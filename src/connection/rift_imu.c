/*
 * rift_imu: the IMU's samples timed on TIMER3 and cut into half-frame slices
 * on the clock model (rift_ts).
 *
 * Sample timing. The IMU's FIFO-threshold interrupt edge is captured in TIMER3
 * CC5 through PPI (from the GPIOTE IN event Zephyr's GPIO driver set up for
 * it), so each read gives a pair: the edge's time, and how many gyro samples
 * the IMU had produced at the edge (the count read just after, less the
 * samples that came out between the edge and the count). A read without a
 * new edge gives none. The lower envelope of (edge time - period x count)
 * over the window gives the sample clock's phase, and the slope between its
 * two halves the period, followed slowly. Without the edge capture (no
 * interrupt pin) the count read's own time stands in, which Smol Slime's
 * interrupt-driven reads bias by their wake-up delay. (The LSM6DSV32X's FIFO
 * timestamps could replace all this later.)
 *
 * Slices. Each gyro sample covers the period up to its time; on the beacon
 * clock it is split at every half-frame boundary (k x period / 2), and each
 * part is integrated into the slice it falls in: the rotation change dq
 * (q = q x exp(w dt)) and the velocity change dv (dv += R(dq) a dt, gravity
 * included), both in the band's frame at the slice's start. The accelerometer
 * reading held is the latest one, so differing ODRs need nothing special.
 */
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <soc.h>
#include <nrfx_gpiote.h>
#include <nrfx_ppi.h>

#include "rift_ts.h"
#include "rift_imu.h"

#define INT_NODE DT_PATH(zephyr_user)   /* Smol Slime's int0-gpios: the IMU's INT1 */

#define READS        512    /* timing window, in FIFO reads (about 3 s at 167 reads/s) */
#define READS_GOOD   64     /* fewer: the timing isn't trusted yet                      */
#define PERIOD_GAIN  32     /* settled, the period takes 1/32 of each new estimate      */
#define BUF          160    /* samples per read kept                                    */
#define SLICES       64     /* kept for the imu command                                 */
#define LATENCY_TICKS 0     /* count read to TIMER3 capture (the driver's hook is close) */
#define G            9.80665f

struct read { uint32_t t, n; };

struct slice {
	uint32_t k;        /* half-frame number on the beacon clock */
	float dq[4];       /* w x y z                               */
	float dv[3];       /* m/s                                   */
	float dt;          /* s of samples in it                    */
	uint16_t parts;
};

/* timing */
static struct read reads[READS];
static uint32_t n_reads, head;        /* reads kept, next slot                 */
static uint32_t cum;                  /* gyro samples produced before this read */
static float period;                  /* ticks per sample                       */
static float nominal_s;
static uint32_t env_t0, env_n0;       /* the oldest read kept                   */
static float env;                     /* lower envelope, from env_t0            */
static float env_step_max;            /* biggest envelope move between reads    */
static uint32_t n_resets;
static uint32_t t_count;              /* this read's capture                    */
static bool int_ready, int_failed;    /* the edge capture                       */
static uint32_t t_edge_last, n_no_edge;

/* this read's samples */
static float gbuf[BUF][3], abuf[BUF][3];
static uint16_t a_at[BUF];            /* gyro samples before each accel sample */
static uint16_t n_g, n_a;

/* slices */
static bool in_slice;
static struct slice cur;
static float a_held[3];
static struct slice done[SLICES];
static uint32_t n_done, n_partial, n_skipped;
static uint16_t parts_min = UINT16_MAX, parts_max;

/* the zero check: slices composed against the fusion */
static float q_fused[4] = {1, 0, 0, 0}, q_zero[4] = {1, 0, 0, 0}, q_comp[4] = {1, 0, 0, 0};
static uint32_t comp_slices;
static float bias[3];   /* the fusion's gyro bias estimate, deg/s */

static void q_mul(float *r, const float *a, const float *b)
{
	float w = a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3];
	float x = a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2];
	float y = a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1];
	float z = a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0];

	r[0] = w; r[1] = x; r[2] = y; r[3] = z;
}

static void q_norm(float *q)
{
	float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

	for (int i = 0; i < 4; i++) {
		q[i] /= n;
	}
}

/* v rotated by q */
static void q_rot(float *r, const float *q, const float *v)
{
	float p[4] = {0, v[0], v[1], v[2]}, qc[4] = {q[0], -q[1], -q[2], -q[3]}, t[4], u[4];

	q_mul(t, q, p);
	q_mul(u, t, qc);
	r[0] = u[1]; r[1] = u[2]; r[2] = u[3];
}

static float q_angle_deg(const float *q)
{
	float v = sqrtf(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

	return 2.0f * atan2f(v, fabsf(q[0])) * 57.29578f;
}

/* Once the sensor has set up its interrupt: its GPIOTE IN event to TIMER3 CC5 */
static void edge_setup(void)
{
#if DT_NODE_HAS_PROP(INT_NODE, int0_gpios)
	static const nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(0);
	uint8_t ch;
	nrf_ppi_channel_t ppi;

	if (nrfx_gpiote_channel_get(&gpiote, NRF_DT_GPIOS_TO_PSEL(INT_NODE, int0_gpios), &ch) !=
	    NRFX_SUCCESS) {
		return;   /* (not set up yet, or by port sense: try again next read) */
	}
	if (nrfx_ppi_channel_alloc(&ppi) != NRFX_SUCCESS) {
		int_failed = true;
		return;
	}
	NRF_PPI->CH[ppi].EEP = (uint32_t)&NRF_GPIOTE->EVENTS_IN[ch];
	NRF_PPI->CH[ppi].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[5];
	NRF_PPI->CHENSET = BIT(ppi);
	t_edge_last = NRF_TIMER3->CC[5];
	int_ready = true;
#else
	int_failed = true;
#endif
}

void rift_imu_read_start(void)
{
	if (!int_ready && !int_failed) {
		edge_setup();
	}
	t_count = rift_ts_now();
	n_g = n_a = 0;
}

void rift_imu_count_read(void)
{
	t_count = rift_ts_now() - LATENCY_TICKS;
}

void rift_imu_gyro(const float g[3])
{
	if (n_g < BUF) {
		memcpy(gbuf[n_g], g, sizeof(gbuf[0]));
	}
	n_g++;
}

void rift_imu_accel(const float a[3])
{
	if (n_a < BUF) {
		memcpy(abuf[n_a], a, sizeof(abuf[0]));
		a_at[n_a] = n_g;
	}
	n_a++;
}

void rift_imu_quat(const float q[4], const float gyro_bias[3])
{
	memcpy(q_fused, q, sizeof(q_fused));
	for (int i = 0; i < 3; i++) {
		bias[i] = gyro_bias[i] * 57.29578f;
	}
}

static void timing_reset(void)
{
	n_reads = head = 0;
	env_step_max = 0;
	in_slice = false;
	n_resets++;
}

static inline const struct read *read_at(uint32_t i) /* 0 = oldest */
{
	return &reads[(head + READS - n_reads + i) % READS];
}

static float env_of(const struct read *r)
{
	return (float)(int32_t)(r->t - env_t0) - period * (float)(int32_t)(r->n - env_n0);
}

static uint32_t sample_time(uint32_t m);

/* The new read into the window; the envelope and period refitted */
static void timing_add(uint32_t t, uint32_t n)
{
	bool track = n_reads >= READS_GOOD;
	uint32_t before = track ? sample_time(n) : 0;

	if (n_reads) {
		float e = (float)(int32_t)(t - env_t0) - period * (float)(int32_t)(n - env_n0);

		/* A sample earlier than the envelope, or a read long after: the
		 * FIFO was reset or the rate changed, start again */
		if (e < env - period / 2 || e > env + 40 * period) {
			timing_reset();
		}
	}
	reads[head] = (struct read){t, n};
	head = (head + 1) % READS;
	if (n_reads < READS) {
		n_reads++;
	}
	env_t0 = read_at(0)->t;
	env_n0 = read_at(0)->n;
	if (n_reads < 8) {
		env = 0;
		for (uint32_t i = 0; i < n_reads; i++) {
			env = MIN(env, env_of(read_at(i)));
		}
		return;
	}

	/* Period: the slope between the two halves' envelopes */
	uint32_t half = n_reads / 2;
	float e1 = INFINITY, e2 = INFINITY;
	uint32_t n1 = 0, n2 = 0;

	for (uint32_t i = 0; i < n_reads; i++) {
		const struct read *r = read_at(i);
		float e = env_of(r);

		if (i < half && e < e1) {
			e1 = e;
			n1 = r->n;
		} else if (i >= half && e < e2) {
			e2 = e;
			n2 = r->n;
		}
	}
	if (n2 > n1) {
		/* each half's minimum is noisy by a sample period over its reads, and
		 * the IMU's rate hardly moves: follow it slowly once settled */
		period += (e2 - e1) / (float)(n2 - n1) / (n_reads >= READS_GOOD ? PERIOD_GAIN : 1);
	}

	/* Phase: the whole window's envelope, with that period */
	env = INFINITY;
	for (uint32_t i = 0; i < n_reads; i++) {
		env = MIN(env, env_of(read_at(i)));
	}
	/* How far this read moved the newest sample's time */
	if (track && n_reads >= READS_GOOD) {
		env_step_max = MAX(env_step_max, fabsf((float)(int32_t)(sample_time(n) - before)));
	}
}

/* TIMER3 when gyro sample m came out */
static uint32_t sample_time(uint32_t m)
{
	return env_t0 + (uint32_t)(int32_t)lroundf(env + period * (float)(int32_t)(m + 1 - env_n0));
}

static float half_s;   /* a slice's length, s */

static void slice_close(void)
{
	if (cur.parts) {
		q_norm(cur.dq);
		done[n_done % SLICES] = cur;
		n_done++;
		if (cur.dt < 0.99f * half_s) {
			n_partial++;   /* (samples missing from it) */
		}
		parts_min = MIN(parts_min, cur.parts);
		parts_max = MAX(parts_max, cur.parts);
		float t[4];

		q_mul(t, q_comp, cur.dq);
		q_norm(t);
		memcpy(q_comp, t, sizeof(t));
		comp_slices++;
	}
}

static void slice_open(uint32_t k)
{
	cur = (struct slice){.k = k, .dq = {1, 0, 0, 0}};
}

/* One part of a gyro sample, dt_s long, into the current slice */
static void integrate(const float w[3], float dt_s)
{
	float h = 0.5f * dt_s;
	float e[4] = {1, w[0] * h, w[1] * h, w[2] * h}, t[4], a[3];

	q_rot(a, cur.dq, a_held);
	for (int i = 0; i < 3; i++) {
		cur.dv[i] += a[i] * G * dt_s;
	}
	q_mul(t, cur.dq, e);
	memcpy(cur.dq, t, sizeof(t));
	cur.dt += dt_s;
	cur.parts++;
}

void rift_imu_read_end(float gyro_period_s)
{
	struct rift_model m;
	uint16_t ng = MIN(n_g, BUF), na = MIN(n_a, BUF);

	if (gyro_period_s != nominal_s || !n_reads) {
		nominal_s = gyro_period_s;
		period = gyro_period_s * 16e6f;
		if (n_reads) {
			timing_reset();
		}
	}
	if (n_g != ng) {
		timing_reset();   /* (more than BUF in one read: lost track) */
	}
	if (int_ready) {
		uint32_t t_edge = NRF_TIMER3->CC[5];
		int32_t since = (int32_t)(t_count - t_edge);

		if (t_edge == t_edge_last) {
			n_no_edge++;   /* no new edge: the FIFO stayed over its threshold */
		} else if (since >= 0) {
			/* (an edge after the count read belongs to the next read) */
			uint32_t after = (uint32_t)((float)since / period);

			timing_add(t_edge, cum + n_g - MIN(after, n_g));
			t_edge_last = t_edge;
		}
	} else {
		timing_add(t_count, cum + n_g);
	}

	rift_ts_model(&m);
	bool timed = n_reads >= READS_GOOD && m.valid;
	double half = 8.0 * m.period_us;    /* half a frame, 1/16 us */

	half_s = m.period_us / 2e6f;
	uint16_t ai = 0;

	for (uint16_t j = 0; j < ng; j++) {
		while (ai < na && a_at[ai] <= j) {
			memcpy(a_held, abuf[ai++], sizeof(a_held));
		}
		if (!timed) {
			continue;
		}

		/* The sample's span on the beacon clock */
		uint32_t s1 = sample_time(cum + j);
		double x1 = m.beacon_at + m.rate * (double)(int32_t)(s1 - m.band_at);
		double x0 = x1 - m.rate * (double)period;
		float w[3];

		for (int i = 0; i < 3; i++) {
			w[i] = (gbuf[j][i] - bias[i]) * 0.01745329f;
		}
		uint32_t k0 = (uint32_t)floor(x0 / half);

		if (!in_slice || cur.k != k0) {
			if (in_slice && cur.k + 1 != k0) {
				n_skipped++;
			}
			if (in_slice) {
				slice_close();
			}
			slice_open(k0);
			in_slice = true;
		}
		double x = x0;

		while ((cur.k + 1) * half < x1) {
			double b = (cur.k + 1) * half;

			integrate(w, (float)((b - x) / 16e6));
			slice_close();
			slice_open(cur.k + 1);
			x = b;
		}
		integrate(w, (float)((x1 - x) / 16e6));
	}
	while (ai < na) {
		memcpy(a_held, abuf[ai++], sizeof(a_held));
	}
	cum += n_g;
}

static void print_f(float v, int decimals)
{
	int32_t s = 1;

	for (int i = 0; i < decimals; i++) {
		s *= 10;
	}
	int32_t x = (int32_t)lroundf(v * s);
	uint32_t a = x < 0 ? -x : x;

	printk("%c%u.%0*u", x < 0 ? '-' : '+', a / s, decimals, a % s);
}

void rift_imu_command(int argc, char **argv)
{
	if (argc == 2 && !strcmp(argv[1], "zero")) {
		unsigned int key = irq_lock();

		memcpy(q_zero, q_fused, sizeof(q_zero));
		q_comp[0] = 1;
		q_comp[1] = q_comp[2] = q_comp[3] = 0;
		comp_slices = 0;
		irq_unlock(key);
		printk("IMU zeroed\n");
		return;
	}
	if (argc != 1) {
		printk("imu [zero]\n");
		return;
	}

	static struct slice last[8];
	unsigned int key = irq_lock();
	uint32_t made = n_done, nr = n_reads, resets = n_resets, skipped = n_skipped;
	uint32_t partial = n_partial;
	float per = period, step = env_step_max, nom = nominal_s;
	uint16_t pmin = parts_min, pmax = parts_max;
	uint32_t nl = MIN(made, ARRAY_SIZE(last));
	float qz[4], qf[4], qc[4];
	uint32_t comp = comp_slices;

	for (uint32_t i = 0; i < nl; i++) {
		last[i] = done[(made - nl + i) % SLICES];
	}
	memcpy(qz, q_zero, sizeof(qz));
	memcpy(qf, q_fused, sizeof(qf));
	memcpy(qc, q_comp, sizeof(qc));
	env_step_max = 0;
	irq_unlock(key);

	/* The fusion's rotation since zero, in the band's frame: qz^-1 qf */
	float qzi[4] = {qz[0], -qz[1], -qz[2], -qz[3]}, rel[4], err[4], qci[4] = {qc[0], -qc[1], -qc[2], -qc[3]};

	q_mul(rel, qzi, qf);
	q_mul(err, qci, rel);

	printk("IMU gyro ");
	print_f(16e6f / per, 2);
	printk(" Hz (nominal ");
	print_f(1.0f / nom, 2);
	printk(" Hz), %u reads timed %s (%u without a new edge), envelope steps up to %d ns, "
	       "%u timing restarts; %u slices (%u short), %u gaps, parts %u-%u\n", nr,
	       int_ready ? "by the interrupt edge" : "by the count read", n_no_edge,
	       (int)(step * 62.5f), resets, made, partial, skipped,
	       pmin == UINT16_MAX ? 0 : pmin, pmax);
	k_msleep(5);
	printk("IMU since zero: %u slices composed ", comp);
	print_f(q_angle_deg(qc), 2);
	printk(" deg, fusion ");
	print_f(q_angle_deg(rel), 2);
	printk(" deg, difference ");
	print_f(q_angle_deg(err), 2);
	printk(" deg\n");
	k_msleep(5);
	for (uint32_t i = 0; i < nl; i++) {
		const struct slice *s = &last[i];
		float n = sqrtf(s->dv[0] * s->dv[0] + s->dv[1] * s->dv[1] + s->dv[2] * s->dv[2]);

		printk("  frame %u.%u  %u parts ", s->k / 2, s->k % 2, s->parts);
		print_f(s->dt * 1000, 3);
		printk(" ms  rot ");
		print_f(q_angle_deg(s->dq), 3);
		printk(" deg  dv ");
		print_f(s->dv[0], 4);
		printk(" ");
		print_f(s->dv[1], 4);
		printk(" ");
		print_f(s->dv[2], 4);
		printk(" |");
		print_f(n, 4);
		printk("| m/s\n");
		k_msleep(5);
	}
}
