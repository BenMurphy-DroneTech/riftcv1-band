/*
 * rift_ts: timestamp pairs and the band's model of the beacon clock.
 *
 * TIMER3 counts at 16 MHz from the crystal, which stays on for good. PPI
 * captures it in CC0 at RADIO ADDRESS, but only the first ADDRESS after each
 * TXREADY (the capture channel is in a group that TXREADY enables and the
 * capture itself disables), so the acknowledgement's own ADDRESS never
 * overwrites the packet's. A retried packet's capture is its last attempt,
 * the one the receiver timed. CC1-CC3 belong to rift_pulse, CC4 captures now
 * (for rift_pulse's interrupt and rift_ts_now) and CC5 the IMU's interrupt
 * edge (rift_imu).
 *
 * Every new single-attempt pair refits the model in a work item: a least-squares
 * line through the newest FIT_PAIRS of them, refitted once without any pair
 * more than OUTLIER_NS off the first line. The fit has its own work queue: the
 * system one's 256-byte stack is far too small for the double maths.
 */
#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <nrfx_ppi.h>

#include "rift_ts.h"
#include "rift_pulse.h"

#define PAIRS       64      /* kept                                              */
#define FIT_PAIRS   32      /* fitted, the newest single-attempt pairs           */
#define MODEL_PAIRS 8       /* fewer than this: no model to pulse from           */
#define OUTLIER_NS  1000    /* dropped from the refit                            */
#define GAP_MS      100000  /* a longer gap starts again (TIMER3 wraps in 268 s) */

struct pair {
	uint64_t band;     /* TIMER3, 1/16 us, unwrapped     */
	uint64_t beacon;   /* beacon clock, 1/16 us          */
	uint32_t attempts;
};

struct fit {
	uint32_t n;
	double mx, my, b;  /* line through (mx, my), slope b, both from the first pair */
	double rms, worst; /* residuals, 1/16 us                                       */
};

static struct pair pairs[PAIRS];
static uint32_t n_pairs, n_retried, n_no_capture, n_held, n_outliers;
static uint32_t period_us;
static uint64_t band_hi;
static uint32_t band_last;
static int64_t last_ms, hold_until;

static struct rift_model model;
static struct k_work fit_work;
static struct k_work_q fit_queue;
static K_THREAD_STACK_DEFINE(fit_stack, 1536);

static nrf_ppi_channel_t addr_ch, txready_ch;
static nrf_ppi_channel_group_t addr_group;
static struct onoff_client hfxo_client;

static void fit_model(struct k_work *work);

static int rift_ts_init(void)
{
	/* The timer has to count the crystal, and ESB releases its own request
	 * after every packet: hold one for good */
	struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);

	sys_notify_init_spinwait(&hfxo_client.notify);
	onoff_request(mgr, &hfxo_client);

	NRF_TIMER3->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER3->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER3->PRESCALER = 0; /* 16 MHz */
	NRF_TIMER3->TASKS_CLEAR = 1;
	NRF_TIMER3->TASKS_START = 1;

	struct k_work_queue_config fit_cfg = {.name = "rift_fit"};

	k_work_queue_init(&fit_queue);
	k_work_queue_start(&fit_queue, fit_stack, K_THREAD_STACK_SIZEOF(fit_stack),
			   K_PRIO_PREEMPT(5), &fit_cfg); /* above ESB and the sensor loop */
	k_work_init(&fit_work, fit_model);

	if (nrfx_ppi_channel_alloc(&addr_ch) != NRFX_SUCCESS ||
	    nrfx_ppi_channel_alloc(&txready_ch) != NRFX_SUCCESS ||
	    nrfx_ppi_group_alloc(&addr_group) != NRFX_SUCCESS) {
		printk("rift_ts: no PPI channels\n");
		return 0;
	}
	NRF_PPI->CH[addr_ch].EEP = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
	NRF_PPI->CH[addr_ch].TEP = (uint32_t)&NRF_TIMER3->TASKS_CAPTURE[0];
	NRF_PPI->FORK[addr_ch].TEP = (uint32_t)&NRF_PPI->TASKS_CHG[addr_group].DIS;
	NRF_PPI->CHG[addr_group] = BIT(addr_ch);
	NRF_PPI->CH[txready_ch].EEP = (uint32_t)&NRF_RADIO->EVENTS_TXREADY;
	NRF_PPI->CH[txready_ch].TEP = (uint32_t)&NRF_PPI->TASKS_CHG[addr_group].EN;
	NRF_PPI->CHENSET = BIT(txready_ch);

	rift_pulse_init();
	return 0;
}

SYS_INIT(rift_ts_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

uint32_t rift_ts_now(void)
{
	unsigned int key = irq_lock();   /* (rift_pulse's interrupt captures CC4 too) */

	NRF_TIMER3->TASKS_CAPTURE[4] = 1;

	uint32_t t = NRF_TIMER3->CC[4];

	irq_unlock(key);
	return t;
}

void rift_ts_ack(const uint8_t *data, uint8_t length, uint32_t attempts)
{
	if (length != RIFT_ACK_LEN || data[0] != RIFT_ACK_TIME) {
		return;
	}
	if (NRF_PPI->CHEN & BIT(addr_ch)) {
		n_no_capture++;   /* no transmit ADDRESS since the last TXREADY */
		return;
	}

	uint32_t c = NRF_TIMER3->CC[0];
	int64_t now = k_uptime_get();

	if (c < band_last) {
		band_hi += 1ull << 32;
	}
	band_last = c;
	if (now < hold_until) {
		n_held++;
		return;
	}

	uint32_t frame = sys_get_le32(&data[1]);
	uint32_t offset = sys_get_le24(&data[5]);
	uint32_t period = sys_get_le16(&data[8]);
	uint64_t beacon = (uint64_t)frame * period * 16 + offset;
	uint64_t band = band_hi | c;
	const struct pair *prev = &pairs[(n_pairs + PAIRS - 1) % PAIRS];

	/* Start again after a long gap, a new period or the beacon clock's wrap */
	if (n_pairs && (now - last_ms > GAP_MS || period != period_us || beacon <= prev->beacon)) {
		n_pairs = n_retried = n_no_capture = n_held = n_outliers = 0;
		model.valid = false;
	}
	last_ms = now;
	period_us = period;

	struct pair *p = &pairs[n_pairs % PAIRS];

	p->band = band;
	p->beacon = beacon;
	p->attempts = attempts;
	n_pairs++;
	if (attempts > 1) {
		n_retried++;
	} else {
		k_work_submit_to_queue(&fit_queue, &fit_work);
	}
}

/* The newest single-attempt pairs, oldest first, into f; returns how many.
 * Call with interrupts locked. */
static uint32_t newest_pairs(struct pair *f)
{
	uint32_t n = 0;

	for (uint32_t i = 0; i < MIN(n_pairs, PAIRS) && n < FIT_PAIRS; i++) {
		const struct pair *p = &pairs[(n_pairs - 1 - i) % PAIRS];

		if (p->attempts == 1) {
			f[FIT_PAIRS - 1 - n++] = *p;
		}
	}
	memmove(f, &f[FIT_PAIRS - n], n * sizeof(*f));
	return n;
}

static double resid(const struct pair *f, uint32_t i, const struct fit *l)
{
	return (double)(f[i].beacon - f[0].beacon) - l->my -
	       l->b * ((double)(f[i].band - f[0].band) - l->mx);
}

/* Least squares, beacon time against band time, both from the first pair; pairs
 * with keep[i] false are left out */
static void fit_line(const struct pair *f, uint32_t n, const bool *keep, struct fit *l)
{
	double sx = 0, sy = 0, sxx = 0, sxy = 0;

	l->n = 0;
	for (uint32_t i = 0; i < n; i++) {
		if (keep && !keep[i]) {
			continue;
		}
		sx += (double)(f[i].band - f[0].band);
		sy += (double)(f[i].beacon - f[0].beacon);
		l->n++;
	}
	l->mx = sx / l->n;
	l->my = sy / l->n;
	for (uint32_t i = 0; i < n; i++) {
		if (keep && !keep[i]) {
			continue;
		}
		double x = (double)(f[i].band - f[0].band) - l->mx;
		double y = (double)(f[i].beacon - f[0].beacon) - l->my;

		sxx += x * x;
		sxy += x * y;
	}
	l->b = sxx > 0 ? sxy / sxx : 1.0;

	double ss = 0;

	l->worst = 0;
	for (uint32_t i = 0; i < n; i++) {
		if (keep && !keep[i]) {
			continue;
		}
		double r = resid(f, i, l);

		ss += r * r;
		l->worst = MAX(l->worst, fabs(r));
	}
	l->rms = sqrt(ss / l->n);
}

static void fit_model(struct k_work *work)
{
	static struct pair f[FIT_PAIRS];
	static bool keep[FIT_PAIRS];
	struct fit l;
	unsigned int key = irq_lock();
	uint32_t n = newest_pairs(f);
	uint32_t period = period_us;
	int64_t last = last_ms;

	irq_unlock(key);
	if (n < 3) {
		return;
	}
	fit_line(f, n, NULL, &l);

	uint32_t dropped = 0;

	for (uint32_t i = 0; i < n; i++) {
		keep[i] = fabs(resid(f, i, &l)) <= OUTLIER_NS / 62.5;
		dropped += !keep[i];
	}
	if (dropped && n - dropped >= 3) {
		fit_line(f, n, keep, &l);
	}

	struct rift_model m = {
		.valid = l.n >= MODEL_PAIRS,
		.pairs = l.n,
		.band_at = (uint32_t)f[n - 1].band,
		.beacon_at = (double)f[0].beacon + l.my +
			     l.b * ((double)(f[n - 1].band - f[0].band) - l.mx),
		.rate = l.b,
		.period_us = period,
		.last_ms = last,
		.rms_ns = (uint32_t)(l.rms * 62.5 + 0.5),
	};

	key = irq_lock();
	if (n_pairs) {          /* not restarted meanwhile */
		model = m;
		n_outliers += dropped;
	}
	irq_unlock(key);
}

void rift_ts_model(struct rift_model *m)
{
	unsigned int key = irq_lock();

	*m = model;
	irq_unlock(key);
}

void rift_ts_hold(uint32_t ms)
{
	hold_until = k_uptime_get() + ms;
}

static int32_t ns(double ticks)
{
	return (int32_t)(ticks * 62.5 + (ticks < 0 ? -0.5 : 0.5));
}

void rift_ts_print(void)
{
	static struct pair f[FIT_PAIRS];
	uint32_t n, total, retried, no_capture, period;
	unsigned int key = irq_lock();

	total = n_pairs;
	retried = n_retried;
	no_capture = n_no_capture;
	period = period_us;
	n = newest_pairs(f);
	irq_unlock(key);

	printk("PAIRS %u fitted of %u (%u retried, %u without a capture, %u outliers dropped)", n,
	       total, retried, no_capture, n_outliers);
	if (n < 3) {
		printk("\n");
		return;
	}

	struct fit l;

	fit_line(f, n, NULL, &l);

	int64_t ppb = (int64_t)((l.b - 1.0) * 1e9);
	uint64_t ppb_abs = ppb < 0 ? -ppb : ppb;
	uint32_t span_ms = (uint32_t)((f[n - 1].band - f[0].band) / 16000);

	printk(" over %u ms: rate %c%u.%03u ppm, residuals rms %d ns, max %d ns\n", span_ms,
	       ppb < 0 ? '-' : '+', (uint32_t)(ppb_abs / 1000), (uint32_t)(ppb_abs % 1000),
	       ns(l.rms), ns(l.worst));
	k_msleep(5);
	printk("  band (us)            beacon frame + offset (us)   residual\n");
	for (uint32_t i = 0; i < n; i++) {
		uint64_t frame = f[i].beacon / (16ull * period);
		uint32_t off = (uint32_t)(f[i].beacon % (16ull * period));

		printk("  %llu.%04u  %llu + %u.%04u  %+d ns\n", f[i].band / 16,
		       (uint32_t)(f[i].band % 16) * 625, frame, off / 16, (off % 16) * 625,
		       ns(resid(f, i, &l)));
		k_msleep(5); /* printk goes through the 1.5 kB log buffer: let it drain */
	}
}
