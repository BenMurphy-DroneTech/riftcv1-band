/*
 * rift_ts: timestamp pairs for the band's model of the beacon clock.
 *
 * TIMER3 counts at 16 MHz from the crystal, which stays on for good. PPI
 * captures it in CC0 at RADIO ADDRESS, but only the first ADDRESS after each
 * TXREADY (the capture channel is in a group that TXREADY enables and the
 * capture itself disables), so the acknowledgement's own ADDRESS never
 * overwrites the packet's. A retried packet's capture is its last attempt,
 * the one the receiver timed.
 */
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <nrfx_ppi.h>

#include "rift_ts.h"

#define PAIRS      64      /* kept                                              */
#define FIT_PAIRS  32      /* fitted, the newest single-attempt pairs           */
#define GAP_MS     100000  /* a longer gap starts again (TIMER3 wraps in 268 s) */

struct pair {
	uint64_t band;     /* TIMER3, 1/16 us, unwrapped     */
	uint64_t beacon;   /* beacon clock, 1/16 us          */
	uint32_t attempts;
};

static struct pair pairs[PAIRS];
static uint32_t n_pairs, n_retried, n_no_capture;
static uint32_t period_us;
static uint64_t band_hi;
static uint32_t band_last;
static int64_t last_ms;

static nrf_ppi_channel_t addr_ch, txready_ch;
static nrf_ppi_channel_group_t addr_group;
static struct onoff_client hfxo_client;

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
	return 0;
}

SYS_INIT(rift_ts_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

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
	uint32_t frame = sys_get_le32(&data[1]);
	uint32_t offset = sys_get_le24(&data[5]);
	uint32_t period = sys_get_le16(&data[8]);
	uint64_t beacon = (uint64_t)frame * period * 16 + offset;

	if (c < band_last) {
		band_hi += 1ull << 32;
	}
	band_last = c;

	uint64_t band = band_hi | c;
	const struct pair *prev = &pairs[(n_pairs + PAIRS - 1) % PAIRS];

	/* Start again after a long gap, a new period or the beacon clock's wrap */
	if (n_pairs && (now - last_ms > GAP_MS || period != period_us || beacon <= prev->beacon)) {
		n_pairs = n_retried = n_no_capture = 0;
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
	}
}

static int32_t ns(double ticks)
{
	return (int32_t)(ticks * 62.5 + (ticks < 0 ? -0.5 : 0.5));
}

void rift_ts_print(void)
{
	static struct pair fit[FIT_PAIRS];
	uint32_t n = 0, total, retried, no_capture, period;
	unsigned int key = irq_lock();

	total = n_pairs;
	retried = n_retried;
	no_capture = n_no_capture;
	period = period_us;
	for (uint32_t i = 0; i < MIN(total, PAIRS) && n < FIT_PAIRS; i++) {
		const struct pair *p = &pairs[(total - 1 - i) % PAIRS];

		if (p->attempts == 1) {
			fit[FIT_PAIRS - 1 - n++] = *p;
		}
	}
	irq_unlock(key);

	const struct pair *f = &fit[FIT_PAIRS - n];

	printk("PAIRS %u fitted of %u (%u retried, %u without a capture)", n, total, retried,
	       no_capture);
	if (n < 3) {
		printk("\n");
		return;
	}

	/* Least squares, beacon time against band time, both from the first pair */
	double sx = 0, sy = 0, sxx = 0, sxy = 0;

	for (uint32_t i = 0; i < n; i++) {
		double x = (double)(f[i].band - f[0].band), y = (double)(f[i].beacon - f[0].beacon);

		sx += x;
		sy += y;
	}
	double mx = sx / n, my = sy / n;

	for (uint32_t i = 0; i < n; i++) {
		double x = (double)(f[i].band - f[0].band) - mx, y = (double)(f[i].beacon - f[0].beacon) - my;

		sxx += x * x;
		sxy += x * y;
	}
	double b = sxy / sxx, ss = 0, worst = 0;

	for (uint32_t i = 0; i < n; i++) {
		double r = (double)(f[i].beacon - f[0].beacon) - my -
			   b * ((double)(f[i].band - f[0].band) - mx);

		ss += r * r;
		worst = MAX(worst, r < 0 ? -r : r);
	}
	int64_t ppb = (int64_t)((b - 1.0) * 1e9);
	uint64_t ppb_abs = ppb < 0 ? -ppb : ppb;
	uint32_t span_ms = (uint32_t)((f[n - 1].band - f[0].band) / 16000);

	printk(" over %u ms: rate %c%u.%03u ppm, residuals rms %d ns, max %d ns\n", span_ms,
	       ppb < 0 ? '-' : '+', (uint32_t)(ppb_abs / 1000), (uint32_t)(ppb_abs % 1000),
	       ns(sqrt(ss / n)), ns(worst));
	k_msleep(5);
	printk("  band (us)            beacon frame + offset (us)   residual\n");
	for (uint32_t i = 0; i < n; i++) {
		double r = (double)(f[i].beacon - f[0].beacon) - my -
			   b * ((double)(f[i].band - f[0].band) - mx);
		uint64_t frame = f[i].beacon / (16ull * period);
		uint32_t off = (uint32_t)(f[i].beacon % (16ull * period));

		printk("  %llu.%04u  %llu + %u.%04u  %+d ns\n", f[i].band / 16,
		       (uint32_t)(f[i].band % 16) * 625, frame, off / 16, (off % 16) * 625, ns(r));
		k_msleep(5); /* printk goes through the 1.5 kB log buffer: let it drain */
	}
}
