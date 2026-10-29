/*
 * rift_pulse: the LED pulse on every camera frame boundary, placed in hardware
 * from the clock model (rift_ts), on the test band's D10 (P1.15) and D9 (P1.14).
 *
 * TIMER3 (rift_ts's 16 MHz timer) compares:
 *   CC1  LED on:  PPI to GPIOTE SET on D10, forked to SET on D9
 *   CC2  LED off: PPI to GPIOTE CLR on both
 *   CC3  wake-up interrupt WAKE_US before the next pulse: it converts that
 *        frame's boundary to TIMER3 ticks from the model, loads CC1 and CC2,
 *        and sets CC3 for the frame after
 *   CC4  the interrupt's capture of now (rift_ts_now uses it too)
 * A boundary is k * period on the beacon clock; the pulse is centred at the
 * boundary + at + trim (at: a test shift, trim: the fixed radio and driver
 * delays). The PPI channels are on only while pulses are being placed, so a
 * stale compare never lights the LEDs. A wake-up that finds the pulse less than
 * MIN_LEAD_US away skips that frame rather than pulsing late. With no model,
 * or no new pair for HOLDOVER_MS, the band stays dark.
 */
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/sys/printk.h>
#include <nrfx_ppi.h>
#include <nrfx_gpiote.h>

#include "rift_ts.h"
#include "rift_pulse.h"

#define WAKE_US      1000   /* wake-up before the pulse                */
#define MIN_LEAD_US  50     /* less than this to the pulse: skip it    */
#define HOLDOVER_MS  500    /* pulse on the last model this long       */
#define WIDTH_US     215
/* The fixed delays, from rift_test.py sweep edges --band against the beacon's
 * own flash: the band pulsed 5.3 us early (the receiver's RX ADDRESS event
 * comes later after the packet's start than the band's TX ADDRESS, plus the
 * test band's driver) */
#define TRIM_NS      5300

static const uint32_t pins[] = {15, 14};   /* P1.15 = D10, P1.14 = D9 */

static bool enabled = true;
static uint32_t width_us = WIDTH_US;
static int32_t at_ns, trim_ns = TRIM_NS;
static uint32_t holdover_ms = HOLDOVER_MS;

static nrf_ppi_channel_t on_ch, off_ch;
static uint8_t gpiote_ch[ARRAY_SIZE(pins)];
static bool ready, pulsing;
static uint32_t n_placed, n_late, n_dark;
static int32_t lead_min_us = INT32_MAX;

static inline uint32_t now_isr(void)
{
	NRF_TIMER3->TASKS_CAPTURE[4] = 1;
	return NRF_TIMER3->CC[4];
}

static void dark(void)
{
	NRF_PPI->CHENCLR = BIT(on_ch) | BIT(off_ch);
	for (int i = 0; i < ARRAY_SIZE(pins); i++) {
		NRF_GPIOTE->TASKS_CLR[gpiote_ch[i]] = 1;
	}
	pulsing = false;
}

static void timer3_isr(const void *arg)
{
	ARG_UNUSED(arg);
	NRF_TIMER3->EVENTS_COMPARE[3] = 0;

	struct rift_model m;
	uint32_t now = now_isr();

	rift_ts_model(&m);
	if (!enabled || !m.valid || k_uptime_get() - m.last_ms > holdover_ms) {
		dark();
		n_dark++;
		NRF_TIMER3->CC[3] = now + 16 * (m.period_us ? m.period_us : 11111);
		return;
	}

	/* The next pulse at least MIN_LEAD_US away, on the beacon clock */
	double period = 16.0 * m.period_us;
	double first = (at_ns + trim_ns) / 62.5 - 8.0 * width_us; /* from the boundary */
	double beacon_now = m.beacon_at + m.rate * (double)(int32_t)(now - m.band_at);
	double k = ceil((beacon_now + 16.0 * MIN_LEAD_US - first) / period);
	double on_beacon = k * period + first;
	uint32_t on = m.band_at + (uint32_t)(int32_t)llround((on_beacon - m.beacon_at) / m.rate);
	uint32_t off = on + (uint32_t)llround(16.0 * width_us / m.rate);
	int32_t lead = (int32_t)(on - now_isr());

	if (lead < 16 * MIN_LEAD_US) {
		n_late++;       /* (the wake-up ran late: skip it, never pulse late) */
	} else {
		NRF_TIMER3->CC[1] = on;
		NRF_TIMER3->CC[2] = off;
		if (!pulsing) {
			NRF_PPI->CHENSET = BIT(on_ch) | BIT(off_ch);
			pulsing = true;
		}
		n_placed++;
		lead_min_us = MIN(lead_min_us, lead / 16);
	}
	uint32_t wake = on + (uint32_t)llround(period / m.rate) - 16 * WAKE_US;

	/* A wake-up already past would wait for the timer to wrap (268 s) */
	if ((int32_t)(wake - now_isr()) < 16 * 100) {
		wake = now_isr() + 16 * 100;
	}
	NRF_TIMER3->CC[3] = wake;
}

void rift_pulse_init(void)
{
	static const nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(0);

	for (int i = 0; i < ARRAY_SIZE(pins); i++) {
		if (nrfx_gpiote_channel_alloc(&gpiote, &gpiote_ch[i]) != NRFX_SUCCESS) {
			printk("rift_pulse: no GPIOTE channel\n");
			return;
		}
	}
	if (nrfx_ppi_channel_alloc(&on_ch) != NRFX_SUCCESS ||
	    nrfx_ppi_channel_alloc(&off_ch) != NRFX_SUCCESS) {
		printk("rift_pulse: no PPI channels\n");
		return;
	}
	for (int i = 0; i < ARRAY_SIZE(pins); i++) {
		/* High-drive output, then hand the pin to a GPIOTE task channel */
		NRF_P1->OUTCLR = BIT(pins[i]);
		NRF_P1->PIN_CNF[pins[i]] =
			(GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
			(GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
			(GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
			(GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos);
		NRF_GPIOTE->CONFIG[gpiote_ch[i]] =
			(GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos) |
			(pins[i] << GPIOTE_CONFIG_PSEL_Pos) |
			(1 << GPIOTE_CONFIG_PORT_Pos) |
			(GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos) |
			(GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos);
	}
	NRF_PPI->CH[on_ch].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[1];
	NRF_PPI->CH[on_ch].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[gpiote_ch[0]];
	NRF_PPI->FORK[on_ch].TEP = (uint32_t)&NRF_GPIOTE->TASKS_SET[gpiote_ch[1]];
	NRF_PPI->CH[off_ch].EEP = (uint32_t)&NRF_TIMER3->EVENTS_COMPARE[2];
	NRF_PPI->CH[off_ch].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[gpiote_ch[0]];
	NRF_PPI->FORK[off_ch].TEP = (uint32_t)&NRF_GPIOTE->TASKS_CLR[gpiote_ch[1]];

	IRQ_CONNECT(TIMER3_IRQn, 1, timer3_isr, NULL, 0);
	NRF_TIMER3->CC[3] = now_isr() + 16 * 10000;
	NRF_TIMER3->EVENTS_COMPARE[3] = 0;
	NRF_TIMER3->INTENSET = TIMER_INTENSET_COMPARE3_Msk;
	irq_enable(TIMER3_IRQn);
	ready = true;
}

static void print_ns(const char *label, int32_t v)
{
	uint32_t a = v < 0 ? -v : v;

	printk("%s %c%u.%03u us", label, v < 0 ? '-' : '+', a / 1000, a % 1000);
}

static void print_status(void)
{
	struct rift_model m;

	rift_ts_model(&m);

	int64_t age = k_uptime_get() - m.last_ms;
	int64_t ppb = (int64_t)((m.rate - 1.0) * 1e9);
	uint64_t ppb_abs = ppb < 0 ? -ppb : ppb;
	bool lit = enabled && m.valid && age <= holdover_ms;

	printk("PULSE %s, width %u us,", !enabled ? "off" : lit ? "on" : "on (dark)", width_us);
	print_ns(" at", at_ns);
	print_ns(", trim", trim_ns);
	printk(", holdover %u ms; model %s, %u pairs, rate %c%u.%03u ppm, rms %u ns, "
	       "last pair %lld ms ago; placed %u, late %u, dark %u, lead down to %d us\n",
	       holdover_ms, m.valid ? "good" : "not yet", m.pairs, ppb < 0 ? '-' : '+',
	       (uint32_t)(ppb_abs / 1000), (uint32_t)(ppb_abs % 1000), m.rms_ns,
	       m.last_ms ? age : -1, n_placed, n_late, n_dark,
	       lead_min_us == INT32_MAX ? -1 : lead_min_us);
}

/* A number of microseconds, maybe with a fraction, as nanoseconds */
static bool parse_us(const char *s, int32_t lo, int32_t hi, int32_t *ns_out)
{
	char *end;
	double v = strtod(s, &end);

	if (end == s || *end || v < lo || v > hi) {
		return false;
	}
	*ns_out = (int32_t)lround(v * 1000);
	return true;
}

void rift_pulse_command(int argc, char **argv)
{
	int32_t v;

	if (!ready) {
		printk("PULSE not set up\n");
		return;
	}
	if (argc == 2 && !strcmp(argv[1], "on")) {
		enabled = true;
	} else if (argc == 2 && !strcmp(argv[1], "off")) {
		enabled = false;
	} else if (argc == 3 && !strcmp(argv[1], "width") && parse_us(argv[2], 1, 2000, &v)) {
		width_us = (v + 500) / 1000;
	} else if (argc == 3 && !strcmp(argv[1], "at") && parse_us(argv[2], -20000, 20000, &v)) {
		at_ns = v;
	} else if (argc == 3 && !strcmp(argv[1], "trim") && parse_us(argv[2], -1000, 1000, &v)) {
		trim_ns = v;
	} else if (argc == 3 && !strcmp(argv[1], "holdover") && parse_us(argv[2], 0, 300000, &v)) {
		holdover_ms = (v + 500) / 1000; /* (a number of ms, read as if us) */
	} else if (argc == 2 && !strcmp(argv[1], "reset")) {
		n_placed = n_late = n_dark = 0;
		lead_min_us = INT32_MAX;
	} else if (argc != 1) {
		printk("pulse [on|off | width <us> | at <us> | trim <us> | holdover <ms> | reset]\n");
		return;
	}
	print_status();
}

void rift_pulse_hold_command(int argc, char **argv)
{
	char *end;
	long ms = argc == 2 ? strtol(argv[1], &end, 10) : -1;

	if (argc != 2 || *end || ms < 0 || ms > 300000) {
		printk("hold <ms>: ignore acknowledgements for 0 to 300000 ms\n");
		return;
	}
	rift_ts_hold(ms);
	printk("HOLD %ld ms\n", ms);
}
