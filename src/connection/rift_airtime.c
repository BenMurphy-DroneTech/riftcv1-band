/*
 * rift_airtime: the radio time of each packet the band sends (rift_airtime.h).
 *
 * TIMER1 is free on the band (MPSL holds TIMER0, ESB TIMER2, rift_ts TIMER3,
 * rift_imu TIMER4). TXREADY also re-arms the first-END capture, which disarms
 * itself, as rift_ts does for the transmit ADDRESS.
 */
#include <stdlib.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/printk.h>
#include <nrfx_ppi.h>

#include "rift_airtime.h"
#include "connection.h"
#include "esb.h"

#define SAMPLES 128   /* kept per kind of packet, the newest */
#define KINDS   4

struct kind {
	uint8_t length;
	bool noack;
	uint32_t n, retried, failed, mixed;
	/* 1/16 us: write to TXREADY, TXREADY to END, END to the acknowledgement's
	 * ADDRESS, its ADDRESS to its END, TXREADY to the last END */
	uint16_t ready[SAMPLES], air[SAMPLES], turn[SAMPLES], ack[SAMPLES], total[SAMPLES];
};

static struct kind kinds[KINDS];
static uint32_t n_lost;   /* no TXREADY captured, or no room for another kind */
static bool armed, pending, pending_noack, pending_busy;
static uint8_t pending_length;

static nrf_ppi_channel_t ready_ch, end1_ch, addr_ch, end_ch;
static nrf_ppi_channel_group_t end1_group;

static int rift_airtime_init(void)
{
	NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit;
	NRF_TIMER1->PRESCALER = 0; /* 16 MHz */
	NRF_TIMER1->TASKS_CLEAR = 1;
	NRF_TIMER1->TASKS_START = 1;

	if (nrfx_ppi_channel_alloc(&ready_ch) != NRFX_SUCCESS ||
	    nrfx_ppi_channel_alloc(&end1_ch) != NRFX_SUCCESS ||
	    nrfx_ppi_channel_alloc(&addr_ch) != NRFX_SUCCESS ||
	    nrfx_ppi_channel_alloc(&end_ch) != NRFX_SUCCESS ||
	    nrfx_ppi_group_alloc(&end1_group) != NRFX_SUCCESS) {
		printk("rift_airtime: no PPI channels\n");
		return 0;
	}
	NRF_PPI->CH[ready_ch].EEP = (uint32_t)&NRF_RADIO->EVENTS_TXREADY;
	NRF_PPI->CH[ready_ch].TEP = (uint32_t)&NRF_TIMER1->TASKS_CAPTURE[0];
	NRF_PPI->FORK[ready_ch].TEP = (uint32_t)&NRF_PPI->TASKS_CHG[end1_group].EN;
	NRF_PPI->CH[end1_ch].EEP = (uint32_t)&NRF_RADIO->EVENTS_END;
	NRF_PPI->CH[end1_ch].TEP = (uint32_t)&NRF_TIMER1->TASKS_CAPTURE[1];
	NRF_PPI->FORK[end1_ch].TEP = (uint32_t)&NRF_PPI->TASKS_CHG[end1_group].DIS;
	NRF_PPI->CHG[end1_group] = BIT(end1_ch);
	NRF_PPI->CH[addr_ch].EEP = (uint32_t)&NRF_RADIO->EVENTS_ADDRESS;
	NRF_PPI->CH[addr_ch].TEP = (uint32_t)&NRF_TIMER1->TASKS_CAPTURE[2];
	NRF_PPI->CH[end_ch].EEP = (uint32_t)&NRF_RADIO->EVENTS_END;
	NRF_PPI->CH[end_ch].TEP = (uint32_t)&NRF_TIMER1->TASKS_CAPTURE[3];
	NRF_PPI->CHENSET = BIT(ready_ch) | BIT(addr_ch) | BIT(end_ch);
	armed = true;
	return 0;
}

SYS_INIT(rift_airtime_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

void rift_airtime_start(uint8_t length, bool noack, bool busy)
{
	if (!armed) {
		return;
	}
	NRF_PPI->TASKS_CHG[end1_group].DIS = 1;
	for (int i = 0; i < 4; i++) {
		NRF_TIMER1->CC[i] = 0;
	}
	pending_length = length;
	pending_noack = noack;
	pending_busy = busy;
	pending = true;
	NRF_TIMER1->TASKS_CLEAR = 1;
}

static struct kind *kind_for(uint8_t length, bool noack)
{
	for (int i = 0; i < KINDS; i++) {
		struct kind *k = &kinds[i];

		if (k->n + k->retried + k->failed + k->mixed == 0) {
			k->length = length;
			k->noack = noack;
			return k;
		}
		if (k->length == length && k->noack == noack) {
			return k;
		}
	}
	return NULL;
}

void rift_airtime_done(bool success, uint32_t attempts)
{
	if (!pending) {
		return;
	}
	pending = false;

	uint32_t ready = NRF_TIMER1->CC[0], end1 = NRF_TIMER1->CC[1];
	uint32_t addr = NRF_TIMER1->CC[2], end = NRF_TIMER1->CC[3];
	struct kind *k = kind_for(pending_length, pending_noack);

	if (!k || !ready || !end1 || end1 < ready) {
		n_lost++;
		return;
	}
	if (!success) {
		k->failed++;
		return;
	}
	if (attempts > 1) {
		k->retried++;
		return;
	}
	if (pending_busy) {
		k->mixed++;
		return;
	}

	uint32_t i = k->n++ % SAMPLES;

	k->ready[i] = MIN(ready, UINT16_MAX);
	k->air[i] = MIN(end1 - ready, UINT16_MAX);
	k->turn[i] = pending_noack ? 0 : MIN(addr - end1, UINT16_MAX);
	k->ack[i] = pending_noack ? 0 : MIN(end - addr, UINT16_MAX);
	k->total[i] = MIN(end - ready, UINT16_MAX);
}

static int cmp_u16(const void *a, const void *b)
{
	return (int)*(const uint16_t *)a - (int)*(const uint16_t *)b;
}

/* min / median / max of n samples, in us */
static void print_spread(const char *name, const uint16_t *v, uint32_t n)
{
	static uint16_t s[SAMPLES];

	memcpy(s, v, n * sizeof(s[0]));
	qsort(s, n, sizeof(s[0]), cmp_u16);
	printk(", %s %u.%u/%u.%u/%u.%u", name, s[0] / 16, s[0] % 16 * 10 / 16, s[n / 2] / 16,
	       s[n / 2] % 16 * 10 / 16, s[n - 1] / 16, s[n - 1] % 16 * 10 / 16);
}

static void print_stats(void)
{
	printk("AIRTIME %s ramp-up; us min/median/max", IS_ENABLED(CONFIG_RIFT_FAST_RAMP_UP) ? "fast" : "normal");
	printk(" (write to TXREADY, TXREADY to END, END to ack ADDRESS, ack ADDRESS to END, TXREADY to last END)\n");
	for (int i = 0; i < KINDS; i++) {
		struct kind *k = &kinds[i];
		uint32_t n = MIN(k->n, SAMPLES);

		if (k->n + k->retried + k->failed + k->mixed == 0) {
			continue;
		}
		printk("AIRTIME %u bytes %s: %u single-attempt (%u kept), %u retried, %u failed, %u after a flush",
		       k->length, k->noack ? "no ack" : "ack", k->n, n, k->retried, k->failed, k->mixed);
		if (n) {
			print_spread("ready", k->ready, n);
			print_spread("air", k->air, n);
			if (!k->noack) {
				print_spread("turn", k->turn, n);
				print_spread("ack", k->ack, n);
			}
			print_spread("total", k->total, n);
		}
		printk("\n");
	}
	printk("AIRTIME %u packets without captures\n", n_lost);
	memset(kinds, 0, sizeof(kinds));
	n_lost = 0;
}

void rift_airtime_command(int argc, char **argv)
{
	if (!armed) {
		printk("AIRTIME not set up\n");
		return;
	}
	if (argc == 1) {
		print_stats();
		return;
	}
	if ((argc == 3 || argc == 4) && !strcmp(argv[1], "b")) {
		int n = atoi(argv[2]);
		int ms = argc == 4 ? atoi(argv[3]) : 5;
		uint8_t p[RIFT_B_TEST_LEN] = {RIFT_B_TEST, connection_get_id()};

		if (n < 1 || n > 10000 || ms < 1 || ms > 1000) {
			printk("airtime b <1-10000 packets> [<1-1000 ms apart>]\n");
			return;
		}
		for (int i = 0; i < n; i++) {
			p[2] = i;
			esb_write_noack(p, sizeof(p));
			k_msleep(ms);
		}
		printk("AIRTIME sent %d test packets B\n", n);
		return;
	}
	printk("airtime [b <n> [<ms>]]\n");
}
