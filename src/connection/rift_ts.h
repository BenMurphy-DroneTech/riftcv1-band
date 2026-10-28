/*
 * rift_ts: timestamp pairs and the band's model of the beacon clock.
 *
 * Each packet the band sends is timed on its own 16 MHz TIMER3 at the radio's
 * transmit ADDRESS event (PPI, no software in the way); the receiver's
 * acknowledgement carries the same packet's arrival on the beacon clock. Each
 * acknowledged packet gives one pair (band time, beacon time), and a line
 * through the newest single-attempt pairs is the clock model.
 */
#ifndef RIFT_TS_H
#define RIFT_TS_H

#include <stdbool.h>
#include <stdint.h>

/* The acknowledgement payload (riftcv1-receiver src/rift/rift.h), 10 bytes:
 * RIFT_ACK_TIME, frame number (u32), offset into the frame in 1/16 us (u24),
 * frame period in us (u16), all little-endian. */
#define RIFT_ACK_TIME 0xe0
#define RIFT_ACK_LEN  10

/* The beacon clock against TIMER3: beacon = beacon_at + rate * (TIMER3 - band_at),
 * both clocks in 1/16 us. */
struct rift_model {
	bool valid;          /* enough pairs to pulse from                  */
	uint32_t pairs;      /* pairs in the fit                            */
	uint32_t band_at;    /* TIMER3 at the newest pair                   */
	double beacon_at;    /* the line's beacon clock there               */
	double rate;         /* beacon ticks per TIMER3 tick                */
	uint32_t period_us;  /* the frame period, from the acknowledgements */
	int64_t last_ms;     /* uptime of the newest pair                   */
	uint32_t rms_ns;     /* the fit's residuals                         */
};

/* From the ESB event handler: an acknowledgement payload, and how many
 * attempts its packet took. */
void rift_ts_ack(const uint8_t *data, uint8_t length, uint32_t attempts);

/* A copy of the current model. */
void rift_ts_model(struct rift_model *m);

/* Ignore acknowledgements for ms milliseconds (the holdover test). */
void rift_ts_hold(uint32_t ms);

/* TIMER3 now. */
uint32_t rift_ts_now(void);

/* The pairs command: the last pairs and their residuals from a straight line. */
void rift_ts_print(void);

#endif
