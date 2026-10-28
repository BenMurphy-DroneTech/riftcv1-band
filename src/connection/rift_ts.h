/*
 * rift_ts: timestamp pairs for the band's model of the beacon clock.
 *
 * Each packet the band sends is timed on its own 16 MHz TIMER3 at the radio's
 * transmit ADDRESS event (PPI, no software in the way); the receiver's
 * acknowledgement carries the same packet's arrival on the beacon clock. Each
 * acknowledged packet gives one pair (band time, beacon time).
 */
#ifndef RIFT_TS_H
#define RIFT_TS_H

#include <stdint.h>

/* The acknowledgement payload (riftcv1-receiver src/rift/rift.h), 10 bytes:
 * RIFT_ACK_TIME, frame number (u32), offset into the frame in 1/16 us (u24),
 * frame period in us (u16), all little-endian. */
#define RIFT_ACK_TIME 0xe0
#define RIFT_ACK_LEN  10

/* From the ESB event handler: an acknowledgement payload, and how many
 * attempts its packet took. */
void rift_ts_ack(const uint8_t *data, uint8_t length, uint32_t attempts);

/* The pairs command: the last pairs and their residuals from a straight line. */
void rift_ts_print(void);

#endif
