/*
 * rift_airtime: how long each packet the band sends holds the radio, for the
 * sub-slots of riftcv1-mocap's Band packet timing plan (Claude Docs).
 *
 * TIMER1 (16 MHz) is cleared as each packet is written and captures, by PPI:
 *   CC0  TXREADY: the transmitter has ramped up (the last attempt's)
 *   CC1  the first END after TXREADY: the packet's end on air
 *   CC2  the last ADDRESS: the acknowledgement's (without one, the packet's own)
 *   CC3  the last END: the acknowledgement's end (without one, the packet's own)
 * The packet's ESB event reads them, and single-attempt packets go into the
 * statistics, by length and whether they asked for an acknowledgement.
 */
#ifndef RIFT_AIRTIME_H
#define RIFT_AIRTIME_H

#include <stdbool.h>
#include <stdint.h>

/* The test packet B (airtime b): RIFT_B_TEST, tracker id, a count, zeros; sent
 * without an acknowledgement. The receiver counts and drops it. */
#define RIFT_B_TEST     0xe2
#define RIFT_B_TEST_LEN 20

/* From esb_write: a packet is about to be written (busy: an earlier one was
 * flushed, so the captures may mix the two). */
void rift_airtime_start(uint8_t length, bool noack, bool busy);

/* From the ESB event handler: the packet's outcome and attempts. */
void rift_airtime_done(bool success, uint32_t attempts);

/* The airtime command: no argument prints the statistics and clears them;
 * b <n> [<ms>] sends n test packets B, <ms> apart (default 5). */
void rift_airtime_command(int argc, char **argv);

#endif
