/*
 * rift_pulse: the LED pulse on every camera frame boundary, placed in hardware
 * from the clock model (rift_ts), on the test band's D10 and D9.
 */
#ifndef RIFT_PULSE_H
#define RIFT_PULSE_H

#include <stdint.h>

/* From rift_ts_init, once TIMER3 runs. */
void rift_pulse_init(void);

/* The pulse console command: pulse [on|off | width <us> | at <us> | trim <us> | holdover <ms> |
 * reset],
 * and hold <ms>. Each answers with a PULSE status line (HOLD for hold). */
void rift_pulse_command(int argc, char **argv);
void rift_pulse_hold_command(int argc, char **argv);

/* The markers console command: markers [<D10 id> <D9 id>], answering MARKERS. */
void rift_pulse_markers_command(int argc, char **argv);

/* From rift_ts: a command from the receiver's acknowledgement, for frame N
 * (RIFT_CMD_* in rift_ts.h). */
void rift_pulse_band_command(uint8_t type, uint32_t frame, uint8_t arg);

#endif
