/*
 * rift_pulse: the LED pulse on every camera frame boundary, placed in hardware
 * from the clock model (rift_ts), on D5-D8 (the breadboard wand's L293D
 * inputs 1A-4A), never high for more than 215 us.
 */
#ifndef RIFT_PULSE_H
#define RIFT_PULSE_H

#include <stdint.h>

/* From rift_ts_init, once TIMER3 runs. */
void rift_pulse_init(void);

/* Dark for good: from power-off, before System OFF (which keeps a pin's level). */
void rift_pulse_stop(void);

/* The pulse console command: pulse [on|off | width <us, up to 215> | at <us> | trim <us> | holdover <ms> |
 * reset],
 * and hold <ms>. Each answers with a PULSE status line (HOLD for hold). */
void rift_pulse_command(int argc, char **argv);
void rift_pulse_hold_command(int argc, char **argv);

/* The markers console command: markers [<D5 id> <D6 id> <D7 id> <D8 id>], answering
 * MARKERS. */
void rift_pulse_markers_command(int argc, char **argv);

/* From rift_ts: a command from the receiver's acknowledgement, for frame N
 * (RIFT_CMD_* in rift_ts.h). */
void rift_pulse_band_command(uint8_t type, uint32_t frame, uint8_t arg);

#endif
