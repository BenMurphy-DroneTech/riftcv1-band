/*
 * rift_pulse: the LED pulse on every camera frame boundary, placed in hardware
 * from the clock model (rift_ts), on the test band's D10 and D9.
 */
#ifndef RIFT_PULSE_H
#define RIFT_PULSE_H

/* From rift_ts_init, once TIMER3 runs. */
void rift_pulse_init(void);

/* The pulse console command: pulse [on|off | width <us> | at <us> | trim <us> | holdover <ms> |
 * reset],
 * and hold <ms>. Each answers with a PULSE status line (HOLD for hold). */
void rift_pulse_command(int argc, char **argv);
void rift_pulse_hold_command(int argc, char **argv);

#endif
