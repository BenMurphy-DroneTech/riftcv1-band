/*
 * rift_imu: the IMU's samples timed on TIMER3 and cut into half-frame slices
 * on the clock model (rift_ts), each slice a rotation change and a velocity
 * change in the band's frame at the slice's start.
 */
#ifndef RIFT_IMU_H
#define RIFT_IMU_H

#include <stdint.h>

/* The frame packet, band to receiver, once per camera frame, little-endian:
 *   0      RIFT_IMU_PACKET
 *   1      tracker id (connection fills it in)
 *   2-3    frame number of its two slices, low 16 bits
 *   4-15   first half-frame slice: rotation vector (3 x int16, 2^-15 rad) and
 *          velocity change (3 x int16, 2^-13 m/s, gravity included), in the
 *          band's frame at the slice's start
 *   16-27  second half-frame slice, the same
 *   28-33  orientation (the fusion's quaternion x, y, z, int16 / 32767, w >= 0)
 *          at the frame's end boundary, which is the next frame's boundary
 *   34     flags: bit 0 first slice whole, bit 1 second slice whole
 *   35     sequence number (connection fills it in)
 * The receiver (riftcv1-receiver) prints it as IMU <hex>; host/rift_imu.py in
 * riftcv1-mocap decodes it. */
#define RIFT_IMU_PACKET 0xe1
#define RIFT_IMU_LEN    36

/* From the sensor loop, in order, for each FIFO read: */
void rift_imu_read_start(void);        /* just before the read                      */
void rift_imu_count_read(void);        /* (IMU driver) just after reading the count */
void rift_imu_gyro(const float g[3]);  /* each gyro sample, calibrated, deg/s       */
void rift_imu_accel(const float a[3]); /* each accel sample, calibrated, g          */
void rift_imu_read_end(float gyro_period_s); /* the gyro's nominal sample period    */
void rift_imu_quat(const float q[4], const float gyro_bias[3]); /* the fusion's
		orientation after the read, and its gyro bias estimate (rad/s, as VQF's)  */

/* From rift_pulse's interrupt once a frame: the boundary is boundary_us away.
 * Schedules the FIFO reads READ_AFTER_US after it and after the half-frame. */
void rift_imu_frame(int32_t boundary_us, uint32_t period_us);

/* The imu console command: imu [zero]. */
void rift_imu_command(int argc, char **argv);

#endif
