/*
 * rift_imu: the IMU's samples timed on TIMER3 and cut into half-frame slices
 * on the clock model (rift_ts), each slice a rotation change and a velocity
 * change in the band's frame at the slice's start.
 */
#ifndef RIFT_IMU_H
#define RIFT_IMU_H

#include <stdint.h>

/* From the sensor loop, in order, for each FIFO read: */
void rift_imu_read_start(void);        /* just before the read                      */
void rift_imu_count_read(void);        /* (IMU driver) just after reading the count */
void rift_imu_gyro(const float g[3]);  /* each gyro sample, calibrated, deg/s       */
void rift_imu_accel(const float a[3]); /* each accel sample, calibrated, g          */
void rift_imu_read_end(float gyro_period_s); /* the gyro's nominal sample period    */
void rift_imu_quat(const float q[4], const float gyro_bias[3]); /* the fusion's
		orientation after the read, and its gyro bias estimate (rad/s, as VQF's)  */

/* The imu console command: imu [zero]. */
void rift_imu_command(int argc, char **argv);

#endif
