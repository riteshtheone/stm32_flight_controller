/*
 * imu.c
 *
 *  Created on: Jul 28, 2024
 *      Author: Ritesh Kumar
 */

#include "imu.h"
#include <math.h>

static void clamp(int16_t *value);

void calculate_angle(IMU *imu, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z) {
	imu->angle_roll  += (float) gyro_x * 0.000061069;
	imu->angle_pitch += (float) gyro_y * 0.000061069;

	imu->angle_roll  += imu->angle_pitch * sin((float) gyro_z * 0.0000010659);
	imu->angle_pitch -= imu->angle_roll  * sin((float) gyro_z * 0.0000010659);

	imu->angle_roll  = imu->angle_roll  * 0.9996 + imu->acc.roll  * 0.0004;
	imu->angle_pitch = imu->angle_pitch * 0.9996 + imu->acc.pitch * 0.0004;
}

void calculate_acc_angle(IMU *imu, int16_t acc_x, int16_t acc_y, int16_t acc_z) {
	acc_z += 4096;
	clamp(&acc_x);
	clamp(&acc_y);
	clamp(&acc_z);
    imu->acc.roll  = (float)atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 57.29578;
    imu->acc.pitch = (float)atan(acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 57.29578;
    imu->acc.pitch *= -1;
}


void calculate_rotational_rate(IMU *imu, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z) {
	imu->rate.roll  = imu->rate.roll  * 0.7 + ((float)gyro_x / 65.5) * 0.3;
	imu->rate.pitch = imu->rate.pitch * 0.7 + ((float)gyro_y / 65.5) * 0.3;
	imu->rate.yaw   = imu->rate.yaw   * 0.7 + ((float)gyro_z / 65.5) * 0.3;
	imu->rate.yaw *= -1;
}

static void clamp(int16_t *value) {
	int16_t max = 4096, min = -4096;
	if (*value > max) *value = max;
	if (*value < min) *value = min;
}
