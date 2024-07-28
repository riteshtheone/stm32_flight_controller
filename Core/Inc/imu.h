/*
 * imu.h
 *
 *  Created on: Jul 28, 2024
 *      Author: Ritesh Kumar
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"

typedef struct {

	float angle_roll, angle_pitch;

	struct Angle_acc{
		float roll, pitch;
	} acc;

	struct Rotational_Rate{
		float roll, pitch, yaw;
	} rate;

} IMU;

void calculate_angle(IMU *imu, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);
void calculate_acc_angle(IMU *imu, int16_t acc_x, int16_t acc_y, int16_t acc_z);
void calculate_rotational_rate(IMU *imu, int16_t gyro_x, int16_t gyro_y, int16_t gyro_z);

#endif /* INC_IMU_H_ */
