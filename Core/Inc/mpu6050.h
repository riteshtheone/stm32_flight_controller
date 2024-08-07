/*
 * mpu6050.h
 *
 *  Created on: Jul 27, 2024
 *      Author: Ritesh Kumar
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "main.h"

#define MPU6050_ADDR (0x68 << 1)

typedef struct {

	float temp;

	struct Accelerometer {
		int16_t x, y, z;
	} acc;

	struct Gyroscope {
		int16_t x, y, z;
	} gyro;

	struct Calibration {
		int16_t ax, ay, az, gx, gy, gz;
	} cal;

} MPU6050;

void mpu6050_init(I2C_HandleTypeDef *hi2c);
void mpu6050_signals(MPU6050 *mpu, I2C_HandleTypeDef *hi2c);
void mpu6050_calibrate(MPU6050 *mpu, I2C_HandleTypeDef *hi2c);

void mpu6050_correct_direction(MPU6050 *mpu);

#endif /* INC_MPU6050_H_ */
