/*
 * mpu6050.c
 *
 *  Created on: Jul 27, 2024
 *      Author: Ritesh Kumar
 */

#include "mpu6050.h"

static void update_acc(MPU6050 *mpu, int16_t raw_ax, int16_t raw_ay, int16_t raw_az);
static void update_gyro(MPU6050 *mpu, int16_t raw_ax, int16_t raw_ay, int16_t raw_az);
static void update_cal(MPU6050 *mpu, int16_t raw_ax, int16_t raw_ay, int16_t raw_az, int16_t raw_gx, int16_t raw_gy, int16_t raw_gz);

void mpu6050_init(I2C_HandleTypeDef *hi2c) {
	if (HAL_I2C_IsDeviceReady(hi2c, MPU6050_ADDR, 1, HAL_MAX_DELAY) == HAL_OK) {
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, (uint8_t*) &(uint8_t ) { 0x00 }, 1, HAL_MAX_DELAY);
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, (uint8_t*) &(uint8_t ) { 0x10 }, 1, HAL_MAX_DELAY);
		HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, (uint8_t*) &(uint8_t ) { 0x08 }, 1, HAL_MAX_DELAY);
	} else Error_Handler();
}

void mpu6050_signals(MPU6050 *mpu, I2C_HandleTypeDef *hi2c) {
	int16_t raw_acc[3]  = { 0 };
	int16_t raw_gyro[3] = { 0 };
	int16_t temp = 0;

	uint8_t data[14] = { 0 };
	HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B, 1, data, 14, HAL_MAX_DELAY);

	raw_acc[0]  = data[0]  << 8 | data[1];
	raw_acc[1]  = data[2]  << 8 | data[3];
	raw_acc[2]  = data[4]  << 8 | data[5];
	temp        = data[6]  << 8 | data[7];
	raw_gyro[0] = data[8]  << 8 | data[9];
	raw_gyro[1] = data[10] << 8 | data[11];
	raw_gyro[2] = data[12] << 8 | data[13];

	update_acc(mpu, raw_acc[0], raw_acc[1], raw_acc[2]);
	update_gyro(mpu, raw_gyro[0], raw_gyro[1], raw_gyro[2]);
	mpu->temp = (float)temp / 340.0 + 36.53;
}

void mpu6050_calibrate(MPU6050 *mpu, I2C_HandleTypeDef *hi2c) {
	int32_t acc_temp[3]  = { 0 };
	int32_t gyro_temp[3] = { 0 };

	for (int i = 0; i < 2000; i++) {
		mpu6050_signals(mpu, hi2c);
		acc_temp[0]  += mpu->acc.x;
		acc_temp[1]  += mpu->acc.y;
		acc_temp[2]  += mpu->acc.z;
		gyro_temp[0] += mpu->gyro.x;
		gyro_temp[1] += mpu->gyro.y;
		gyro_temp[2] += mpu->gyro.z;
		HAL_Delay(4);
	}

	acc_temp[0]  /= 2000;
	acc_temp[1]  /= 2000;
	acc_temp[2]  /= 2000;
	gyro_temp[0] /= 2000;
	gyro_temp[1] /= 2000;
	gyro_temp[2] /= 2000;

	update_cal(mpu, acc_temp[0], acc_temp[1], acc_temp[2], gyro_temp[0], gyro_temp[1], gyro_temp[2]);
}

static void update_acc(MPU6050 *mpu, int16_t raw_ax, int16_t raw_ay, int16_t raw_az) {
	mpu->acc.x = raw_ax - mpu->cal.ax;
	mpu->acc.y = raw_ay - mpu->cal.ay;
	mpu->acc.z = raw_az - mpu->cal.az;
}

static void update_gyro(MPU6050 *mpu, int16_t raw_gx, int16_t raw_gy, int16_t raw_gz) {
	mpu->gyro.x = raw_gx - mpu->cal.gx;
	mpu->gyro.y = raw_gy - mpu->cal.gy;
	mpu->gyro.z = raw_gz - mpu->cal.gz;
}

static void update_cal(MPU6050 *mpu, int16_t raw_ax, int16_t raw_ay, int16_t raw_az, int16_t raw_gx, int16_t raw_gy, int16_t raw_gz) {
	mpu->cal.ax = raw_ax;
	mpu->cal.ay = raw_ay;
	mpu->cal.az = raw_az;
	mpu->cal.gx = raw_gx;
	mpu->cal.gy = raw_gy;
	mpu->cal.gz = raw_gz;
}
