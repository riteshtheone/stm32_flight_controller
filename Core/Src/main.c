#include "main.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "imu.h"
#include "pid.h"
#include "utils.h"

uint32_t iteration_timer;
uint16_t esc[4];

IMU imu;
MPU6050 mpu;
PID_Controller pid;
/* USER CODE END Includes */

I2C_HandleTypeDef hi2c1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();

	/* USER CODE BEGIN 2 */
	mpu6050_init(&hi2c1);
	mpu6050_calibrate(&mpu, &hi2c1);
	/* USER CODE END 2 */

	while (1) {
		/* USER CODE BEGIN 3 */
		if (micros() - iteration_timer >= 4000) {
			iteration_timer = micros();

			mpu6050_signals(&mpu, &hi2c1);
			calculate_rotational_rate(&imu, mpu.gyro.x, mpu.gyro.y, mpu.gyro.z);
			calculate_acc_angle(&imu, mpu.acc.x, mpu.acc.y, mpu.acc.z);
			calculate_angle(&imu, mpu.gyro.x, mpu.gyro.y, mpu.gyro.z);

			pid.setpoint.roll = 0;
			pid.setpoint.roll -= imu.angle_roll * 18;
			pid.setpoint.roll /= 3.0;

			pid.setpoint.pitch = 0;
			pid.setpoint.pitch -= imu.angle_pitch * 18;
			pid.setpoint.pitch /= 3.0;

			pid.setpoint.yaw = 0;

			calculate_pid(&pid, imu.rate.roll, imu.rate.pitch, imu.rate.yaw);

			esc[0] = 1300 + pid.output.roll - pid.output.pitch -  pid.output.yaw;
			esc[1] = 1300 + pid.output.roll + pid.output.pitch +  pid.output.yaw;
			esc[2] = 1300 - pid.output.roll + pid.output.pitch -  pid.output.yaw;
			esc[3] = 1300 - pid.output.roll - pid.output.pitch +  pid.output.yaw;

			clamp(&esc[0], 1170, 2000);
			clamp(&esc[1], 1170, 2000);
			clamp(&esc[2], 1170, 2000);
			clamp(&esc[3], 1170, 2000);
		}
	}
	/* USER CODE END 3 */
}



void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 25;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_I2C1_Init(void) {
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void Error_Handler(void) {
	__disable_irq();
	while (1) {}
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line){}
#endif /* USE_FULL_ASSERT */
