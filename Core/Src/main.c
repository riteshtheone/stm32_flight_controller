#include "main.h"

/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "imu.h"
#include "pid.h"
#include "utils.h"

uint32_t iteration_timer;
uint16_t esc[4];
uint8_t start;
extern __IO uint32_t usPulse[4];

IMU imu;
MPU6050 mpu;
PID_Controller pid;
/* USER CODE END Includes */

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

int main(void) {

	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();

	/* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4);

	mpu6050_init(&hi2c1);
	mpu6050_calibrate(&mpu, &hi2c1);

	while (usPulse[0] < 990 || usPulse[0] > 2100
			|| usPulse[1] < 990 || usPulse[1] > 2100
			|| usPulse[3] < 990 || usPulse[3] > 2100
			|| usPulse[2] < 990 || usPulse[2] > 1050){
		HAL_Delay(200);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !GPIO_PIN_SET);
	/* USER CODE END 2 */

	while (1) {
		/* USER CODE BEGIN 3 */
		if (micros() - iteration_timer >= 4000) {
			iteration_timer = micros();

			if (usPulse[0] < 990 || usPulse[0] > 2100
				    || usPulse[1] < 990 || usPulse[1] > 2100
					|| usPulse[2] < 990 || usPulse[2] > 2100
					|| usPulse[3] < 990 || usPulse[3] > 2100) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
				start = 0;
			}
			if (start != 2 && usPulse[2] < 1050 && usPulse[3] < 1050) start = 1;
			if (start == 1 && usPulse[2] > 1050) start = 0;
			if (start == 1 && usPulse[2] < 1050 && usPulse[3] > 1450) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
				start = 2;
				imu.angle_roll = imu.acc.roll;
				imu.angle_pitch = imu.acc.pitch;
				flush_pid();
			}
			if (start == 2 && usPulse[2] < 1050 && usPulse[3] > 1950) {
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
				start = 0;
			}

			mpu6050_signals(&mpu, &hi2c1);
			mpu6050_correct_direction(&mpu);
			calculate_rotational_rate(&imu, mpu.gyro.x, mpu.gyro.y, mpu.gyro.z);
			calculate_acc_angle(&imu, mpu.acc.x, mpu.acc.y, mpu.acc.z);
			calculate_angle(&imu, mpu.gyro.x, mpu.gyro.y, mpu.gyro.z);

			pid.setpoint.roll = 0;
			if (usPulse[0] > 1508) pid.setpoint.roll = usPulse[0] - 1508;
			else if (usPulse[0] < 1492) pid.setpoint.roll = usPulse[0] - 1492;
			pid.setpoint.roll -= imu.angle_roll * 18;
			pid.setpoint.roll /= 3.0;

			pid.setpoint.pitch = 0;
			if (usPulse[1] > 1508) pid.setpoint.pitch = usPulse[1] - 1508;
			else if (usPulse[1] < 1492) pid.setpoint.pitch = usPulse[1] - 1492;
			pid.setpoint.pitch -= imu.angle_pitch * 18;
			pid.setpoint.pitch /= 3.0;

			pid.setpoint.yaw = 0;
			if (usPulse[2] > 1050) {
				if (usPulse[3] > 1508) pid.setpoint.yaw = (usPulse[3] - 1508) / 3.0;
				else if (usPulse[3] < 1492) pid.setpoint.yaw = (usPulse[3] - 1492) / 3.0;
			}

			calculate_pid(&pid, imu.rate.roll, imu.rate.pitch, imu.rate.yaw);

			uint16_t throttle = usPulse[2];

			if (start == 2) {
				clamp(&throttle, 1000, 1800);

				esc[0] = throttle + pid.output.roll - pid.output.pitch - pid.output.yaw;
				esc[1] = throttle + pid.output.roll + pid.output.pitch + pid.output.yaw;
				esc[2] = throttle - pid.output.roll + pid.output.pitch - pid.output.yaw;
				esc[3] = throttle - pid.output.roll - pid.output.pitch + pid.output.yaw;

				clamp(&esc[0], 1170, 2000);
				clamp(&esc[1], 1170, 2000);
				clamp(&esc[2], 1170, 2000);
				clamp(&esc[3], 1170, 2000);
			} else
				esc[0] = esc[1] = esc[2] = esc[3] = 1000;

			TIM2->CCR1 = esc[0];
			TIM2->CCR2 = esc[1];
			TIM2->CCR3 = esc[2];
			TIM2->CCR4 = esc[3];
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

static void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 96 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 2500 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
	HAL_TIM_MspPostInit(&htim2);
}

static void MX_TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 96 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
		Error_Handler();
	}
}

static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, !GPIO_PIN_RESET);

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
