/*
 * util.c
 *
 *  Created on: Jul 28, 2024
 *      Author: Ritesh Kumar
 */

#include <utils.h>

#define counter    (0xffff + 1)

static const float mFactor = 1000000 / (96000000 / 96);
static uint32_t prev[4], now[4];

__IO uint32_t usPulse[4];


uint32_t millis(void) {
    return HAL_GetTick();
}

uint32_t micros(void) {
	uint32_t m0, m1;
	__IO uint32_t u0, u1;
    m0 = HAL_GetTick();
    u0 = SysTick->VAL;
    m1 = HAL_GetTick();
    u1 = SysTick->VAL;
    const uint32_t tms = SysTick->LOAD + 1;
    return (m1 != m0)
    		? (m1 * 1000 + ((tms - u1) * 1000) / tms)
    		: (m0 * 1000 + ((tms - u0) * 1000) / tms);
}

void clamp(uint16_t *value, uint16_t min, uint16_t max) {
	if (*value > max) *value = max;
	if (*value < min) *value = min;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))
			prev[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		else {
			now[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			uint32_t diff = now[0] > prev[0] ? now[0] - prev[0] : counter - prev[0] + now[0];
			usPulse[0] = diff * mFactor;
		}
	}
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5))
			prev[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		else {
			now[1] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			uint32_t diff = now[1] > prev[1] ? now[1] - prev[1] : counter - prev[1] + now[1];
			usPulse[1] = diff * mFactor;
		}
	}
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0))
			prev[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		else {
			now[2] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
			uint32_t diff = now[2] > prev[2] ? now[2] - prev[2] : counter - prev[2] + now[2];
			usPulse[2] = diff * mFactor;
		}
	}
	if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1))
			prev[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		else {
			now[3] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
			uint32_t diff = now[3] > prev[3] ? now[3] - prev[3] : counter - prev[3] + now[3];
			usPulse[3] = diff * mFactor;
		}
	}
}
