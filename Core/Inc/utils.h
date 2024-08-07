/*
 * utils.h
 *
 *  Created on: Jul 28, 2024
 *      Author: Ritesh Kumar
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"

uint32_t millis(void);
uint32_t micros(void);
void clamp(uint16_t *value, uint16_t min, uint16_t max);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);

#endif /* INC_UTILS_H_ */
