/*
 * util.c
 *
 *  Created on: Jul 28, 2024
 *      Author: Ritesh Kumar
 */

#include <utils.h>

uint32_t millis(void) {
    return HAL_GetTick();
}

uint32_t micros(void) {
    uint32_t m0 = HAL_GetTick();
    __IO uint32_t u0 = SysTick->VAL;
    uint32_t m1 = HAL_GetTick();
    __IO uint32_t u1 = SysTick->VAL;
    const uint32_t tms = SysTick->LOAD + 1;

    return (m1 != m0)
               ? (m1 * 1000 + ((tms - u1) * 1000) / tms)
               : (m0 * 1000 + ((tms - u0) * 1000) / tms);
}

void clamp(uint16_t *value, uint16_t min, uint16_t max) {
	if (*value > max) *value = max;
	if (*value < min) *value = min;
}
