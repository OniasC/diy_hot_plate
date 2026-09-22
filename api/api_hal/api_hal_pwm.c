/*
 * pi_hal_pwm.c
 *
 *  Created on: Jun 15, 2022
 *      Author: onias
 */

#include "api_hal_pwm.h"

static uint32_t pwmValCheck(const TIM_TypeDef* const tim, uint32_t value) {
    if (value > tim->ARR) {
        value = tim->ARR;
    }
    return value;
}

__weak void analogWrite(pwm_t pwmPin, uint32_t value) {
    // value = 0xFFFF - value;
    value = pwmValCheck(pwmPin.htim->Instance, value);
    switch (pwmPin.Channel) {
    case TIM_CHANNEL_1:
        pwmPin.htim->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        pwmPin.htim->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        pwmPin.htim->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        pwmPin.htim->Instance->CCR4 = value;
        break;
    }
    return;
}
