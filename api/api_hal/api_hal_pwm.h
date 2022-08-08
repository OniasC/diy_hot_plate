/*
 * api_hal_pwm.h
 *
 *  Created on: Jun 15, 2022
 *      Author: onias
 */

#ifndef API_API_HAL_API_HAL_PWM_H_
#define API_API_HAL_API_HAL_PWM_H_


#include"../api.h"

#ifdef __cplusplus
extern "C" {
#endif

__weak void analogWrite(pwm_t pwm_pin, uint32_t pwm_value);

#ifdef __cplusplus
}
#endif

#endif /* API_API_HAL_API_HAL_PWM_H_ */
