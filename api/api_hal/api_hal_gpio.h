/*
 * api_hal_gpio.h
 *
 *  Created on: 7 Mar 2022
 *      Author: onias
 */

#ifndef API_API_HAL_API_HAL_GPIO_H_
#define API_API_HAL_API_HAL_GPIO_H_

#include"../api.h"

#ifdef __cplusplus
extern "C" {
#endif

__weak void digitalWrite(io_pin_t * const pin, uint8_t mode);
__weak uint8_t digitalRead(io_pin_t * const pin);

#ifdef __cplusplus
}
#endif


#endif /* API_API_HAL_API_HAL_GPIO_H_ */
