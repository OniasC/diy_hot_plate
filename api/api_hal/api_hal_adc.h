/*
 * api_hal_adc.h
 *
 *  Created on: Jun 12, 2022
 *      Author: onias
 */

#ifndef API_API_HAL_API_HAL_ADC_H_
#define API_API_HAL_API_HAL_ADC_H_

#include"../api.h"

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;

__weak uint32_t analogRead(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* API_API_HAL_API_HAL_ADC_H_ */
