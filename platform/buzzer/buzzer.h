/*
 * buzzer.h
 *
 *  Created on: Sep 18, 2021
 *      Author: onias
 */

#ifndef API_BUZZER_BUZZER_H_
#define API_BUZZER_BUZZER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../platform.h"
#include "themes.h"

typedef enum {
    BUZZER_NOT_INIT,
    BUZZER_NO_ERROR,
    BUZZER_ERROR,
} buzzer_status_e;

typedef struct {
    pwm_t pwm;
    uint32_t frequency;
    uint8_t volume;
} buzzer_t;

buzzer_status_e buzzer_ctor(buzzer_t* const buzzer, TIM_HandleTypeDef* htim, uint32_t Channel, io_pin_t* const pin);

void BUZZER_play(buzzer_t* const buzzer);

void BUZZER_SetVolume(buzzer_t* const buzzer, uint32_t volume);

void BUZZER_tone(buzzer_t* const buzzer, uint32_t frequency, uint32_t durationMs);

void BUZZER_Play_Pirates(buzzer_t* const buzzer);

#ifdef __cplusplus
}
#endif

#endif /* API_BUZZER_BUZZER_H_ */
