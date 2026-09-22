/*
 * hot_plate.h
 *
 *  Created on: Aug 8, 2022
 *      Author: onias
 */

#ifndef APP_HOT_PLATE_HOT_PLATE_H_
#define APP_HOT_PLATE_HOT_PLATE_H_

#include "main.h"
#include "platform/platform.h"
#include "platform/buzzer/buzzer.h"
#include "platform/api_hal/api_hal.h"
#include "ui/menu/graph_lib/graph.h"
#include "board/bsp.h"

#ifdef __cplusplus
#include "third_party/NTC_Thermistor_hpp/Thermistor.h"
extern "C" {
#endif

typedef enum hotPlateState { hotPlateState_OFF = 0U, hotPlateState_REFLOW = 1U, hotPlateState_TRANSITION = 2U, hotPlateState_COOLDOWN = 3U } hotPlateState_e;

typedef enum { selectedMode_0 = 0U, selectedMode_1 = 1U, selectedMode_2 = 2U, selectedMode_3 = 3U } selectedMode_e;

typedef struct {
    float temp;
    float second;
} ReflowKeyPoint_t;

typedef struct {
    ReflowKeyPoint_t ramp;
    ReflowKeyPoint_t soak;
    ReflowKeyPoint_t reflow;
    ReflowKeyPoint_t cooldown; // second here doesnt matter
} TempProfile_t;

static const TempProfile_t TempProfile_SAC305_LowDensity = {{150.0, 75.0}, {175.0, 135.0}, {230.0, 180.0}, {40.0, 300.0}};
static const TempProfile_t TempProfile_SAC305_HighDensity = {{150.0, 90.0}, {175.0, 165.0}, {230.0, 225.0}, {40.0, 300.0}};

/* All defined in Core/Src/main.cpp */
extern float temperature;
extern float temp_setpoint;
extern float seconds;
extern float pwm_value;
extern float refresh_rate;
extern float pid_refresh_rate;
extern float Kp, Ki, Kd;
extern float PID_P, PID_I, PID_D;
extern float PID_ERROR, PREV_ERROR, PID_Output;
extern float MIN_PID_VALUE, MAX_PID_VALUE;
extern unsigned int millis_now, millis_before, millis_before_2;
extern TempProfile_t selectedProfile;
extern hotPlateState_e running_mode;
extern selectedMode_e selected_mode;
extern bool but_3_state, but_4_state;
extern io_pin_t but_1, but_2, but_3, but_4;
extern pwm_t SSR;
extern u8g2_t u8g2;

void reflow(float temperature);

void loop();

#ifdef __cplusplus
}

extern Thermistor* therm1;
void refreshDisplay(void);
#endif

#endif /* APP_HOT_PLATE_HOT_PLATE_H_ */
