/*
 * graph.h
 *
 *  Created on: Aug 8, 2022
 *      Author: onias
 */

#ifndef APP_HOT_PLATE_HOT_PLATE_H_
#define APP_HOT_PLATE_HOT_PLATE_H_

#include "main.h"
#include "../api/api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum hotPlateState {
    hotPlateState_OFF = 0U,
    hotPlateState_REFLOW = 1U,
    hotPlateState_TRANSITION = 2U,
    hotPlateState_COOLDOWN = 3U
} hotPlateState_e;

typedef enum {
    selectedMode_0 = 0U,
    selectedMode_1 = 1U,
    selectedMode_2 = 2U,
    selectedMode_3 = 3U
} selectedMode_e;

typedef struct {
    float temp;
    float second;
} ReflowKeyPoint_t;

typedef struct {
    ReflowKeyPoint_t ramp;
    ReflowKeyPoint_t soak;
    ReflowKeyPoint_t reflow;
    ReflowKeyPoint_t cooldown; //second here doesnt matter
} TempProfile_t;

static const TempProfile_t TempProfile_SAC305_LowDensity = { {150.0, 75.0},{175.0,135.0},{230.0,180.0},{40.0,300.0} };
static const TempProfile_t TempProfile_SAC305_HighDensity = { {150.0, 90.0},{175.0,165.0},{230.0,225.0}, {40.0,300.0} };


void reflow(float temperature);

void loop();

#ifdef __cplusplus
}
#endif

#endif /* APP_HOT_PLATE_HOT_PLATE_H_ */
