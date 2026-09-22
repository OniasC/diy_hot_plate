/*
 * max6675.h
 *
 *  Created on: Sep 3, 2022
 *      Author: onias
 */

#ifndef API_THERMOCOUPLE_K_TYPE_MAX6675_H_
#define API_THERMOCOUPLE_K_TYPE_MAX6675_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "../api.h"
#include "../gpio/gpio.h"

#define SPI_TIMEOUT 1000
#define FILTER_LVL 0.10

#define BUFFER_SIZE 2
#define MAX_TEMP 1024
#define MAX6675_RESOLUTION 4095 //12 bits high

typedef struct {
  float	_celcius;
  float	_cFiltered;
  bool 	_connected;
  float _filterLvl;
  hspi_t *hspi;
  io_pin_t *csPin;
} max6675_t ;

void thermocouple_max6675_ctor(max6675_t * const max6675, io_pin_t const * csPin, hspi_t const *hspi);

void MAX6675_initMeasure(max6675_t * const max6675);
float MAX6675_readCelsius(max6675_t * const max6675, const bool filter);
float MAX6675_get_celcius(max6675_t * const max6675, const bool filter);
bool MAX6675_get_status(max6675_t * const max6675);
void MAX6675_set_filterLevel(max6675_t * const max6675, uint8_t level);



#ifdef __cplusplus
}
#endif

#endif /* API_THERMOCOUPLE_K_TYPE_MAX6675_H_ */
