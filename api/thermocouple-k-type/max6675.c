/*
 * max6675.cpp
 *
 *  Created on: Sep 3, 2022
 *      Author: onias
 */

#include "max6675.h"

//**************************************************************************
//    Constructor
//    Input: ^SPI GPIO PIN
//**************************************************************************

void thermocouple_max6675_ctor(max6675_t * const max6675, io_pin_t const * csPin, hspi_t const *hspi)
{
  max6675->hspi = (hspi_t *) hspi;
  max6675->csPin = (io_pin_t *)csPin;
  max6675->_filterLvl = FILTER_LVL;
  gpio_write(max6675->csPin, GPIO_PIN_SET); // CS High
}

//**************************************************************************
//    Init first reading
//    Give the time for the first reading
//    Initialise cFiltered
//**************************************************************************
void MAX6675_initMeasure(max6675_t * const max6675)
{
	HAL_Delay(250); // 4Hz max wait for the first reading
	max6675->_cFiltered = MAX6675_readCelsius(max6675, API_FALSE);
}
//**************************************************************************
//    Read Celsius temperature
//    input bool true if request filtered value
//    @returns float last Celcius measure (member)
//**************************************************************************
float MAX6675_readCelsius(max6675_t * const max6675, bool filter)
{
	uint8_t buffer[BUFFER_SIZE];
	volatile uint16_t sensorValue = 0U;

	//SELECT();
	gpio_write(max6675->csPin, GPIO_PIN_RESET); // CS Low

	HAL_SPI_Receive(max6675->hspi, buffer, 1, HAL_MAX_DELAY);
	sensorValue = buffer[1] << 8 | buffer[0];
	sensorValue = sensorValue >> 3;

	max6675->_celcius = (sensorValue * MAX_TEMP)/MAX6675_RESOLUTION;
	//DESELECT();
	gpio_write(max6675->csPin, GPIO_PIN_SET); // CS High
	return max6675->_celcius;
}

//**************************************************************************
//  input bool true if request filtered value
//  @returns float last Celcius measure (member)
/**************************************************************************/
float MAX6675_get_celcius(max6675_t * const max6675, const bool filter)
{
	if (filter) return (max6675->_cFiltered);
	return (max6675->_celcius);
}

//**************************************************************************
//  input -
//  @returns TRUE if connected
/**************************************************************************/
bool MAX6675_get_status(max6675_t * const max6675)
{
	return (max6675->_connected);
}

//**************************************************************************
//  input level value between 1-99
//  @returns -
/**************************************************************************/
void MAX6675_set_filterLevel(max6675_t * const max6675, uint8_t level)
{
	if (level>=100)level = 99;
	max6675->_filterLvl=level/100;
}

