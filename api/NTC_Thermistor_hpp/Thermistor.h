/**
	Thermistor - interface describes a set of methods
	for working with a thermistor sensor and reading
	a temperature in Celsius, Fahrenheit and Kelvin.

	v.2.0.0
	- created

	v.2.0.3
	- added virtual destructor

	https://github.com/YuriiSalimov/NTC_Thermistor

	Created by Yurii Salimov, May, 2019.
	Released into the public domain.
*/
#ifndef THERMISTOR_H
#define THERMISTOR_H

#if defined(ARDUINO) && (ARDUINO >= 100)
	#include <Arduino.h>
#else
//	#include <WProgram.h>
	#include "../api/api_hal/api_hal.h"
#endif

typedef float ntc_float_t;

class Thermistor {

	public:

		/**
			Destructor
			Deletes Thermistor instance.
		*/
		virtual ~Thermistor() {};

		/**
			Reads a temperature in Celsius from the thermistor.

			@return temperature in degree Celsius
		*/
		virtual ntc_float_t readCelsius() = 0;

		/**
			Reads a temperature in Kelvin from the thermistor.

			@return temperature in degree Kelvin
		*/
		virtual ntc_float_t readKelvin() = 0;

		/**
			Reads a temperature in Fahrenheit from the thermistor.

			@return temperature in degree Fahrenheit
		*/
		virtual ntc_float_t readFahrenheit() = 0;
};

#endif
