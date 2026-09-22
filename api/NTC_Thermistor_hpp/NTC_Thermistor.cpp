/**
	Created by Yurii Salimov, February, 2018.
	Released into the public domain.
*/
#include "../NTC_Thermistor_hpp/NTC_Thermistor.h"

NTC_Thermistor::NTC_Thermistor(
	const int pin,
	const ntc_float_t referenceResistance,
	const ntc_float_t nominalResistance,
	const ntc_float_t nominalTemperatureCelsius,
	const ntc_float_t bValue,
	const int adcResolution
) {
	//pinMode(this->pin = pin, INPUT);
	this->referenceResistance = referenceResistance;
	this->nominalResistance = nominalResistance;
	this->nominalTemperature = celsiusToKelvins(nominalTemperatureCelsius);
	this->bValue = bValue;
	this->adcResolution = max(adcResolution, 0);
}

/**
	Reads and returns a temperature in Celsius.
	Reads the temperature in Kelvin,
	converts in Celsius and return it.

	@return temperature in Celsius.
*/
ntc_float_t NTC_Thermistor::readCelsius() {
	return kelvinsToCelsius(readKelvin());
}

/**
	Returns a temperature in Fahrenheit.
	Reads a temperature in Kelvin,
	converts in Fahrenheit and return it.

	@return temperature in Fahrenheit.
*/
ntc_float_t NTC_Thermistor::readFahrenheit() {
	return kelvinsToFahrenheit(readKelvin());
}

/**
	Returns a temperature in Kelvin.
	Reads the thermistor resistance,
	converts in Kelvin and return it.

	@return temperature in Kelvin.
*/
ntc_float_t NTC_Thermistor::readKelvin() {
	return resistanceToKelvins(readResistance());
}

inline ntc_float_t NTC_Thermistor::resistanceToKelvins(const ntc_float_t resistance) {
	const ntc_float_t inverseKelvin = 1.0 / this->nominalTemperature +
		log(resistance / this->nominalResistance) / this->bValue;
	return (1.0 / inverseKelvin);
}

inline ntc_float_t NTC_Thermistor::readResistance() {
  ntc_float_t readVoltage;
  uint32_t analog = analogRead((uint8_t)this->pin);
  this->voltageReading = (ntc_float_t)analog/(ntc_float_t)this->adcResolution;
  ntc_float_t resistance = this->referenceResistance / (1.0/this->voltageReading - 1.0);
  return resistance;
	//return this->referenceResistance / (this->adcResolution / readVoltage() - 1);
}

inline ntc_float_t NTC_Thermistor::readVoltage() {
	return analogRead((uint8_t)this->pin)*3.3/0xFFFFF;
}

inline ntc_float_t NTC_Thermistor::celsiusToKelvins(const ntc_float_t celsius) {
	return (celsius + 273.15);
}

inline ntc_float_t NTC_Thermistor::kelvinsToCelsius(const ntc_float_t kelvins) {
	return (kelvins - 273.15);
}

inline ntc_float_t NTC_Thermistor::celsiusToFahrenheit(const ntc_float_t celsius) {
	return (celsius * 1.8 + 32);
}

/**
	Kelvin to Fahrenheit conversion:
	F = (K - 273.15) * 1.8 + 32
	Where C = (K - 273.15) is Kelvins To Celsius conversion.
	Then F = C * 1.8 + 32 is Celsius to Fahrenheit conversion.
	=> Kelvin convert to Celsius, then Celsius to Fahrenheit.
*/
inline ntc_float_t NTC_Thermistor::kelvinsToFahrenheit(const ntc_float_t kelvins) {
	return celsiusToFahrenheit(kelvinsToCelsius(kelvins));
}
