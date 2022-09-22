/*
 * BME280_Unit_Test_All_Funcs.c
 *
 *  Created on: Sep 21, 2022
 *      Author: h4z3m
 */

#include "bme280.h"

void test1(){
	BME280_Handle handler1 = { 0 };
	BME280_Result result = { 0 };

	sint32 temp_fixed = 0;
	float64 temp_float = 0;

	uint32 pressure_fixed = 0;
	float64 pressure_float = 0;

	uint32 humidity_fixed = 0;
	float64 humidity_float = 0;

	BME280_Settings settings;

	result = BME280_init(&handler1);

	result = BME280_setTemperatureOversampling(&handler1,
			BME280_OVERSAMPLING_x1);

	result = BME280_setPressureOversampling(&handler1, BME280_OVERSAMPLING_x2);

	result = BME280_setHumidityOversampling(&handler1, BME280_OVERSAMPLING_x8);

	result = BME280_setFilterCoefficient(&handler1, BME280_FILTER_COEFF_2);

	result = BME280_setMode(&handler1, BME280_Mode_Normal);
	result = BME280_setStandbyTime(&handler1, BME280_T_STDBY_125_MS);

	result = BME280_getSensorSettings(&handler1, &settings);

	result = BME280_getTemperature_fixedPoint(&handler1, &temp_fixed);
	result = BME280_getTemperature_floatingPoint(&handler1, &temp_float);

	result = BME280_getPressure_fixedPoint(&handler1, &pressure_fixed);
	result = BME280_getPressure_floatingPoint(&handler1, &pressure_float);

	result = BME280_getHumidity_fixedPoint(&handler1, &humidity_fixed);
	result = BME280_getHumidity_floatingPoint(&handler1, &humidity_float);

	return;
}
