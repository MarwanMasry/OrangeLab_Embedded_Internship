/******************************************************************************
 *
 * Module: Bosch Sensortec BME280
 *
 * File Name: bme280_unitTests.c
 *
 * Description: Source file for BME280 Sensor unit test functions.
 *
 * Date Created: 21/2022
 *
 * Author: Hazem Montasser
 *
 *******************************************************************************/

#include <bme280_unitTests.h>

/*!
 *	Unit testing functions for all options. Assumes initialized handle.
 */

BME280_Status BME280_UnitTest_PressureOversampling(BME280_Handle *a_cfgPtr) {
	/**************************************
	 * 		Pressure oversampling check
	 **************************************/

	BME280_Status result = BME280_NOT_YET_OBTAINED;
	BME280_Settings settings = { 0 };

	/****************************************************************************/
	result = BME280_setPressureOversampling(a_cfgPtr, BME280_OVERSAMPLING_OFF);

	result = BME280_getPressureOversampling(a_cfgPtr, &settings.osrs_p);
	if (settings.osrs_p != BME280_OVERSAMPLING_OFF || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setPressureOversampling(a_cfgPtr, BME280_OVERSAMPLING_x1);

	result = BME280_getPressureOversampling(a_cfgPtr, &settings.osrs_p);
	if (settings.osrs_p != BME280_OVERSAMPLING_x1 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setPressureOversampling(a_cfgPtr, BME280_OVERSAMPLING_x2);

	result = BME280_getPressureOversampling(a_cfgPtr, &settings.osrs_p);
	if (settings.osrs_p != BME280_OVERSAMPLING_x2 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setPressureOversampling(a_cfgPtr, BME280_OVERSAMPLING_x4);

	result = BME280_getPressureOversampling(a_cfgPtr, &settings.osrs_p);
	if (settings.osrs_p != BME280_OVERSAMPLING_x4 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setPressureOversampling(a_cfgPtr, BME280_OVERSAMPLING_x8);

	result = BME280_getPressureOversampling(a_cfgPtr, &settings.osrs_p);
	if (settings.osrs_p != BME280_OVERSAMPLING_x8 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setPressureOversampling(a_cfgPtr, BME280_OVERSAMPLING_x16);

	result = BME280_getPressureOversampling(a_cfgPtr, &settings.osrs_p);
	if (settings.osrs_p != BME280_OVERSAMPLING_x16 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/\
return result;

}

BME280_Status BME280_UnitTest_TemperatureOversampling(BME280_Handle *a_cfgPtr) {
	/**************************************
	 * 		Temperature oversampling check
	 **************************************/

	BME280_Status result = BME280_NOT_YET_OBTAINED;
	BME280_Settings settings = { 0 };
	/****************************************************************************/
	result = BME280_setTemperatureOversampling(a_cfgPtr,
			BME280_OVERSAMPLING_OFF);

	result = BME280_getTemperatureOversampling(a_cfgPtr, &settings.osrs_t);
	if (settings.osrs_t != BME280_OVERSAMPLING_OFF || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setTemperatureOversampling(a_cfgPtr,
			BME280_OVERSAMPLING_x1);

	result = BME280_getTemperatureOversampling(a_cfgPtr, &settings.osrs_t);
	if (settings.osrs_t != BME280_OVERSAMPLING_x1 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setTemperatureOversampling(a_cfgPtr,
			BME280_OVERSAMPLING_x2);

	result = BME280_getTemperatureOversampling(a_cfgPtr, &settings.osrs_t);
	if (settings.osrs_t != BME280_OVERSAMPLING_x2 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setTemperatureOversampling(a_cfgPtr,
			BME280_OVERSAMPLING_x4);

	result = BME280_getTemperatureOversampling(a_cfgPtr, &settings.osrs_t);
	if (settings.osrs_t != BME280_OVERSAMPLING_x4 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setTemperatureOversampling(a_cfgPtr,
			BME280_OVERSAMPLING_x8);

	result = BME280_getTemperatureOversampling(a_cfgPtr, &settings.osrs_t);
	if (settings.osrs_t != BME280_OVERSAMPLING_x8 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setTemperatureOversampling(a_cfgPtr,
			BME280_OVERSAMPLING_x16);

	result = BME280_getTemperatureOversampling(a_cfgPtr, &settings.osrs_t);
	if (settings.osrs_t != BME280_OVERSAMPLING_x16 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	return result;
}

BME280_Status BME280_UnitTest_HumdityOversampling(BME280_Handle *a_cfgPtr) {

	/**************************************
	 * 		Humidity oversampling check
	 **************************************/

	BME280_Status result = BME280_NOT_YET_OBTAINED;
	BME280_Settings settings = { 0 };
	/****************************************************************************/
	result = BME280_setHumidityOversampling(a_cfgPtr, BME280_OVERSAMPLING_OFF);

	result = BME280_getHumidityOversampling(a_cfgPtr, &settings.osrs_h);
	if (settings.osrs_h != BME280_OVERSAMPLING_OFF || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setHumidityOversampling(a_cfgPtr, BME280_OVERSAMPLING_x1);

	result = BME280_getHumidityOversampling(a_cfgPtr, &settings.osrs_h);
	if (settings.osrs_h != BME280_OVERSAMPLING_x1 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setHumidityOversampling(a_cfgPtr, BME280_OVERSAMPLING_x2);

	result = BME280_getHumidityOversampling(a_cfgPtr, &settings.osrs_h);
	if (settings.osrs_h != BME280_OVERSAMPLING_x2 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setHumidityOversampling(a_cfgPtr, BME280_OVERSAMPLING_x4);

	result = BME280_getHumidityOversampling(a_cfgPtr, &settings.osrs_h);
	if (settings.osrs_h != BME280_OVERSAMPLING_x4 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setHumidityOversampling(a_cfgPtr, BME280_OVERSAMPLING_x8);

	result = BME280_getHumidityOversampling(a_cfgPtr, &settings.osrs_h);
	if (settings.osrs_h != BME280_OVERSAMPLING_x8 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setHumidityOversampling(a_cfgPtr, BME280_OVERSAMPLING_x16);

	result = BME280_getHumidityOversampling(a_cfgPtr, &settings.osrs_h);
	if (settings.osrs_h != BME280_OVERSAMPLING_x16 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/
	return result;
}

BME280_Status BME280_UnitTest_FilterCoefficient(BME280_Handle *a_cfgPtr) {
	/**************************************
	 * 		Filter coefficient check
	 **************************************/

	BME280_Status result = BME280_NOT_YET_OBTAINED;
	BME280_Settings settings = { 0 };
	/****************************************************************************/
	result = BME280_setFilterCoefficient(a_cfgPtr, BME280_FILTER_COEFF_OFF);

	result = BME280_getFilterCoefficient(a_cfgPtr, &settings.filter);
	if (settings.filter != BME280_FILTER_COEFF_OFF || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setFilterCoefficient(a_cfgPtr, BME280_FILTER_COEFF_2);

	result = BME280_getFilterCoefficient(a_cfgPtr, &settings.filter);
	if (settings.filter != BME280_FILTER_COEFF_2 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/
	result = BME280_setFilterCoefficient(a_cfgPtr, BME280_FILTER_COEFF_4);

	result = BME280_getFilterCoefficient(a_cfgPtr, &settings.filter);
	if (settings.filter != BME280_FILTER_COEFF_4 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setFilterCoefficient(a_cfgPtr, BME280_FILTER_COEFF_8);

	result = BME280_getFilterCoefficient(a_cfgPtr, &settings.filter);
	if (settings.filter != BME280_FILTER_COEFF_8 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setFilterCoefficient(a_cfgPtr, BME280_FILTER_COEFF_16);

	result = BME280_getFilterCoefficient(a_cfgPtr, &settings.filter);
	if (settings.filter != BME280_FILTER_COEFF_16 || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/
	return result;
}

BME280_Status BME280_UnitTest_StandbyTime(BME280_Handle *a_cfgPtr) {
	/**************************************
	 * 		Filter coefficient check
	 **************************************/

	BME280_Status result = BME280_NOT_YET_OBTAINED;
	BME280_Settings settings = { 0 };
	/****************************************************************************/
	result = BME280_setStandbyTime(a_cfgPtr, BME280_T_STDBY_0_5_MS);

	result = BME280_getStandbyTime(a_cfgPtr, &settings.t_stby);
	if (settings.t_stby != BME280_T_STDBY_0_5_MS || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setStandbyTime(a_cfgPtr, BME280_T_STDBY_1000_MS);

	result = BME280_getStandbyTime(a_cfgPtr, &settings.t_stby);
	if (settings.t_stby != BME280_T_STDBY_1000_MS || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/
	result = BME280_setStandbyTime(a_cfgPtr, BME280_T_STDBY_10_MS);

	result = BME280_getStandbyTime(a_cfgPtr, &settings.t_stby);
	if (settings.t_stby != BME280_T_STDBY_10_MS || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setStandbyTime(a_cfgPtr, BME280_T_STDBY_125_MS);

	result = BME280_getStandbyTime(a_cfgPtr, &settings.t_stby);
	if (settings.t_stby != BME280_T_STDBY_125_MS || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setStandbyTime(a_cfgPtr, BME280_T_STDBY_20_MS);

	result = BME280_getStandbyTime(a_cfgPtr, &settings.t_stby);
	if (settings.t_stby != BME280_T_STDBY_20_MS || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setStandbyTime(a_cfgPtr, BME280_T_STDBY_250_MS);

	result = BME280_getStandbyTime(a_cfgPtr, &settings.t_stby);
	if (settings.t_stby != BME280_T_STDBY_250_MS || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setStandbyTime(a_cfgPtr, BME280_T_STDBY_500_MS);

	result = BME280_getStandbyTime(a_cfgPtr, &settings.t_stby);
	if (settings.t_stby != BME280_T_STDBY_500_MS || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/

	result = BME280_setStandbyTime(a_cfgPtr, BME280_T_STDBY_62_5_MS);

	result = BME280_getStandbyTime(a_cfgPtr, &settings.t_stby);
	if (settings.t_stby != BME280_T_STDBY_62_5_MS || result != BME280_OK)
		return BME280_SETTING_FAILED;
	/****************************************************************************/
	return result;
}

/*!
 * Complete unit testing functions for various settings of the sensor. Checks all settings
 * 	and obtains readings.
 */

BME280_Status BME280_UnitTest_GeneralFuncs_SPI() {
	BME280_Handle a_cfgPtr = { 0 };
	BME280_Status result = 0;

	BME280_sint32 temp_fixed = 0;
	BME280_float64 temp_float = 0;

	BME280_uint32 pressure_fixed = 0;
	BME280_float64 pressure_float = 0;

	BME280_uint32 humidity_fixed = 0;
	BME280_float64 humidity_float = 0;

	BME280_Settings settings;
	do {
		result = BME280_getInstance(&a_cfgPtr);

		if (result != BME280_FOUND_EMPTY_INSTANCE)
			break;

		result = BME280_setInterfaceType(&a_cfgPtr, BME280_Interface_SPI);
		if (result != BME280_OK)
			break;
		result = BME280_init(&a_cfgPtr);
		if (result != BME280_OK)
			break;
		BME280_setAssertNSSCallback(&a_cfgPtr, BME280_GPIO_setSlaveSelectPin);
		BME280_setReleaseNSSCallback(&a_cfgPtr,
				BME280_GPIO_resetSlaveSelectPin);
		/* Test all oversampling settings */
		result = BME280_UnitTest_TemperatureOversampling(&a_cfgPtr);
		if (result != BME280_OK)
			break;
		result = BME280_UnitTest_PressureOversampling(&a_cfgPtr);
		if (result != BME280_OK)
			break;
		result = BME280_UnitTest_HumdityOversampling(&a_cfgPtr);
		if (result != BME280_OK)
			break;
		result = BME280_UnitTest_FilterCoefficient(&a_cfgPtr);
		if (result != BME280_OK)
			break;
		result = BME280_UnitTest_StandbyTime(&a_cfgPtr);
		if (result != BME280_OK)
			break;

		/* Set test settings before getting readings from sensor */

		result = BME280_setFilterCoefficient(&a_cfgPtr, BME280_FILTER_COEFF_2);
		if (result != BME280_OK)
			break;
		result = BME280_setTemperatureOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x16);
		if (result != BME280_OK)
			break;
		result = BME280_setHumidityOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x8);
		if (result != BME280_OK)
			break;
		result = BME280_setPressureOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x4);
		if (result != BME280_OK)
			break;
		result = BME280_setStandbyTime(&a_cfgPtr, BME280_T_STDBY_125_MS);
		if (result != BME280_OK)
			break;
		result = BME280_getSensorSettings(&a_cfgPtr, &settings);
		if (result != BME280_OK)
			break;
		result = BME280_setMode(&a_cfgPtr, BME280_Mode_Normal);
		if (result != BME280_OK)
			break;

		/* Get readings from sensor*/
		result = BME280_getTemperature_fixedPoint(&a_cfgPtr, &temp_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getTemperature_floatingPoint(&a_cfgPtr, &temp_float);
		if (result != BME280_OK)
			break;
		result = BME280_getPressure_fixedPoint(&a_cfgPtr, &pressure_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getPressure_floatingPoint(&a_cfgPtr, &pressure_float);
		if (result != BME280_OK)
			break;
		result = BME280_getHumidity_fixedPoint(&a_cfgPtr, &humidity_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getHumidity_floatingPoint(&a_cfgPtr, &humidity_float);
		if (result != BME280_OK)
			break;
		if (result != BME280_OK)
			break;
	} while (0);
	BME280_DeInit(&a_cfgPtr);
	return result;
}

BME280_Status BME280_UnitTest_ReadingsCheck_SPI() {

	BME280_Handle a_cfgPtr = { 0 };
	BME280_Status result = 0;

	BME280_sint32 temp_fixed = 0;
	BME280_float64 temp_float = 0;

	BME280_uint32 pressure_fixed = 0;
	BME280_float64 pressure_float = 0;

	BME280_uint32 humidity_fixed = 0;
	BME280_float64 humidity_float = 0;
	do {
		result = BME280_getInstance(&a_cfgPtr);

		if (result != BME280_FOUND_EMPTY_INSTANCE)
			break;
		result = BME280_setInterfaceType(&a_cfgPtr, BME280_Interface_SPI);
		if (result != BME280_OK)
			break;
		BME280_setAssertNSSCallback(&a_cfgPtr, BME280_GPIO_setSlaveSelectPin);
		BME280_setReleaseNSSCallback(&a_cfgPtr,
				BME280_GPIO_resetSlaveSelectPin);
		result = BME280_init(&a_cfgPtr);
		if (result != BME280_OK)
			break;

		result = BME280_setFilterCoefficient(&a_cfgPtr, BME280_FILTER_COEFF_2);
		if (result != BME280_OK)
			break;
		result = BME280_setTemperatureOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x16);
		if (result != BME280_OK)
			break;
		result = BME280_setHumidityOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x8);
		if (result != BME280_OK)
			break;
		result = BME280_setPressureOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x4);
		if (result != BME280_OK)
			break;
		result = BME280_setStandbyTime(&a_cfgPtr, BME280_T_STDBY_125_MS);
		if (result != BME280_OK)
			break;

		result = BME280_setMode(&a_cfgPtr, BME280_Mode_Normal);
		if (result != BME280_OK)
			break;

		result = BME280_getTemperature_fixedPoint(&a_cfgPtr, &temp_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getTemperature_floatingPoint(&a_cfgPtr, &temp_float);
		if (result != BME280_OK)
			break;
		result = BME280_getPressure_fixedPoint(&a_cfgPtr, &pressure_fixed);
		if (result != BME280_OK)
			break;

		result = BME280_getPressure_floatingPoint(&a_cfgPtr, &pressure_float);
		if (result != BME280_OK)
			break;
		result = BME280_getHumidity_fixedPoint(&a_cfgPtr, &humidity_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getHumidity_floatingPoint(&a_cfgPtr, &humidity_float);
		if (result != BME280_OK)
			break;

	} while (0);
	BME280_DeInit(&a_cfgPtr);
	return result;
}

BME280_Status BME280_UnitTest_GeneralFuncs_I2C() {
	BME280_Handle a_cfgPtr = { 0 };
	BME280_Status result = 0;

	BME280_sint32 temp_fixed = 0;
	BME280_float64 temp_float = 0;

	BME280_uint32 pressure_fixed = 0;
	BME280_float64 pressure_float = 0;

	BME280_uint32 humidity_fixed = 0;
	BME280_float64 humidity_float = 0;

	BME280_Settings settings;
	do {
		result = BME280_getInstance(&a_cfgPtr);

		if (result != BME280_FOUND_EMPTY_INSTANCE)
			break;

		result = BME280_setInterfaceType(&a_cfgPtr, BME280_Interface_I2C);
		if (result != BME280_OK)
			break;
		result = BME280_init(&a_cfgPtr);
		if (result != BME280_OK)
			break;
		/* Test all oversampling settings */
		result = BME280_UnitTest_TemperatureOversampling(&a_cfgPtr);
		if (result != BME280_OK)
			break;
		result = BME280_UnitTest_PressureOversampling(&a_cfgPtr);
		if (result != BME280_OK)
			break;
		result = BME280_UnitTest_HumdityOversampling(&a_cfgPtr);
		if (result != BME280_OK)
			break;

		/* Set settings before getting readings from sensor */

		result = BME280_setFilterCoefficient(&a_cfgPtr, BME280_FILTER_COEFF_2);
		if (result != BME280_OK)
			break;
		result = BME280_setTemperatureOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x16);
		if (result != BME280_OK)
			break;
		result = BME280_setHumidityOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x8);
		if (result != BME280_OK)
			break;
		result = BME280_setPressureOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x4);
		if (result != BME280_OK)
			break;
		result = BME280_setStandbyTime(&a_cfgPtr, BME280_T_STDBY_125_MS);
		if (result != BME280_OK)
			break;
		result = BME280_getSensorSettings(&a_cfgPtr, &settings);
		if (result != BME280_OK)
			break;
		result = BME280_setMode(&a_cfgPtr, BME280_Mode_Normal);
		if (result != BME280_OK)
			break;

		/* Get readings from sensor*/
		result = BME280_getTemperature_fixedPoint(&a_cfgPtr, &temp_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getTemperature_floatingPoint(&a_cfgPtr, &temp_float);
		if (result != BME280_OK)
			break;
		result = BME280_getPressure_fixedPoint(&a_cfgPtr, &pressure_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getPressure_floatingPoint(&a_cfgPtr, &pressure_float);
		if (result != BME280_OK)
			break;
		result = BME280_getHumidity_fixedPoint(&a_cfgPtr, &humidity_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getHumidity_floatingPoint(&a_cfgPtr, &humidity_float);
		if (result != BME280_OK)
			break;
		if (result != BME280_OK)
			break;
	} while (0);
	BME280_DeInit(&a_cfgPtr);
	return result;
}

BME280_Status BME280_UnitTest_ReadingsCheck_I2C() {

	BME280_Handle a_cfgPtr = { 0 };
	BME280_Status result = 0;

	BME280_sint32 temp_fixed = 0;
	BME280_float64 temp_float = 0;

	BME280_uint32 pressure_fixed = 0;
	BME280_float64 pressure_float = 0;

	BME280_uint32 humidity_fixed = 0;
	BME280_float64 humidity_float = 0;
	do {
		result = BME280_getInstance(&a_cfgPtr);

		if (result != BME280_FOUND_EMPTY_INSTANCE)
			break;
		result = BME280_setInterfaceType(&a_cfgPtr, BME280_Interface_I2C);
		if (result != BME280_OK)
			break;
		result = BME280_init(&a_cfgPtr);
		if (result != BME280_OK)
			break;

		result = BME280_setFilterCoefficient(&a_cfgPtr, BME280_FILTER_COEFF_2);
		if (result != BME280_OK)
			break;
		result = BME280_setTemperatureOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x16);
		if (result != BME280_OK)
			break;
		result = BME280_setHumidityOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x8);
		if (result != BME280_OK)
			break;
		result = BME280_setPressureOversampling(&a_cfgPtr,
				BME280_OVERSAMPLING_x4);
		if (result != BME280_OK)
			break;
		result = BME280_setStandbyTime(&a_cfgPtr, BME280_T_STDBY_125_MS);
		if (result != BME280_OK)
			break;

		result = BME280_setMode(&a_cfgPtr, BME280_Mode_Normal);
		if (result != BME280_OK)
			break;

		result = BME280_getTemperature_fixedPoint(&a_cfgPtr, &temp_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getTemperature_floatingPoint(&a_cfgPtr, &temp_float);
		if (result != BME280_OK)
			break;
		result = BME280_getPressure_fixedPoint(&a_cfgPtr, &pressure_fixed);
		if (result != BME280_OK)
			break;

		result = BME280_getPressure_floatingPoint(&a_cfgPtr, &pressure_float);
		if (result != BME280_OK)
			break;
		result = BME280_getHumidity_fixedPoint(&a_cfgPtr, &humidity_fixed);
		if (result != BME280_OK)
			break;
		result = BME280_getHumidity_floatingPoint(&a_cfgPtr, &humidity_float);
		if (result != BME280_OK)
			break;

	} while (0);
	BME280_DeInit(&a_cfgPtr);
	return result;
}
