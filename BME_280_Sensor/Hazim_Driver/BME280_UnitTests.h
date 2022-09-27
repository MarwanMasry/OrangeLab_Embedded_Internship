/*
 * BME280_UnitTest_AllFuncs.h
 *
 *  Created on: Sep 25, 2022
 *      Author: h4z3m
 */

#ifndef BME280_UNITTESTS_H_
#define BME280_UNITTESTS_H_
#include "bme280.h"

/**
 * @fn BME280_Status BME280_UnitTest_PressureOversampling(BME280_Handle*)
 * @brief Tests setting all oversampling settings. Already assumes initialized handle.
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @return
 */
BME280_Status BME280_UnitTest_PressureOversampling(
		BME280_Handle *a_cfgPtr);
/**
 * @fn BME280_Status BME280_UnitTest_TemperatureOversampling(BME280_Handle*)
 * @brief Tests setting all oversampling settings. Already assumes initialized handle.
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @return
 */
BME280_Status BME280_UnitTest_TemperatureOversampling(
		BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_UnitTest_HumdityOversampling(BME280_Handle*)
 * @brief Tests setting all oversampling settings. Already assumes initialized handle.
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @return
 */
BME280_Status BME280_UnitTest_HumdityOversampling(
		BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_UnitTest_GeneralFuncs()
 * @brief Tests oversampling functions and gets readings from sensor and checks them.
 *
 * @return
 */
BME280_Status BME280_UnitTest_GeneralFuncs();


/**
 * @fn BME280_Status BME280_UnitTest_ReadingsCheck();
 * @brief Sets specific options to the sensor and gets all readings.
 *
 * @return
 */
BME280_Status BME280_UnitTest_ReadingsCheck();
#endif /* BME280_UNITTESTS_H_ */
