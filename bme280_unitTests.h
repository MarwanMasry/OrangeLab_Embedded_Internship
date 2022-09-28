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
BME280_Status BME280_UnitTest_PressureOversampling(BME280_Handle *a_cfgPtr);
/**
 * @fn BME280_Status BME280_UnitTest_TemperatureOversampling(BME280_Handle*)
 * @brief Tests setting all oversampling settings. Already assumes initialized handle.
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @return
 */
BME280_Status BME280_UnitTest_TemperatureOversampling(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_UnitTest_HumdityOversampling(BME280_Handle*)
 * @brief Tests setting all oversampling settings. Already assumes initialized handle.
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @return
 */
BME280_Status BME280_UnitTest_HumdityOversampling(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_UnitTest_FilterCoefficient(BME280_Handle*)
 * @brief Tests setting all filter coefficient settings. Already assumes initialized handle.
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @return
 */
BME280_Status BME280_UnitTest_FilterCoefficient(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_UnitTest_StandbyTime(BME280_Handle*)
 * @brief Tests setting all standby time settings. Already assumes initialized handle.
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @return
 */
BME280_Status BME280_UnitTest_StandbyTime(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_UnitTest_GeneralFuncs_SPI()
 * @brief Tests oversampling functions and gets readings from sensor and checks them.
 * 		Used for SPI mode.
 *
 * @return
 */
BME280_Status BME280_UnitTest_GeneralFuncs_SPI();

/**
 * @fn BME280_Status BME280_UnitTest_ReadingsCheck_SPI();
 * @brief Sets specific options to the sensor and gets all readings.
 *		Used for SPI mode.
 * @return
 */
BME280_Status BME280_UnitTest_ReadingsCheck_SPI();

/**
 * @fn BME280_Status BME280_UnitTest_GeneralFuncs_I2C()
 * @brief Tests oversampling functions and gets readings from sensor and checks them.
 * 		Used for I2C mode.
 *
 * @return
 */
BME280_Status BME280_UnitTest_GeneralFuncs_I2C();

/**
 * @fn BME280_Status BME280_UnitTest_ReadingsCheck_I2C();
 * @brief Sets specific options to the sensor and gets all readings.
 *		Used for I2C mode.
 * @return
 */
BME280_Status BME280_UnitTest_ReadingsCheck_I2C();

extern void BME280_GPIO_setSlaveSelectPin(void);

extern void BME280_GPIO_resetSlaveSelectPin(void);
#endif /* BME280_UNITTESTS_H_ */
