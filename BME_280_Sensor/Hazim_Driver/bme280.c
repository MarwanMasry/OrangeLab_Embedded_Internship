/*
 * bme280.c
 *
 *  Created on: Sep 12, 2022
 *      Author: h4z3m
 */
#include "bme280.h"
#include "bme280_types.h"
#include "bme280_registers.h"
#include "stm32wbxx_hal.h"
struct BME280_ConfigType sensorPool[BME280_MAX_SENSOR_POOL_SIZE];

/*******************************************************************************
 *                          Private function prototypes                        *
 *******************************************************************************/

/**
 * @fn BME280_Result BME280_Read(BME280_Handle *cfgPtr, uint8 regAddr, uint8 *data, uint8 len)
 * @brief Abstract function used to read data from the sensor based on
 * the selected interface
 *
 * @param cfgPtr: Pointer to the sensor configuration struct
 * @param regAddr: Register address to read data from
 * @param data: Buffer to read data into
 * @param len: Length of the data to read
 *
 * @return BME280_Result
 */
static BME280_Result BME280_Read(
		BME280_Handle *cfgPtr,
		uint8 regAddr,
		uint8 *data,
		uint8 len);

/**
 * @fn BME280_Result BME280_Write(BME280_Handle *cfgPtr, uint8 regAddr, uint8 data_byte)
 * @brief Abstract function used to write data to the sensor based on
 * the selected interface
 *
 * @param cfgPtr: Pointer to the sensor configuration struct
 * @param regAddr: Register address to write to
 * @param data_byte: Data byte to write into the register address
 *
 * @return BME280_Result
 */
static BME280_Result BME280_Write(
		BME280_Handle *cfgPtr,
		uint8 regAddr,
		uint8 data_byte);

/**
 * @fn BME280_Result BME280_SPI_ReadWrapper(SPI_HandleTypeDef*, uint8, uint8*, uint8)
 * @brief Wraps HAL_SPI functions to be used according to the
 * selected interface
 *
 * @param regAddr: Register address to read from the sensor
 * @param recv_buff: Buffer to read data into
 * @param len: Length of data to read
 *
 * @return BME280_Result
 */
static BME280_Result BME280_SPI_ReadWrapper(
		uint8 regAddr,
		uint8 *recv_buff,
		uint8 len);

/**
 * @fn BME280_Result BME280_SPI_WriteWrapper(SPI_HandleTypeDef*, uint8, uint8)
 * @brief Wraps HAL_SPI functions to be used according to the
 * selected interface
 *
 * @param regAddr: Register to write to
 * @param data_byte: Data to write to the register
 *
 * @return BME280_Result
 */
static BME280_Result BME280_SPI_WriteWrapper(
		uint8 regAddr,
		uint8 data_byte);

/**
 * @fn BME280_Result BME280_getUncompensatedReadings(BME280_Handle*, BME280_PressureReading*, BME280_TemperatureReading*, BME280_HumidityReading*)
 * @brief  Reads ALL _raw_ un-compensated readings from the sensor in burst-read mode for
 * efficiency, all readings are aligned into the readingsPtr union.
 *
 *
 * @param cfgPtr		Configuration struct used to interface with the sensor
 * @param pressure		Pointer to pressure reading union
 * @param temperature	Pointer to temperature reading union
 * @param humidity		Pointer to humidity reading union
 * @return
 */
static BME280_Result BME280_getUncompensatedReadings(
		BME280_Handle *cfgPtr,
		BME280_PressureReading *pressure,
		BME280_TemperatureReading *temperature,
		BME280_HumidityReading *humidity);

/**
 * @fn BME280_Result BME280_getCalibData(BME280_Handle*)
 * @brief Reads ALL calibration data from the sensor
 * 		  Calibration data:
 * 		  	- dig_T1 to dig_T3
 * 		  	- dig_P1 to dig_P9
 * 		  	- dig_H1 to dig_H6
 * 		  Stores them in cfgPtr Calib1 and Calib2 structures
 *
 * @param cfgPtr
 * @return
 */
static BME280_Result BME280_getCalibData(
		BME280_Handle *cfgPtr);

/**
 * @fn BME280_Result BME280_getConfigRegister(BME280_Handle*, BME280_ConfigRegisterUnion*)
 * @brief Retrieves config register from the sensor
 *
 * @param cfgPtr	Pointer to sensor handle
 * @param cfgRegPtr Pointer to config register variable which will contain the sensor config register
 * @return
 */
static BME280_Result BME280_getConfigRegister(
		BME280_Handle *cfgPtr,
		BME280_ConfigRegisterUnion *cfgRegPtr);

/**
 * @fn BME280_Result BME280_getCtrlMeasRegister(BME280_Handle*, BME280_CtrlMeasRegisterUnion*)
 * @brief Retrieves ctrl_meas register from the sensor
 *
 * @param cfgPtr	Pointer to sensor handle
 * @param cfgRegPtr Pointer to ctrl_meas register variable which will contain the sensor ctrl_meas register
 * @return
 */
static BME280_Result BME280_getCtrlMeasRegister(
		BME280_Handle *cfgPtr,
		BME280_CtrlMeasRegisterUnion *cfgRegPtr);

/**
 * @fn BME280_Result BME280_getCtrlHumRegister(BME280_Handle*, BME280_CtrlHumRegisterUnion*)
 * @brief Retrieves ctrl_hum register from the sensor
 *
 * @param cfgPtr	Pointer to sensor handle
 * @param cfgRegPtr Pointer to ctrl_hum register variable which will contain the sensor ctrl_hum register
 * @return
 */
static BME280_Result BME280_getCtrlHumRegister(
		BME280_Handle *cfgPtr,
		BME280_CtrlHumRegisterUnion *cfgRegPtr);

/**
 * @fn BME280_Result BME280_getStatusRegister(BME280_Handle*, BME280_StatusRegisterUnion*)
 * @brief Retrieves status register from the sensor
 *
 * @param cfgPtr	Pointer to sensor handle
 * @param cfgRegPtr Pointer to status register variable which will contain the sensor status register
 * @return
 */
static BME280_Result BME280_getStatusRegister(
		BME280_Handle *cfgPtr,
		BME280_StatusRegisterUnion *cfgRegPtr);

/**
 * @fn BME280_Result BME280_isInstance(BME280_Handle*)
 * @brief Verifies whether the handle is tied to an existing instance in the sensor
 * 		pool or not. Returns certain error codes if pool is full or handle points to null
 * 		- BME280_IS_INSTANCE  -> Handle points to an instance
 * 		- BME280_NULL_ERROR   -> Handle points to null
 * 		- BME280_NOT_INSTANCE -> Handle points to something else other than sensor instances
 *
 * @param cfgPtr
 * @return
 */
static BME280_Result BME280_isInstance(
		BME280_Handle *cfgPtr);

/**
 * @fn BME280_Result BME280_getInstance(BME280_Handle*)
 * @brief
 *
 * @param cfgPtr
 * @return
 */
static BME280_Result BME280_getInstance(
		BME280_Handle *cfgPtr);
#if BME280_FLOATING_POINT == BME280_FEATURE_ENABLE
/**
 * @fn float64 BME280_compensateTemperature_floatingPoint(BME280_Handle*, sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate temperature as floating point notation.
 * 		Ex: returns 32.52 -> 32.52 c
 *
 * @param cfgPtr		 Pointer to sensor handle
 * @param rawTemperature Raw temperature data read by the sensor
 * @return Temperature in degrees Celsius
 */
static float64 BME280_compensateTemperature_floatingPoint(
		BME280_Handle *cfgPtr,
		sint32 rawTemperature);

/**
 * @fn float64 BME280_compensatePressure_floatingPoint(BME280_Handle*, sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate pressure as fixed point notation.
 * 		Ex: returns 9338.2 -> 9338.2 Pa
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param rawPressure	Raw pressure data read by the sensor
 * @return Pressure in Pascal
 */
static float64 BME280_compensatePressure_floatingPoint(
		BME280_Handle *cfgPtr,
		sint32 rawPressure);

/**
 * @fn float64 BME280_compensateHumidity_floatingPoint(BME280_Handle*, sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate pressure as fixed point notation.
 * 		Ex: returns 3252 -> 32.52c
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param rawHumidity	Raw humidity data read by the sensor
 * @return Relative humidity in percentage (%)
 */
static float64 BME280_compensateHumidity_floatingPoint(
		BME280_Handle *cfgPtr,
		sint32 rawHumidity);
#endif

#if BME280_FIXED_POINT == BME280_FEATURE_ENABLE
/**
 * @fn sint32 BME280_compensateTemperature_fixedPoint(BME280_Handle*, sint32)
 * @brief Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate temperature as fixed point notation.
 * 		Ex: returns 3252 -> 32.52 c
 *
 * @param cfgPtr			Pointer to sensor handle
 * @param rawTemperature	Raw temperature data read by the sensor
 * @return Temperature in Celsius (fixed point)
 */
static sint32 BME280_compensateTemperature_fixedPoint(
		BME280_Handle *cfgPtr,
		sint32 rawTemperature);

/**
 * @fn uint32 BME280_compensatePressure_fixedPoint(BME280_Handle*, sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate pressure as fixed point notation.
 * 		Ex: returns 93211 -> 93.211 Pa = 932.11 hPa
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param rawPressure	Raw pressure data read by the sensor
 * @return Pressure in Pascal (fixed point)
 */
static uint32 BME280_compensatePressure_fixedPoint(
		BME280_Handle *cfgPtr,
		sint32 rawPressure);

/**
 * @fn uint32 BME280_compensateHumidity_fixedPoint(BME280_Handle*, sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate relative humidity as fixed point notation.
 * 		Ex: returns 4288 -> 42284/1024 -> 41.29% rH
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param rawHumidity	Raw humdity data read by the sensor
 * @return	Relative humidity in percent, to be divided by 1024 to get exact value
 */
static uint32 BME280_compensateHumidity_fixedPoint(
		BME280_Handle *cfgPtr,
		sint32 rawHumidity);
#endif
/*******************************************************************************
 *                          Private function definitions                       *
 *******************************************************************************/

static BME280_Result BME280_SPI_ReadWrapper(
		uint8 regAddr,
		uint8 *recv_buff,
		uint8 len){
	/* Status of SPI transmission */

	uint8 status = 0;
	/* Mask read address in SPI mode */
	regAddr = BME280_READ_MASK(regAddr);
	/* Pull SS pin low to write*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	/* Send control byte */
	status = BME280_SPI_TransmitReceive(&regAddr, recv_buff, 1,
	BME280_SPI_TIMEOUT_MS);
	/* Receive data from sensor */
	status = BME280_SPI_TransmitReceive(&regAddr, recv_buff, len,
	BME280_SPI_TIMEOUT_MS);
	/* Pull up SS pin to indicate end of transmission */

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	if (status == HAL_OK)
		return BME280_OK;
	else
		return BME280_COMM_ERROR;
}

static BME280_Result BME280_SPI_WriteWrapper(
		uint8 regAddr,
		uint8 data_byte){
	/* Status of SPI transmission */
	uint8 status = 0;
	/* Dummy byte to receive in */
	uint8 dummy = 0;
	/* Mask write address in SPI mode */
	regAddr = BME280_WRITE_MASK(regAddr);
	/* Pull SS pin low to write*/
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	/* Send control byte */
	status = BME280_SPI_TransmitReceive(&regAddr, &dummy, 1,
	BME280_SPI_TIMEOUT_MS);
	/* Send data to write */
	status = BME280_SPI_TransmitReceive(&data_byte, &dummy, 1,
	BME280_SPI_TIMEOUT_MS);
	/* Pull up SS pin to indicate end of transmission */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	if (status == HAL_OK)
		return BME280_OK;
	else
		return BME280_COMM_ERROR;
}

static BME280_Result BME280_Read(
		BME280_Handle *cfgPtr,
		uint8 regAddr,
		uint8 *data,
		uint8 len){
	/* Interface return value */
	BME280_Result result;

	/* Check for used interface, if neither is specified return no interface specified*/
	result = BME280_NO_INTERFACE_SPECIFIED; /* Using I2C interface*/
	if ((*cfgPtr)->Intf == BME280_Interface_I2C) {
		//return BME280_I2C_ReadWrapper(cfgPtr->I2C_Handle, &regAddr, data, len);
	}

	/* Using SPI interface */
	else if ((*cfgPtr)->Intf == BME280_Interface_SPI) {
		result = BME280_SPI_ReadWrapper(regAddr, data, len);
	}
	return result;
}

static BME280_Result BME280_Write(
		BME280_Handle *cfgPtr,
		uint8 regAddr,
		uint8 data_byte){
	/* Interface return value */
	BME280_Result result;

	/* Check for used interface, if neither is specified return no interface specified*/
	result = BME280_NO_INTERFACE_SPECIFIED;
	/* Using I2C interface*/
	if ((*cfgPtr)->Intf == BME280_Interface_I2C) {
		//return BME280_I2C_WriteWrapper((*cfgPtr)->I2C_Handle, regAddr, data, len);
	}
	/* Using SPI interface */
	else if ((*cfgPtr)->Intf == BME280_Interface_SPI) {
		result = BME280_SPI_WriteWrapper(regAddr, data_byte);

	}
	return result;

}

static BME280_Result BME280_isInstance(
		BME280_Handle *cfgPtr){

	/* Null check */
	if (*cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;

	for (uint8 c = 0; c < BME280_MAX_SENSOR_POOL_SIZE; ++c) {
		/* Check if the passed pointer already points to an occupied instance */
		if ((*cfgPtr) == &sensorPool[c] && sensorPool[c].occupied == TRUE) {
			return BME280_IS_INSTANCE;
		}
	}

	/* Not an active instance of the sensor pool*/
	return BME280_NOT_INSTANCE;
}

static BME280_Result BME280_getInstance(
		BME280_Handle *cfgPtr){
	for (uint8 c = 0; c < BME280_MAX_SENSOR_POOL_SIZE; ++c) {
		/* Search for an empty instance in the pool */
		if ((*cfgPtr) != &sensorPool[c] && sensorPool[c].occupied == FALSE) {
			(*cfgPtr) = &sensorPool[c];
			sensorPool[c].occupied = TRUE;
			return BME280_FOUND_EMPTY_INSTANCE;
		} else if ((*cfgPtr) == &sensorPool[c] && sensorPool[c].occupied == TRUE) {
			return BME280_IS_INSTANCE;
		}
	}

	if ((*cfgPtr) == NULL_PTR)
		return BME280_POOL_FULL;

	return BME280_OK;
}

/* Getter functions for registers */

static BME280_Result BME280_getConfigRegister(
		BME280_Handle *cfgPtr,
		BME280_ConfigRegisterUnion *cfgRegPtr){
	if (cfgRegPtr == NULL_PTR || cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;
	/* Read config register */
	return BME280_Read(cfgPtr, BME280_CONFIG_REGISTER, &cfgRegPtr->config, 1);

}

static BME280_Result BME280_getCtrlMeasRegister(
		BME280_Handle *cfgPtr,
		BME280_CtrlMeasRegisterUnion *ctrlMeasRegPtr){
	if (ctrlMeasRegPtr == NULL_PTR || cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;

	/* Read ctrl-meas register */
	return BME280_Read(cfgPtr, BME280_CTRL_MEAS_REGISTER,
			&ctrlMeasRegPtr->config, 1);
}

static BME280_Result BME280_getCtrlHumRegister(
		BME280_Handle *cfgPtr,
		BME280_CtrlHumRegisterUnion *ctrlHumRegPtr){
	if (ctrlHumRegPtr == NULL_PTR || cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;

	/* Read ctrl-meas register */
	return BME280_Read(cfgPtr, BME280_CTRL_HUM_REGISTER, &ctrlHumRegPtr->config,
			1);
}

static BME280_Result BME280_getStatusRegister(
		BME280_Handle *cfgPtr,
		BME280_StatusRegisterUnion *statusRegPtr){
	if (statusRegPtr == NULL_PTR || cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;

	/* Read ctrl-meas register */
	return BME280_Read(cfgPtr, BME280_STATUS_REGISTER, &statusRegPtr->config, 1);
}

static BME280_Result BME280_getUncompensatedReadings(
		BME280_Handle *cfgPtr,
		BME280_PressureReading *pressure,
		BME280_TemperatureReading *temperature,
		BME280_HumidityReading *humidity){
	/* If all reading pointers are null, function returns null error and no measurement reading is done*/
	boolean readingPtrsBool = (pressure != NULL_PTR)
			|| (temperature != NULL_PTR) || (humidity != NULL_PTR);
	/* Operation result status flag */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE) {
			break;
		}
		/* Null check*/
		if (readingPtrsBool == FALSE) {
			result = BME280_NULL_ERROR;
			break;
		}

		/* Readings union in which to receive raw data */
		BME280_UncompensatedReadings readings;

		/* Burst read for readings from sensor, incoming bytes will be stored and aligned
		 * into readings union */
		result = BME280_Read(cfgPtr, BME280_START_READINGS_ADDRESS,
				readings.arr,
				BME280_READINGS_BYTES_LENGTH);

		/* Check for successful read */
		if (result != BME280_OK)
			break;

		if (pressure != NULL_PTR) {
			/* Zero value so padding is not garbage */
			pressure->pressure = 0;
			pressure->Data.xlsb = readings.Bytes.press_xlsb;
			pressure->Data.lsb = readings.Bytes.press_lsb;
			pressure->Data.msb = readings.Bytes.press_msb;
		}
		if (temperature != NULL_PTR) {
			/* Zero value so padding is not garbage */
			temperature->temperature = 0;
			temperature->Data.xlsb = readings.Bytes.temp_xlsb >> 4;
			temperature->Data.lsb = readings.Bytes.temp_lsb;
			temperature->Data.msb = readings.Bytes.temp_msb;
		}
		if (humidity != NULL_PTR) {
			/* Zero value so padding is not garbage */
			humidity->humidity = 0;
			humidity->Data.lsb = readings.Bytes.hum_lsb;
			humidity->Data.msb = readings.Bytes.hum_msb;
		}
	} while (0);
	return result;
}

static BME280_Result BME280_getCalibData(
		BME280_Handle *cfgPtr){

	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;

		/** Read calibration parameters for:
		 * dig_T1-dig_T3
		 * dig_P1-dig_P9
		 * dig_H1
		 * */
		result = BME280_Read(cfgPtr,
		BME280_TEMP_PRESS_BLOCK_START_ADDRESS,
				(uint8*) ((*cfgPtr)->calib_data_1.arr),
				BME280_TEMP_PRESS_CALIB_BLOCK_SIZE);
		/* Check for successful read */
		if (result != BME280_OK)
			break;

		/** Read calibration parameters for:
		 * dig_H2-dig_H6
		 * */
		result = BME280_Read(cfgPtr, BME280_HUM_BLOCK_START_ADDRESS,
				(uint8*) ((*cfgPtr)->calib_data_2.arr),
				BME280_HUM_CALIB_BLOCK_SIZE);

		/* Check for successful read */
		if (result != BME280_OK)
			break;

		/* Parse calibration data for humidity */
		(*cfgPtr)->calib_data_2.Bytes.dig_H4 =
				((*cfgPtr)->calib_data_2.Bytes.dig_H4_lsb & BME280_LSBYTE_MASK)
						| (((sint16) (*cfgPtr)->calib_data_2.Bytes.dig_H4_msb)
								<< 4);
		(*cfgPtr)->calib_data_2.Bytes.dig_H5 =
				((*cfgPtr)->calib_data_2.Bytes.dig_H5_lsb & BME280_LSBYTE_MASK)
						| (((sint16) (*cfgPtr)->calib_data_2.Bytes.dig_H5_msb)
								<< 4);
	} while (0);
	return result;

}

#if BME280_FLOATING_POINT == BME280_FEATURE_ENABLE
static float64 BME280_compensateTemperature_floatingPoint(
		BME280_Handle *cfgPtr,
		sint32 rawTemperature){
	float64 var1;
	float64 var2;
	float64 temperature;
	float64 temperature_min = -40;
	float64 temperature_max = 85;

	var1 = ((float64) rawTemperature) / 16384.0
			- ((float64) (*cfgPtr)->calib_data_1.words.dig_T1) / 1024.0;
	var1 = var1 * ((float64) (*cfgPtr)->calib_data_1.words.dig_T2);
	var2 = (((float64) rawTemperature) / 131072.0
			- ((float64) (*cfgPtr)->calib_data_1.words.dig_T1) / 8192.0);
	var2 = (var2 * var2) * ((float64) (*cfgPtr)->calib_data_1.words.dig_T3);
	(*cfgPtr)->calib_data_2.Bytes.t_fine_float = (sint32) (var1 + var2);
	temperature = (var1 + var2) / 5120.0;

	if (temperature < temperature_min) {
		temperature = temperature_min;
	} else if (temperature > temperature_max) {
		temperature = temperature_max;
	}

	return temperature;
}

static float64 BME280_compensatePressure_floatingPoint(
		BME280_Handle *cfgPtr,
		sint32 rawPressure){
	float64 var1;
	float64 var2;
	float64 var3;
	float64 pressure;
	float64 pressure_min = 30000.0;
	float64 pressure_max = 110000.0;

	var1 = ((float64) (*cfgPtr)->calib_data_2.Bytes.t_fine_float / 2.0) - 64000.0;
	var2 = var1 * var1 * ((float64) (*cfgPtr)->calib_data_1.words.dig_P6)
			/ 32768.0;
	var2 = var2 + var1 * ((float64) (*cfgPtr)->calib_data_1.words.dig_P5) * 2.0;
	var2 = (var2 / 4.0)
			+ (((float64) (*cfgPtr)->calib_data_1.words.dig_P4) * 65536.0);
	var3 = ((float64) (*cfgPtr)->calib_data_1.words.dig_P3) * var1 * var1
			/ 524288.0;
	var1 = (var3 + ((float64) (*cfgPtr)->calib_data_1.words.dig_P2) * var1)
			/ 524288.0;
	var1 = (1.0 + var1 / 32768.0)
			* ((float64) (*cfgPtr)->calib_data_1.words.dig_P1);

	/* avoid exception caused by division by zero */
	if (var1 > (0.0)) {
		pressure = 1048576.0 - (float64) rawPressure;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((float64) (*cfgPtr)->calib_data_1.words.dig_P9) * pressure
				* pressure / 2147483648.0;
		var2 = pressure * ((float64) (*cfgPtr)->calib_data_1.words.dig_P8)
				/ 32768.0;
		pressure = pressure
				+ (var1 + var2
						+ ((float64) (*cfgPtr)->calib_data_1.words.dig_P7))
						/ 16.0;

		if (pressure < pressure_min) {
			pressure = pressure_min;
		} else if (pressure > pressure_max) {
			pressure = pressure_max;
		}
	} else /* Invalid case */
	{
		pressure = pressure_min;
	}

	return pressure;
}

static float64 BME280_compensateHumidity_floatingPoint(
		BME280_Handle *cfgPtr,
		sint32 rawHumidity){
	float64 humidity;
	float64 humidity_min = 0.0;
	float64 humidity_max = 100.0;
	float64 var1;
	float64 var2;
	float64 var3;
	float64 var4;
	float64 var5;
	float64 var6;

	var1 = ((float64) (*cfgPtr)->calib_data_2.Bytes.t_fine_float) - 76800.0;
	var2 = (((float64) (*cfgPtr)->calib_data_2.Bytes.dig_H4) * 64.0
			+ (((float64) (*cfgPtr)->calib_data_2.Bytes.dig_H5) / 16384.0)
					* var1);
	var3 = rawHumidity - var2;
	var4 = ((float64) (*cfgPtr)->calib_data_2.Bytes.dig_H2) / 65536.0;
	var5 = (1.0
			+ (((float64) (*cfgPtr)->calib_data_2.Bytes.dig_H3) / 67108864.0)
					* var1);
	var6 = 1.0
			+ (((float64) (*cfgPtr)->calib_data_2.Bytes.dig_H6) / 67108864.0)
					* var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6
			* (1.0
					- ((float64) (*cfgPtr)->calib_data_1.words.dig_H1) * var6
							/ 524288.0);

	if (humidity > humidity_max) {
		humidity = humidity_max;
	} else if (humidity < humidity_min) {
		humidity = humidity_min;
	}

	return humidity;
}

#endif
/* 32 bit compensation for pressure data */
#if BME280_FIXED_POINT == BME280_FEATURE_ENABLE
static sint32 BME280_compensateTemperature_fixedPoint(
		BME280_Handle *cfgPtr,
		sint32 rawTemperature){
	sint32 var1, var2, compensatedTemperature;
	var1 = (sint32) ((rawTemperature / 8)
			- ((sint32) (*cfgPtr)->calib_data_1.words.dig_T1 * 2));

	var1 = (var1 * ((sint32) (*cfgPtr)->calib_data_1.words.dig_T2)) / 2048;

	var2 = (sint32) ((rawTemperature / 16)
			- ((sint32) (*cfgPtr)->calib_data_1.words.dig_T1));

	var2 = (((var2 * var2) / 4096)
			* ((sint32) (*cfgPtr)->calib_data_1.words.dig_T3)) / 16384;

	(*cfgPtr)->calib_data_2.Bytes.t_fine = var1 + var2;

	compensatedTemperature = ((*cfgPtr)->calib_data_2.Bytes.t_fine * 5 + 128)
			/ 256;

	return compensatedTemperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
static uint32 BME280_compensatePressure_fixedPoint(
		BME280_Handle *a_configPtr,
		sint32 rawPressure){

	sint32 var1 = 0, var2 = 0, var3 = 0, var4 = 0;
	uint32 var5 = 0;
	uint32 a_PressureAfterCalibrating = 0;

	uint32 pressure_min = 30000;
	uint32 pressure_max = 110000;
	/* Sensor Calibration calculations */

	var1 = (((int32_t) (*a_configPtr)->calib_data_2.Bytes.t_fine) / 2)
			- (int32_t) 64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048)
			* ((int32_t) (*a_configPtr)->calib_data_1.words.dig_P6);
	var2 = var2
			+ ((var1 * ((int32_t) ((*a_configPtr)->calib_data_1.words.dig_P5)))
					* 2);
	var2 = (var2 / 4)
			+ (((int32_t) ((*a_configPtr)->calib_data_1.words.dig_P4)) * 65536);
	var3 = (((*a_configPtr)->calib_data_1.words.dig_P3)
			* (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t) ((*a_configPtr)->calib_data_1.words.dig_P2)) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1))
			* ((int32_t) ((*a_configPtr)->calib_data_1.words.dig_P1))) / 32768;

	/* avoid exception caused by division by zero */
	if (var1) {
		var5 = (uint32_t) ((uint32_t) 1048576) - (rawPressure);

		a_PressureAfterCalibrating = ((uint32_t) (var5
				- (uint32_t) (var2 / 4096))) * 3125;

		if (a_PressureAfterCalibrating < 0x80000000) {
			a_PressureAfterCalibrating = ((a_PressureAfterCalibrating) << 1)
					/ ((uint32_t) var1);
		} else {
			(a_PressureAfterCalibrating) = ((a_PressureAfterCalibrating)
					/ (uint32_t) var1) * 2;
		}

		var1 = (((int32_t) ((*a_configPtr)->calib_data_1.words.dig_P9))
				* ((int32_t) ((((a_PressureAfterCalibrating) / 8)
						* ((a_PressureAfterCalibrating) / 8)) / 8192))) / 4096;
		var2 = (((int32_t) ((a_PressureAfterCalibrating) / 4))
				* ((int32_t) ((*a_configPtr)->calib_data_1.words.dig_P8)))
				/ 8192;
		(a_PressureAfterCalibrating) =
				(uint32_t) ((int32_t) (a_PressureAfterCalibrating)
						+ ((var1 + var2
								+ ((*a_configPtr)->calib_data_1.words.dig_P7))
								/ 16));

		/* Check if we exceeded the temperature ranges */

		if ((a_PressureAfterCalibrating) < pressure_min) {
			(a_PressureAfterCalibrating) = pressure_min;

		} else if ((a_PressureAfterCalibrating) > pressure_max) {
			(a_PressureAfterCalibrating) = pressure_max;

		}
	} else /* Check if we exceeded the temperature ranges */

	{
		(a_PressureAfterCalibrating) = pressure_min;

	}

	return a_PressureAfterCalibrating;
}

static uint32 BME280_compensateHumidity_fixedPoint(
		BME280_Handle *cfgPtr,
		sint32 rawHumidity){
	sint32 var1;
	sint32 var2;
	sint32 var3;
	sint32 var4;
	sint32 var5;
	uint32 compensatedHumidity;
	uint32 humidity_max = 102400;

	var1 = (*cfgPtr)->calib_data_2.Bytes.t_fine - ((sint32) 76800);
	var2 = (sint32) (rawHumidity * 16384);
	var3 = (sint32) (((sint32) (*cfgPtr)->calib_data_2.Bytes.dig_H4) * 1048576);
	var4 = ((sint32) (*cfgPtr)->calib_data_2.Bytes.dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (sint32) 16384) / 32768;
	var2 = (var1 * ((sint32) (*cfgPtr)->calib_data_2.Bytes.dig_H6)) / 1024;
	var3 = (var1 * ((sint32) (*cfgPtr)->calib_data_2.Bytes.dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (sint32) 32768)) / 1024) + (sint32) 2097152;
	var2 = ((var4 * ((sint32) (*cfgPtr)->calib_data_2.Bytes.dig_H2)) + 8192)
			/ 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3
			- ((var4 * ((sint32) (*cfgPtr)->calib_data_1.words.dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	compensatedHumidity = (uint32) (var5 / 4096);

	if (compensatedHumidity > humidity_max) {
		compensatedHumidity = humidity_max;
	}

	return compensatedHumidity;
}

#endif
/*******************************************************************************
 *                          Public function definitions                        *
 *******************************************************************************/

BME280_Result BME280_getChipID(
		BME280_Handle *cfgPtr,
		uint8 *chipID){
	/* Null check */
	if (chipID == NULL_PTR)
		return BME280_NULL_ERROR;
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);

	do {
		result = BME280_isInstance(cfgPtr);

		if (result != BME280_IS_INSTANCE)
			break;
		result = BME280_Read(cfgPtr, BME280_ID_REGISTER, chipID, 1);
	} while (0);

	return result;
}

BME280_Result BME280_init(
		BME280_Handle *cfgPtr){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {

		result = BME280_getInstance(cfgPtr);
		/**
		 Check if an empty sensor instance is found or
		 if it's already an existing instance
		 */
		if (result == BME280_POOL_FULL)
			break;
		/* Discover sensor try count */
		uint8 tryCount = BME280_MAX_DISCOVERY_COUNT;
		/* Buffer to receive data in */
		uint8 chipID = 0;

		do {

			/* Attempt to get ID from ID register */
			result = BME280_getChipID(cfgPtr, &chipID);
			/* If ID is not returned successfully, try again */
			if (chipID != BME280_CHIP_ID || result != BME280_OK) {
				++tryCount;
				chipID = 0;
				continue;
			}
			/* Sensor communication is OK */
			else {

				/* Attempt to reset the sensor by writing the reset word into the reset register*/
				result = BME280_softReset(cfgPtr);

				if (result != BME280_OK)
					return BME280_COMM_ERROR;
				/* Get calibration data from the sensor */
				result = BME280_getCalibData(cfgPtr);

//				test1(cfgPtr);

				if (result != BME280_OK)
					break;
				/* Set up user configuration for the sensor*/

				if (result != BME280_OK)
					return BME280_COMM_ERROR;

				/* Send control measurement byte */

				if (result != BME280_OK)
					return BME280_COMM_ERROR;
				/* Send control measurement byte */

				if (result != BME280_OK)
					return BME280_COMM_ERROR;

				/* Break out of loop as config and communication was successful*/
				break;
			}
		} while (tryCount > BME280_MAX_DISCOVERY_COUNT);
	} while (0);
	return result;
}

BME280_Result BME280_softReset(
		BME280_Handle *cfgPtr){

	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {

		result = BME280_isInstance(cfgPtr);

		if (result != BME280_IS_INSTANCE)
			break;

		result = BME280_NOT_YET_OBTAINED;

		/* Reset register address*/
		uint8 regAddr = BME280_RESET_REGISTER;

		/* Reset byte which resets sensor when written into the reset register*/
		uint8 reset_byte = BME280_RESET_WORD;

		/* Send reset word */
		result = BME280_Write(cfgPtr, regAddr, reset_byte);

		/* Status register which contains IM_UPDATE bit*/
		regAddr = BME280_STATUS_REGISTER;
		uint8 reg_data = 0xFF;
		result = BME280_NOT_YET_OBTAINED;

		/*Wait on sensor to boot and read the IM_UPDATE bit to reset indicating successful boot*/
		/* Wait for NVM data to be copied */
		do {
			/* Now we wait for start up time for the sensor to boot */
			BME280_delayMs(BME280_START_UP_TIME_MS);
			result = BME280_Read(cfgPtr, regAddr, &reg_data, 1);
			/* Loop on IM_UDATE flag to be ready*/
		} while (result != BME280_OK
				&& BME280_CONFIG_IM_UPDATE_MASK(reg_data)
						!= BME280_IM_UPDATE_READY);
	} while (0);
	return result;

}

uint16 BME280_calculateMeasurementDelayMs(
		BME280_Settings *a_settings){
	/**
	 * Relation between sampling settings and real xX values:
	 * Input BME280_OVERSAMPLING_x16 = 0x05
	 * Output 16
	 *
	 * 2<<(0x05-1) = 2<<(0x04) = 2<<4 = 16
	 *
	 * Input BME280_OVERSAMPLING_x8 = 0x04
	 * Output 8
	 * 2<<(0x04-1) = 2<<(0x03) = 2<<3 = 8
	 *
	 */

	uint8 osrs_t, osrs_p, osrs_h;
	float32 ODR;
	float32 tMeasure_ms;
	osrs_t =
			((a_settings->osrs_t) == (BME280_OVERSAMPLING_OFF)) ?
					0 : (2 << (a_settings->osrs_t - 1));
	osrs_p =
			((a_settings->osrs_p) == (BME280_OVERSAMPLING_OFF)) ?
					0 : (2 << (a_settings->osrs_p - 1));
	osrs_h =
			((a_settings->osrs_h) == (BME280_OVERSAMPLING_OFF)) ?
					0 : (2 << (a_settings->osrs_h - 1));

	tMeasure_ms = 1.25 + (2.3 * ((float32) osrs_t))
			+ (2 * ((float32) osrs_p) + 0.5) + (2 * ((float32) osrs_h) + 0.5);
	/* Filter is enabled */
	if (a_settings->filter != BME280_FILTER_COEFF_OFF) {
		/* Map standby time to their values*/
		float32 tStandby_map[] = { 0.5, 62.5, 125.0, 250.0, 500.0, 1000.0, 10.0,
				20.0 };
		float32 filter_map[] = { 1, 2, 5, 11, 22 };

		switch (a_settings->Mode) {
		case BME280_Mode_Forced:
			ODR = 1000 / tMeasure_ms;
			break;
		case BME280_Mode_Normal:
			ODR = (1000) / ((tMeasure_ms + tStandby_map[a_settings->t_stby]));
			break;
		default:
			ODR = 1;
			break;
		}
		return (uint16) (((float32) 1000 * filter_map[a_settings->filter]) / ODR);
	}
	/* Filter disabled*/
	else
		return (uint16) tMeasure_ms;
}

BME280_Result BME280_getTemperature_fixedPoint(
		BME280_Handle *cfgPtr,
		sint32 *temperature){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (temperature == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(cfgPtr, &rawPress, &rawTemp,
				&rawHum);
		if (result != BME280_OK)
			break;

		/* Compensate readings */
		*temperature = BME280_compensateTemperature_fixedPoint(cfgPtr,
				rawTemp.temperature);

	} while (0);
	return result;
}

BME280_Result BME280_getPressure_fixedPoint(
		BME280_Handle *cfgPtr,
		uint32 *pressure){

	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (pressure == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(cfgPtr, &rawPress, &rawTemp,
				&rawHum);
		if (result != BME280_OK)
			break;
		/*
		 * For pressure or humidity:
		 * We must get t_fine parameter by reading the temperature data, so we need to compensate temperature data
		 * as well
		 *
		 * */
		/* Compensate readings */
		BME280_compensateTemperature_fixedPoint(cfgPtr, rawTemp.temperature);
		*pressure = BME280_compensatePressure_fixedPoint(cfgPtr,
				rawPress.pressure);

	} while (0);
	return result;

}

BME280_Result BME280_getHumidity_fixedPoint(
		BME280_Handle *cfgPtr,
		uint32 *humidity){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (humidity == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(cfgPtr, &rawPress, &rawTemp,
				&rawHum);
		if (result != BME280_OK)
			break;
		/*
		 * For pressure or humidity:
		 * We must get t_fine parameter by reading the temperature data, so we need to compensate temperature data
		 * as well
		 *
		 * */
		/* Compensate readings */
		BME280_compensateTemperature_fixedPoint(cfgPtr, rawTemp.temperature);
		*humidity = BME280_compensateHumidity_fixedPoint(cfgPtr,
				rawHum.humidity);

	} while (0);
	return result;
}




BME280_Result BME280_getTemperature_floatingPoint(
		BME280_Handle *cfgPtr,
		float64 *temperature){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (temperature == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(cfgPtr, &rawPress, &rawTemp,
				&rawHum);
		if (result != BME280_OK)
			break;

		/* Compensate readings */
		*temperature = BME280_compensateTemperature_floatingPoint(cfgPtr,
				rawTemp.temperature);

	} while (0);
	return result;
}

BME280_Result BME280_getPressure_floatingPoint(
		BME280_Handle *cfgPtr,
		float64 *pressure){

	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (pressure == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(cfgPtr, &rawPress, &rawTemp,
				&rawHum);
		if (result != BME280_OK)
			break;
		/*
		 * For pressure or humidity:
		 * We must get t_fine parameter by reading the temperature data, so we need to compensate temperature data
		 * as well
		 *
		 * */
		/* Compensate readings */
		BME280_compensateTemperature_floatingPoint(cfgPtr, rawTemp.temperature);
		*pressure = BME280_compensatePressure_floatingPoint(cfgPtr,
				rawPress.pressure);

	} while (0);
	return result;

}

BME280_Result BME280_getHumidity_floatingPoint(
		BME280_Handle *cfgPtr,
		float64 *humidity){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (humidity == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(cfgPtr, &rawPress, &rawTemp,
				&rawHum);
		if (result != BME280_OK)
			break;
		/*
		 * For pressure or humidity:
		 * We must get t_fine parameter by reading the temperature data, so we need to compensate temperature data
		 * as well
		 *
		 * */
		/* Compensate readings */
		BME280_compensateTemperature_floatingPoint(cfgPtr, rawTemp.temperature);
		*humidity = BME280_compensateHumidity_floatingPoint(cfgPtr,
				rawHum.humidity);

	} while (0);
	return result;
}

/* Setter functions for settings */

BME280_Result BME280_setPressureOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting pressureOversampling){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {

		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current Ctrl-meas register value */
		BME280_CtrlMeasRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_CtrlMeasRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getCtrlMeasRegister(cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set pressure over-sampling setting then write it to the sensor*/
		reg.Bits.osrs_p = pressureOversampling;
		result = BME280_Write(cfgPtr, BME280_CTRL_MEAS_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getCtrlMeasRegister(cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;

	} while (0);
	return result;

}

BME280_Result BME280_setTemperatureOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting temperatureOversampling){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {

		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current Ctrl-meas register value */
		BME280_CtrlMeasRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_CtrlMeasRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getCtrlMeasRegister(cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set temperature over-sampling setting then write it to the sensor*/
		reg.Bits.osrs_t = temperatureOversampling;
		result = BME280_Write(cfgPtr, BME280_CTRL_MEAS_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getCtrlMeasRegister(cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;

	} while (0);
	return result;

}

BME280_Result BME280_setHumidityOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting humidityOversampling){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current Ctrl-hum register value */
		BME280_CtrlHumRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_CtrlHumRegisterUnion verity_reg = { 0 };
		BME280_CtrlMeasRegisterUnion reg_meas = { 0 };

		/* Read register from sensor */
		result = BME280_getCtrlHumRegister(cfgPtr, &reg);
		if (result != BME280_OK)
			return result;
		/* Set humidity over-sampling setting then write it to the sensor*/
		reg.Bits.osrs_h = humidityOversampling;

		/* Send control humidity byte, must be modified AFTER modifying
		 * ctrl_meas register as per data-sheet:
		 *
		 * "Changes to this register only become effective after
		 *  a write operation to "ctrl_meas"."
		 * */
		result = BME280_getCtrlMeasRegister(cfgPtr, &reg_meas);
		result = BME280_setMode(cfgPtr, reg_meas.Bits.mode);

		//result = BME280_Write(cfgPtr, BME280_CTRL_MEAS_REGISTER, reg.config);
		//BME280_delayMs(10);
		result = BME280_Write(cfgPtr, BME280_CTRL_HUM_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getCtrlHumRegister(cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;

	} while (0);
	return result;
}

BME280_Result BME280_setStandbyTime(
		BME280_Handle *cfgPtr,
		BME280_StandbyTime standbyTime){
	/* Interface return value */
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {

		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current config register value */
		BME280_ConfigRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_ConfigRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getConfigRegister(cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set standby time setting then write it to the sensor*/
		reg.Bits.t_sb = standbyTime;
		result = BME280_Write(cfgPtr, BME280_CONFIG_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getConfigRegister(cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;

	} while (0);
	return result;

}

BME280_Result BME280_setMode(
		BME280_Handle *cfgPtr,
		BME280_ModeType mode){

	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current Ctrl-meas register value */
		BME280_CtrlMeasRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_CtrlMeasRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getCtrlMeasRegister(cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set mode setting then write it to the sensor*/
		reg.Bits.mode = mode;
		result = BME280_Write(cfgPtr, BME280_CTRL_MEAS_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getCtrlMeasRegister(cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;

	} while (0);

	return result;

}

BME280_Result BME280_setFilterCoefficient(
		BME280_Handle *cfgPtr,
		BME280_FilterCoeff filterCoeff){
	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current config register value */
		BME280_ConfigRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_ConfigRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getConfigRegister(cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set filter coefficient setting then write it to the sensor*/
		reg.Bits.filter_coeff = filterCoeff;
		result = BME280_Write(cfgPtr, BME280_CONFIG_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getConfigRegister(cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;
	} while (0);

	return result;
}

/* Getter functions for settings */

BME280_Result BME280_getUpdateStatus(
		BME280_Handle *cfgPtr,
		BME280_UpdateStatus *updateFlag){
	BME280_Result result = BME280_isInstance(cfgPtr);

	do {
		if (result != BME280_IS_INSTANCE) {
			break;
		}
		if (updateFlag == NULL_PTR) {
			result = BME280_NULL_ERROR;
		}

		BME280_StatusRegisterUnion statusReg = { 0 };
		/* Get status register */
		result = BME280_getStatusRegister(cfgPtr, &statusReg);
		if (result != BME280_OK) {
			break;
		}
		if (statusReg.Bits.im_update == BME280_IM_UPDATE_READY) {
			*updateFlag = BME280_Update_Finished;
		} else {

			*updateFlag = BME280_Update_Copying;
		}
		break;
	} while (0);

	return result;
}

BME280_Result BME280_getMeasuringStatus(
		BME280_Handle *cfgPtr,
		BME280_MeasuringStatus *measureFlag){
	BME280_Result result = BME280_isInstance(cfgPtr);

	do {
		if (result != BME280_IS_INSTANCE) {
			break;
		}
		if (measureFlag == NULL_PTR) {
			result = BME280_NULL_ERROR;
		}

		BME280_StatusRegisterUnion statusReg = { 0 };
		/* Get status register */
		result = BME280_getStatusRegister(cfgPtr, &statusReg);
		if (result != BME280_OK) {
			break;
		}
		if (statusReg.Bits.measuring == BME280_MEASURING_DONE) {
			*measureFlag = BME280_Measuring_Finished;
		} else {

			*measureFlag = BME280_Measuring_Running;
		}
		break;
	} while (0);
	return result;
}

BME280_Result BME280_getMode(
		BME280_Handle *cfgPtr,
		BME280_ModeType *mode){
	if (*cfgPtr == NULL_PTR || mode == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_CtrlMeasRegisterUnion ctrlMeasReg;
	BME280_Result result = BME280_getCtrlMeasRegister(cfgPtr,
			&ctrlMeasReg);
	*mode = (BME280_ModeType) ctrlMeasReg.Bits.mode;
	return result;
}

BME280_Result BME280_getTemperatureOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting *oversampling){
	if (*cfgPtr == NULL_PTR || oversampling == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_CtrlMeasRegisterUnion ctrlMeasReg;
	BME280_Result result = BME280_getCtrlMeasRegister(cfgPtr,
			&ctrlMeasReg);
	*oversampling = (BME280_Oversampling_setting) ctrlMeasReg.Bits.osrs_t;
	return result;
}

BME280_Result BME280_getPressureOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting *oversampling){
	if (*cfgPtr == NULL_PTR || oversampling == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_CtrlMeasRegisterUnion ctrlMeasReg = { 0 };
	BME280_Result result = BME280_getCtrlMeasRegister(cfgPtr,
			&ctrlMeasReg);
	*oversampling = (BME280_Oversampling_setting) ctrlMeasReg.Bits.osrs_p;
	return result;
}

BME280_Result BME280_getHumidityOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting *oversampling){
	if (*cfgPtr == NULL_PTR || oversampling == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_CtrlHumRegisterUnion ctrlHumReg = { 0 };
	BME280_Result result = BME280_getCtrlHumRegister(cfgPtr,
			&ctrlHumReg);
	*oversampling = (BME280_Oversampling_setting) ctrlHumReg.Bits.osrs_h;
	return result;
}

BME280_Result BME280_getFilterCoefficient(
		BME280_Handle *cfgPtr,
		BME280_FilterCoeff *filterCoeff){
	if (*cfgPtr == NULL_PTR || filterCoeff == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_ConfigRegisterUnion configReg = { 0 };
	BME280_Result result = BME280_getConfigRegister(cfgPtr,
			&configReg);
	*filterCoeff = (BME280_FilterCoeff) configReg.Bits.filter_coeff;
	return result;
}

BME280_Result BME280_getStandbyTime(
		BME280_Handle *cfgPtr,
		BME280_StandbyTime *standbyTime){
	if (*cfgPtr == NULL_PTR || standbyTime == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_ConfigRegisterUnion configReg = { 0 };
	BME280_Result result = BME280_getConfigRegister(cfgPtr,
			&configReg);
	*standbyTime = (BME280_StandbyTime) configReg.Bits.t_sb;
	return result;
}

BME280_Result BME280_getSensorSettings(
		BME280_Handle *cfgPtr,
		BME280_Settings *settings){

	BME280_Result result = BME280_isInstance(cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (settings == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}

		result = BME280_getHumidityOversampling(cfgPtr, &settings->osrs_h);
		if (result != BME280_OK)
			break;
		result = BME280_getTemperatureOversampling(cfgPtr,& settings->osrs_t);
		if (result != BME280_OK)
			break;
		result = BME280_getPressureOversampling(cfgPtr, &settings->osrs_p);
		if (result != BME280_OK)
			break;
		result = BME280_getFilterCoefficient(cfgPtr, &settings->filter);
		if (result != BME280_OK)
			break;
		result = BME280_getStandbyTime(cfgPtr, &settings->t_stby);
		if (result != BME280_OK)
			break;
		result = BME280_getMode(cfgPtr, &settings->Mode);
		if (result != BME280_OK)
			break;

	} while (0);
	return result;

}
__attribute__((weak))BME280_Result BME280_SPI_TransmitReceive(
		uint8 *txData,
		uint8 *rxData,
		uint16 size,
		uint32 timeout){
	/* To be implemented by the user */
	return BME280_NOT_IMPLEMENTED;
}

__attribute__((weak)) void BME280_delayMs(
		uint32 a_milliseconds){
	/* To be implemented by the user */
	return BME280_NOT_IMPLEMENTED;
}
