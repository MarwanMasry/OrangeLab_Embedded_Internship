/******************************************************************************
 *
 * Module: Bosch Sensortec BME280
 *
 * File Name: bme280.c
 *
 * Description: Source file for BME280 Sensor API function implementation as well
 * 			as private functions.
 *
 * Date Created: 12/9/2022
 *
 * Author: Hazem Montasser
 *
 *******************************************************************************/

#include <bme280_private_defs.h>
#include <bme280_private_types.h>
#include "bme280.h"
#include <std_types.h>
/*******************************************************************************
 *                          Private global variables						   *
 *******************************************************************************/

static struct BME280_ConfigType BME280_sensorPool[BME280_MAX_SENSOR_POOL_SIZE] =
		{ 0 };

/*******************************************************************************
 *                          Private function prototypes                        *
 *******************************************************************************/

/**
 * @fn BME280_Status BME280_Read(BME280_Handle *a_cfgPtr, BME280_uint8 a_regAddr, BME280_uint8 *data, BME280_uint8 len)
 * @brief Abstract function used to read data from the sensor based on
 * the selected interface
 *
 * @param a_cfgPtr: Pointer to the sensor configuration struct
 * @param a_regAddr: Register address to read data from
 * @param data: Buffer to read data into
 * @param len: Length of the data to read
 *
 * @return BME280_Status
 */
static BME280_Status BME280_Read(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 *a_data, BME280_uint8 a_len);

/**
 * @fn BME280_Status BME280_Write(BME280_Handle *a_cfgPtr, BME280_uint8 a_regAddr, BME280_uint8 data_byte)
 * @brief Abstract function used to write data to the sensor based on
 * the selected interface
 *
 * @param a_cfgPtr: Pointer to the sensor configuration struct
 * @param a_regAddr: Register address to write to
 * @param data_byte: Data byte to write into the register address
 *
 * @return BME280_Status
 */
static BME280_Status BME280_Write(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 data_byte);

/**
 * @fn BME280_Status BME280_SPI_ReadWrapper(BME280_Handle*, BME280_uint8, BME280_uint8*, BME280_uint8)
 * @brief	Wraps user implemented SPI write function to be used when sensor is in I2C interface.
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_regAddr		Address of register to read from in the sensor
 * @param a_recvBuff	Receive buffer to receive data in
 * @param a_len			Number of bytes to receive into a_recvBuff
 * @return
 */
static BME280_Status BME280_SPI_ReadWrapper(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 *a_recvBuff, BME280_uint8 a_len);

/**
 * @fn BME280_Status BME280_SPI_WriteWrapper(BME280_Handle*, BME280_uint8, BME280_uint8)
 * @brief	Wraps user implemented SPI write function to be used when sensor is in I2C interface.
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_regAddr		Address of register to read from in the sensor
 * @param a_dataByte	Data byte to write at register in the sensor
 * @return
 */
static BME280_Status BME280_SPI_WriteWrapper(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 a_dataByte);

/**
 * @fn BME280_Status BME280_I2C_ReadWrapper(BME280_Handle*, BME280_uint8, BME280_uint8*, BME280_uint8)
 * @brief	Wraps user implemented I2C read function to be used when sensor is in I2C interface.
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_regAddr		Address of register to read from in the sensor
 * @param a_recvBuff	Receive buffer to receive data in
 * @param a_len			Number of bytes to receive into a_recvBuff
 * @return
 */
static BME280_Status BME280_I2C_ReadWrapper(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 *a_recvBuff, BME280_uint8 a_len);

/**
 * @fn BME280_Status BME280_I2C_WriteWrapper(BME280_Handle*, BME280_uint8, BME280_uint8)
 * @brief 	Wraps user implemented I2C write function to be used when sensor is in I2C interface.
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_regAddr		Address of register to read from in the sensor
 * @param a_dataByte	Data byte to write at register in the sensor
 * @return
 */
static BME280_Status BME280_I2C_WriteWrapper(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 a_dataByte);

/**
 * @fn BME280_Status BME280_getUncompensatedReadings(BME280_Handle*, BME280_PressureReading*, BME280_TemperatureReading*, BME280_HumidityReading*)
 * @brief  Reads ALL _raw_ un-compensated readings from the sensor in burst-read mode for
 * efficiency, all readings are aligned into the readingsPtr union.
 *
 *
 * @param a_cfgPtr		Configuration struct used to interface with the sensor
 * @param a_pressure		Pointer to pressure reading union
 * @param a_temperature	Pointer to temperature reading union
 * @param a_humidity		Pointer to humidity reading union
 * @return
 */
static BME280_Status BME280_getUncompensatedReadings(BME280_Handle *a_cfgPtr,
		BME280_PressureReading *a_pressure,
		BME280_TemperatureReading *a_temperature,
		BME280_HumidityReading *a_humidity);

/**
 * @fn BME280_Status BME280_getCalibData(BME280_Handle*)
 * @brief Reads ALL calibration data from the sensor
 * 		  Calibration data:
 * 		  	- dig_T1 to dig_T3
 * 		  	- dig_P1 to dig_P9
 * 		  	- dig_H1 to dig_H6
 * 		  Stores them in a_cfgPtr Calib1 and Calib2 structures
 *
 * @param a_cfgPtr
 * @return
 */
static BME280_Status BME280_getCalibData(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_getConfigRegister(BME280_Handle*, BME280_ConfigRegisterUnion*)
 * @brief Retrieves config register from the sensor
 *
 * @param a_cfgPtr	Pointer to sensor handle
 * @param a_cfgRegPtr Pointer to config register variable which will contain the sensor config register
 * @return
 */
static BME280_Status BME280_getConfigRegister(BME280_Handle *a_cfgPtr,
		BME280_ConfigRegisterUnion *a_cfgRegPtr);

/**
 * @fn BME280_Status BME280_getCtrlMeasRegister(BME280_Handle*, BME280_CtrlMeasRegisterUnion*)
 * @brief Retrieves ctrl_meas register from the sensor
 *
 * @param a_cfgPtr	Pointer to sensor handle
 * @param a_cfgRegPtr Pointer to ctrl_meas register variable which will contain the sensor ctrl_meas register
 * @return
 */
static BME280_Status BME280_getCtrlMeasRegister(BME280_Handle *a_cfgPtr,
		BME280_CtrlMeasRegisterUnion *a_cfgRegPtr);

/**
 * @fn BME280_Status BME280_getCtrlHumRegister(BME280_Handle*, BME280_CtrlHumRegisterUnion*)
 * @brief Retrieves ctrl_hum register from the sensor
 *
 * @param a_cfgPtr	Pointer to sensor handle
 * @param a_cfgRegPtr Pointer to ctrl_hum register variable which will contain the sensor ctrl_hum register
 * @return
 */
static BME280_Status BME280_getCtrlHumRegister(BME280_Handle *a_cfgPtr,
		BME280_CtrlHumRegisterUnion *a_cfgRegPtr);

/**
 * @fn BME280_Status BME280_getStatusRegister(BME280_Handle*, BME280_StatusRegisterUnion*)
 * @brief Retrieves status register from the sensor
 *
 * @param a_cfgPtr	Pointer to sensor handle
 * @param a_cfgRegPtr Pointer to status register variable which will contain the sensor status register
 * @return
 */
static BME280_Status BME280_getStatusRegister(BME280_Handle *a_cfgPtr,
		BME280_StatusRegisterUnion *a_cfgRegPtr);

/**
 * @fn BME280_Status BME280_isInstance(BME280_Handle*)
 * @brief Verifies whether the handle is tied to an existing instance in the sensor
 * 		pool or not. Returns certain error codes if pool is full or handle points to null
 * 		- BME280_IS_INSTANCE  -> Handle points to an instance
 * 		- BME280_NULL_ERROR   -> Handle points to null
 * 		- BME280_NOT_INSTANCE -> Handle points to something else other than sensor instances
 *
 * @param a_cfgPtr
 * @return
 */
static BME280_Status BME280_isInstance(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_float64 BME280_compensateTemperature_floatingPoint(BME280_Handle*, BME280_sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate temperature as floating point notation.
 * 		Ex: returns 32.52 -> 32.52 c
 *
 * @param a_cfgPtr		 Pointer to sensor handle
 * @param rawTemperature Raw temperature data read by the sensor
 * @return Temperature in degrees Celsius
 */
static BME280_float64 BME280_compensateTemperature_floatingPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawTemperature);

/**
 * @fn BME280_float64 BME280_compensatePressure_floatingPoint(BME280_Handle*, BME280_sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate pressure as fixed point notation.
 * 		Ex: returns 9338.2 -> 9338.2 Pa
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_rawPressure	Raw pressure data read by the sensor
 * @return Pressure in Pascal
 */
static BME280_float64 BME280_compensatePressure_floatingPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawPressure);

/**
 * @fn BME280_float64 BME280_compensateHumidity_floatingPoint(BME280_Handle*, BME280_sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate pressure as fixed point notation.
 * 		Ex: returns 3252 -> 32.52c
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_rawHumidity	Raw humidity data read by the sensor
 * @return Relative humidity in percentage (%)
 */
static BME280_float64 BME280_compensateHumidity_floatingPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawHumidity);

/**
 * @fn BME280_sint32 BME280_compensateTemperature_fixedPoint(BME280_Handle*, BME280_sint32)
 * @brief Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate temperature as fixed point notation.
 * 		Ex: returns 3252 -> 32.52 c
 *
 * @param a_cfgPtr			Pointer to sensor handle
 * @param a_rawTemperature	Raw temperature data read by the sensor
 * @return Temperature in Celsius (fixed point)
 */
static BME280_sint32 BME280_compensateTemperature_fixedPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawTemperature);

/**
 * @fn BME280_uint32 BME280_compensatePressure_fixedPoint(BME280_Handle*, BME280_sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate pressure as fixed point notation.
 * 		Ex: returns 93211 -> 93.211 Pa = 932.11 hPa
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_rawPressure	Raw pressure data read by the sensor
 * @return Pressure in Pascal (fixed point)
 */
static BME280_uint32 BME280_compensatePressure_fixedPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawPressure);

/**
 * @fn BME280_uint32 BME280_compensateHumidity_fixedPoint(BME280_Handle*, BME280_sint32)
 * @brief	Uses calibration data and formulas from data-sheet to
 * 		compensate and calculate relative humidity as fixed point notation.
 * 		Ex: returns 4288 -> 42284/1024 -> 41.29% rH
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_rawHumidity	Raw humdity data read by the sensor
 * @return	Relative humidity in percent, to be divided by 1024 to get exact value
 */
static BME280_uint32 BME280_compensateHumidity_fixedPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawHumidity);

/**
 * @fn void BME280_DeInitHandle(BME280_Handle*)
 * @brief Function used to zero out all handle parameters for the passed
 * 		handle. Handle is assumed to already be an instance as this function
 * 		is private and used inside public API functions.
 *
 * @param a_cfgPtr	Pointer to sensor handle
 */
static void BME280_DeInitHandle(BME280_Handle *a_cfgPtr);

/*******************************************************************************
 *                          Private function definitions                       *
 *******************************************************************************/

static BME280_Status BME280_SPI_ReadWrapper(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 *a_recvBuff, BME280_uint8 a_len) {
	if ((*a_cfgPtr)->GPIOCallback_SetNSS == NULL_PTR
			|| (*a_cfgPtr)->GPIOCallback_resetNSS == NULL_PTR)
		return BME280_CALLBACK_NOT_SET;

	/* Status of SPI transmission */
	BME280_Comm_Status status = 0;

	/* Mask read address in SPI mode */
	a_regAddr = BME280_SPI_READ_MASK(a_regAddr);

	/* Pull SS pin low to write by calling user implemented GPIO callback*/
	(*a_cfgPtr)->GPIOCallback_resetNSS();

	/* Send control byte */
	status = BME280_SPI_TransmitReceive(&a_regAddr, a_recvBuff, 1,
	BME280_SPI_TIMEOUT_MS);

	/* Receive data from sensor */
	status = BME280_SPI_TransmitReceive(&a_regAddr, a_recvBuff, a_len,
	BME280_SPI_TIMEOUT_MS);

	/* Pull up SS pin to indicate end of transmission by calling user implemented GPIO callback */
	(*a_cfgPtr)->GPIOCallback_SetNSS();

	if (status == BME280_Comm_OK)
		return BME280_OK;
	else
		return BME280_COMM_ERROR;
}

static BME280_Status BME280_SPI_WriteWrapper(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 a_dataByte) {
	if ((*a_cfgPtr)->GPIOCallback_SetNSS == NULL_PTR
			|| (*a_cfgPtr)->GPIOCallback_resetNSS == NULL_PTR)
		return BME280_CALLBACK_NOT_SET;
	/* Status of SPI transmission */
	BME280_Comm_Status status = 0;

	/* Dummy byte to receive in */
	BME280_uint8 dummy = 0;

	/* Mask write address in SPI mode */
	a_regAddr = BME280_SPI_WRITE_MASK(a_regAddr);

	/* Pull SS pin low to write by calling user implemented GPIO callback*/
	(*a_cfgPtr)->GPIOCallback_resetNSS();

	/* Send control byte */
	status = BME280_SPI_TransmitReceive(&a_regAddr, &dummy, 1,
	BME280_SPI_TIMEOUT_MS);

	/* Send data to write */
	status = BME280_SPI_TransmitReceive(&a_dataByte, &dummy, 1,
	BME280_SPI_TIMEOUT_MS);

	/* Pull up SS pin to indicate end of transmission by calling user implemented GPIO callback */
	(*a_cfgPtr)->GPIOCallback_SetNSS();

	if (status == BME280_Comm_OK)
		return BME280_OK;
	else
		return BME280_COMM_ERROR;
}

static BME280_Status BME280_I2C_ReadWrapper(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 *a_recvBuff, BME280_uint8 a_len) {

	/* Status of SPI transmission */
	BME280_Comm_Status status = 0;

	/* First, the sensor address must be sent in R/W = 0 for write mode followed by control byte (register address)*/
	status = BME280_I2C_Master_Transmit(
			BME280_I2C_WRITE_MASK((*a_cfgPtr)->I2C_SlaveAddr), &a_regAddr, 1,
			BME280_I2C_TIMEOUT_MS);

	/* Next, we re-send the slave address in R/W = 1 for read mode and start reading data (auto-incremented) */
	status = BME280_I2C_Master_Receive(
			BME280_I2C_READ_MASK((*a_cfgPtr)->I2C_SlaveAddr), a_recvBuff, a_len,
			BME280_I2C_TIMEOUT_MS);

	if (status == BME280_Comm_OK)
		return BME280_OK;
	else
		return BME280_COMM_ERROR;
}

static BME280_Status BME280_I2C_WriteWrapper(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 a_dataByte) {
	/* Status of SPI transmission */
	BME280_Comm_Status status = 0;

	/* Send sensor address with R/W = 0 for write mode, followed by register address and data byte*/
	BME280_uint8 arr[] = { a_regAddr, a_dataByte };
	status = BME280_I2C_Master_Transmit(
			BME280_I2C_WRITE_MASK((*a_cfgPtr)->I2C_SlaveAddr), arr, 2,
			BME280_I2C_TIMEOUT_MS);

	if (status == BME280_Comm_OK)
		return BME280_OK;
	else
		return BME280_COMM_ERROR;
}

static BME280_Status BME280_Read(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 *a_data, BME280_uint8 a_len) {
	/* Interface return value */
	BME280_Status result;

	/* Check for used interface, if neither is specified return no interface specified*/
	result = BME280_NO_INTERFACE_SPECIFIED; /* Using I2C interface*/
	if ((*a_cfgPtr)->Intf == BME280_Interface_I2C) {
		return BME280_I2C_ReadWrapper(a_cfgPtr, a_regAddr, a_data, a_len);
	}

	/* Using SPI interface */
	else if ((*a_cfgPtr)->Intf == BME280_Interface_SPI) {
		result = BME280_SPI_ReadWrapper(a_cfgPtr, a_regAddr, a_data, a_len);
	}
	return result;
}

static BME280_Status BME280_Write(BME280_Handle *a_cfgPtr,
		BME280_uint8 a_regAddr, BME280_uint8 a_data_byte) {
	/* Interface return value */
	BME280_Status result;

	/* Check for used interface, if neither is specified return no interface specified*/
	result = BME280_NO_INTERFACE_SPECIFIED;

	/* Using I2C interface*/
	if ((*a_cfgPtr)->Intf == BME280_Interface_I2C) {
		return BME280_I2C_WriteWrapper(a_cfgPtr, a_regAddr, a_data_byte);
	}

	/* Using SPI interface */
	else if ((*a_cfgPtr)->Intf == BME280_Interface_SPI) {
		result = BME280_SPI_WriteWrapper(a_cfgPtr, a_regAddr, a_data_byte);
	}
	return result;

}

static BME280_Status BME280_isInstance(BME280_Handle *a_cfgPtr) {

	/* Null check */
	if (*a_cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;

	for (BME280_uint8 c = 0; c < BME280_MAX_SENSOR_POOL_SIZE; ++c) {
		/* Check if the passed pointer already points to an occupied instance */
		if ((*a_cfgPtr)
				== &BME280_sensorPool[c]&& BME280_sensorPool[c].occupied == TRUE) {
			return BME280_IS_INSTANCE;
		}
	}

	/* Not an active instance of the sensor pool*/
	return BME280_NOT_INSTANCE;
}


/*******************************************************************************
 *                          Getter functions for registers                     *
 *******************************************************************************/

static BME280_Status BME280_getConfigRegister(BME280_Handle *a_cfgPtr,
		BME280_ConfigRegisterUnion *a_cfgRegPtr) {
	if (a_cfgRegPtr == NULL_PTR || a_cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;
	/* Read config register */
	return BME280_Read(a_cfgPtr, BME280_CONFIG_REGISTER, &a_cfgRegPtr->config,
			1);

}

static BME280_Status BME280_getCtrlMeasRegister(BME280_Handle *a_cfgPtr,
		BME280_CtrlMeasRegisterUnion *ctrlMeasRegPtr) {
	if (ctrlMeasRegPtr == NULL_PTR || a_cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;

	/* Read ctrl-meas register */
	return BME280_Read(a_cfgPtr, BME280_CTRL_MEAS_REGISTER,
			&ctrlMeasRegPtr->config, 1);
}

static BME280_Status BME280_getCtrlHumRegister(BME280_Handle *a_cfgPtr,
		BME280_CtrlHumRegisterUnion *ctrlHumRegPtr) {
	if (ctrlHumRegPtr == NULL_PTR || a_cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;

	/* Read ctrl-meas register */
	return BME280_Read(a_cfgPtr, BME280_CTRL_HUM_REGISTER,
			&ctrlHumRegPtr->config, 1);
}

static BME280_Status BME280_getStatusRegister(BME280_Handle *a_cfgPtr,
		BME280_StatusRegisterUnion *statusRegPtr) {
	if (statusRegPtr == NULL_PTR || a_cfgPtr == NULL_PTR)
		return BME280_NULL_ERROR;

	/* Read ctrl-meas register */
	return BME280_Read(a_cfgPtr, BME280_STATUS_REGISTER, &statusRegPtr->config,
			1);
}

static BME280_Status BME280_getUncompensatedReadings(BME280_Handle *a_cfgPtr,
		BME280_PressureReading *a_pressure,
		BME280_TemperatureReading *a_temperature,
		BME280_HumidityReading *a_humidity) {
	/* If all reading pointers are null, function returns null error and no measurement reading is done*/
	boolean readingPtrsBool = (a_pressure != NULL_PTR)
			|| (a_temperature != NULL_PTR) || (a_humidity != NULL_PTR);
	/* Operation result status flag */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
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
		result = BME280_Read(a_cfgPtr, BME280_START_READINGS_ADDRESS,
				readings.arr,
				BME280_READINGS_BYTES_LENGTH);

		/* Check for successful read */
		if (result != BME280_OK)
			break;

		if (a_pressure != NULL_PTR) {
			/* Zero value so padding is not garbage */
			a_pressure->pressure = 0;
			a_pressure->Data.xlsb = readings.Bytes.press_xlsb;
			a_pressure->Data.lsb = readings.Bytes.press_lsb;
			a_pressure->Data.msb = readings.Bytes.press_msb;
		}
		if (a_temperature != NULL_PTR) {
			/* Zero value so padding is not garbage */
			a_temperature->temperature = 0;
			a_temperature->Data.xlsb = readings.Bytes.temp_xlsb >> 4;
			a_temperature->Data.lsb = readings.Bytes.temp_lsb;
			a_temperature->Data.msb = readings.Bytes.temp_msb;
		}
		if (a_humidity != NULL_PTR) {
			/* Zero value so padding is not garbage */
			a_humidity->humidity = 0;
			a_humidity->Data.lsb = readings.Bytes.hum_lsb;
			a_humidity->Data.msb = readings.Bytes.hum_msb;
		}
	} while (0);
	return result;
}

static BME280_Status BME280_getCalibData(BME280_Handle *a_cfgPtr) {

	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;

		/* Read calibration parameters for:
		 * dig_T1-dig_T3
		 * dig_P1-dig_P9
		 * dig_H1
		 * */
		result = BME280_Read(a_cfgPtr,
		BME280_TEMP_PRESS_BLOCK_START_ADDRESS,
				(BME280_uint8*) ((*a_cfgPtr)->calib_data_1.arr),
				BME280_TEMP_PRESS_CALIB_BLOCK_SIZE);
		/* Check for successful read */
		if (result != BME280_OK)
			break;

		/* Read calibration parameters for:
		 * dig_H2-dig_H6
		 * */
		result = BME280_Read(a_cfgPtr, BME280_HUM_BLOCK_START_ADDRESS,
				(BME280_uint8*) ((*a_cfgPtr)->calib_data_2.arr),
				BME280_HUM_CALIB_BLOCK_SIZE);

		/* Check for successful read */
		if (result != BME280_OK)
			break;

		/* Parse calibration data for humidity */
		(*a_cfgPtr)->calib_data_2.Bytes.dig_H4 =
				((*a_cfgPtr)->calib_data_2.Bytes.dig_H4_lsb & BME280_LSBYTE_MASK)
						| (((BME280_sint16) (*a_cfgPtr)->calib_data_2.Bytes.dig_H4_msb)
								<< 4);
		(*a_cfgPtr)->calib_data_2.Bytes.dig_H5 =
				((*a_cfgPtr)->calib_data_2.Bytes.dig_H5_lsb & BME280_LSBYTE_MASK)
						| (((BME280_sint16) (*a_cfgPtr)->calib_data_2.Bytes.dig_H5_msb)
								<< 4);
	} while (0);
	return result;

}

#if BME280_FLOATING_POINT == BME280_FEATURE_ENABLE
static BME280_float64 BME280_compensateTemperature_floatingPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawTemperature) {
	BME280_float64 var1;
	BME280_float64 var2;
	BME280_float64 temperature;

	var1 = ((BME280_float64) a_rawTemperature) / 16384.0
			- ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_T1)
					/ 1024.0;
	var1 = var1 * ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_T2);
	var2 =
			(((BME280_float64) a_rawTemperature) / 131072.0
					- ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_T1)
							/ 8192.0);
	var2 = (var2 * var2)
			* ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_T3);
	(*a_cfgPtr)->calib_data_2.Bytes.t_fine_float =
			(BME280_sint32) (var1 + var2);
	temperature = (var1 + var2) / 5120.0;

	if (temperature < BME280_MIN_TEMPERATURE_FLOATING_POINT) {
		temperature = BME280_MIN_TEMPERATURE_FLOATING_POINT;
	} else if (temperature > BME280_MAX_TEMPERATURE_FLOATING_POINT) {
		temperature = BME280_MAX_TEMPERATURE_FLOATING_POINT;
	}

	return temperature;
}

static BME280_float64 BME280_compensatePressure_floatingPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawPressure) {
	BME280_float64 var1;
	BME280_float64 var2;
	BME280_float64 var3;
	BME280_float64 pressure;

	var1 = ((BME280_float64) (*a_cfgPtr)->calib_data_2.Bytes.t_fine_float / 2.0)
			- 64000.0;
	var2 = var1 * var1
			* ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P6)
			/ 32768.0;
	var2 = var2
			+ var1 * ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P5)
					* 2.0;
	var2 = (var2 / 4.0)
			+ (((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P4)
					* 65536.0);
	var3 = ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P3) * var1
			* var1 / 524288.0;
	var1 = (var3
			+ ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P2) * var1)
			/ 524288.0;
	var1 = (1.0 + var1 / 32768.0)
			* ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P1);

	/* avoid exception caused by division by zero */
	if (var1 > (0.0)) {
		pressure = 1048576.0 - (BME280_float64) a_rawPressure;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P9)
				* pressure * pressure / 2147483648.0;
		var2 = pressure
				* ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P8)
				/ 32768.0;
		pressure =
				pressure
						+ (var1 + var2
								+ ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_P7))
								/ 16.0;

		if (pressure < BME280_MIN_PRESSURE_FLOATING_POINT) {
			pressure = BME280_MIN_PRESSURE_FLOATING_POINT;
		} else if (pressure > BME280_MAX_PRESSURE_FLOATING_POINT) {
			pressure = BME280_MAX_PRESSURE_FLOATING_POINT;
		}
	} else /* Invalid case */
	{
		pressure = BME280_MIN_PRESSURE_FLOATING_POINT;
	}

	return pressure;
}

static BME280_float64 BME280_compensateHumidity_floatingPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawHumidity) {
	BME280_float64 humidity;
	BME280_float64 var1;
	BME280_float64 var2;
	BME280_float64 var3;
	BME280_float64 var4;
	BME280_float64 var5;
	BME280_float64 var6;

	var1 = ((BME280_float64) (*a_cfgPtr)->calib_data_2.Bytes.t_fine_float)
			- 76800.0;
	var2 = (((BME280_float64) (*a_cfgPtr)->calib_data_2.Bytes.dig_H4) * 64.0
			+ (((BME280_float64) (*a_cfgPtr)->calib_data_2.Bytes.dig_H5)
					/ 16384.0) * var1);
	var3 = a_rawHumidity - var2;
	var4 = ((BME280_float64) (*a_cfgPtr)->calib_data_2.Bytes.dig_H2) / 65536.0;
	var5 = (1.0
			+ (((BME280_float64) (*a_cfgPtr)->calib_data_2.Bytes.dig_H3)
					/ 67108864.0) * var1);
	var6 = 1.0
			+ (((BME280_float64) (*a_cfgPtr)->calib_data_2.Bytes.dig_H6)
					/ 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6
			* (1.0
					- ((BME280_float64) (*a_cfgPtr)->calib_data_1.words.dig_H1)
							* var6 / 524288.0);

	if (humidity > BME280_MAX_HUMIDITY_FLOATING_POINT) {
		humidity = BME280_MAX_HUMIDITY_FLOATING_POINT;
	} else if (humidity < BME280_MIN_HUMIDITY_FLOATING_POINT) {
		humidity = BME280_MIN_HUMIDITY_FLOATING_POINT;
	}

	return humidity;
}

#endif

/* 32 bit compensation for pressure data */
static BME280_sint32 BME280_compensateTemperature_fixedPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawTemperature) {
	BME280_sint32 var1, var2, compensatedTemperature;
	var1 = (BME280_sint32) ((a_rawTemperature / 8)
			- ((BME280_sint32) (*a_cfgPtr)->calib_data_1.words.dig_T1 * 2));

	var1 = (var1 * ((BME280_sint32) (*a_cfgPtr)->calib_data_1.words.dig_T2))
			/ 2048;

	var2 = (BME280_sint32) ((a_rawTemperature / 16)
			- ((BME280_sint32) (*a_cfgPtr)->calib_data_1.words.dig_T1));

	var2 = (((var2 * var2) / 4096)
			* ((BME280_sint32) (*a_cfgPtr)->calib_data_1.words.dig_T3)) / 16384;

	(*a_cfgPtr)->calib_data_2.Bytes.t_fine = var1 + var2;

	compensatedTemperature = ((*a_cfgPtr)->calib_data_2.Bytes.t_fine * 5 + 128)
			/ 256;

	if (compensatedTemperature < BME280_MIN_TEMPERATURE_FIXED_POINT) {
		compensatedTemperature = BME280_MIN_TEMPERATURE_FIXED_POINT;
	} else if (compensatedTemperature > BME280_MAX_TEMPERATURE_FIXED_POINT) {
		compensatedTemperature = BME280_MAX_TEMPERATURE_FIXED_POINT;
	}

	return compensatedTemperature;
}

static BME280_uint32 BME280_compensatePressure_fixedPoint(
		BME280_Handle *a_configPtr, BME280_sint32 a_rawPressure) {

	BME280_sint32 var1 = 0, var2 = 0, var3 = 0, var4 = 0;
	BME280_uint32 var5 = 0;
	BME280_uint32 a_PressureAfterCalibrating = 0;

	/* Sensor Calibration calculations */

	var1 = (((BME280_sint32) (*a_configPtr)->calib_data_2.Bytes.t_fine) / 2)
			- (BME280_sint32) 64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048)
			* ((BME280_sint32) (*a_configPtr)->calib_data_1.words.dig_P6);
	var2 = var2
			+ ((var1 * ((BME280_sint32) ((*a_configPtr)->calib_data_1.words.dig_P5)))
					* 2);
	var2 = (var2 / 4)
			+ (((BME280_sint32) ((*a_configPtr)->calib_data_1.words.dig_P4)) * 65536);
	var3 = (((*a_configPtr)->calib_data_1.words.dig_P3)
			* (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((BME280_sint32) ((*a_configPtr)->calib_data_1.words.dig_P2)) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1))
			* ((BME280_sint32) ((*a_configPtr)->calib_data_1.words.dig_P1))) / 32768;

	/* avoid exception caused by division by zero */
	if (var1) {
		var5 = (BME280_uint32) ((BME280_uint32) 1048576) - (a_rawPressure);

		a_PressureAfterCalibrating = ((BME280_uint32) (var5
				- (BME280_uint32) (var2 / 4096))) * 3125;

		if (a_PressureAfterCalibrating < 0x80000000) {
			a_PressureAfterCalibrating = ((a_PressureAfterCalibrating) << 1)
					/ ((BME280_uint32) var1);
		} else {
			(a_PressureAfterCalibrating) = ((a_PressureAfterCalibrating)
					/ (BME280_uint32) var1) * 2;
		}

		var1 = (((BME280_sint32) ((*a_configPtr)->calib_data_1.words.dig_P9))
				* ((BME280_sint32) ((((a_PressureAfterCalibrating) / 8)
						* ((a_PressureAfterCalibrating) / 8)) / 8192))) / 4096;
		var2 = (((BME280_sint32) ((a_PressureAfterCalibrating) / 4))
				* ((BME280_sint32) ((*a_configPtr)->calib_data_1.words.dig_P8)))
				/ 8192;
		(a_PressureAfterCalibrating) =
				(BME280_uint32) ((BME280_sint32) (a_PressureAfterCalibrating)
						+ ((var1 + var2
								+ ((*a_configPtr)->calib_data_1.words.dig_P7))
								/ 16));

		/* Check if we exceeded the temperature ranges */

		if ((a_PressureAfterCalibrating) < BME280_MIN_PRESSURE_FIXED_POINT) {
			(a_PressureAfterCalibrating) = BME280_MIN_PRESSURE_FIXED_POINT;

		} else if ((a_PressureAfterCalibrating)
				> BME280_MAX_PRESSURE_FIXED_POINT) {
			(a_PressureAfterCalibrating) = BME280_MAX_PRESSURE_FIXED_POINT;

		}
	} else /* Check if we exceeded the temperature ranges */

	{
		(a_PressureAfterCalibrating) = BME280_MIN_PRESSURE_FIXED_POINT;

	}

	return a_PressureAfterCalibrating;
}

static BME280_uint32 BME280_compensateHumidity_fixedPoint(
		BME280_Handle *a_cfgPtr, BME280_sint32 a_rawHumidity) {
	BME280_sint32 var1;
	BME280_sint32 var2;
	BME280_sint32 var3;
	BME280_sint32 var4;
	BME280_sint32 var5;
	BME280_uint32 compensatedHumidity;

	var1 = (*a_cfgPtr)->calib_data_2.Bytes.t_fine - ((BME280_sint32) 76800);
	var2 = (BME280_sint32) (a_rawHumidity * 16384);
	var3 =
			(BME280_sint32) (((BME280_sint32) (*a_cfgPtr)->calib_data_2.Bytes.dig_H4)
					* 1048576);
	var4 = ((BME280_sint32) (*a_cfgPtr)->calib_data_2.Bytes.dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (BME280_sint32) 16384) / 32768;
	var2 = (var1 * ((BME280_sint32) (*a_cfgPtr)->calib_data_2.Bytes.dig_H6))
			/ 1024;
	var3 = (var1 * ((BME280_sint32) (*a_cfgPtr)->calib_data_2.Bytes.dig_H3))
			/ 2048;
	var4 = ((var2 * (var3 + (BME280_sint32) 32768)) / 1024)
			+ (BME280_sint32) 2097152;
	var2 = ((var4 * ((BME280_sint32) (*a_cfgPtr)->calib_data_2.Bytes.dig_H2))
			+ 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3
			- ((var4 * ((BME280_sint32) (*a_cfgPtr)->calib_data_1.words.dig_H1))
					/ 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	compensatedHumidity = (BME280_uint32) (var5 / 4096);

	if (compensatedHumidity > BME280_MAX_HUMIDITY_FIXED_POINT) {
		compensatedHumidity = BME280_MAX_HUMIDITY_FIXED_POINT;
	}

	return compensatedHumidity;
}

static void BME280_DeInitHandle(BME280_Handle *a_cfgPtr) {

	/* De-init configuration struct*/

	(*a_cfgPtr)->ID = 0x00;
	(*a_cfgPtr)->occupied = FALSE;
	(*a_cfgPtr)->Intf = BME280_Interface_Not_Specified;
	(*a_cfgPtr)->GPIOCallback_SetNSS = NULL_PTR;
	(*a_cfgPtr)->GPIOCallback_resetNSS = NULL_PTR;
	/* Zero calibration data */
	for (BME280_uint8 c = 0; c < BME280_TEMP_PRESS_CALIB_BLOCK_SIZE; c++)
		(*a_cfgPtr)->calib_data_1.arr[c] = 0;

	for (BME280_uint8 c = 0; c < BME280_HUM_CALIB_BLOCK_SIZE; c++)
		(*a_cfgPtr)->calib_data_2.arr[c] = 0;
}

/*******************************************************************************
 *                          Public function definitions                        *
 *******************************************************************************/

BME280_Status BME280_getInstance(BME280_Handle *a_cfgPtr) {

	for (BME280_uint8 c = 0; c < BME280_MAX_SENSOR_POOL_SIZE; ++c) {
		/* Search for an empty instance in the pool */
		if ((*a_cfgPtr)
				!= &BME280_sensorPool[c]&& BME280_sensorPool[c].occupied == FALSE) {
			(*a_cfgPtr) = &BME280_sensorPool[c];
			/* Set default values for instance parameters */
			BME280_DeInitHandle(a_cfgPtr);
			/* Set occupied flag to true as it was de-initialized or unknown*/
			BME280_sensorPool[c].occupied = TRUE;
			return BME280_FOUND_EMPTY_INSTANCE;
		}
		/* Already an existing and occupied instance, cfgPtr already points to a used instance*/
		else if ((*a_cfgPtr)
				== &BME280_sensorPool[c]&& BME280_sensorPool[c].occupied == TRUE) {
			return BME280_IS_INSTANCE;
		}
	}

	if ((*a_cfgPtr) == NULL_PTR)
		return BME280_POOL_FULL;

	return BME280_OK;
}

BME280_Status BME280_setInterfaceType(BME280_Handle *a_cfgPtr,
		BME280_InterfaceType a_intf) {

	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);

	do {

		if (result != BME280_IS_INSTANCE)
			break;

		/* Flag used if interface is I2C*/
		BME280_uint8 flag = FALSE;

		/* Set selected interface */
		switch (a_intf) {
		case BME280_Interface_I2C:
			/* Must check if more than the maxmimum I2C instances are occupied or not*/
			for (BME280_uint8 c = 0; c < BME280_MAX_SENSOR_POOL_SIZE; ++c) {
				if (BME280_sensorPool[c].Intf
						== BME280_Interface_I2C&& BME280_sensorPool[c].I2C_SlaveAddr == BME280_I2C_Addr && BME280_sensorPool[c].occupied==TRUE) {
					flag = TRUE;
					break;
				}
			}
			if (flag == TRUE) {
				result = BME280_POOL_FULL;
				break;
			} else {
				(*a_cfgPtr)->Intf = BME280_Interface_I2C;
				(*a_cfgPtr)->I2C_SlaveAddr = BME280_I2C_Addr;
				result = BME280_OK;
			}
			break;
		case BME280_Interface_SPI:
			(*a_cfgPtr)->Intf = BME280_Interface_SPI;
			result = BME280_OK;
			break;
		default:
			(*a_cfgPtr)->Intf = BME280_Interface_Not_Specified;
			result = BME280_NO_INTERFACE_SPECIFIED;
			break;
		}

	} while (0);

	return result;
}

BME280_Status BME280_getChipID(BME280_Handle *a_cfgPtr, BME280_uint8 *a_chipID) {
	/* Null check */
	if (a_chipID == NULL_PTR)
		return BME280_NULL_ERROR;
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);

	do {

		if (result != BME280_IS_INSTANCE)
			break;
		result = BME280_Read(a_cfgPtr, BME280_ID_REGISTER, a_chipID, 1);
	} while (0);

	return result;
}

BME280_Status BME280_init(BME280_Handle *a_cfgPtr) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		/* Check if passed handle is an instance from the pool*/
		if (result != BME280_IS_INSTANCE)
			break;
		/* Discover sensor try count */
		BME280_uint8 tryCount = BME280_MAX_DISCOVERY_COUNT;
		/* Buffer to receive data in */
		BME280_uint8 chipID = 0;

		do {

			/* Attempt to get ID from ID register */
			result = BME280_getChipID(a_cfgPtr, &chipID);
			/* If ID is not returned successfully, try again */
			if (chipID != BME280_CHIP_ID || result != BME280_OK) {
				++tryCount;
				chipID = 0;
				continue;
			}
			/* Sensor communication is OK */
			else {

				/* Attempt to reset the sensor by writing the reset word into the reset register*/
				result = BME280_softReset(a_cfgPtr);

				if (result != BME280_OK)
					return BME280_COMM_ERROR;
				/* Get calibration data from the sensor */
				result = BME280_getCalibData(a_cfgPtr);

				/* Break out of loop as communication was successful*/
				break;
			}
		} while (tryCount > BME280_MAX_DISCOVERY_COUNT);
	} while (0);
	return result;
}

BME280_Status BME280_softReset(BME280_Handle *a_cfgPtr) {

	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {

		result = BME280_isInstance(a_cfgPtr);

		if (result != BME280_IS_INSTANCE)
			break;

		result = BME280_NOT_YET_OBTAINED;

		/* Reset register address*/
		BME280_uint8 a_regAddr = BME280_RESET_REGISTER;

		/* Reset byte which resets sensor when written into the reset register*/
		BME280_uint8 reset_byte = BME280_RESET_WORD;

		/* Send reset word */
		result = BME280_Write(a_cfgPtr, a_regAddr, reset_byte);

		/* Status register which contains IM_UPDATE bit*/
		a_regAddr = BME280_STATUS_REGISTER;
		BME280_uint8 reg_data = 0xFF;
		result = BME280_NOT_YET_OBTAINED;

		/*Wait on sensor to boot and read the IM_UPDATE bit to reset indicating successful boot*/
		/* Wait for NVM data to be copied */
		do {
			/* Now we wait for start up time for the sensor to boot */
			BME280_delayMs(BME280_START_UP_TIME_MS);
			result = BME280_Read(a_cfgPtr, a_regAddr, &reg_data, 1);
			/* Loop on IM_UDATE flag to be ready*/
		} while (result != BME280_OK
				&& BME280_CONFIG_IM_UPDATE_MASK(reg_data)
						!= BME280_IM_UPDATE_READY);
	} while (0);
	return result;

}

BME280_uint16 BME280_calculateMeasurementDelayMs(BME280_Settings *a_settings) {
	/*
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

	BME280_uint8 osrs_t, osrs_p, osrs_h;
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
		return (BME280_uint16) (((float32) 1000 * filter_map[a_settings->filter])
				/ ODR);
	}
	/* Filter disabled*/
	else
		return (BME280_uint16) tMeasure_ms;
}

BME280_Status BME280_getTemperature_fixedPoint(BME280_Handle *a_cfgPtr,
		BME280_sint32 *a_temperature) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (a_temperature == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		BME280_uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(a_cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(
				&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(a_cfgPtr, &rawPress, &rawTemp,
				&rawHum);
		if (result != BME280_OK)
			break;

		/* Compensate readings */
		*a_temperature = BME280_compensateTemperature_fixedPoint(a_cfgPtr,
				rawTemp.temperature);

	} while (0);
	return result;
}

BME280_Status BME280_getPressure_fixedPoint(BME280_Handle *a_cfgPtr,
		BME280_uint32 *a_pressure) {

	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (a_pressure == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		BME280_uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(a_cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(
				&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(a_cfgPtr, &rawPress, &rawTemp,
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
		BME280_compensateTemperature_fixedPoint(a_cfgPtr, rawTemp.temperature);
		*a_pressure = BME280_compensatePressure_fixedPoint(a_cfgPtr,
				rawPress.pressure);

	} while (0);
	return result;

}

BME280_Status BME280_getHumidity_fixedPoint(BME280_Handle *a_cfgPtr,
		BME280_uint32 *a_humidity) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (a_humidity == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		BME280_uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(a_cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(
				&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(a_cfgPtr, &rawPress, &rawTemp,
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
		BME280_compensateTemperature_fixedPoint(a_cfgPtr, rawTemp.temperature);
		*a_humidity = BME280_compensateHumidity_fixedPoint(a_cfgPtr,
				rawHum.humidity);

	} while (0);
	return result;
}

BME280_Status BME280_getTemperature_floatingPoint(BME280_Handle *a_cfgPtr,
		BME280_float64 *a_temperature) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (a_temperature == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		BME280_uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(a_cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(
				&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(a_cfgPtr, &rawPress, &rawTemp,
				&rawHum);
		if (result != BME280_OK)
			break;

		/* Compensate readings */
		*a_temperature = BME280_compensateTemperature_floatingPoint(a_cfgPtr,
				rawTemp.temperature);

	} while (0);
	return result;
}

BME280_Status BME280_getPressure_floatingPoint(BME280_Handle *a_cfgPtr,
		BME280_float64 *a_pressure) {

	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (a_pressure == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		BME280_uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(a_cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(
				&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(a_cfgPtr, &rawPress, &rawTemp,
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
		BME280_compensateTemperature_floatingPoint(a_cfgPtr,
				rawTemp.temperature);
		*a_pressure = BME280_compensatePressure_floatingPoint(a_cfgPtr,
				rawPress.pressure);

	} while (0);
	return result;

}

BME280_Status BME280_getHumidity_floatingPoint(BME280_Handle *a_cfgPtr,
		BME280_float64 *a_humidity) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (a_humidity == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}
		BME280_Settings current_settings = { 0 };
		BME280_TemperatureReading rawTemp = { 0 };
		BME280_HumidityReading rawHum = { 0 };
		BME280_PressureReading rawPress = { 0 };

		BME280_uint32 measurementDelay = 0;

		/* Read current sensor settings from the sensor */
		result = BME280_getSensorSettings(a_cfgPtr, &current_settings);
		if (result != BME280_OK)
			break;

		/* Calculate needed delay per current sensor settings */
		measurementDelay = BME280_calculateMeasurementDelayMs(
				&current_settings);

		/* Delay by calculated delay to allow the sensor to convert */
		BME280_delayMs(measurementDelay);

		/* Get uncompensated readings from the sensor*/
		result = BME280_getUncompensatedReadings(a_cfgPtr, &rawPress, &rawTemp,
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
		BME280_compensateTemperature_floatingPoint(a_cfgPtr,
				rawTemp.temperature);
		*a_humidity = BME280_compensateHumidity_floatingPoint(a_cfgPtr,
				rawHum.humidity);

	} while (0);
	return result;
}

/*******************************************************************************
 *                          Setter functions for settings  					   *
 *******************************************************************************/

BME280_Status BME280_setPressureOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting a_pressureOversampling) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {

		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current Ctrl-meas register value */
		BME280_CtrlMeasRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_CtrlMeasRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getCtrlMeasRegister(a_cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set pressure over-sampling setting then write it to the sensor*/
		reg.Bits.osrs_p = a_pressureOversampling;
		result = BME280_Write(a_cfgPtr, BME280_CTRL_MEAS_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getCtrlMeasRegister(a_cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;

	} while (0);
	return result;

}

BME280_Status BME280_setTemperatureOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting a_temperatureOversampling) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {

		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current Ctrl-meas register value */
		BME280_CtrlMeasRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_CtrlMeasRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getCtrlMeasRegister(a_cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set temperature over-sampling setting then write it to the sensor*/
		reg.Bits.osrs_t = a_temperatureOversampling;
		result = BME280_Write(a_cfgPtr, BME280_CTRL_MEAS_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getCtrlMeasRegister(a_cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;
	} while (0);
	return result;

}

BME280_Status BME280_setHumidityOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting a_humidityOversampling) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current Ctrl-hum register value */
		BME280_CtrlHumRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_CtrlHumRegisterUnion verity_reg = { 0 };
		BME280_CtrlMeasRegisterUnion reg_meas = { 0 };

		/* Read register from sensor */
		result = BME280_getCtrlHumRegister(a_cfgPtr, &reg);
		if (result != BME280_OK)
			return result;
		/* Set humidity over-sampling setting then write it to the sensor*/
		reg.Bits.osrs_h = a_humidityOversampling;

		/* Send control humidity byte, must be modified AFTER modifying
		 * ctrl_meas register as per data-sheet:
		 *
		 * "Changes to this register only become effective after
		 *  a write operation to "ctrl_meas"."
		 * */
		result = BME280_getCtrlMeasRegister(a_cfgPtr, &reg_meas);
		result = BME280_setMode(a_cfgPtr, reg_meas.Bits.mode);

		result = BME280_Write(a_cfgPtr, BME280_CTRL_HUM_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getCtrlHumRegister(a_cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;
	} while (0);
	return result;
}

BME280_Status BME280_setStandbyTime(BME280_Handle *a_cfgPtr,
		BME280_StandbyTime a_standbyTime) {
	/* Interface return value */
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {

		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current config register value */
		BME280_ConfigRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_ConfigRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getConfigRegister(a_cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set standby time setting then write it to the sensor*/
		reg.Bits.t_sb = a_standbyTime;
		result = BME280_Write(a_cfgPtr, BME280_CONFIG_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getConfigRegister(a_cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;
	} while (0);
	return result;

}

BME280_Status BME280_setMode(BME280_Handle *a_cfgPtr, BME280_ModeType a_mode) {

	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current Ctrl-meas register value */
		BME280_CtrlMeasRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_CtrlMeasRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getCtrlMeasRegister(a_cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set mode setting then write it to the sensor*/
		reg.Bits.mode = a_mode;
		result = BME280_Write(a_cfgPtr, BME280_CTRL_MEAS_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getCtrlMeasRegister(a_cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;

	} while (0);

	return result;

}

BME280_Status BME280_setFilterCoefficient(BME280_Handle *a_cfgPtr,
		BME280_FilterCoeff a_filterCoeff) {
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		/* Register in which to receive current config register value */
		BME280_ConfigRegisterUnion reg = { 0 };

		/* Variable in which we will re-read the data to verify it changed */
		BME280_ConfigRegisterUnion verity_reg = { 0 };

		/* Read register from sensor */
		result = BME280_getConfigRegister(a_cfgPtr, &reg);
		if (result != BME280_OK)
			return result;

		/* Set filter coefficient setting then write it to the sensor*/
		reg.Bits.filter_coeff = a_filterCoeff;
		result = BME280_Write(a_cfgPtr, BME280_CONFIG_REGISTER, reg.config);

		if (result != BME280_OK)
			break;

		/* Read register from sensor into verity reg */
		result = BME280_getConfigRegister(a_cfgPtr, &verity_reg);

		if (result != BME280_OK)
			break;

		/* Check if values changed correctly */
		if (verity_reg.config != reg.config)
			result = BME280_SETTING_FAILED;
	} while (0);

	return result;
}

/*******************************************************************************
 *                          Getter functions for settings                      *
 *******************************************************************************/

BME280_Status BME280_getUpdateStatus(BME280_Handle *a_cfgPtr,
		BME280_UpdateStatus *a_updateFlag) {
	BME280_Status result = BME280_isInstance(a_cfgPtr);

	do {
		if (result != BME280_IS_INSTANCE) {
			break;
		}
		if (a_updateFlag == NULL_PTR) {
			result = BME280_NULL_ERROR;
		}

		BME280_StatusRegisterUnion statusReg = { 0 };
		/* Get status register */
		result = BME280_getStatusRegister(a_cfgPtr, &statusReg);
		if (result != BME280_OK) {
			break;
		}
		if (statusReg.Bits.im_update == BME280_IM_UPDATE_READY) {
			*a_updateFlag = BME280_Update_Finished;
		} else {

			*a_updateFlag = BME280_Update_Copying;
		}
		break;
	} while (0);

	return result;
}

BME280_Status BME280_getMeasuringStatus(BME280_Handle *a_cfgPtr,
		BME280_MeasuringStatus *a_measureFlag) {
	BME280_Status result = BME280_isInstance(a_cfgPtr);

	do {
		if (result != BME280_IS_INSTANCE) {
			break;
		}
		if (a_measureFlag == NULL_PTR) {
			result = BME280_NULL_ERROR;
		}

		BME280_StatusRegisterUnion statusReg = { 0 };
		/* Get status register */
		result = BME280_getStatusRegister(a_cfgPtr, &statusReg);
		if (result != BME280_OK) {
			break;
		}
		if (statusReg.Bits.measuring == BME280_MEASURING_DONE) {
			*a_measureFlag = BME280_Measuring_Finished;
		} else {

			*a_measureFlag = BME280_Measuring_Running;
		}
		break;
	} while (0);
	return result;
}

BME280_Status BME280_getMode(BME280_Handle *a_cfgPtr, BME280_ModeType *a_mode) {
	if (*a_cfgPtr == NULL_PTR || a_mode == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_CtrlMeasRegisterUnion ctrlMeasReg;
	BME280_Status result = BME280_getCtrlMeasRegister(a_cfgPtr, &ctrlMeasReg);
	*a_mode = (BME280_ModeType) ctrlMeasReg.Bits.mode;
	return result;
}

BME280_Status BME280_getTemperatureOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting *a_oversampling) {
	if (*a_cfgPtr == NULL_PTR || a_oversampling == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_CtrlMeasRegisterUnion ctrlMeasReg;
	BME280_Status result = BME280_getCtrlMeasRegister(a_cfgPtr, &ctrlMeasReg);
	*a_oversampling = (BME280_Oversampling_setting) ctrlMeasReg.Bits.osrs_t;
	return result;
}

BME280_Status BME280_getPressureOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting *a_oversampling) {
	if (*a_cfgPtr == NULL_PTR || a_oversampling == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_CtrlMeasRegisterUnion ctrlMeasReg = { 0 };
	BME280_Status result = BME280_getCtrlMeasRegister(a_cfgPtr, &ctrlMeasReg);
	*a_oversampling = (BME280_Oversampling_setting) ctrlMeasReg.Bits.osrs_p;
	return result;
}

BME280_Status BME280_getHumidityOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting *a_oversampling) {
	if (*a_cfgPtr == NULL_PTR || a_oversampling == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_CtrlHumRegisterUnion ctrlHumReg = { 0 };
	BME280_Status result = BME280_getCtrlHumRegister(a_cfgPtr, &ctrlHumReg);
	*a_oversampling = (BME280_Oversampling_setting) ctrlHumReg.Bits.osrs_h;
	return result;
}

BME280_Status BME280_getFilterCoefficient(BME280_Handle *a_cfgPtr,
		BME280_FilterCoeff *a_filterCoeff) {
	if (*a_cfgPtr == NULL_PTR || a_filterCoeff == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_ConfigRegisterUnion configReg = { 0 };
	BME280_Status result = BME280_getConfigRegister(a_cfgPtr, &configReg);
	*a_filterCoeff = (BME280_FilterCoeff) configReg.Bits.filter_coeff;
	return result;
}

BME280_Status BME280_getStandbyTime(BME280_Handle *a_cfgPtr,
		BME280_StandbyTime *a_standbyTime) {
	if (*a_cfgPtr == NULL_PTR || a_standbyTime == NULL_PTR)
		return BME280_NULL_ERROR;
	BME280_ConfigRegisterUnion configReg = { 0 };
	BME280_Status result = BME280_getConfigRegister(a_cfgPtr, &configReg);
	*a_standbyTime = (BME280_StandbyTime) configReg.Bits.t_sb;
	return result;
}

BME280_Status BME280_getSensorSettings(BME280_Handle *a_cfgPtr,
		BME280_Settings *a_settings) {

	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		if (result != BME280_IS_INSTANCE)
			break;
		if (a_settings == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}

		result = BME280_getHumidityOversampling(a_cfgPtr, &a_settings->osrs_h);
		if (result != BME280_OK)
			break;
		result = BME280_getTemperatureOversampling(a_cfgPtr,
				&a_settings->osrs_t);
		if (result != BME280_OK)
			break;
		result = BME280_getPressureOversampling(a_cfgPtr, &a_settings->osrs_p);
		if (result != BME280_OK)
			break;
		result = BME280_getFilterCoefficient(a_cfgPtr, &a_settings->filter);
		if (result != BME280_OK)
			break;
		result = BME280_getStandbyTime(a_cfgPtr, &a_settings->t_stby);
		if (result != BME280_OK)
			break;
		result = BME280_getMode(a_cfgPtr, &a_settings->Mode);
		if (result != BME280_OK)
			break;

	} while (0);
	return result;

}

BME280_Status BME280_DeInit(BME280_Handle *a_cfgPtr) {
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		/* If it's not an existing instance, break*/
		if (result != BME280_IS_INSTANCE)
			break;

		/* Put device to sleep */
		result = BME280_setMode(a_cfgPtr, BME280_Mode_Sleep);
		if (result != BME280_OK)
			break;
		/* Soft reset device to restore default values and configuration */
		result = BME280_softReset(a_cfgPtr);
		if (result != BME280_OK)
			break;
		/* De-init configuration struct*/
		BME280_DeInitHandle(a_cfgPtr);

		/* Declare this instance as not occupied*/
		(*a_cfgPtr)->occupied = FALSE;

		/* Let handle point to null*/
		(*a_cfgPtr) = NULL_PTR;

		/* De-init complete */
	} while (0);
	return result;
}

/*******************************************************************************
 *                          Callback setters		                           *
 *******************************************************************************/

BME280_Status BME280_setAssertNSSCallback(BME280_Handle *a_cfgPtr,
		void (*a_callback)(void)) {
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		/* If it's not an existing instance, break*/
		if (result != BME280_IS_INSTANCE)
			break;
		/* If the callback is null, break*/
		if (a_callback == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}

		(*a_cfgPtr)->GPIOCallback_SetNSS = a_callback;
	} while (0);
	return result;
}

BME280_Status BME280_setReleaseNSSCallback(BME280_Handle *a_cfgPtr,
		void (*a_callback)(void)) {
	BME280_Status result = BME280_isInstance(a_cfgPtr);
	do {
		/* If it's not an existing instance, break*/
		if (result != BME280_IS_INSTANCE)
			break;
		/* If the callback is null, break*/
		if (a_callback == NULL_PTR) {
			result = BME280_NULL_ERROR;
			break;
		}

		(*a_cfgPtr)->GPIOCallback_resetNSS = a_callback;
	} while (0);
	return result;
}

/*******************************************************************************
 *                          Weak functions definitions                         *
 *******************************************************************************/

__attribute__((weak)) BME280_Comm_Status BME280_SPI_TransmitReceive(
		BME280_uint8 *txData, BME280_uint8 *rxData, BME280_uint16 size,
		BME280_uint32 timeout) {
	/* To be implemented by the user */
	return BME280_NOT_IMPLEMENTED;
}

__attribute__((weak)) BME280_Status BME280_delayMs(BME280_uint32 a_milliseconds) {
	/* To be implemented by the user */
	return BME280_NOT_IMPLEMENTED;
}

__attribute__((weak)) BME280_Status BME280_I2C_Master_Transmit(
		BME280_uint8 sensorAddr, BME280_uint8 *txData, BME280_uint16 size,
		BME280_uint32 timeout) {
	/* To be implemented by the user */
	return BME280_NOT_IMPLEMENTED;
}

__attribute__((weak)) BME280_Status BME280_I2C_Master_Receive(
		BME280_uint8 sensorAddr, BME280_uint8 *rxData, BME280_uint16 size,
		BME280_uint32 timeout) {
	/* To be implemented by the user */
	return BME280_NOT_IMPLEMENTED;
}

