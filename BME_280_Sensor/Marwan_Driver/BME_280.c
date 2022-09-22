/******************************************************************************
 *
 * Module: BME_280
 *
 * File Name: BME_280.c
 *
 * Description: Source file for BME_280 Driver
 *
 * Author: Marwan Abdelhakim Elmasry
 ******************************************************************************/

/*******************************************************************************
 *                              Includes Needed                                *
 *******************************************************************************/ 
/* All Registers need for BME_280 Driver */
#include "BME_280_Registers.h"

/* Including HAL layer layers APIs */
#include "stm32wbxx_hal.h"

/* Needed Types*/
#include "BME_280_Private_Types.h"

#include "BME_280.h"

/*******************************************************************************
 *                      Private Global Variables Definitions                   *
 *******************************************************************************/
STATIC struct BME_280_Configurations g_bme280_instances[MAX_INSTANCE_OF_BME_280_SENSOR] = {NOT_OCCUIPIED};

/*******************************************************************************
 *                          Private Function Prototype	                       *
 *******************************************************************************/
/******************************************************************************
 * @fn BME_280_Status BME_280_SPI_ReadWrapper(uint8, uint8*, uint16, BME_280_Config*)
 * @brief This function will use SPI API to read data from the sensor
 *
 * @param a_TxAddress: Pointer to the address we want to read
 * @param a_RxBuffer: Buffer to store the data read
 * @param a_Size: size of data buffer
 * @param configPtr: pointer to configuration structure
 * @return the status of Read procedure
 *******************************************************************************/
STATIC BME_280_Status BME_280_SPI_ReadWrapper
(
		uint8 a_TxAddress,
		uint8 *a_RxBuffer,
		uint16 a_Size,
		BME_280_Config* a_configPtr
);

/******************************************************************************
 * @fn BME_280_Status BME_280_SPI_WriteWrapper(uint8, uint8*, BME_280_Config*)
 * @brief This function will use SPI API to write data to the sensor
 *
 * @param a_SPI_handlePtr:
 * @param a_TxAddress: Pointer to the address we want to write
 * @param a_Buffer: Buffer of data to be written
 * @param configPtr:  pointer to configuration structure
 * @return The status of write procedure
 *******************************************************************************/
STATIC BME_280_Status BME_280_SPI_WriteWrapper
(
		uint8 a_TxAddress,
		uint8 *a_Buffer,
		BME_280_Config* a_configPtr
);


/******************************************************************************
 * @fn BME_280_Status BME_280_read(uint8, uint8*, uint16, BME_280_Config*)
 * @brief This is a a function that read from sensor either you use
 *		  SPI or I2C interface
 *
 * @param a_Address: address of data to read
 * @param a_Buffer: buffer to data read
 * @param a_size: the size of the buffer
 * @param a_configPtr: pointer to configuration structure
 * @return the status of this API
 *******************************************************************************/
STATIC BME_280_Status BME_280_read
(
		uint8 a_Address,
		uint8 *a_Buffer,
		uint16 a_size,
		BME_280_Config* a_configPtr
);


/******************************************************************************
 * @fn BME_280_Status BME_280_write(uint8, uint8*, BME_280_Config*)
 * @brief This is a a function that write to sensor either you use
 *               SPI or I2C interface
 *
 * @param a_Address: address of data to write
 * @param a_Buffer: buffer to data write
 * @param a_configPtr: pointer to configuration structure
 * @return the status of this API
 *******************************************************************************/
STATIC BME_280_Status BME_280_write
(
		uint8 a_Address,
		uint8 *a_Buffer,
		BME_280_Config* a_configPtr
);

/******************************************************************************
 * @fn BME_280_Status BME_280_getUncompansatedData(BME_280_Config*, BME280_UncompensatedReadings*)
 * @brief  This function will get the current Uncompensated data from the sensor and
 * 		   return it
 *
 * @param a_configPtr
 * @param a_uncompansatedData: the API will put the data in this parameter.
 * @return the status of this API
 *******************************************************************************/
STATIC BME_280_Status BME_280_getUncompensatedData
(
		BME_280_Config* a_configPtr,
		BME280_UncompensatedReadings* a_uncompansatedData
);


/******************************************************************************
 * @fn  BME_280_getCalibratedData(BME_280_Calib1*, BME_280_Calib2*, BME_280_Config*)
 * @brief this function will get save the calibrated data in the configuration
 *        structure.
 *
 * @param a_configPtr: pointer to configuration structure
 * @return the status of this API
 *******************************************************************************/
STATIC BME_280_Status BME_280_getCalibratedData
(
		BME_280_Config* a_configPtr
);




#if (BME280_32BIT_COMPENSATING_MEASURMENTS == _ENABLE_)

/******************************************************************************
 * @fn BME_280_Status BME_280_convertTemperature_fixedPoint(BME_280_Config*, BME280_TemperatureReading*, sint32*)
 * @brief this function will calibrate the temperature in a fixed point
 *
 * @param a_configPtr
 * @param a_tempBeforeCalibrating
 * @param a_tempAfterCalibrating
 * @return the status of this API
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertTemperature_fixedPoint
(
		BME_280_Config* a_configPtr,
		BME280_TemperatureReading* a_tempBeforeCalibrating,
		sint32*	a_tempAfterCalibrating
);


/******************************************************************************
 * @fn BME_280_Status BME_280_convertPressure_fixedPoint(BME_280_Config*, BME280_PressureReading*, uint32*)
 * @brief this function will calibrate the pressure in a fixed point
 *
 * @param a_configPtr
 * @param a_PressureBeforeCalibrating
 * @param a_PressureAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertPressure_fixedPoint
(
		BME_280_Config* a_configPtr,
		BME280_PressureReading* a_PressureBeforeCalibrating,
		uint32*	a_PressureAfterCalibrating
);



/******************************************************************************
 * @fn BME_280_Status BME_280_convertHumidity_fixedPoint(BME_280_Config*, BME280_HumidityReading*, sint32*)
 * @brief this function will calibrate the humidity in a fixed point
 *
 * @param a_configPtr
 * @param a_HumidityBeforeCalibrating
 * @param a_HumidityAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertHumidity_fixedPoint
(
		BME_280_Config* a_configPtr,
		BME280_HumidityReading* a_HumidityBeforeCalibrating,
		uint32*	a_HumidityAfterCalibrating
);

#endif

#if (BME280_FLOAT_COMPENSATING_MEASURMENTS == _ENABLE_)

/******************************************************************************
 * @fn BME_280_Status BME_280_convertTemperature_floatingPoint(BME_280_Config*, BME280_TemperatureReading*, sint32*)
 * @brief this function will calibrate the temperature in a floating point
 *
 * @param a_configPtr
 * @param a_tempBeforeCalibrating
 * @param a_tempAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertTemperature_floatingPoint
(
		BME_280_Config* a_configPtr,
		BME280_TemperatureReading* a_tempBeforeCalibrating,
		float64*	a_tempAfterCalibrating
);


/******************************************************************************
 * @fn BME_280_Status BME_280_convertPressure_floatingPoint(BME_280_Config*, BME280_PressureReading*, sint32*)
 * @brief this function will calibrate the pressure in a floating point
 *
 * @param a_configPtr
 * @param a_PressureBeforeCalibrating
 * @param a_PressureAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertPressure_floatingPoint
(
		BME_280_Config* a_configPtr,
		BME280_PressureReading* a_PressureBeforeCalibrating,
		float64*	a_PressureAfterCalibrating
);


/******************************************************************************
 * @fn BME_280_Status BME_280_convertHumidity_floatingPoint(BME_280_Config*, BME280_HumidityReading*, sint32*)
 * @brief this function will calibrate the humidity in a floating point
 *
 * @param a_configPtr
 * @param a_HumidityBeforeCalibrating
 * @param a_HumidityAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertHumidity_floatingPoint
(
		BME_280_Config* a_configPtr,
		BME280_HumidityReading* a_HumidityBeforeCalibrating,
		float64*	a_HumidityAfterCalibrating
);

#endif

/******************************************************************************
 * @fn BME_280_Status BME_280_doesInstanceExsits(BME_280_Config*)
 * @brief this function will take the configuration pointer and check if it
 *        already take instance of this driver or not and return the result
 *
 * @param a_configPtr
 * @return return if this instance occupy a instance or not
 *******************************************************************************/
STATIC BME_280_Status BME_280_doesInstanceExsits
(
		BME_280_Config* a_configPtr
);


/*******************************************************************************
 *                             Weak Functions			                       *
 *******************************************************************************/
/******************************************************************************
 * @fn BME_280_Status BME_280_SPI_TransmitReceive(uint8*, uint8*, uint16)
 * @brief this is a weak function implemented by the user
 *
 * @param a_TxAddress
 * @param a_RxBuffer
 * @param a_Size
 * @return status of API
 *******************************************************************************/
__weak BME_280_Status BME_280_SPI_TransmitReceive
(
		uint8 *a_TxAddress,
		uint8 *a_RxBuffer,
		uint16 a_Size,
		uint16 timeout
)
{
	while(1);
}

/******************************************************************************
 * @fn void BME_280_Delay(uint32)
 * @brief this function provide delay in ms this function must be implemented
 *        by the user to provide a delay function
 *
 * @param a_delay
 *******************************************************************************/
__weak void BME_280_Delay
(
		uint32 a_delay
)
{
	while(1);
}

/*******************************************************************************
 *                        Private Function Definitions	                       *
 *******************************************************************************/

/******************************************************************************
 * @fn BME_280_Status BME_280_doesInstanceExsits(BME_280_Config*)
 * @brief this function will take the configuration pointer and check if it
 *        already take instance of this driver or not and return the result
 *
 * @param a_configPtr
 * @return return if this instance occupy a instance or not
 *******************************************************************************/
STATIC BME_280_Status BME_280_doesInstanceExsits
(
		BME_280_Config* a_configPtr
)
{
	BME_280_Status result = BME280_OK;
	uint8 i = ZERO;
	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		result = INSTANCE_NOT_TAKEN;

		for( i = ZERO ; i < sizeof(g_bme280_instances)/sizeof(g_bme280_instances[ZERO]) ; i++)
		{
			if( *a_configPtr == &g_bme280_instances[i])
			{
				result = INSTANCE_TAKEN;
				break;
			}
		}

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_SPI_ReadWrapper(SPI_HandleTypeDef*, uint8, uint8*, uint16, BME_280_Config*)
 * @brief This function will use SPI API to read data from the sensor
 *
 * @param a_TxAddress: Pointer to the address we want to read
 * @param a_RxBuffer: Buffer to store the data read
 * @param a_Size: size of data buffer
 * @param configPtr: pointer to configuration structure
 * @return the status of Read procedure
 *******************************************************************************/
STATIC BME_280_Status BME_280_SPI_ReadWrapper
(
		uint8 a_TxAddress,
		uint8 *a_RxBuffer,
		uint16 a_Size,
		BME_280_Config* a_configPtr
)
{
	BME_280_Status result = BME280_OK;
	uint8 Dummy = 10;

	/* Making the MSB 1 so that we make a read command */
	uint8 readCommand = a_TxAddress | SPI_READ_MASK;

	if ( (NULL_PTR != a_configPtr )  )
	{
		/* Setting CSB pin to low to start Read procedure */
		HAL_GPIO_WritePin(CSB_PIN_PORT, CSB_PIN, RESET);

		/* Sending read command and receive dummy data form sensor */
		BME_280_SPI_TransmitReceive( &readCommand, &Dummy, 1, 100);

		/* Receiving the data from sensor */
		BME_280_SPI_TransmitReceive(&readCommand, a_RxBuffer, a_Size, 100);

		/* Termination Read procedure */
		HAL_GPIO_WritePin(CSB_PIN_PORT, CSB_PIN, SET);

		result = BME280_OK;
	}
	else
	{
		result = BME280_NULL_ERROR;
	}

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_SPI_WriteWrapper(SPI_HandleTypeDef*, uint8, uint8*, BME_280_Config*)
 * @brief This function will use SPI API to write data to the sensor
 *
 * @param a_SPI_handlePtr:
 * @param a_TxAddress: Pointer to the address we want to write
 * @param a_Buffer: Buffer of data to be written
 * @param configPtr:  pointer to configuration structure
 * @return The status of write procedure
 *******************************************************************************/
STATIC BME_280_Status BME_280_SPI_WriteWrapper
(
		uint8 a_TxAddress,
		uint8 *a_Buffer,
		BME_280_Config* a_configPtr
)
{
	BME_280_Status result = BME280_OK;

	uint8 Dummy = 10;

	/* Making the MSB 1 so that we make a read command */
	uint8 writeCommand = a_TxAddress & SPI_WRITE_MASK;


	if ( (NULL_PTR != a_configPtr ))
	{
		/* Setting CSB pin to low to start Read procedure */
		HAL_GPIO_WritePin(CSB_PIN_PORT, CSB_PIN, RESET);

		/* Sending write command and receive dummy data form sensor */
		BME_280_SPI_TransmitReceive(&writeCommand, &Dummy, 1, 100);

		/* Writing data to sensor */
		BME_280_SPI_TransmitReceive(a_Buffer, &Dummy, 1, 100);

		/* Termination Read procedure */
		HAL_GPIO_WritePin(CSB_PIN_PORT, CSB_PIN, SET);

		result = BME280_OK;
	}
	else
	{
		result = BME280_NULL_ERROR;
	}

	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_read(uint8, uint8*, uint16, BME_280_Config*)
 * @brief This is a a function that read from sensor either you use
 *		  SPI or I2C interface
 *
 * @param a_Address: address of data to read
 * @param a_Buffer: buffer to data read
 * @param a_size: the size of the buffer
 * @param a_configPtr: pointer to configuration structure
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_read
(
		uint8 a_Address,
		uint8 *a_Buffer,
		uint16 a_size,
		BME_280_Config* a_configPtr
)
{
	BME_280_Status result = BME280_ERROR;
	do
	{
		/* check if we are in SPI mode to interface with SPI */
		if ( (NULL_PTR != a_configPtr) && (BME_280_INTERFACE_SPI == (*a_configPtr)->ProtocolUsed) )
		{
			result = BME_280_SPI_ReadWrapper( a_Address, a_Buffer, a_size, a_configPtr);
			break;
		}
		/* check if we are in I2C mode to interface with I2C */
		else if ( (NULL_PTR != a_configPtr) && (BME_280_INTERFACE_I2C == (*a_configPtr)->ProtocolUsed))
		{
			break; ;// return I2C wrapper when implemented
		}
		else
		{
			result =  BME280_NULL_ERROR;
		}

	}while(FALSE);

	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_write(uint8, uint8*, BME_280_Config*)
 * @brief This is a a function that write to sensor either you use
 *               SPI or I2C interface
 *
 * @param a_Address: address of data to write
 * @param a_Buffer: buffer to data write
 * @param a_configPtr: pointer to configuration structure
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_write
(
		uint8 a_Address,
		uint8 *a_Buffer,
		BME_280_Config* a_configPtr
)
{
	BME_280_Status result = BME280_ERROR;
	do
	{
		/* check if we are in SPI mode to interface with SPI */
		if ( (NULL_PTR != a_configPtr) && (BME_280_INTERFACE_SPI == (*a_configPtr)->ProtocolUsed) )
		{
			result = BME_280_SPI_WriteWrapper(a_Address, a_Buffer, a_configPtr);
			break;
		}
		/* check if we are in I2C mode to interface with I2C */
		else if ( (NULL_PTR == a_configPtr) && (BME_280_INTERFACE_I2C == (*a_configPtr)->ProtocolUsed))
		{
			break ;// return I2C wrapper when implemented
		}
		else
		{
			return BME280_NULL_ERROR;
		}

	}while(FALSE);

	return result;
}


/******************************************************************************
 * @fn  BME_280_getCalibratedData(BME_280_Calib1*, BME_280_Calib2*, BME_280_Config*)
 * @brief this function will get save the calibrated data in the configuration
 *        structure.
 *
 * @param a_configPtr: pointer to configuration structure
 * @return the status of this API
 *******************************************************************************/
STATIC BME_280_Status BME_280_getCalibratedData
(
		BME_280_Config* a_configPtr
)
{
	BME_280_Status result = BME280_OK;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read Temperature and pressure Calibration data */
		result = BME_280_read
				(
						BME280_TEMP_PRESS_CALIB_DATA_ADDR,
						(uint8*)(((*a_configPtr)->calib1).arr),
						BME280_TEMP_PRESS_CALIB_DATA_LEN,
						a_configPtr
				);

		if( BME280_OK != result)
		{
			break;
		}


		/* Read Humidity Calibration data */
		result = BME_280_read
				(
						BME280_HUMIDITY_CALIB_DATA_ADDR,
						((*a_configPtr)->calib2).arr,
						BME280_HUMIDITY_CALIB_DATA_LEN,
						a_configPtr
				);

		/* If the result is OK then procced */
		if( BME280_OK == result)
		{
			/* making dig_h4 ready */
			(*a_configPtr)->calib2.words.dig_H4 = \
					( ( (sint16)((*a_configPtr)->calib2.words.dig_H4_MSB) ) << 4 ) | \
					( ((*a_configPtr)->calib2.words.dig_H4_LSB) & (0x0F) );

			/* making dig_h5 ready */
			(*a_configPtr)->calib2.words.dig_H5 = \
					( ( (sint16)((*a_configPtr)->calib2.words.dig_H5_MSB) ) << 4 ) | \
					( ((*a_configPtr)->calib2.words.dig_H5_LSB) & (0x0F) ) ;
		}

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_getUncompansatedData(BME_280_Config*, BME280_UncompensatedReadings*)
 * @brief  This function will get the current Uncompensated data from the sensor and
 * 		   return it
 *
 * @param a_configPtr
 * @param a_uncompansatedData: the API will put the data in this parameter.
 * @return the status of this API
 *******************************************************************************/
STATIC BME_280_Status BME_280_getUncompensatedData
(
		BME_280_Config* a_configPtr,
		BME280_UncompensatedReadings* a_uncompansatedData
)
{
	BME_280_Status result = BME280_OK;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read uncompensated data from sensor first */
		result = BME_280_read(BME280_DATA_START_ADDR, (a_uncompansatedData->arr), BME280_PRESS_TEMP_HUMI_DATA_LEN, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}


#if (BME280_32BIT_COMPENSATING_MEASURMENTS == _ENABLE_)

/******************************************************************************
 * @fn BME_280_Status BME_280_convertTemperature_fixedPoint(BME_280_Config*, BME280_TemperatureReading*, sint32*)
 * @brief this function will calibrate the temperature in a fixed point
 *
 * @param a_configPtr
 * @param a_tempBeforeCalibrating
 * @param a_tempAfterCalibrating
 * @return the status of this API
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertTemperature_fixedPoint
(
		BME_280_Config* a_configPtr,
		BME280_TemperatureReading* a_tempBeforeCalibrating,
		sint32*	a_tempAfterCalibrating
)
{
	BME_280_Status result = BME280_OK;
	sint32 var1=ZERO, var2=ZERO;
	sint32 temperature_min = -4000;
	sint32 temperature_max = 8500;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Sensor Calibration calculations */
		var1 = (sint32) ((a_tempBeforeCalibrating->temperature / 8) - ((sint32) (*a_configPtr)->calib1.words.dig_T1 * 2));

		var1 = (var1 * ((sint32) (*a_configPtr)->calib1.words.dig_T2)) / 2048;

		var2 = (sint32) ((a_tempBeforeCalibrating->temperature / 16) - ((sint32) (*a_configPtr)->calib1.words.dig_T1));

		var2 = (((var2 * var2) / 4096) * ((sint32) (*a_configPtr)->calib1.words.dig_T3)) / 16384;

		(*a_configPtr)->calib2.words.t_fine = var1 + var2;

		*a_tempAfterCalibrating = (((*a_configPtr)->calib2.words.t_fine) * 5 + 128) / 256;

		/* Check if we exceeded the temperature ranges */
		if (*a_tempAfterCalibrating < temperature_min)
		{
			*a_tempAfterCalibrating = temperature_min;
			result = BME280_EXCEEDED_MIN_RANGE;
			break;
		}
		else if (*a_tempAfterCalibrating > temperature_max)
		{
			*a_tempAfterCalibrating = temperature_max;
			result = BME280_EXCEEDED_MAX_RANGE;
			break;
		}


		/* Success */
		result = BME280_OK;
	}while(FALSE);

	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_convertPressure_fixedPoint(BME_280_Config*, BME280_PressureReading*, uint32*)
 * @brief this function will calibrate the pressure in a fixed point
 *
 * @param a_configPtr
 * @param a_PressureBeforeCalibrating
 * @param a_PressureAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertPressure_fixedPoint
(
		BME_280_Config* a_configPtr,
		BME280_PressureReading* a_PressureBeforeCalibrating,
		uint32*	a_PressureAfterCalibrating
)
{
	BME_280_Status result = BME280_OK;

	sint32 var1=ZERO, var2=ZERO, var3=ZERO, var4=ZERO;
	uint32 var5=ZERO;

	uint32 pressure_min = 30000;
	uint32 pressure_max = 110000;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Sensor Calibration calculations */

		var1 = (((sint32)(*a_configPtr)->calib2.words.t_fine) / 2) - (sint32)64000;
		var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((sint32)(*a_configPtr)->calib1.words.dig_P6);
		var2 = var2 + ((var1 * ((sint32)((*a_configPtr)->calib1.words.dig_P5))) * 2);
		var2 = (var2 / 4) + (((sint32)((*a_configPtr)->calib1.words.dig_P4)) * 65536);
		var3 = (((*a_configPtr)->calib1.words.dig_P3) * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
		var4 = (((sint32)((*a_configPtr)->calib1.words.dig_P2)) * var1) / 2;
		var1 = (var3 + var4) / 262144;
		var1 = (((32768 + var1)) * ((sint32)((*a_configPtr)->calib1.words.dig_P1))) / 32768;

		/* avoid exception caused by division by zero */
		if (var1)
		{
			var5 = (uint32)((uint32)1048576) - (a_PressureBeforeCalibrating->pressure);

			*a_PressureAfterCalibrating = ((uint32)(var5 - (uint32)(var2 / 4096))) * 3125;

			if (*a_PressureAfterCalibrating < 0x80000000)
			{
				*a_PressureAfterCalibrating = ( (*a_PressureAfterCalibrating) << 1) / ((uint32)var1);
			}
			else
			{
				(*a_PressureAfterCalibrating) = ((*a_PressureAfterCalibrating) / (uint32)var1) * 2;
			}

			var1 = (((sint32)((*a_configPtr)->calib1.words.dig_P9)) * ((sint32)((((*a_PressureAfterCalibrating) / 8) * ((*a_PressureAfterCalibrating) / 8)) / 8192))) / 4096;
			var2 = (((sint32)((*a_PressureAfterCalibrating) / 4)) * ((sint32)((*a_configPtr)->calib1.words.dig_P8))) / 8192;
			(*a_PressureAfterCalibrating) = (uint32)((sint32)(*a_PressureAfterCalibrating) + ((var1 + var2 + ((*a_configPtr)->calib1.words.dig_P7)) / 16));

			/* Check if we exceeded the temperature ranges */

			if ((*a_PressureAfterCalibrating) < pressure_min)
			{
				(*a_PressureAfterCalibrating) = pressure_min;
				result = BME280_EXCEEDED_MIN_RANGE;
				break;
			}
			else if ((*a_PressureAfterCalibrating) > pressure_max)
			{
				(*a_PressureAfterCalibrating) = pressure_max;
				result = BME280_EXCEEDED_MAX_RANGE;
				break;
			}
		}
		else  		/* Check if we exceeded the temperature ranges */

		{
			(*a_PressureAfterCalibrating) = pressure_min;
			result = BME280_EXCEEDED_MIN_RANGE;
			break;
		}


		/* Success */
		result = BME280_OK;
	}while(FALSE);

	return result;
}



/******************************************************************************
 * @fn BME_280_Status BME_280_convertHumidity_fixedPoint(BME_280_Config*, BME280_HumidityReading*, sint32*)
 * @brief this function will calibrate the humidity in a fixed point
 *
 * @param a_configPtr
 * @param a_HumidityBeforeCalibrating
 * @param a_HumidityAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertHumidity_fixedPoint
(
		BME_280_Config* a_configPtr,
		BME280_HumidityReading* a_HumidityBeforeCalibrating,
		uint32*	a_HumidityAfterCalibrating
)
{
	BME_280_Status result = BME280_OK;

	sint32 var1=ZERO, var2=ZERO, var3=ZERO, var4=ZERO, var5=ZERO;

	uint32 humidity_max = 102400;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Sensor Calibration calculations */
		var1 = (*a_configPtr)->calib2.words.t_fine - ((sint32)76800);
		var2 = (sint32)((a_HumidityBeforeCalibrating->humidity) * 16384);
		var3 = (sint32)(((sint32)((*a_configPtr)->calib2.words.dig_H4)) * 1048576);
		var4 = ((sint32)((*a_configPtr)->calib2.words.dig_H5)) * var1;
		var5 = (((var2 - var3) - var4) + (sint32)16384) / 32768;
		var2 = (var1 * ((sint32)((*a_configPtr)->calib2.words.dig_H6))) / 1024;
		var3 = (var1 * ((sint32)((*a_configPtr)->calib2.words.dig_H3))) / 2048;
		var4 = ((var2 * (var3 + (sint32)32768)) / 1024) + (sint32)2097152;
		var2 = ((var4 * ((sint32)((*a_configPtr)->calib2.words.dig_H2))) + 8192) / 16384;
		var3 = var5 * var2;
		var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
		var5 = var3 - ((var4 * ((sint32)((*a_configPtr)->calib1.words.dig_H1))) / 16);
		var5 = (var5 < 0 ? 0 : var5);
		var5 = (var5 > 419430400 ? 419430400 : var5);
		*a_HumidityAfterCalibrating = (uint32)(var5 / 4096);

		if (*a_HumidityAfterCalibrating > humidity_max)
		{
			*a_HumidityAfterCalibrating = humidity_max;
			result = BME280_EXCEEDED_MAX_RANGE;
			break;
		}

		/* Success */
		result = BME280_OK;
	}while(FALSE);

	return result;

}

#endif

#if (BME280_FLOAT_COMPENSATING_MEASURMENTS == _ENABLE_)

/******************************************************************************
 * @fn BME_280_Status BME_280_convertTemperature_floatingPoint(BME_280_Config*, BME280_TemperatureReading*, sint32*)
 * @brief this function will calibrate the temperature in a floating point
 *
 * @param a_configPtr
 * @param a_tempBeforeCalibrating
 * @param a_tempAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertTemperature_floatingPoint
(
		BME_280_Config* a_configPtr,
		BME280_TemperatureReading* a_tempBeforeCalibrating,
		float64*	a_tempAfterCalibrating
)
{
	BME_280_Status result = BME280_OK;
	float64 var1=ZERO, var2=ZERO ;
	float64 temperature_min = -40;
	float64 temperature_max = 85;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Sensor Calibration calculations */
		var1 = ((float64)a_tempBeforeCalibrating->temperature) / 16384.0 - ((float64)((*a_configPtr)->calib1.words.dig_T1)) / 1024.0;
		var1 = var1 * ((float64)((*a_configPtr)->calib1.words.dig_T2));
		var2 = (((float64)a_tempBeforeCalibrating->temperature) / 131072.0 - ((float64)((*a_configPtr)->calib1.words.dig_T1)) / 8192.0);
		var2 = (var2 * var2) * ((float64)((*a_configPtr)->calib1.words.dig_T3));

		(*a_configPtr)->calib2.words.t_fine_float = (sint32)(var1 + var2);

		*a_tempAfterCalibrating = (var1 + var2) / 5120.0;


		/* Check if we exceeded the temperature ranges */
		if (*a_tempAfterCalibrating < temperature_min)
		{
			*a_tempAfterCalibrating = temperature_min;
			result = BME280_EXCEEDED_MIN_RANGE;
			break;
		}
		else if (*a_tempAfterCalibrating > temperature_max)
		{
			*a_tempAfterCalibrating = temperature_max;
			result = BME280_EXCEEDED_MAX_RANGE;
			break;
		}


		/* Success */
		result = BME280_OK;
	}while(FALSE);

	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_convertPressure_floatingPoint(BME_280_Config*, BME280_PressureReading*, sint32*)
 * @brief this function will calibrate the pressure in a floating point
 *
 * @param a_configPtr
 * @param a_PressureBeforeCalibrating
 * @param a_PressureAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertPressure_floatingPoint
(
		BME_280_Config* a_configPtr,
		BME280_PressureReading* a_PressureBeforeCalibrating,
		float64*	a_PressureAfterCalibrating
)
{
	BME_280_Status result = BME280_OK;
	float64 var1 = ZERO;
	float64 var2 = ZERO;
	float64 var3 = ZERO;
	float64 pressure_min = 30000.0;
	float64 pressure_max = 110000.0;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Sensor Calibration calculations */

		var1 = ((float64)((*a_configPtr)->calib2.words.t_fine_float) / 2.0) - 64000.0;
		var2 = var1 * var1 * ((float64)(((*a_configPtr)->calib1.words.dig_P6))) / 32768.0;
		var2 = var2 + var1 * ((float64)((*a_configPtr)->calib1.words.dig_P5)) * 2.0;
		var2 = (var2 / 4.0) + (((float64)((*a_configPtr)->calib1.words.dig_P4)) * 65536.0);
		var3 = ((float64)((*a_configPtr)->calib1.words.dig_P3)) * var1 * var1 / 524288.0;
		var1 = (var3 + ((float64)((*a_configPtr)->calib1.words.dig_P2)) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0) * ((float64)((*a_configPtr)->calib1.words.dig_P1));

		/* avoid exception caused by division by zero */
		if (var1 > (0.0))
		{
			(*a_PressureAfterCalibrating) = 1048576.0 - (float64) (a_PressureBeforeCalibrating->pressure);
			(*a_PressureAfterCalibrating) = ((*a_PressureAfterCalibrating) - (var2 / 4096.0)) * 6250.0 / var1;
			var1 = ((float64)((*a_configPtr)->calib1.words.dig_P9)) * (*a_PressureAfterCalibrating) * (*a_PressureAfterCalibrating) / 2147483648.0;
			var2 = (*a_PressureAfterCalibrating) * ((float64)((*a_configPtr)->calib1.words.dig_P8)) / 32768.0;
			(*a_PressureAfterCalibrating) = (*a_PressureAfterCalibrating) + (var1 + var2 + ((double)((*a_configPtr)->calib1.words.dig_P7))) / 16.0;

			if ((*a_PressureAfterCalibrating) < pressure_min)
			{
				(*a_PressureAfterCalibrating) = pressure_min;
				result = BME280_EXCEEDED_MIN_RANGE;
				break;
			}
			else if ((*a_PressureAfterCalibrating) > pressure_max)
			{
				(*a_PressureAfterCalibrating) = pressure_max;
				result = BME280_EXCEEDED_MAX_RANGE;
				break;
			}
		}
		else /* Invalid case */
		{
			(*a_PressureAfterCalibrating) = pressure_min;
			result = BME280_EXCEEDED_MIN_RANGE;
			break;
		}

		/* Success */
		result = BME280_OK;
	}while(FALSE);

	return result;

}


/******************************************************************************
 * @fn BME_280_Status BME_280_convertHumidity_floatingPoint(BME_280_Config*, BME280_HumidityReading*, sint32*)
 * @brief this function will calibrate the humidity in a floating point
 *
 * @param a_configPtr
 * @param a_HumidityBeforeCalibrating
 * @param a_HumidityAfterCalibrating
 * @return
 *******************************************************************************/
STATIC BME_280_Status BME_280_convertHumidity_floatingPoint
(
		BME_280_Config* a_configPtr,
		BME280_HumidityReading* a_HumidityBeforeCalibrating,
		float64*	a_HumidityAfterCalibrating
)
{
	BME_280_Status result = BME280_OK;
	float64 humidity_min = 0.0;
	float64 humidity_max = 100.0;
	float64 var1 = ZERO;
	float64 var2 = ZERO;
	float64 var3 = ZERO;
	float64 var4 = ZERO;
	float64 var5 = ZERO;
	float64 var6 = ZERO;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Sensor Calibration calculations */
		var1 = ((float64)((*a_configPtr)->calib2.words.t_fine_float)) - 76800.0;
		var2 = (((float64)((*a_configPtr)->calib2.words.dig_H4)) * 64.0 + (((double)((*a_configPtr)->calib2.words.dig_H5)) / 16384.0) * var1);
		var3 = a_HumidityBeforeCalibrating->humidity - var2;
		var4 = ((float64)((*a_configPtr)->calib2.words.dig_H2)) / 65536.0;
		var5 = (1.0 + (((float64)((*a_configPtr)->calib2.words.dig_H3)) / 67108864.0) * var1);
		var6 = 1.0 + (((float64)((*a_configPtr)->calib2.words.dig_H6)) / 67108864.0) * var1 * var5;
		var6 = var3 * var4 * (var5 * var6);
		(*a_HumidityAfterCalibrating) = var6 * (1.0 - ((float64)((*a_configPtr)->calib1.words.dig_H1)) * var6 / 524288.0);

		if ((*a_HumidityAfterCalibrating) > humidity_max)
		{
			(*a_HumidityAfterCalibrating) = humidity_max;
			result = BME280_EXCEEDED_MAX_RANGE;
			break;
		}
		else if ((*a_HumidityAfterCalibrating) < humidity_min)
		{
			(*a_HumidityAfterCalibrating) = humidity_min;
			result = BME280_EXCEEDED_MIN_RANGE;
			break;
		}

		/* Success */
		result = BME280_OK;
	}while(FALSE);

	return result;
}

#endif



/*******************************************************************************
 *                          Public Function Definitions	                       *
 *******************************************************************************/

/******************************************************************************
 * @fn BME_280_Status BME_280_SoftReset(BME_280_Config*)
 * @brief This function will make a reset to sensor and make it return to
 *        default state.
 *
 * @param a_configPtr: configPtr: pointer to configuration structure
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SoftReset
(
		BME_280_Config* a_configPtr
)
{
	BME_280_Status result = BME280_OK;
	uint8 resetValue = BME280_SOFT_RESET_COMMAND;
	uint8 statusRegister = 1;

	/* Checking if the input is not null pointer */
	if( NULL_PTR != a_configPtr)
	{
		/* Writing reset command to the sensor */
		result = BME_280_write(BME280_RESET_ADDR, &resetValue, a_configPtr);

		/* If our write is successful enter */
		if(BME280_OK == result )
		{
			do
			{
				/* Delay needed for the sensor to reset */
				BME_280_Delay(2);

				/* get the status register from the sensor */
				BME_280_read(BME280_STATUS_ADDR, &statusRegister, 1, a_configPtr);

				/* Mask to get only the LSB <IM_Update bit> */
				statusRegister &= IM_UPDATE_MASK;

				/* Further delay if sensor need more time to reset */
			}while( ZERO != statusRegister);

			/* Success */
			result = BME280_OK;

		}
		else
		{
			/* return error */
			result = BME280_COMM_ERROR;
		}
	}
	else
	{
		/* return error */
		result = BME280_NULL_ERROR;
	}

	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_Init(BME_280_Config*)
 * @brief This function will initialize the sensor by making soft reset and
 *        check the ID of this sensor and save the calibrated data in configuration
 *        structure.
 *
 * @param a_configPtr
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_Init
(
		BME_280_Config* a_configPtr,
		uint8			a_protocolUsed
)
{
	BME_280_Status result = BME280_OK;
	uint8 i = ZERO;
	uint8 tryCount = FIVE; 	/* Discover sensor try count */
	uint8 ID_value = ZERO ;

	do
	{
		/* Check if the input is null or not */
		if(NULL_PTR == a_configPtr )
		{
			result = BME280_NULL_ERROR;
			break;
		}

		if( (BME_280_INTERFACE_SPI != a_protocolUsed) || (BME_280_INTERFACE_SPI != a_protocolUsed))
		{
			result = BME280_INTERFACE_UNKNOWN;
			break;
		}

		/* Break if the input already has an instance */
		if( INSTANCE_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_ERROR_INSTANCE_ALEARDY_TAKEN;
			break;
		}

		/* search if their is an instance occupied or not and give it one if not */
		for( i = ZERO ; i < sizeof(g_bme280_instances)/sizeof(g_bme280_instances[ZERO]) ; i++)
		{
			if( NOT_OCCUIPIED == g_bme280_instances[i].occupied)
			{

				*a_configPtr = &g_bme280_instances[i];
				(*a_configPtr)->occupied = OCCUIPIED;
				(*a_configPtr)->ProtocolUsed = a_protocolUsed;
				break;
			}
		}

		/* tell the user that their is enough instances available */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NO_INSTANCES_AVALIABLE;
			break;
		}

		while(tryCount)
		{
			/* Attempt to get ID from ID register */
			result = BME_280_read(BME280_CHIP_ID_ADDR, &ID_value, ONE , a_configPtr);

			/* If ID is not returned successfully, try again */
			if ( (ID_value == BME280_CHIP_ID)  && (result == BME280_OK))
			{

				/* Attempt to reset the sensor by writing the reset word into the reset register*/
				result = BME_280_SoftReset(a_configPtr);

				if (result == BME280_OK)
				{
					/* Read Calibration Data */
					result = BME_280_getCalibratedData(a_configPtr);
					/* Set mode to sleep mode */
					result = BME_280_SetMode(a_configPtr, SLEEP_MODE);
					break;
				}
			}

			/* wait for 1 MS then try again if needed */
			BME_280_Delay(ONE);
			--tryCount;
		}

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_DeInit(BME_280_Config*)
 * @brief This function will deInit your instance of the sensor
 *
 * @param a_configPtr
 * @return the status of this API
 *******************************************************************************/
BME_280_Status BME_280_DeInit
(
		BME_280_Config* a_configPtr
)
{
	BME_280_Status result = BME280_OK;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Set sensor mode to sleep mode */
		result = BME_280_SetMode(a_configPtr, SLEEP_MODE);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Free the resources */
		(*a_configPtr)->occupied = ZERO;
		a_configPtr = NULL_PTR;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}



/******************************************************************************
 * @fn BME_280_Status BME_280_SetMode(BME_280_Config*, BME_280_Mode)
 * @brief This function will set the mode of the sensor
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_mode: the mode of the sensor to be set
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SetMode
(
		BME_280_Config* a_configPtr,
		BME_280_Mode    a_mode
)
{
	BME_280_Status result = BME280_OK;
	BME280_CtrlMeasRegisterUnion CtrlMeasRegister = {ZERO};
	BME_280_Mode checkMode = ZERO;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration measurement register from sensor first */
		result = BME_280_read(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Update the mode */
		CtrlMeasRegister.Bits.mode = a_mode;

		/* Write again the configuration in the sensor */
		result = BME_280_write(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), a_configPtr);
		BME_280_Delay(10);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Check that the Settings is set successfully */
		result = BME_280_GetMode(a_configPtr, &checkMode);

		if(a_mode != checkMode)
		{
			result = BME280_SETTINGS_FAILED_TO_BE_SET;
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Mode BME_280_GetMode(BME_280_Config*)
 * @brief This function will get the current mode of the sensor
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_mode: the mode of the sensor, the user enter this parameter and the
 *                 function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_GetMode
(
		BME_280_Config* a_configPtr,
		BME_280_Mode*    a_mode
)
{
	BME_280_Status result = BME280_OK;
	BME280_CtrlMeasRegisterUnion CtrlMeasRegister = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration measurement register from sensor first */
		result = BME_280_read(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* return the mode to the user */
		*a_mode = CtrlMeasRegister.Bits.mode;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_SetFilterSettings(BME_280_Config*, BME_280_FilterCoefficient)
 * @brief this function will set the filter coefficient of the sensor.
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_filterCoef: the filter coefficient to be set
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SetFilterSettings
(
		BME_280_Config* 			 a_configPtr,
		BME_280_FilterCoefficient    a_filterCoef
)
{
	BME_280_Status result = BME280_OK;
	BME_280_ConfigRegisterUnion ConfigRegister  = {ZERO};
	BME_280_FilterCoefficient checkFilterCoef  = {ZERO};
	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration  register from sensor first */
		result = BME_280_read(BME280_CONFIG_ADDR, &(ConfigRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Update the filter coefficient */
		ConfigRegister.Bits.filter_coeff = a_filterCoef;

		/* Write again the configuration in the sensor */
		result = BME_280_write(BME280_CONFIG_ADDR, &(ConfigRegister.config), a_configPtr);
		BME_280_Delay(10);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Check that the Settings is set successfully */
		result = BME_280_GetFilterSettings(a_configPtr, &checkFilterCoef);

		if(a_filterCoef != checkFilterCoef)
		{
			result = BME280_SETTINGS_FAILED_TO_BE_SET;
			break;
		}


		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_FilterCoefficient BME_280_GetFilterSettings(BME_280_Config*)
 * @brief  this function will get the current filter coefficient of the sensor.
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_filterCoef: the filter coefficient of the sensor, the user enter this
 * 					     parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_GetFilterSettings
(
		BME_280_Config* 			 a_configPtr,
		BME_280_FilterCoefficient*    a_filterCoef
)
{
	BME_280_Status result = BME280_OK;
	BME_280_ConfigRegisterUnion ConfigRegister  = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration  register from sensor first */
		result = BME_280_read(BME280_CONFIG_ADDR, &(ConfigRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Return the filter coefficient to the user */
		*a_filterCoef  = ConfigRegister.Bits.filter_coeff;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;

}

/******************************************************************************
 * @fn BME_280_Status BME_280_SetStandbyTime(BME_280_Config*, BME_280_StandbyTimeInNormalMode)
 * @brief this function will set the standby time in case of using normal Mode
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_standbytime: the standby time to be set
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SetStandbyTime
(
		BME_280_Config* 			       a_configPtr,
		BME_280_StandbyTimeInNormalMode    a_standbytime
)
{
	BME_280_Status result = BME280_OK;
	BME_280_ConfigRegisterUnion ConfigRegister  = {ZERO};
	BME_280_Mode	mode  = {ZERO};
	BME_280_StandbyTimeInNormalMode checkStandbytime  = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* get the operating mode */
		result = BME_280_GetMode(a_configPtr , &mode);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		if(NORMAL_MODE != mode)
		{
			result = BME280_ERROR_CANNOT_SET_T_SB_NOT_IN_NORMAL_MODE;
			break;
		}

		/* Read configuration  register from sensor first */
		result = BME_280_read(BME280_CONFIG_ADDR, &(ConfigRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Update the standby time */
		ConfigRegister.Bits.t_sb = a_standbytime;

		/* Write again the configuration in the sensor */
		result = BME_280_write(BME280_CONFIG_ADDR, &(ConfigRegister.config), a_configPtr);
		BME_280_Delay(10);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Check that the Settings is set successfully */
		result = BME_280_GetStandbyTime(a_configPtr, &checkStandbytime);

		if(a_standbytime != checkStandbytime)
		{
			result = BME280_SETTINGS_FAILED_TO_BE_SET;
			break;
		}

		/* Success */
		result = BME280_OK;
	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_StandbyTimeInNormalMode BME_280_GetStandbyTime(BME_280_Config*)
 * @brief this function will get the current standby time in case of using normal Mode
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_standbytime: the standby time of the sensor, the user enter this
 * 					     parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_GetStandbyTime
(
		BME_280_Config* a_configPtr,
		BME_280_StandbyTimeInNormalMode*    a_standbytime
)
{
	BME_280_Status result = BME280_OK;
	BME_280_ConfigRegisterUnion ConfigRegister  = {ZERO};
	BME_280_Mode	mode = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* get the operating mode */
		result = BME_280_GetMode(a_configPtr , &mode);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		if(NORMAL_MODE != mode)
		{
			result = BME280_ERROR_CANNOT_GET_T_SB_NOT_IN_NORMAL_MODE;
			break;
		}

		/* Read configuration  register from sensor first */
		result = BME_280_read(BME280_CONFIG_ADDR, &(ConfigRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Get standby time for the user */
		*a_standbytime = ConfigRegister.Bits.t_sb;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_SetTempOverSamplingSetting(BME_280_OversamplingValue)
 * @brief this function will set the over sampling settings for the temperature
 *        in this function
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_tempOverSampleValue: the temperature over-sampling value
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SetTempOverSamplingSetting
(
		BME_280_Config* a_configPtr,
		BME_280_OversamplingValue a_tempOverSampleValue
)
{
	BME_280_Status result = BME280_OK;
	BME280_CtrlMeasRegisterUnion CtrlMeasRegister = {ZERO};
	BME_280_OversamplingValue checkTempOverSampleValue = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration measurement register from sensor first */
		result = BME_280_read(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Update the temperature over-sampling value */
		CtrlMeasRegister.Bits.osrs_t = a_tempOverSampleValue;

		/* Write again the configuration in the sensor */
		result = BME_280_write(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), a_configPtr);
		BME_280_Delay(10);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Check that the Settings is set successfully */
		result = BME_280_GetTempOverSamplingSetting(a_configPtr, &checkTempOverSampleValue);

		if(a_tempOverSampleValue != checkTempOverSampleValue)
		{
			result = BME280_SETTINGS_FAILED_TO_BE_SET;
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);
	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_GetTempOverSamplingSetting(BME_280_OversamplingValue)
 * @brief  this function will communicate with the sensor and will read the
 *         over sampling value
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_standbytime: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_GetTempOverSamplingSetting
(
		BME_280_Config* a_configPtr,
		BME_280_OversamplingValue* a_tempOverSampleValue
)
{
	BME_280_Status result = BME280_OK;
	BME280_CtrlMeasRegisterUnion CtrlMeasRegister = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration measurement register from sensor first */
		result = BME_280_read(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Get the over sampling value of temperature for the user */
		*a_tempOverSampleValue = CtrlMeasRegister.Bits.osrs_t;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_SetPressureOverSamplingSetting(BME_280_Config*, BME_280_OversamplingValue)
 * @brief  this function will set the over sampling settings for the pressure
 *         in this function
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_pressureOverSampleValue: the pressure over-sampling value
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SetPressureOverSamplingSetting
(
		BME_280_Config* a_configPtr,
		BME_280_OversamplingValue a_pressureOverSampleValue
)
{
	BME_280_Status result = BME280_OK;
	BME280_CtrlMeasRegisterUnion CtrlMeasRegister = {ZERO};
	BME_280_OversamplingValue checkPressureOverSampleValue = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration measurement register from sensor first */
		result = BME_280_read(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Update the pressure over-sampling value */
		CtrlMeasRegister.Bits.osrs_p = a_pressureOverSampleValue;

		/* Write again the configuration in the sensor */
		result = BME_280_write(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), a_configPtr);
		BME_280_Delay(10);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Check that the Settings is set successfully */
		result = BME_280_GetPressureOverSamplingSetting(a_configPtr, &checkPressureOverSampleValue);

		if(a_pressureOverSampleValue != checkPressureOverSampleValue)
		{
			result = BME280_SETTINGS_FAILED_TO_BE_SET;
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);
	return result;
}

/******************************************************************************
 * @fn BME_280_OversamplingValue BME_280_GetPressureOverSamplingSetting(BME_280_Config*)
 * @brief  this function will communicate with the sensor and will read the
 *         over sampling value
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_pressureOverSampleValue: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_GetPressureOverSamplingSetting
(
		BME_280_Config* a_configPtr,
		BME_280_OversamplingValue*  a_pressureOverSampleValue
)
{
	BME_280_Status result = BME280_OK;
	BME280_CtrlMeasRegisterUnion CtrlMeasRegister = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration measurement register from sensor first */
		result = BME_280_read(BME280_CTRL_MEAS_ADDR, &(CtrlMeasRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Get the over sampling value of pressure for the user */
		*a_pressureOverSampleValue = CtrlMeasRegister.Bits.osrs_p;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_SetHumidityOverSamplingSetting(BME_280_Config*, BME_280_OversamplingValue)
 * @brief  this function will set the over sampling settings for the humidity
 *         in this function
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param a_humidityOverSampleValue: the humidity over-sampling value
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SetHumidityOverSamplingSetting
(
		BME_280_Config* a_configPtr,
		BME_280_OversamplingValue a_humidityOverSampleValue
)
{
	BME_280_Status result = BME280_OK;
	BME280_CtrlHumRegisterUnion HumidityRegister = {ZERO};
	BME_280_Mode mode = {ZERO};
	BME_280_OversamplingValue checkHumidityOverSampleValue = {ZERO};
	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration measurement register from sensor first */
		result = BME_280_read(BME280_CTRL_HUM_ADDR, &(HumidityRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Update the humidity over-sampling value */
		HumidityRegister.Bits.ctrl_hum = a_humidityOverSampleValue;

		/* We must write in Control measurement register to wirte in humidity register */
		BME_280_GetMode(a_configPtr, &mode);
		BME_280_SetMode(a_configPtr, mode);


		/* Write again the configuration in the sensor */
		result = BME_280_write(BME280_CTRL_HUM_ADDR, &(HumidityRegister.config), a_configPtr);
		BME_280_Delay(10);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Check that the Settings is set successfully */
		result = BME_280_GetHumidityOverSamplingSetting(a_configPtr, &checkHumidityOverSampleValue);

		if(a_humidityOverSampleValue != checkHumidityOverSampleValue)
		{
			result = BME280_SETTINGS_FAILED_TO_BE_SET;
			break;
		}

		/* Success */
		result = BME280_OK;
	}while(FALSE);

	return result;
}


/******************************************************************************
 * @fn BME_280_OversamplingValue BME_280_GetHumidityOverSamplingSetting(BME_280_Config*)
 * @brief  this function will communicate with the sensor and will read the
 *         over sampling value
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_humidityOverSampleValue: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_GetHumidityOverSamplingSetting
(
		BME_280_Config* a_configPtr,
		BME_280_OversamplingValue* a_humidityOverSampleValue
)
{
	BME_280_Status result = BME280_OK;
	BME280_CtrlHumRegisterUnion HumidityRegister = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read configuration measurement register from sensor first */
		result = BME_280_read(BME280_CTRL_HUM_ADDR, &(HumidityRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Get the humidity over-sampling value for the user */
		*a_humidityOverSampleValue = HumidityRegister.Bits.ctrl_hum;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_getID(BME_280_Config*, uint8*)
 * @brief This function will get the ID of the sensor
 *
 * @param a_configPtr: pointer to configuration structure
 * @param a_ID: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getID
(
		BME_280_Config* a_configPtr,
		uint8*			a_ID
)
{
	BME_280_Status result = BME280_OK;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read ID from sensor first */
		result = BME_280_read(BME280_CHIP_ID_ADDR, a_ID, ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_getUpdateStatus(BME_280_Config*, uint8*)
 * @brief
 *
 * @param  a_configPtr
 * @param  a_status: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getUpdateStatus
(
		BME_280_Config* a_configPtr,
		uint8*			a_status
)
{
	BME_280_Status result = BME280_OK;
	BME280_StatusRegisterUnion StatusRegister = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read update flag from sensor first */
		result = BME_280_read(BME280_STATUS_ADDR, &(StatusRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Return the update flag to the user*/
		*a_status = StatusRegister.Bits.im_update;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;

}

/******************************************************************************
 * @fn BME_280_Status BME_280_getMeasurmentStatus(BME_280_Config*, uint8*)
 * @brief
 *
 * @param  a_configPtr
 * @param  a_status: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getMeasurmentStatus
(
		BME_280_Config* a_configPtr,
		uint8*			a_status
)
{
	BME_280_Status result = BME280_OK;
	BME280_StatusRegisterUnion StatusRegister  = {ZERO};
	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		/* Read update flag from sensor first */
		result = BME_280_read(BME280_STATUS_ADDR, &(StatusRegister.config), ONE, a_configPtr);

		/* Error occurred */
		if(result != BME280_OK)
		{
			break;
		}

		/* Return the measuring flag to the user*/
		*a_status = StatusRegister.Bits.measuring;

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

#if (BME280_32BIT_COMPENSATING_MEASURMENTS == _ENABLE_)
/******************************************************************************
 * @fn BME_280_Status BME_280_getTemperature_fixedPoint(BME_280_Config*, sint32*)
 * @brief  output the temperature in this format: value of "5123" equal 51.23 degree
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_Temperature: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getTemperature_fixedPoint
(
		BME_280_Config* a_configPtr,
		sint32*	a_Temperature
)
{
	BME_280_Status result = BME280_OK;
	BME280_UncompensatedReadings unCompensatedData  = {ZERO};
	BME280_TemperatureReading    unCompensatedTemp = {ZERO};
	uint32 measurementDelay = ZERO;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		if(result != BME280_OK)
		{
			break;
		}

		/* wait for measurement delay */
		result = BME_280_getMeasurmentTimeInMsec(a_configPtr, &measurementDelay);
		if(result != BME280_OK)
		{
			break;
		}
		BME_280_Delay(measurementDelay);

		/* Get uncompensated Readings from the sensor */
		result = BME_280_getUncompensatedData(a_configPtr, &unCompensatedData);

		if(result != BME280_OK)
		{
			break;
		}

		/* Get uncompensated Temperature from uncompensated data */
		unCompensatedTemp.Data.xlsb = unCompensatedData.Bytes.temp_xlsb;
		unCompensatedTemp.Data.lsb = unCompensatedData.Bytes.temp_lsb;
		unCompensatedTemp.Data.msb = unCompensatedData.Bytes.temp_msb;

		/* Convert Temperature to a real value */
		result = BME_280_convertTemperature_fixedPoint(a_configPtr, &unCompensatedTemp, a_Temperature);

		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;


	}while(FALSE);

	return result;
}

/******************************************************************************
 * @fn BME_280_Status BME_280_getPressure_fixedPoint(BME_280_Config*, uint32*)
 * @brief  output the Pressure in this format: value of "101325" equals to
 *         101325 Pa
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_Pressure: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getPressure_fixedPoint
(
		BME_280_Config* a_configPtr,
		uint32*	        a_Pressure
)
{
	BME_280_Status result = BME280_OK;
	BME280_UncompensatedReadings unCompensatedData  = {ZERO};
	BME280_PressureReading       unCompensatedPressure = {ZERO};
	sint32 temp = {ZERO};
	uint32 measurementDelay = ZERO;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		if(result != BME280_OK)
		{
			break;
		}

		/* wait for measurement delay */
		result = BME_280_getMeasurmentTimeInMsec(a_configPtr, &measurementDelay);
		if(result != BME280_OK)
		{
			break;
		}
		BME_280_Delay(measurementDelay);

		/* Get uncompensated Readings from the sensor */
		result = BME_280_getUncompensatedData(a_configPtr, &unCompensatedData);

		if(result != BME280_OK)
		{
			break;
		}

		/* Get uncompensated Pressure from uncompensated data */
		unCompensatedPressure.Data.xlsb = unCompensatedData.Bytes.press_xlsb;
		unCompensatedPressure.Data.lsb = unCompensatedData.Bytes.press_lsb;
		unCompensatedPressure.Data.msb = unCompensatedData.Bytes.press_msb;

		/* Calculate temperature as its calibrated data is needed */
		result = BME_280_getTemperature_fixedPoint(a_configPtr, &temp);
		if(result != BME280_OK)
		{
			break;
		}

		/* Convert Pressure to a real value */
		result = BME_280_convertPressure_fixedPoint(a_configPtr, &unCompensatedPressure, a_Pressure);

		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}


/******************************************************************************
 * @fn BME_280_Status BME_280_getHumidity_fixedPoint(BME_280_Config*, uint32*)
 * @brief  output the Humidity in this format: value of "47445" equals to
 *         47.445% humidity
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_Humidity: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getHumidity_fixedPoint
(
		BME_280_Config* a_configPtr,
		uint32*	a_Humidity
)
{

	BME_280_Status result = BME280_OK;
	BME280_UncompensatedReadings unCompensatedData = {ZERO};
	BME280_HumidityReading       unCompensatedHumidity = {ZERO};
	sint32 temp = {ZERO};
	uint32 measurementDelay = ZERO;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		if(result != BME280_OK)
		{
			break;
		}

		/* wait for measurement delay */
		result = BME_280_getMeasurmentTimeInMsec(a_configPtr, &measurementDelay);
		if(result != BME280_OK)
		{
			break;
		}
		BME_280_Delay(measurementDelay);

		/* Get uncompensated Readings from the sensor */
		result = BME_280_getUncompensatedData(a_configPtr, &unCompensatedData);

		if(result != BME280_OK)
		{
			break;
		}

		/* Get uncompensated Pressure from uncompensated data */
		unCompensatedHumidity.Data.lsb = unCompensatedData.Bytes.hum_lsb;
		unCompensatedHumidity.Data.msb = unCompensatedData.Bytes.hum_msb;

		/* Calculate temperature as its calibrated data is needed */
		result = BME_280_getTemperature_fixedPoint(a_configPtr, &temp);
		if(result != BME280_OK)
		{
			break;
		}

		/* Convert Pressure to a real value */
		result = BME_280_convertHumidity_fixedPoint(a_configPtr, &unCompensatedHumidity, a_Humidity);

		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

#endif

#if (BME280_FLOAT_COMPENSATING_MEASURMENTS == _ENABLE_)
/******************************************************************************
 * @fn BME_280_Status BME_280_getTemperature_floatingPoint(BME_280_Config*, sint32*)
 * @brief  output the temperature in floating point format, the output is in celsius
 *         degree
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_Temperature: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getTemperature_floatingPoint
(
		BME_280_Config* a_configPtr,
		float64*	    a_Temperature
)
{
	BME_280_Status result = BME280_OK;
	BME280_UncompensatedReadings unCompensatedData = {ZERO};
	BME280_TemperatureReading    unCompensatedTemp = {ZERO};
	uint32 measurementDelay = ZERO;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		if(result != BME280_OK)
		{
			break;
		}

		/* wait for measurement delay */
		result = BME_280_getMeasurmentTimeInMsec(a_configPtr, &measurementDelay);
		if(result != BME280_OK)
		{
			break;
		}
		BME_280_Delay(measurementDelay);

		/* Get uncompensated Readings from the sensor */
		result = BME_280_getUncompensatedData(a_configPtr, &unCompensatedData);

		if(result != BME280_OK)
		{
			break;
		}

		/* Get uncompensated Temperature from uncompensated data */
		unCompensatedTemp.Data.xlsb = unCompensatedData.Bytes.temp_xlsb;
		unCompensatedTemp.Data.lsb = unCompensatedData.Bytes.temp_lsb;
		unCompensatedTemp.Data.msb = unCompensatedData.Bytes.temp_msb;

		/* Convert Temperature to a real value */
		result = BME_280_convertTemperature_floatingPoint(a_configPtr, &unCompensatedTemp, a_Temperature);

		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;


	}while(FALSE);

	return result;

}

/******************************************************************************
 * @fn BME_280_Status BME_280_getPressure_floatingPoint(BME_280_Config*, sint32*)
 * @brief  output the Pressure in floating point format, The output is in Pa
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_Pressure: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getPressure_floatingPoint
(
		BME_280_Config* a_configPtr,
		float64*	    a_Pressure
)
{
	BME_280_Status result = BME280_OK;
	BME280_UncompensatedReadings unCompensatedData = {ZERO};
	BME280_PressureReading       unCompensatedPressure = {ZERO};
	float64 temp = {ZERO};
	uint32 measurementDelay = ZERO;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		if(result != BME280_OK)
		{
			break;
		}

		/* wait for measurement delay */
		result = BME_280_getMeasurmentTimeInMsec(a_configPtr, &measurementDelay);
		if(result != BME280_OK)
		{
			break;
		}
		BME_280_Delay(measurementDelay);

		/* Get uncompensated Readings from the sensor */
		result = BME_280_getUncompensatedData(a_configPtr, &unCompensatedData);

		if(result != BME280_OK)
		{
			break;
		}

		/* Get uncompensated Pressure from uncompensated data */
		unCompensatedPressure.Data.xlsb = unCompensatedData.Bytes.press_xlsb;
		unCompensatedPressure.Data.lsb  =  unCompensatedData.Bytes.press_lsb;
		unCompensatedPressure.Data.msb  =  unCompensatedData.Bytes.press_msb;

		/* Calculate temperature as its calibrated data is needed */
		result = BME_280_getTemperature_floatingPoint(a_configPtr, &temp);
		if(result != BME280_OK)
		{
			break;
		}

		/* Convert Pressure to a real value */
		result = BME_280_convertPressure_floatingPoint(a_configPtr, &unCompensatedPressure, a_Pressure);

		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}



/******************************************************************************
 * @fn BME_280_Status BME_280_getHumidity_floatingPoint(BME_280_Config*, sint32*)
 * @brief  output the Humidity in floating point format, the output is the %
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_Humidity: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getHumidity_floatingPoint
(
		BME_280_Config* a_configPtr,
		float64*	    a_Humidity
)
{

	BME_280_Status result = BME280_OK;
	BME280_UncompensatedReadings unCompensatedData = {ZERO};
	BME280_HumidityReading       unCompensatedHumidity = {ZERO};
	float64 temp = {ZERO};
	uint32 measurementDelay = ZERO;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		if(result != BME280_OK)
		{
			break;
		}

		/* wait for measurement delay */
		result = BME_280_getMeasurmentTimeInMsec(a_configPtr, &measurementDelay);
		if(result != BME280_OK)
		{
			break;
		}
		BME_280_Delay(measurementDelay);

		/* Get uncompensated Readings from the sensor */
		result = BME_280_getUncompensatedData(a_configPtr, &unCompensatedData);

		if(result != BME280_OK)
		{
			break;
		}

		/* Get uncompensated Pressure from uncompensated data */
		unCompensatedHumidity.Data.lsb = unCompensatedData.Bytes.hum_lsb;
		unCompensatedHumidity.Data.msb = unCompensatedData.Bytes.hum_msb;

		/* Calculate temperature as its calibrated data is needed */
		result = BME_280_getTemperature_floatingPoint(a_configPtr, &temp);
		if(result != BME280_OK)
		{
			break;
		}

		/* Convert Pressure to a real value */
		result = BME_280_convertHumidity_floatingPoint(a_configPtr, &unCompensatedHumidity, a_Humidity);

		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;

	}while(FALSE);

	return result;
}

#endif



/******************************************************************************
 * @fn  BME_280_getSensorSettings(BME_280_Config*, BME_280_settings*)
 * @brief this function will get the current sensor settings
 *
 * @param a_configPtr
 * @param a_settings: This is provided to the user and it will be filled with
 *                    sensor settings.
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getSensorSettings
(
		BME_280_Config*   a_configPtr,
		BME_280_settings* a_settings
)
{
	BME_280_Status result = BME280_OK;

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		if(result != BME280_OK)
		{
			break;
		}

		/* Get the mode */
		result = BME_280_GetMode(a_configPtr, &(a_settings->mode));
		if(result != BME280_OK)
		{
			break;
		}

		/* Get the StandbyTime */
		result = BME_280_GetStandbyTime(a_configPtr, &(a_settings->standbyTime));
		if(result != BME280_OK)
		{
			break;
		}

		/* Get the Filter Setting */
		result = BME_280_GetFilterSettings(a_configPtr, &(a_settings->filterCoeff));
		if(result != BME280_OK)
		{
			break;
		}

		/* Get the Pressure OverSampling Setting */
		result = BME_280_GetPressureOverSamplingSetting(a_configPtr, &(a_settings->overSampling_settings.Pressure));
		if(result != BME280_OK)
		{
			break;
		}

		/* Get the temperature OverSampling Setting */
		result = BME_280_GetTempOverSamplingSetting(a_configPtr, &(a_settings->overSampling_settings.Temperture));
		if(result != BME280_OK)
		{
			break;
		}

		/* Get the Humidity OverSampling Setting */
		result = BME_280_GetHumidityOverSamplingSetting(a_configPtr, &(a_settings->overSampling_settings.humidity));
		if(result != BME280_OK)
		{
			break;
		}

		/* Success */
		result = BME280_OK;


	}while(FALSE);

	return result;
}



/******************************************************************************
 * @fn BME_280_Status BME_280_getMeasurmentTimeInMsec(BME_280_Config*, uint32*, BME_280_settings*)
 * @brief  This function will return the measurement time taken by the sensor
 *
 * @param  a_config
 * @param  a_measurmentTime: the user enter this parameter and the function will fill it
 * @param  a_settings: the function will set the current settings for the taken
 *                     measurement
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getMeasurmentTimeInMsec
(
		BME_280_Config*   a_configPtr,
		uint32*			  a_measurmentTime
)
{
	BME_280_Status result = BME280_OK;
	uint8 osrs_t = {ZERO}, osrs_p = {ZERO}, osrs_h = {ZERO};
	float32 ODR = {ZERO};
	float32 tMeasure_ms = {ZERO};
	BME_280_settings settings = {ZERO};

	do
	{
		if(NULL_PTR == a_configPtr)
		{
			result = BME280_NULL_ERROR;
			break;
		}

		/* Break if the input already has no instance */
		if( INSTANCE_NOT_TAKEN == BME_280_doesInstanceExsits(a_configPtr) )
		{
			result = BME280_NOT_INITIALIZED;
			break;
		}

		if(result != BME280_OK)
		{
			break;
		}

		/* Get the sensor Settings */
		result = BME_280_getSensorSettings(a_configPtr, &settings);
		if(result != BME280_OK)
		{
			break;
		}

		/**
		 * Relation between sampling settings and real xX values:
		 * Input BME280_OVERSAMPLING_x16 = 0x05
		 * Output 16
		 *
		 * 2<<(0x05>>1) = 2<<(0x04) = 2<<4 = 16
		 *
		 * Input BME280_OVERSAMPLING_x8 = 0x04
		 * Output 16
		 */
		osrs_t = ((settings.overSampling_settings.Temperture) == (SKIPPED)) ?	0 : (2 << (settings.overSampling_settings.Temperture - 1));

		osrs_p = ((settings.overSampling_settings.Pressure) == (SKIPPED)) ? 0 : (2 << (settings.overSampling_settings.Pressure - 1));

		osrs_h = ((settings.overSampling_settings.humidity) == (SKIPPED)) ? 0 : (2 << (settings.overSampling_settings.humidity - 1));

		tMeasure_ms = 1.25 + (2.3 * ((float32) osrs_t)) + (2 * ((float32) osrs_p) + 0.5) + (2 * ((float32) osrs_h) + 0.5);

		/* Filter is enabled */
		if (settings.filterCoeff != FILTER_OFF)
		{
			/* Map standby time to their values*/
			float32 tStandby_map[] = { 0.5, 62.5, 125.0, 250.0, 500.0, 1000.0, 10.0, 20.0 };
			float32 filter_map[] = { 1, 2, 5, 11, 22 };

			switch (settings.mode)
			{
			case FORCED_MODE:
				ODR = 1000 / tMeasure_ms;
				break;
			case NORMAL_MODE:
				ODR = (1000) / ((tMeasure_ms + tStandby_map[settings.standbyTime]));
				break;

			case SLEEP_MODE:
				break;
			}

			*a_measurmentTime = (uint32) (((float) 1000 * filter_map[settings.filterCoeff]) / ODR);
		}
		/* Filter disabled*/
		else
		{
			*a_measurmentTime =  (uint32) tMeasure_ms;
		}


		/* Success */
		result = BME280_OK;


	}while(FALSE);

	return result;
}

