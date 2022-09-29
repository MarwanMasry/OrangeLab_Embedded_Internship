/******************************************************************************
 *
 * Module: BME_280
 *
 * File Name: BME_280.h
 *
 * Description: Header file for BME_280 Driver
 *
 * Author: Marwan Abdelhakim Elmasry
 ******************************************************************************/
#ifndef BME_280_H_
#define BME_280_H_

/*******************************************************************************
 *                             ID - Version Info                               *
 *******************************************************************************/
#define BME280_CHIP_ID                            (0x60)

/*******************************************************************************
 *                              Includes Needed                                *
 *******************************************************************************/  
/* Standard types */
#include "Std_Types.h"

/* BME_280 Types Header file */
#include "BME_280_Public_Types.h"

/*******************************************************************************
 *                          Public Function Prototype	                       *
 *******************************************************************************/   

/******************************************************************************
 * @fn BME_280_Status BME_280_getInstance(BME_280_Config*)
 * @brief This function will get you an instance of the driver if you get instance
 *        you will have status of BME_280_OK, if not then the instance is either
 *        exists or their is no instance exists
 *
 * @param a_configPtr
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getInstance
(
		BME_280_Config* a_configPtr
);

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
		BME_280_Config* a_configPtr
);


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
);


/******************************************************************************
 * @fn    BME_280_Status BME_280_SoftReset(BME_280_Config*)
 * @brief This function will make a reset to sensor and make it return to
 *        default state.
 *
 * @param  a_configPtr: pointer to configuration structure
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SoftReset
(
		BME_280_Config* a_configPtr
);


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
);


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
);


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
);

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
);

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
);

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
);

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
);


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
		BME_280_OversamplingValue*  a_tempOverSampleValue
);


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
);

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
);

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
);


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
		BME_280_OversamplingValue*  a_humidityOverSampleValue
);


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
		BME_280_TempType_fixedPoint*	a_Temperature
);

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
		BME_280_PressureType_fixedPoint*	a_Pressure
);


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
		BME_280_HumidityType_fixedPoint*	a_Humidity
);

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
		BME_280_TempType_floatingPoint*	    a_Temperature
);

/******************************************************************************
 * @fn BME_280_Status BME_280_getPressure_floatingPoint(BME_280_Config*, sint32*)
 * @brief  output the Pressure in floating point format,The output is in Pa.
 *
 * @param  a_configPtr: pointer to configuration structure
 * @param  a_Pressure: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getPressure_floatingPoint
(
		BME_280_Config* a_configPtr,
		BME_280_PressureType_floatingPoint*	    a_Pressure
);

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
		BME_280_HumidityType_floatingPoint*	  a_Humidity
);


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
);

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
);

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
);

/******************************************************************************
 * @fn BME_280_Status BME_280_getMeasurmentTimeInMsec(BME_280_Config*, uint32*, BME_280_settings*)
 * @brief  This function will return the measurement time taken by the sensor
 *
 * @param  a_config
 * @param  a_measurmentTime: the user enter this parameter and the function will fill it
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_getMeasurmentTimeInMsec
(
		BME_280_Config*   a_configPtr,
		uint32*			  a_measurmentTime
);


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
);



/******************************************************************************
 * @fn BME_280_Status BME_280_setInterfaceProtocol(BME_280_Config*, BME_280_CommunicationProtocol)
 * @brief the user must set the protocol used to interface with the sensor
 *
 * @param a_configPtr
 * @param a_protocol: I2C or SPI
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_setInterfaceProtocol
(
		BME_280_Config*   a_configPtr,
		BME_280_CommunicationProtocol a_protocol
);

/******************************************************************************
 * @fn void BME_280_SetAssertSlaveSelectCallback(void(*)(void))
 * @brief The user must provide the API that set the Slave select pin
 *
 * @param a_configPtr
 * @param a_assertSlaveSelect_ptr: pointer to function to set Slave select API
 * @return status of this procedural
 *******************************************************************************/
BME_280_Status BME_280_SetAssertSlaveSelectCallback
(
		BME_280_Config*   a_configPtr,
		void (*a_assertSlaveSelect_ptr)(void)
);

/******************************************************************************
 * @fn void BME_280_SetReleaseSlaveSelectCallback(void(*)(void))
 * @brief The user must provide the API that reset the Slave select pin
 *
 * @param a_configPtr
 * @param a_releaseSlaveSelect_ptr:: pointer to function to set Slave select API
 *******************************************************************************/
BME_280_Status BME_280_SetReleaseSlaveSelectCallback
(
		BME_280_Config*   a_configPtr,
		void (*a_releaseSlaveSelect_ptr)(void)
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
BME_280_SPI_Status BME_280_SPI_TransmitReceive
(
		uint8 *a_TxAddress,
		uint8 *a_RxBuffer,
		uint16 a_Size,
		uint16 timeout
);


/******************************************************************************
 * @fn void BME_280_Delay(uint32)
 * @brief this function provide delay in ms this function must be implemented
 *        by the user to provide a delay function
 *
 * @param a_delay
 *******************************************************************************/
void BME_280_Delay
(
		uint32 a_delay
);


/******************************************************************************
 * @fn BME_280_I2C_Status BME280_I2C_Master_Transmit(uint8, uint8*, uint16, uint32)
 * @brief
 *
 * @param sensorAddr: Sensor address, assumed to be masked with the R/W bit from the internal API functions
 * @param txData: Pointer to data to be transmitted
 * @param size: Number of bytes to transmit
 * @param timeout: Timeout in milliseconds
 * @return
 *******************************************************************************/
BME_280_I2C_Status BME280_I2C_Master_Transmit
(
		uint8 sensorAddr,
		uint8 *txData,
		uint16 size,
		uint32 timeout
);

/******************************************************************************
 * @fn BME_280_I2C_Status BME280_I2C_Master_Receive(uint8, uint8*, uint16, uint32)
 * @brief
 *
 * @param sensorAddr: Sensor address, assumed to be masked with the R/W bit from the internal API functions
 * @param rxData: Pointer to buffer to receive data in
 * @param size: Number of bytes to receive in rxData
 * @param timeout: Timeout in milliseconds
 * @return
 *******************************************************************************/
BME_280_I2C_Status BME280_I2C_Master_Receive
(
		uint8 sensorAddr,
		uint8 *rxData,
		uint16 size,
		uint32 timeout
);


#endif /* BME_280_H_ */
