/******************************************************************************
 *
 * Module: BME_280
 *
 * File Name: BME_280_Types.h
 *
 * Description: Header file that has all the needed public type decelerations for
 *              BME_280 Driver
 *
 * Author: Marwan Abdelhakim Elmasry
 ******************************************************************************/

#ifndef BME_280_TYPES_H_
#define BME_280_TYPES_H_

/*******************************************************************************
 *                              Includes Needed                                *
 *******************************************************************************/      
/* Standard types */
#include "Std_Types.h"

/*******************************************************************************
 *                             Types Deceleration	                       *
 *******************************************************************************/   

/******************************************************************************
 * @enum  BME_280_CommunicationProtocol
 * @brief This ENUM is used to define what is the interface used the the sensor
 *
 *******************************************************************************/

typedef enum
{
	BME_280_INTERFACE_SPI = ONE,BME_280_INTERFACE_I2C
}BME_280_CommunicationProtocol;


/******************************************************************************
 * @enum  BME_280_OversamplingValue
 * @brief ENUM is used to define a values for Temperature, pressure,
 *        Humidity over-sampling settings
 *
 *******************************************************************************/
typedef enum
{
	SKIPPED,OVERSAMPLING_1,OVERSAMPLING_2,OVERSAMPLING_4,OVERSAMPLING_8,OVERSAMPLING_16
}BME_280_OversamplingValue;

/******************************************************************************
 * @enum  BME_280_Mode
 * @brief This ENUM is used to set the mode of operation of BME sensor
 *
 *******************************************************************************/
typedef enum
{
	SLEEP_MODE , FORCED_MODE , NORMAL_MODE = 3
}BME_280_Mode;

/******************************************************************************
 * @enum  BME_280_FilterCoefficient
 * @brief This ENUM represents the values of filter coefficient in the sensor
 *
 *******************************************************************************/
typedef enum
{
	FILTER_OFF, _2, _4 , _8 , _16
}BME_280_FilterCoefficient;

/******************************************************************************
 * @enum FlagStatus
 * @brief  this enum is uset to set values of set and reset
 *******************************************************************************/

typedef enum
{
  _RESET = 0,
  _SET = !_RESET
} BME280_FlagStatus;

/******************************************************************************
 * @enum  BME_280_StandbyTimeInNormalMode
 * @brief This ENUM represents the values of standby time in case of using normal mode
 *
 *******************************************************************************/
typedef enum
{
	_HALF_MS, _62_AND_HALF_MS, _125_MS, _250_MS, _500_MS,
	_1000_MS, _10_MS, _20_MS , NO_STANDBY_TIME_NOT_IN_NORMAL_MODE
}BME_280_StandbyTimeInNormalMode;


/******************************************************************************
 * @enum BME280_Result
 * @brief Enum for API return status states
 *
 *******************************************************************************/
typedef enum {
	BME280_OK = 0,
	BME280_NULL_ERROR = 0xFF,
	BME280_COMM_ERROR = 0x0A,
	BME280_ERROR,
	BME280_NOT_OK,
	BME280_ERROR_INSTANCE_ALEARDY_TAKEN,
	BME280_NO_INSTANCES_AVALIABLE,
	BME280_INTERFACE_UNKNOWN,
	BME280_NOT_INITIALIZED,
	BME280_ERROR_CANNOT_SET_T_SB_NOT_IN_NORMAL_MODE,
	BME280_ERROR_CANNOT_GET_T_SB_NOT_IN_NORMAL_MODE,
	BME280_EXCEEDED_MIN_RANGE,
	BME280_EXCEEDED_MAX_RANGE,
	BME280_SETTINGS_FAILED_TO_BE_SET,
	BME280_SPI_TRANSIMISSION_FAILED,
	BME280_I2C_TRANSIMISSION_FAILED,
	BME280_SET_CS_FAILED,
	BME280_RESET_CS_FAILED,
	BME280_CALL_BACK_IS_NULL_ERROR,
	BME280_CANNOT_HAVE_MORE_THAN_ONE_INSTANCE_OF_I2C,
	BME280_WEAK_FUNCTION_NOT_IMPLEMENTED
} BME_280_Status;


/******************************************************************************
 * @enum    BME_280_SPI_Status
 * @brief   the user will supply this status to the driver
 *
 *******************************************************************************/
typedef enum
{
	SPI_TRANSIMISSION_SUCCESSED = 9,
	SPI_TRANSIMISSION_FAILED
}BME_280_SPI_Status;

/******************************************************************************
 * @enum    BME_280_I2C_Status
 * @brief   the user will supply this status to the driver
 *
 *******************************************************************************/
typedef enum
{
	I2C_TRANSIMISSION_SUCCESSED = 9,
	I2C_TRANSIMISSION_FAILED
}BME_280_I2C_Status;


/******************************************************************************
 * @brief: Type defines for temperature/pressure/humidity in Fixed point
 *******************************************************************************/
typedef 	sint32  BME_280_TempType_fixedPoint;
typedef 	uint32  BME_280_PressureType_fixedPoint;
typedef 	uint32  BME_280_HumidityType_fixedPoint;

/******************************************************************************
 * @brief: Type defines for temperature/pressure/humidity in Floating point
 *******************************************************************************/
typedef 	float64 BME_280_TempType_floatingPoint;
typedef 	float64 BME_280_PressureType_floatingPoint;
typedef 	float64 BME_280_HumidityType_floatingPoint;



/******************************************************************************
 * @struct BME_280_OversamplingSettings
 * @brief  This structure will encapsulate the over-sampling values for Temperature,
 *         pressure, Humidity over-sampling.
 *******************************************************************************/
typedef struct
{
	BME_280_OversamplingValue Temperture;
	BME_280_OversamplingValue humidity;
	BME_280_OversamplingValue Pressure;
}BME_280_OversamplingSettings;



/******************************************************************************
 * @struct BME_280_settings;
 * @brief  struct for all the setting of the sensor
 *******************************************************************************/
typedef struct
{
	BME_280_OversamplingSettings     overSampling_settings;
	BME_280_Mode				     mode;
	BME_280_StandbyTimeInNormalMode  standbyTime;
	BME_280_FilterCoefficient        filterCoeff;
}BME_280_settings;


/******************************************************************************
 * This is pointer to configurations structure that the user will supply
 *******************************************************************************/
typedef struct BME_280_Configurations *BME_280_Config;




#endif /* BME_280_TYPES_H_ */
