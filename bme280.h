/******************************************************************************
 *
 * Module: Bosch Sensortec BME280
 *
 * File Name: bme280.h
 *
 * Description: Header file for BME280 Sensor API functions.
 *
 * Date Created: 12/9/2022
 *
 * Author: Hazem Montasser
 *
 *******************************************************************************/

#ifndef BME280_DRIVER_BME280_H_
#define BME280_DRIVER_BME280_H_

/*******************************************************************************
 *                          Typedefs		                                   *
 *******************************************************************************/

/**
 * @def BME280_Handle
 * @brief  BME280 API handle
 */
typedef struct BME280_ConfigType *BME280_Handle;

/**
 * @def BME280_boolean
 * @brief 8 bit unsigned integer number
 */
typedef unsigned char BME280_boolean;

/**
 * @def BME280_uint8
 * @brief 8 bit unsigned integer number
 */
typedef unsigned char BME280_uint8;

/**
 * @def BME280_sint8
 * @brief 8 bit signed integer number
 */
typedef unsigned char BME280_sint8;

/**
 * @def BME280_uint16
 * @brief 16 bit unsigned integer number
 */
typedef unsigned short BME280_uint16;

/**
 * @def BME280_sint16
 * @brief 16 bit signed integer number
 */
typedef signed short BME280_sint16;

/**
 * @def BME280_uint32
 * @brief 32 bit unsigned integer number
 */
typedef unsigned long BME280_uint32;

/**
 * @def BME280_sint32
 * @brief 32 bit signed integer number
 */
typedef signed long BME280_sint32;

/**
 * @def BME280_float64
 * @brief 64 bit floating point number
 */
typedef double BME280_float64;

/**
 * @enum BME280_StandbyTime
 * @brief Enum for available standby times used in configuration
 *
 */
typedef enum {
	BME280_T_STDBY_0_5_MS = (0x00), /**< 0.5ms*/
	BME280_T_STDBY_62_5_MS = (0x01), /**< 62.5ms*/
	BME280_T_STDBY_125_MS = (0x02), /**< 125ms*/
	BME280_T_STDBY_250_MS = (0x03), /**< 250ms*/
	BME280_T_STDBY_500_MS = (0x04), /**< 500ms*/
	BME280_T_STDBY_1000_MS = (0x05), /**< 1000ms*/
	BME280_T_STDBY_10_MS = (0x06), /**< 10ms*/
	BME280_T_STDBY_20_MS = (0x07), /**< 20ms*/
	BME280_T_STDBY_NOT_SPECIFIED = (0xFF)
} BME280_StandbyTime;

/**
 * @enum BME280_FilterCoeff
 * @brief Enum for available IIR filter coefficients used in configuration
 */
typedef enum {
	BME280_FILTER_COEFF_OFF = (0x00), /**< Filter off*/
	BME280_FILTER_COEFF_2 = (0x02), /**<  2		*/
	BME280_FILTER_COEFF_4 = (0x04), /**<  4		*/
	BME280_FILTER_COEFF_8 = (0x08), /**<  8		*/
	BME280_FILTER_COEFF_16 = (0x10), /**<  16		*/
	BME280_FILTER_NOT_SPECIFIED = (0xFF)
} BME280_FilterCoeff;

/******************************************************************************
 * @enum BME280_Oversampling_setting
 * @brief Enum for available oversampling settings
 *******************************************************************************/
typedef enum {
	BME280_OVERSAMPLING_OFF = 0x00, /**< Skipped measurement */
	BME280_OVERSAMPLING_x1 = 0x01, /**<  x1		*/
	BME280_OVERSAMPLING_x2 = 0x02, /**<  x2		*/
	BME280_OVERSAMPLING_x4 = 0x03, /**<  x4		*/
	BME280_OVERSAMPLING_x8 = 0x04, /**<  x8		*/
	BME280_OVERSAMPLING_x16 = 0x05, /**< x16	*/
	BME280_OVERSAMPLING_NOT_SPECIFIED = (0xFF)
} BME280_Oversampling_setting;

/**
 * @enum BME280_Status
 * @brief Enum for API return status states
 *******************************************************************************/
typedef enum {

	BME280_OK = 0, /**< General code for operation success */
	BME280_IS_INSTANCE = 0xF1, /**< Code indicating that the handle is already an instance in the sensor pool */
	BME280_FOUND_EMPTY_INSTANCE = 0x50, /**< Code indicating that there is an empty instance found in the sensor pool */

	BME280_NULL_ERROR = 0xFF,/**< General code for null arguments to functions */
	BME280_COMM_ERROR = 0x0A,/**< General code for communication failure,
	 not to be confused with users' BME280_Comm_Error
	 in BME280_Comm_Status which the user uses to implement weak functions
	 */
	BME280_NOT_YET_OBTAINED = 0x3F, /**< General code used for status flag when the result has not been obtained yet through API functions */
	BME280_NOT_IMPLEMENTED = 0xAC, /**< General error code for when weak functions are not implemented by the user*/
	BME280_POOL_FULL = 0x94, /**< Error code indicating that the sensor pool is full */
	BME280_NOT_INSTANCE = 0x43, /**< Error code indicating that the handle is not an existing instance in the pool*/
	BME280_NO_INTERFACE_SPECIFIED = 0x58, /**< Error code indicating that there was no communication interface specified, see BME280_setInterfaceType function */
	BME280_SETTING_FAILED = 0x62, /**< General error code returned if a sensor setting was not set and validated */
	BME280_CALLBACK_NOT_SET = 0xBD /**< Error code when callbacks for GPIO functions are not set by the user*/
} BME280_Status;

/**
 * @enum BME280_Comm_Status
 * @brief Used by the user to return status if communication failed or succeeded
 * 		based on target implementation. Should be returned through weak functions to be
 * 		implemented by the user.
 *
 */
typedef enum {
	BME280_Comm_Error,/**< Communication occurred during transmission */
	BME280_Comm_OK /**< Communication success */
} BME280_Comm_Status;

/**
 * @enum BME280_ModeType
 * @brief Enum for sensor mode (sleep, normal, forced) used in configuration
 *******************************************************************************/
typedef enum {
	BME280_Mode_Sleep = 0b00,
	BME280_Mode_Forced = 0b10,
	BME280_Mode_Normal = 0b11,
	BME280_Mode_Not_Specified = (0xFF)
} BME280_ModeType;

/**
 * @enum BME280_MeasuringStatus
 * @brief Indicates if the sensor is measuring or has finished measuring
 *
 */
typedef enum {
	BME280_Measuring_Finished = 0x00, BME280_Measuring_Running = 0x01
} BME280_MeasuringStatus;

/**
 * @enum BME280_UpdateStatus
 * @brief Indicates if the sensor is copying data to registers or has finished
 * 		copying
 *
 */
typedef enum {
	BME280_Update_Finished = 0x00, BME280_Update_Copying = 0x01
} BME280_UpdateStatus;

/**
 * @enum BME280_InterfaceType
 * @brief Enum for interface used by the sensor
 */
typedef enum {
	BME280_Interface_SPI = 0, /**< Selected SPI interface */
	BME280_Interface_I2C = 1, /**< Selected I2C interface */
	BME280_Interface_Not_Specified = (0xFF)
} BME280_InterfaceType;

/**
 * @struct BME280_Settings
 * @brief Struct that contains sensor user settings
 *
 * 1. BME280_StandbyTime t_stby: 		  Standby time between measurements
 * 2. BME280_FilterCoeff filter: 		  IIR filter coefficient
 * 3. BME280_Oversampling_setting osrs_t: Over-sampling multiplier for temperature
 * 4. BME280_Oversampling_setting osrs_p: Over-sampling multiplier for temperature
 * 5. BME280_Oversampling_setting osrs_h: Over-sampling multiplier for humidity
 * 6. BME280_ModeType Mode: 	 		  Sensor mode (sleep, forced, normal)
 *
 */
typedef struct {
	BME280_ModeType Mode; /**< Specifies the current mode for BME280 */
	BME280_StandbyTime t_stby; /**< Standby time of sensor*/
	BME280_FilterCoeff filter; /**< Filter coefficient*/
	BME280_Oversampling_setting osrs_t;/**< Temperature over-sampling setting*/
	BME280_Oversampling_setting osrs_p;/**< Pressure over-sampling setting*/
	BME280_Oversampling_setting osrs_h;/**< Humidityover-sampling setting*/
} BME280_Settings;

/*******************************************************************************
 *                          Public function prototypes                         *
 *******************************************************************************/

/**
 * @fn BME280_Status BME280_getInstance(BME280_Handle*)
 * @brief Obtains an instance from the sensor pool if there is an available instance.
 * 		Instance is uninitialized and must be initialized by calling BME280_init function
 * 		to verify and initialize the sensor.
 *
 * 		__Note: If handle is already an occupied instance, the handle is unchanged.
 *
 * @param a_cfgPtr	Pointer to sensor handle
 * @return Instance for an available sensor in the pool.
 */
BME280_Status BME280_getInstance(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_setInterfaceType(BME280_Handle*, BME280_InterfaceType)
 * @brief API to set interface type (I2C/SPI) before initializing the sensor. Must be set
 * 		according to the used interface before calling any sensor functions.
 *
 * @param a_cfgPtr 	Pointer to sensor handle.
 * @param a_intf	Interface type selected. See BME280_InterfaceType enum.
 * @return
 */
BME280_Status BME280_setInterfaceType(BME280_Handle *a_cfgPtr,
		BME280_InterfaceType a_intf);

/**
 * @fn BME280_Status BME280_init(BME280_Handle *a_cfgPtr)
 * @brief Initialization function for the BME280 sensor
 * 		  - Initializes sensor by reading its chip ID and soft-resetting it
 *		  - Returns an empty handle for a new sensor (if pool limit is not reached)
 * @param a_cfgPtr		 Pointer to the sensor configuration struct
 * @return
 */
BME280_Status BME280_init(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_softReset(BME280_Handle *a_cfgPtr)
 * @brief Soft-resets the sensor by writing the reset byte into
 * reset register

 * @param a_cfgPtr: Pointer to the sensor configuration struct
 * @return BME280_Status
 */
BME280_Status BME280_softReset(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_uint16 BME280_calculateMeasurementDelayMs(BME280_Settings*)
 * @brief Calculates measurement time needed by the sensor to
 * 		finish conversion based on the selected settings.
 *
 * @param a_settings	Pointer to the settings struct which contains
 * 				- Filter coefficient
 * 				- Standby time
 * 				- Sensor mode
 * 				- Over-sampling settings for all parameters
 * @return
 */
BME280_uint16 BME280_calculateMeasurementDelayMs(BME280_Settings *a_settings);

/**
 * @fn BME280_uint8 BME280_getChipID(BME280_Handle*)
 * @brief Reads chip ID from the sensor.
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @param a_chipID Pointer to variable to receive the ID in
 * @return
 */
BME280_uint8 BME280_getChipID(BME280_Handle *a_cfgPtr, BME280_uint8 *a_chipID);

/**
 * @fn BME280_Status BME280_getTemperature_floatingPoint(BME280_Handle*, BME280_float64*)
 * @brief
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_temperature	Pointer to variable to store the temperature in as floating point (double)
 * @return
 */
BME280_Status BME280_getTemperature_floatingPoint(BME280_Handle *a_cfgPtr,
		BME280_float64 *a_temperature);

/**
 * @fn BME280_Status BME280_getPressure_floatingPoint(BME280_Handle*, BME280_float64*)
 * @brief
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_pressure 		Pointer to variable to store the pressure in as floating point (double)
 * @return
 */
BME280_Status BME280_getPressure_floatingPoint(BME280_Handle *a_cfgPtr,
		BME280_float64 *a_pressure);

/**
 * @fn BME280_Status BME280_getHumidity_floatingPoint(BME280_Handle*, BME280_float64*)
 * @brief
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_humidity		Pointer to variable to store the humidity in as floating point (double)
 * @return
 */
BME280_Status BME280_getHumidity_floatingPoint(BME280_Handle *a_cfgPtr,
		BME280_float64 *a_humidity);

/**
 * @fn BME280_Status BME280_getTemperature_fixedPoint(BME280_Handle*, BME280_sint32*)
 * @brief
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_temperature	Pointer to variable to store the temperature in as fixed point (signed 32 bit int)
 * @return
 */
BME280_Status BME280_getTemperature_fixedPoint(BME280_Handle *a_cfgPtr,
		BME280_sint32 *a_temperature);

/**
 * @fn BME280_Status BME280_getPressure_fixedPoint(BME280_Handle*, BME280_uint32*)
 * @brief
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_pressure 		Pointer to variable to store the pressure in as fixed point (unsigned 32 bit int)
 * @return
 */
BME280_Status BME280_getPressure_fixedPoint(BME280_Handle *a_cfgPtr,
		BME280_uint32 *a_pressure);

/**
 * @fn BME280_Status BME280_getHumidity_fixedPoint(BME280_Handle*, BME280_uint32*)
 * @brief
 *
 * @param a_cfgPtr		Pointer to sensor handle
 * @param a_humidity		Pointer to variable to store the humidity in as fixed point (unsigned 32 bit int)
 * @return
 */
BME280_Status BME280_getHumidity_fixedPoint(BME280_Handle *a_cfgPtr,
		BME280_uint32 *a_humidity);

/* Getter functions for registers and settings */

/**
 * @fn BME280_Status BME280_getUpdateStatus(BME280_Handle*, BME280_UpdateStatus*)
 * @brief
 *
 * @param a_cfgPtr 		Pointer to sensor handle
 * @param a_updateFlag	Pointer to update status variable which will contain the update status
 * @return
 */
BME280_Status BME280_getUpdateStatus(BME280_Handle *a_cfgPtr,
		BME280_UpdateStatus *a_updateFlag);

/**
 * @fn BME280_Status BME280_getMeasuringStatus(BME280_Handle*, BME280_MeasuringStatus*)
 * @brief
 *
 * @param a_cfgPtr 		Pointer to sensor handle
 * @param a_measureFlag	Pointer to update status variable which will contain the measuring status
 * @return
 */
BME280_Status BME280_getMeasuringStatus(BME280_Handle *a_cfgPtr,
		BME280_MeasuringStatus *a_measureFlag);

/**
 * @fn BME280_Status BME280_getMode(BME280_Handle*, BME280_ModeType*)
 * @brief
 *
 * @param a_cfgPtr 		Pointer to sensor handle
 * @param a_mode		Pointer to measure status variable which will contain the sensor mode
 * @return
 */
BME280_Status BME280_getMode(BME280_Handle *a_cfgPtr, BME280_ModeType *a_mode);

/**
 * @fn BME280_Status BME280_getSensorSettings(BME280_Handle*, BME280_Settings*)
 * @brief
 *
 * @param a_cfgPtr	Pointer to sensor handle
 * @param a_settings Pointer to settings variable which will be filled with current sensor settings
 * @return
 */
BME280_Status BME280_getSensorSettings(BME280_Handle *a_cfgPtr,
		BME280_Settings *a_settings);

/**
 * @fn BME280_Status BME280_getTemperatureOversampling(BME280_Handle*, BME280_Oversampling_setting*)
 * @brief
 *
 * @param a_cfgPtr 		Pointer to sensor handle
 * @param a_oversampling	Pointer to variable which will contain the temperature oversampling setting
 * @return
 */
BME280_Status BME280_getTemperatureOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting *a_oversampling);

/**
 * @fn BME280_Status BME280_getPressureOversampling(BME280_Handle*, BME280_Oversampling_setting*)
 * @brief
 *
 * @param a_cfgPtr 		Pointer to sensor handle
 * @param a_oversampling	Pointer to variable which will contain the pressure oversampling setting
 * @return
 */
BME280_Status BME280_getPressureOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting *a_oversampling);

/**
 * @fn BME280_Status BME280_getHumidityOversampling(BME280_Handle*, BME280_Oversampling_setting*)
 * @brief
 *
 * @param a_cfgPtr 		Pointer to sensor handle
 * @param a_oversampling	Pointer to variable which will contain the humidity oversampling setting
 * @return
 */
BME280_Status BME280_getHumidityOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting *a_oversampling);

/**
 * @fn BME280_Status BME280_getFilterCoefficient(BME280_Handle*, BME280_FilterCoeff*)
 * @brief
 *
 * @param a_cfgPtr 		Pointer to sensor handle
 * @param a_filterCoeff	Pointer to variable which will contain the filter coefficient setting
 * @return
 */
BME280_Status BME280_getFilterCoefficient(BME280_Handle *a_cfgPtr,
		BME280_FilterCoeff *a_filterCoeff);

/**
 * @fn BME280_Status BME280_getStandbyTime(BME280_Handle*, BME280_StandbyTime*)
 * @brief
 *
 * @param a_cfgPtr 		Pointer to sensor handle
 * @param a_standbyTime	Pointer to variable which will contain the sensor standby time
 * @return
 */
BME280_Status BME280_getStandbyTime(BME280_Handle *a_cfgPtr,
		BME280_StandbyTime *a_standbyTime);

/* Setter functions for registers and settings */

/**
 * @fn BME280_Status BME280_setPressureOversampling(BME280_Handle*, BME280_Oversampling_setting)
 * @brief
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @param a_pressureOversampling
 * @return Sets pressure over-sampling setting and status flag if operation succeeded
 */
BME280_Status BME280_setPressureOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting a_pressureOversampling);

/**
 * @fn BME280_Status BME280_setTemperatureOversampling(BME280_Handle*, BME280_Oversampling_setting)
 * @brief
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @param a_temperatureOversampling
 * @return Sets temperature over-sampling setting and status flag if operation succeeded
 */
BME280_Status BME280_setTemperatureOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting a_temperatureOversampling);
/**
 * @fn BME280_Status BME280_setHumidityOversampling(BME280_Handle*, BME280_Oversampling_setting)
 * @brief
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @param a_humidityOversampling
 * @return Sets humidity over-sampling setting and status flag if operation succeeded
 */
BME280_Status BME280_setHumidityOversampling(BME280_Handle *a_cfgPtr,
		BME280_Oversampling_setting a_humidityOversampling);

/**
 * @fn BME280_Status BME280_setStandbyTime(BME280_Handle*, BME280_StandbyTime)
 * @brief
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @param a_standbyTime
 * @return Sets standby time and status flag if operation succeeded
 */
BME280_Status BME280_setStandbyTime(BME280_Handle *a_cfgPtr,
		BME280_StandbyTime a_standbyTime);

/**
 * @fn BME280_Status BME280_setMode(BME280_Handle*, BME280_ModeType)
 * @brief
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @param a_mode
 * @return Sets mode and status flag if operation succeeded
 */
BME280_Status BME280_setMode(BME280_Handle *a_cfgPtr, BME280_ModeType a_mode);

/**
 * @fn BME280_Status BME280_setFilterCoefficient(BME280_Handle*, BME280_FilterCoeff)
 * @brief
 *
 * @param a_cfgPtr Pointer to sensor handle
 * @param a_filterCoeff
 * @return	Sets filter coefficient and status flag if operation succeeded
 */
BME280_Status BME280_setFilterCoefficient(BME280_Handle *a_cfgPtr,
		BME280_FilterCoeff a_filterCoeff);

/**
 * @fn BME280_Status BME280_DeInit(BME280_Handle*)
 * @brief	De-initializes sensor for the passed handle.
 * 			De-init sequence is:
 *
 * 			1. Set sensor mode to sleep mode
 * 			2. Soft reset the sensor to restore default register values
 * 			3. Lets handle point to NULL.
 *
 * 		 	__Note: Does nothing if the handle is NULL or already de-initialized.
 *
 * @param a_cfgPtr
 * @return	a_cfgPtr is NULL and status flag
 */
BME280_Status BME280_DeInit(BME280_Handle *a_cfgPtr);

/**
 * @fn BME280_Status BME280_setAssertNSSCallback(BME280_Handle*, void(*)(void))
 * @brief
 *
 * @param a_cfgPtr
 * @param a_callback
 * @return
 */
BME280_Status BME280_setAssertNSSCallback(BME280_Handle *a_cfgPtr,
		void (*a_callback)(void));

/**
 * @fn BME280_Status BME280_setReleaseNSSCallback(BME280_Handle*, void(*)(void))
 * @brief
 *
 * @param a_cfgPtr
 * @param a_callback
 * @return
 */
BME280_Status BME280_setReleaseNSSCallback(BME280_Handle *a_cfgPtr,
		void (*a_callback)(void));

/**
 * @fn BME280_Comm_Status BME280_SPI_TransmitReceive(BME280_uint8*, BME280_uint8*, BME280_uint16, BME280_uint16)
 * @brief SPI transmit and receive function used to read from SPI interface. Implementation
 * 		is up to the user and target hardware.
 *
 *		** Must be implemented by the user. **
 *
 * @param txData  Pointer to data to be transmitted.
 * @param rxData  Pointer to buffer to receive data in.
 * @param size    Number of bytes to send & receive.
 * @param timeout Timeout in milliseconds
 * @return BME280_Comm_Status
 */
extern BME280_Comm_Status BME280_SPI_TransmitReceive(BME280_uint8 *txData,
		BME280_uint8 *rxData, BME280_uint16 size, BME280_uint32 timeout);

/**
 * @fn BME280_Status BME280_delayMs(BME280_uint16)
 * @brief Function that implements the delay passed to it in milliseconds. Used in
 * 		internal communication sensor functions as some functions need delay.
 *
 *		** Must be implemented by the user. **
 *
 * @param a_milliseconds
 */
extern BME280_Status BME280_delayMs(BME280_uint32 a_milliseconds);

/**
 * @fn BME280_Status BME280_I2C_Master_Transmit(BME280_uint8, BME280_uint8*, BME280_uint16, BME280_uint32)
 * @brief
 *
 * @param sensorAddr	Sensor address, assumed to be masked with the R/W bit from the internal API functions
 * @param txData		Pointer to data to be transmitted
 * @param size			Number of bytes to transmit
 * @param timeout		Timeout in milliseconds
 * @return
 */
extern BME280_Status BME280_I2C_Master_Transmit(BME280_uint8 sensorAddr,
		BME280_uint8 *txData, BME280_uint16 size, BME280_uint32 timeout);

/**
 * @fn BME280_Status BME280_I2C_Master_Receive(BME280_uint8, BME280_uint8*, BME280_uint16, BME280_uint32)
 * @brief
 *
 * @param sensorAddr	Sensor address, assumed to be masked with the R/W bit from the internal API functions
 * @param rxData		Pointer to buffer to receive data in
 * @param size			Number of bytes to receive in rxData
 * @param timeout		Timeout in milliseconds
 * @return
 */
extern BME280_Status BME280_I2C_Master_Receive(BME280_uint8 sensorAddr,
		BME280_uint8 *rxData, BME280_uint16 size, BME280_uint32 timeout);

#endif /* BME280_DRIVER_BME280_H_ */
