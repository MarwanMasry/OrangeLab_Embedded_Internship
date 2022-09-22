/**
 * @def BME280_DRIVER_BME280_H_
 * @brief BME280 driver header file for API function declarations.
 *
 */
#ifndef BME280_DRIVER_BME280_H_
#define BME280_DRIVER_BME280_H_

/*******************************************************************************
 *                          Includes		                                   *
 *******************************************************************************/

#include <std_types.h>

/*******************************************************************************
 *                          Definitions		                                   *
 *******************************************************************************/

#define BME280_FEATURE_ENABLE 1
#define BME280_FEATURE_DISABLE 0

#define BME280_FLOATING_POINT BME280_FEATURE_ENABLE
#define BME280_FIXED_POINT BME280_FEATURE_ENABLE

/*******************************************************************************
 *                          Typedefs		                                   *
 *******************************************************************************/

/**
 * BME280 API handle
 */
typedef struct BME280_ConfigType *BME280_Handle;

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
	BME280_T_STDBY_20_MS = (0x07) /**< 20ms*/
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
	BME280_FILTER_COEFF_16 = (0x10) /**<  16		*/
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
} BME280_Oversampling_setting;

/**
 * @enum BME280_Result
 * @brief Enum for API return status states
 *******************************************************************************/
typedef enum {
	/** Communication state codes
	 */
	BME280_OK = 0,
	BME280_NULL_ERROR = 0xFF,
	BME280_COMM_ERROR = 0x0A,
	BME280_NOT_YET_OBTAINED = 0x3F,
	BME280_NOT_IMPLEMENTED = 0xAC,
	/**
	 *  Error codes related to handler
	 */
	BME280_POOL_FULL = 0x94,
	BME280_NOT_INSTANCE = 0x43,
	BME280_IS_INSTANCE = 0xF1,
	BME280_FOUND_EMPTY_INSTANCE = 0x50,
	BME280_NO_INTERFACE_SPECIFIED = 0x58,
	/**
	 * Sensor related errors and states
	 */
	BME280_SETTING_FAILED = 0x62,
} BME280_Result;

/**
 * @enum BME280_ModeType
 * @brief Enum for sensor mode (sleep, normal, forced) used in configuration
 *******************************************************************************/
typedef enum {
	BME280_Mode_Sleep = 0b00,
	BME280_Mode_Forced = 0b10,
	BME280_Mode_Normal = 0b11
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
 * @struct BME280_Settings
 * @brief Struct that contains sensor user settings
 *
 * 1. BME280_StandbyTime t_stby: 		  Standby time between measurements
 * 2. BME280_FilterCoeff filter: 		  IIR filter coefficient
 * 3. BME280_Oversampling_setting osrs_t: Oversampling multiplier for temperature
 * 4. BME280_Oversampling_setting osrs_p: Oversampling multiplier for temperature
 * 5. BME280_Oversampling_setting osrs_h: Oversampling multiplier for humidity
 * 6. BME280_ModeType Mode: 	 		  Sensor mode (sleep, forced, normal)
 *
 */
typedef struct {
	BME280_ModeType Mode; /* Specifies the current mode for BME280 */
	BME280_StandbyTime t_stby; /* Standby time of sensor*/
	BME280_FilterCoeff filter;
	BME280_Oversampling_setting osrs_t;
	BME280_Oversampling_setting osrs_p;
	BME280_Oversampling_setting osrs_h;
} BME280_Settings;

/*******************************************************************************
 *                          Public function prototypes                         *
 *******************************************************************************/

/**
 * @fn BME280_Result BME280_init(BME280_Handle *cfgPtr)
 * @brief Initialization function for the BME280 sensor
 * 		  - Initializes sensor by reading its chip ID and soft-resetting it
 *		  - Returns an empty handle for a new sensor (if pool limit is not reached)
 * @param cfgPtr		 Pointer to the sensor configuration struct
 * @param interfaceType  Sensor interface type from BME280_InterfaceType enumerations
 * @return BME280_Result
 */
BME280_Result BME280_init(
		BME280_Handle *cfgPtr, BME280_InterfaceType interfaceType);

/**
 * @fn BME280_Result BME280_softReset(BME280_Handle *cfgPtr)
 * @brief Soft-resets the sensor by writing the reset byte into
 * reset register

 * @param cfgPtr: Pointer to the sensor configuration struct
 * @return BME280_Result
 */
BME280_Result BME280_softReset(
		BME280_Handle *cfgPtr);

/**
 * @fn uint16 BME280_calculateMeasurementDelayMs(BME280_Settings*)
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
uint16 BME280_calculateMeasurementDelayMs(
		BME280_Settings *a_settings);

/**
 * @fn uint8 BME280_getChipID(BME280_Handle*)
 * @brief Reads chip ID from the sensor.
 *
 * @param cfgPtr Pointer to sensor handle
 * @param chipID Pointer to variable to receive the ID in
 * @return
 */
uint8 BME280_getChipID(
		BME280_Handle *cfgPtr,
		uint8 *chipID);

#if BME280_FLOATING_POINT ==BME280_FEATURE_ENABLE

/**
 * @fn BME280_Result BME280_getTemperature_floatingPoint(BME280_Handle*, float64*)
 * @brief
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param temperature	Pointer to variable to store the temperature in as floating point (double)
 * @return
 */
BME280_Result BME280_getTemperature_floatingPoint(
		BME280_Handle *cfgPtr,
		float64 *temperature);

/**
 * @fn BME280_Result BME280_getPressure_floatingPoint(BME280_Handle*, float64*)
 * @brief
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param pressure 		Pointer to variable to store the pressure in as floating point (double)
 * @return
 */
BME280_Result BME280_getPressure_floatingPoint(
		BME280_Handle *cfgPtr,
		float64 *pressure);

/**
 * @fn BME280_Result BME280_getHumidity_floatingPoint(BME280_Handle*, float64*)
 * @brief
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param humidity		Pointer to variable to store the humidity in as floating point (double)
 * @return
 */
BME280_Result BME280_getHumidity_floatingPoint(
		BME280_Handle *cfgPtr,
		float64 *humidity);
#endif

#if BME280_FIXED_POINT == BME280_FEATURE_ENABLE
/**
 * @fn BME280_Result BME280_getTemperature_fixedPoint(BME280_Handle*, sint32*)
 * @brief
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param temperature	Pointer to variable to store the temperature in as fixed point (signed 32 bit int)
 * @return
 */
BME280_Result BME280_getTemperature_fixedPoint(
		BME280_Handle *cfgPtr,
		sint32 *temperature);

/**
 * @fn BME280_Result BME280_getPressure_fixedPoint(BME280_Handle*, uint32*)
 * @brief
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param pressure 		Pointer to variable to store the pressure in as fixed point (unsigned 32 bit int)
 * @return
 */
BME280_Result BME280_getPressure_fixedPoint(
		BME280_Handle *cfgPtr,
		uint32 *pressure);

/**
 * @fn BME280_Result BME280_getHumidity_fixedPoint(BME280_Handle*, uint32*)
 * @brief
 *
 * @param cfgPtr		Pointer to sensor handle
 * @param humidity		Pointer to variable to store the humidity in as fixed point (unsigned 32 bit int)
 * @return
 */
BME280_Result BME280_getHumidity_fixedPoint(
		BME280_Handle *cfgPtr,
		uint32 *humidity);
#endif
/* Getter functions for registers and settings */

/**
 * @fn BME280_Result BME280_getUpdateStatus(BME280_Handle*, BME280_UpdateStatus*)
 * @brief
 *
 * @param cfgPtr 		Pointer to sensor handle
 * @param updateFlag	Pointer to update status variable which will contain the update status
 * @return
 */
BME280_Result BME280_getUpdateStatus(
		BME280_Handle *cfgPtr,
		BME280_UpdateStatus *updateFlag);

/**
 * @fn BME280_Result BME280_getMeasuringStatus(BME280_Handle*, BME280_MeasuringStatus*)
 * @brief
 *
 * @param cfgPtr 		Pointer to sensor handle
 * @param measureFlag	Pointer to update status variable which will contain the measuring status
 * @return
 */
BME280_Result BME280_getMeasuringStatus(
		BME280_Handle *cfgPtr,
		BME280_MeasuringStatus *measureFlag);

/**
 * @fn BME280_Result BME280_getMode(BME280_Handle*, BME280_ModeType*)
 * @brief
 *
 * @param cfgPtr 		Pointer to sensor handle
 * @param mode			Pointer to measure status variable which will contain the sensor mode
 * @return
 */
BME280_Result BME280_getMode(
		BME280_Handle *cfgPtr,
		BME280_ModeType *mode);


/**
 * @fn BME280_Result BME280_getSensorSettings(BME280_Handle*, BME280_Settings*)
 * @brief
 *
 * @param cfgPtr
 * @param settings
 * @return
 */
BME280_Result BME280_getSensorSettings(
		BME280_Handle *cfgPtr,
		BME280_Settings *settings);

/**
 * @fn BME280_Result BME280_getTemperatureOversampling(BME280_Handle*, BME280_Oversampling_setting*)
 * @brief
 *
 * @param cfgPtr 		Pointer to sensor handle
 * @param oversampling	Pointer to variable which will contain the temperature oversampling setting
 * @return
 */
BME280_Result BME280_getTemperatureOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting *oversampling);

/**
 * @fn BME280_Result BME280_getPressureOversampling(BME280_Handle*, BME280_Oversampling_setting*)
 * @brief
 *
 * @param cfgPtr 		Pointer to sensor handle
 * @param oversampling	Pointer to variable which will contain the pressure oversampling setting
 * @return
 */
BME280_Result BME280_getPressureOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting *oversampling);

/**
 * @fn BME280_Result BME280_getHumidityOversampling(BME280_Handle*, BME280_Oversampling_setting*)
 * @brief
 *
 * @param cfgPtr 		Pointer to sensor handle
 * @param oversampling	Pointer to variable which will contain the humidity oversampling setting
 * @return
 */
BME280_Result BME280_getHumidityOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting *oversampling);

/**
 * @fn BME280_Result BME280_getFilterCoefficient(BME280_Handle*, BME280_FilterCoeff*)
 * @brief
 *
 * @param cfgPtr 		Pointer to sensor handle
 * @param filterCoeff	Pointer to variable which will contain the filter coefficient setting
 * @return
 */
BME280_Result BME280_getFilterCoefficient(
		BME280_Handle *cfgPtr,
		BME280_FilterCoeff *filterCoeff);

/**
 * @fn BME280_Result BME280_getStandbyTime(BME280_Handle*, BME280_StandbyTime*)
 * @brief
 *
 * @param cfgPtr 		Pointer to sensor handle
 * @param standbyTime	Pointer to variable which will contain the sensor standby time
 * @return
 */
BME280_Result BME280_getStandbyTime(
		BME280_Handle *cfgPtr,
		BME280_StandbyTime *standbyTime);

/* Setter functions for registers and settings */

/**
 * @fn BME280_Result BME280_setPressureOversampling(BME280_Handle*, BME280_Oversampling_setting)
 * @brief
 *
 * @param cfgPtr Pointer to sensor handle
 * @param pressureOversampling
 * @return
 */
BME280_Result BME280_setPressureOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting pressureOversampling);

/**
 * @fn BME280_Result BME280_setTemperatureOversampling(BME280_Handle*, BME280_Oversampling_setting)
 * @brief
 *
 * @param cfgPtr Pointer to sensor handle
 * @param temperatureOversampling
 * @return
 */
BME280_Result BME280_setTemperatureOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting temperatureOversampling);
/**
 * @fn BME280_Result BME280_setHumidityOversampling(BME280_Handle*, BME280_Oversampling_setting)
 * @brief
 *
 * @param cfgPtr Pointer to sensor handle
 * @param humidityOversampling
 * @return
 */
BME280_Result BME280_setHumidityOversampling(
		BME280_Handle *cfgPtr,
		BME280_Oversampling_setting humidityOversampling);

/**
 * @fn BME280_Result BME280_setStandbyTime(BME280_Handle*, BME280_StandbyTime)
 * @brief
 *
 * @param cfgPtr Pointer to sensor handle
 * @param standbyTime
 * @return
 */
BME280_Result BME280_setStandbyTime(
		BME280_Handle *cfgPtr,
		BME280_StandbyTime standbyTime);

/**
 * @fn BME280_Result BME280_setMode(BME280_Handle*, BME280_ModeType)
 * @brief
 *
 * @param cfgPtr Pointer to sensor handle
 * @param mode
 * @return
 */
BME280_Result BME280_setMode(
		BME280_Handle *cfgPtr,
		BME280_ModeType mode);

/**
 * @fn BME280_Result BME280_setFilterCoefficient(BME280_Handle*, BME280_FilterCoeff)
 * @brief
 *
 * @param cfgPtr Pointer to sensor handle
 * @param filterCoeff
 * @return
 */
BME280_Result BME280_setFilterCoefficient(
		BME280_Handle *cfgPtr,
		BME280_FilterCoeff filterCoeff);

/**
 * @fn BME280_Result BME280_SPI_TransmitReceive(uint8*, uint8*, uint16, uint16)
 * @brief SPI transmit and receive function used to read from SPI interface. Implementation
 * 		is up to the user and target hardware.
 *
 * @param txData  Pointer to data to be transmitted.
 * @param rxData  Pointer to buffer to receive data in.
 * @param size    Number of bytes to send & receive.
 * @param timeout Timeout in milliseconds
 * @return
 */
extern BME280_Result BME280_SPI_TransmitReceive(
		uint8 *txData,
		uint8 *rxData,
		uint16 size,
		uint32 timeout);

/**
 * @fn void BME280_delayMs(uint16)
 * @brief Function that implements the delay passed to it in milliseconds. Used in
 * 		internal communication sensor functions as some functions need delay.
 *
 * @param a_milliseconds
 */
extern void BME280_delayMs(
		uint32 a_milliseconds);
#endif /* BME280_DRIVER_BME280_H_ */
