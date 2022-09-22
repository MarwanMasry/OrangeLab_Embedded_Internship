/*
 * bme280_defs.h
 *
 *  Created on: Sep 12, 2022
 *      Author: h4z3m
 */

#ifndef BME280_DRIVER_BME280_DEFS_H_
#define BME280_DRIVER_BME280_DEFS_H_

/*******************************************************************************
 *                          Includes		                                   *
 *******************************************************************************/
#include <std_types.h>
/*******************************************************************************
 *                          Typedefs		                                   *
 *******************************************************************************/

/**
 * @enum BME280_InterfaceType
 * @brief Enum for interface used by the sensor
 */
typedef enum {
	BME280_Interface_SPI = 0, /**< Selected SPI interface */
	BME280_Interface_I2C = 1 /**< Selected I2C interface */
} BME280_InterfaceType;

/**
 * @union BME280_ConfigRegisterUnion
 * @brief Union used to create configuration register byte
 *******************************************************************************/
typedef union {
	uint8 config;
	struct {
		uint8 spi3w_en :1;
		uint8 :1;/**< Padding */
		uint8 filter_coeff :3;
		uint8 t_sb :3;
	} Bits;
} BME280_ConfigRegisterUnion;
/**
 * @union Calib1
 * @brief Union used to receive and access calibration parameters
 *******************************************************************************/
typedef union __attribute__((packed, aligned(1))) {
	uint16 arr[13];
	struct __attribute__((packed, aligned(1))) {
		uint16 dig_T1;
		sint16 dig_T2;
		sint16 dig_T3;

		uint16 dig_P1;
		sint16 dig_P2;
		sint16 dig_P3;
		sint16 dig_P4;
		sint16 dig_P5;
		sint16 dig_P6;
		sint16 dig_P7;
		sint16 dig_P8;
		sint16 dig_P9;

		uint8:8;/**< Padding */

		sint8 dig_H1;
	} words;
} Calib1;

/**
 * @struct Calib2
 * @brief Struct used to receive and access calibration parameters
 *******************************************************************************/
typedef union __attribute__((packed, aligned(1))) {
	uint8 arr[19];

	struct __attribute__((packed, aligned(1))) {
		sint16 dig_H2 :16;

		uint8 dig_H3 :8;

		uint8 dig_H4_msb :8;
		uint8 dig_H4_lsb :4;

		uint8 dig_H5_lsb :4;
		uint8 dig_H5_msb :8;

		sint8 dig_H6 :8;
		sint16 dig_H4 :16;

		sint16 dig_H5 :16;

		sint32 t_fine;
		sint32 t_fine_float;
	} Bytes;

} Calib2;

/**
 * @union BME280_UncompensatedReadings
 * @brief Union used to receive and access temp, press, & hum. data.
 *
 */
typedef union __attribute__((packed, aligned(1))) {
	uint8 arr[8];
	struct __attribute__((packed, aligned(1))) {
		uint8 press_msb :8;
		uint8 press_lsb :8;
		uint8 press_pad :4;
		uint8 press_xlsb :4;

		uint8 temp_msb :8;
		uint8 temp_lsb :8;
		uint8 temp_pad :4;
		uint8 temp_xlsb :4;

		uint8 hum_msb :8;
		uint8 hum_lsb :8;

	} Bytes;
} BME280_UncompensatedReadings;

/**
 * @union BME280_TemperatureReading
 * @brief Used to store readings from the sensor and access them directly from
 * 'temperature' member
 *
 */
typedef union {
	uint32 temperature;
	struct {
		uint32 xlsb :4;
		uint32 lsb :8;
		uint32 msb :8;
		uint32 :12;/**< Padding */
	} Data;
} BME280_TemperatureReading;

/**
 * @union BME280_PressureReading
 * @brief Used to store readings from the sensor and access them directly from
 * 'pressure' member
 *
 */
typedef union {
	sint32 pressure;
	struct {
		sint32 xlsb :4;
		sint32 lsb :8;
		sint32 msb :8;
		sint32 :12;/**< Padding */
	} Data;

} BME280_PressureReading;

/**
 * @union BME280_HumidityReading
 * @brief Used to store readings from the sensor and access them directly from
 * 'humidity' member
 *
 */
typedef union {
	sint16 humidity;
	struct {
		sint16 lsb :8;
		sint16 msb :8;
	} Data;
} BME280_HumidityReading;

/**
 * @union BME280_CtrlMeasRegisterUnion
 * @brief Union used to create control measurements register byte
 *******************************************************************************/
typedef union {
	uint8 config;
	struct {
		uint8 mode :2;
		uint8 osrs_p :3;
		uint8 osrs_t :3;
	} Bits;
} BME280_CtrlMeasRegisterUnion;

/**
 * @union BME280_CtrlHumRegisterUnion
 * @brief Union used to create control humidity register byte
 */
typedef union {
	uint8 config;
	struct {
		uint8 osrs_h :3; /**< Humidity over-sampling setting**/
		uint8 :5; /**< Padding */
	} Bits;
} BME280_CtrlHumRegisterUnion;

/**
 * @union BME280_StatusRegisterUnion
 * @brief Union used to create status register byte
 *******************************************************************************/
typedef union {
	uint8 config;
	struct {
		uint8 im_update :1;
		uint8 :2;/**< Padding */
		uint8 measuring :1;
		uint8 :4;/**< Padding */
	} Bits;
} BME280_StatusRegisterUnion;

/**
 * @struct BME280_ConfigType
 * @brief Configuration structure used for setup, configuration,
 *  	  and sensor functions.
 *
 * 1. uint8 ID: 						  Custom ID issued to the sensor by the user
 * 2. BME280_Settings:					  Struct that contains user settings
 * 3. BME280_InterfaceType Intf: 		  Interface type used with the sensor (I2C/SPI)
 * 4. Calib1 calib_data_1:				  Contains calibration data obtained from sensor
 * 5. Calib2 calib_data_2:				  Contains calibration data obtained from sensor
 */
struct BME280_ConfigType {
	uint8 ID; /* ID of sensor*/
	boolean occupied;
	BME280_Settings Settings;
	BME280_InterfaceType Intf; /* Specifies interface used to communicate with BME280*/
	Calib1 calib_data_1;
	Calib2 calib_data_2;
};

#endif /* BME280_DRIVER_BME280_DEFS_H_ */
