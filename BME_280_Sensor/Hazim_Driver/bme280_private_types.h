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
#include <bme280.h>
/*******************************************************************************
 *                          Typedefs		                                   *
 *******************************************************************************/

/**
 * @union BME280_ConfigRegisterUnion
 * @brief Union used to create configuration register byte
 *******************************************************************************/
typedef union {
	BME280_uint8 config;
	struct {
		BME280_uint8 spi3w_en :1;
		BME280_uint8 :1;/**< Padding */
		BME280_uint8 filter_coeff :3;
		BME280_uint8 t_sb :3;
	} Bits;
} BME280_ConfigRegisterUnion;

/**
 * @union Calib1
 * @brief Union used to receive and access calibration parameters
 *******************************************************************************/
typedef union __attribute__((packed, aligned(1))) {
	BME280_uint16 arr[BME280_CALIB_1_BLOCK_SIZE];
	struct __attribute__((packed, aligned(1))) {
		BME280_uint16 dig_T1;
		BME280_sint16 dig_T2;
		BME280_sint16 dig_T3;

		BME280_uint16 dig_P1;
		BME280_sint16 dig_P2;
		BME280_sint16 dig_P3;
		BME280_sint16 dig_P4;
		BME280_sint16 dig_P5;
		BME280_sint16 dig_P6;
		BME280_sint16 dig_P7;
		BME280_sint16 dig_P8;
		BME280_sint16 dig_P9;

		BME280_uint8 :8;/**< Padding */

		BME280_sint8 dig_H1;
	} words;
} Calib1;

/**
 * @struct Calib2
 * @brief Struct used to receive and access calibration parameters
 *******************************************************************************/
typedef union __attribute__((packed, aligned(1))) {
	BME280_uint8 arr[BME280_CALIB_2_BLOCK_SIZE];

	struct __attribute__((packed, aligned(1))) {
		BME280_sint16 dig_H2 :16;

		BME280_uint8 dig_H3 :8;

		BME280_uint8 dig_H4_msb :8;
		BME280_uint8 dig_H4_lsb :4;

		BME280_uint8 dig_H5_lsb :4;
		BME280_uint8 dig_H5_msb :8;

		BME280_sint8 dig_H6 :8;
		BME280_sint16 dig_H4 :16;

		BME280_sint16 dig_H5 :16;

		BME280_sint32 t_fine;
		BME280_sint32 t_fine_float;
	} Bytes;

} Calib2;

/**
 * @union BME280_UncompensatedReadings
 * @brief Union used to receive and access temp, press, & hum. data.
 *
 */
typedef union __attribute__((packed, aligned(1))) {
	BME280_uint8 arr[BME280_READINGS_BYTES_LENGTH];
	struct __attribute__((packed, aligned(1))) {
		BME280_uint8 press_msb :8;
		BME280_uint8 press_lsb :8;
		BME280_uint8 press_pad :4;
		BME280_uint8 press_xlsb :4;

		BME280_uint8 temp_msb :8;
		BME280_uint8 temp_lsb :8;
		BME280_uint8 temp_pad :4;
		BME280_uint8 temp_xlsb :4;

		BME280_uint8 hum_msb :8;
		BME280_uint8 hum_lsb :8;

	} Bytes;
} BME280_UncompensatedReadings;

/**
 * @union BME280_TemperatureReading
 * @brief Used to store readings from the sensor and access them directly from
 * 'temperature' member
 *
 */
typedef union {
	BME280_uint32 temperature;
	struct {
		BME280_uint32 xlsb :4;
		BME280_uint32 lsb :8;
		BME280_uint32 msb :8;
		BME280_uint32 :12;/**< Padding */
	} Data;
} BME280_TemperatureReading;

/**
 * @union BME280_PressureReading
 * @brief Used to store readings from the sensor and access them directly from
 * 'pressure' member
 *
 */
typedef union {
	BME280_sint32 pressure;
	struct {
		BME280_sint32 xlsb :4;
		BME280_sint32 lsb :8;
		BME280_sint32 msb :8;
		BME280_sint32 :12;/**< Padding */
	} Data;

} BME280_PressureReading;

/**
 * @union BME280_HumidityReading
 * @brief Used to store readings from the sensor and access them directly from
 * 'humidity' member
 *
 */
typedef union {
	BME280_sint16 humidity;
	struct {
		BME280_sint16 lsb :8;
		BME280_sint16 msb :8;
	} Data;
} BME280_HumidityReading;

/**
 * @union BME280_CtrlMeasRegisterUnion
 * @brief Union used to create control measurements register byte
 *******************************************************************************/
typedef union {
	BME280_uint8 config;
	struct {
		BME280_uint8 mode :2;
		BME280_uint8 osrs_p :3;
		BME280_uint8 osrs_t :3;
	} Bits;
} BME280_CtrlMeasRegisterUnion;

/**
 * @union BME280_CtrlHumRegisterUnion
 * @brief Union used to create control humidity register byte
 */
typedef union {
	BME280_uint8 config;
	struct {
		BME280_uint8 osrs_h :3; /**< Humidity over-sampling setting**/
		BME280_uint8 :5; /**< Padding */
	} Bits;
} BME280_CtrlHumRegisterUnion;

/**
 * @union BME280_StatusRegisterUnion
 * @brief Union used to create status register byte
 *******************************************************************************/
typedef union {
	BME280_uint8 config;
	struct {
		BME280_uint8 im_update :1;
		BME280_uint8 :2;/**< Padding */
		BME280_uint8 measuring :1;
		BME280_uint8 :4;/**< Padding */
	} Bits;
} BME280_StatusRegisterUnion;

/**
 * @enum BME280_I2C_SensorSlaveAddress
 * @brief  Used to identify BME280 slave address in I2C interface mode. Must be set
 * 		if the device's SDO is connected to __GND__ datasheet.
 */
typedef enum {
	BME280_I2C_Addr = (0x76),
}BME280_I2C_SensorSlaveAddress;

/**
 * @struct BME280_ConfigType
 * @brief Configuration structure used for setup, configuration,
 *  	  and sensor functions.
 */
struct BME280_ConfigType {
	BME280_uint8 ID; /**< Custom ID issued to the sensor by the user*/
	BME280_boolean occupied; /**< Flag used to determine if instance is occupied or empty*/
	BME280_InterfaceType Intf; /**< Interface type used with the sensor (I2C/SPI) */
	BME280_I2C_SensorSlaveAddress I2C_SlaveAddr;
	void (*GPIOCallback_SetNSS)(void);			/**< Callback function for setting NSS pin */
	void (*GPIOCallback_resetNSS)(void);			/**< Callback function for resetting NSS pin */
	Calib1 calib_data_1; /**<Contains calibration data obtained from sensor*/
	Calib2 calib_data_2; /**<Contains calibration data obtained from sensor*/
};

#endif /* BME280_DRIVER_BME280_DEFS_H_ */
