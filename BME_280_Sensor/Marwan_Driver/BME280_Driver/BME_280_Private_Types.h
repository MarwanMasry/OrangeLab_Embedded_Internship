/******************************************************************************
 *
 * Module: BME_280
 *
 * File Name: BME_280_Types.h
 *
 * Description: Header file that has all the needed private type decelerations for
 *              BME_280 Driver
 *
 * Author: Marwan Abdelhakim Elmasry
 ******************************************************************************/

#ifndef BME_280_DRIVER_BME_280_PRIVATE_TYPES_H_
#define BME_280_DRIVER_BME_280_PRIVATE_TYPES_H_

/*******************************************************************************
 *                              Includes Needed                                *
 *******************************************************************************/
/* Standard types */
#include "Std_Types.h"

#include "BME_280_Public_Types.h"

/*******************************************************************************
 *                          Pre-Processors Definition                          *
 *******************************************************************************/
/* The number of Sensors to be configured */
#define MAX_INSTANCE_OF_BME_280_SENSOR				10

#define BME280_TEMP_PRESS_CALIB_DATA_LEN          (26U)
#define BME280_HUMIDITY_CALIB_DATA_LEN            (7U)
#define BME280_PRESS_TEMP_HUMI_DATA_LEN           (8U)

#define SPI_READ_MASK							  (0x80)
#define SPI_WRITE_MASK							  (0x7F)

#define I2C_READ_MASK							  (0x01)
#define I2C_WRITE_MASK							  (0xFE)


#define BME280_I2C_TIMEOUT_MS						100
#define BME280_SPI_TIMEOUT_MS						100


#define IM_UPDATE_MASK							  (0x01)

#define OCCUIPIED								  1U
#define NOT_OCCUIPIED							  0U
#define INSTANCE_TAKEN							  2U
#define INSTANCE_NOT_TAKEN						  3U

#define SENSOR_ADDRESS_IN_I2C_BUS				  (0x76U)

#define BME280_SOFT_RESET_COMMAND                 (0xB6)


/*******************************************************************************
 *                             Types Deceleration	                       *
 *******************************************************************************/

/******************************************************************************
 * @union BME_280_ConfigRegisterUnion
 * @brief Union used to create configuration register byte
 *
 *******************************************************************************/
typedef union
{
	uint8 config;
	struct
	{
		uint8 spi3w_en :1;
		uint8  :1;
		uint8 filter_coeff :3;
		uint8 t_sb :3;
	} Bits;
} BME_280_ConfigRegisterUnion;



/******************************************************************************
 * @union   BME_280_Calib1
 * @brief   Union used to receive and access calibration parameters
 *
 *******************************************************************************/
typedef union __attribute__((packed, aligned(1)))
{
	uint16 arr[13];
	struct __attribute__((packed, aligned(1)))
	{
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

		uint8 :8;

		sint8 dig_H1;
	} words;
} BME_280_Calib1;


/******************************************************************************
 * @struct BME_280_Calib2
 * @brief  Union used to receive and access calibration parameters
 *
 *******************************************************************************/
#pragma pack(1)
typedef union __attribute__((packed, aligned(1)))
{
	uint8 arr[19];
	struct __attribute__((packed, aligned(1)))
	{
		sint16 dig_H2 		:16;
		uint8 dig_H3 		:8;

		uint8 dig_H4_MSB  :8;
		uint8 dig_H4_LSB  :4;

		uint8 dig_H5_LSB  :4;
		uint8 dig_H5_MSB  :8;

		sint8  dig_H6		:8;

		sint16   dig_H4;
		sint16   dig_H5;
		sint32	 t_fine;
		sint32	 t_fine_float;

	}words;


} BME_280_Calib2;

/******************************************************************************
 * Union Name: BME280_UncompensatedReadings
 * Union Description:  Union used to receive and access temp, press, & hum. data.
 *******************************************************************************/
typedef union __attribute__((packed, aligned(1)))
{
	uint8 arr[8];

	struct __attribute__((packed, aligned(1)))
	{
		uint8 press_msb;
		uint8 press_lsb;
		uint8 :4;
		uint8 press_xlsb :4;


		uint8 temp_msb;
		uint8 temp_lsb;
		uint8 :4;
		uint8 temp_xlsb :4;


		uint8 hum_msb;
		uint8 hum_lsb;
	} Bytes;

} BME280_UncompensatedReadings;

/******************************************************************************
 * @union BME280_TemperatureReading
 * @brief union that contain the temperature reading before calibrating
 *
 *******************************************************************************/
typedef union
{
	uint32 temperature;
	struct
	{
		uint32 xlsb :4;
		uint32 lsb :8;
		uint32 msb :8;
		uint32  :12;
	} Data;
} BME280_TemperatureReading;

/******************************************************************************
 * @union BME280_PressureReading
 * @brief union that contain the pressure reading before calibrating
 *
 *******************************************************************************/
typedef union
{
	uint32 pressure;
	struct
	{
		uint32 xlsb :4;
		uint32 lsb :8;
		uint32 msb :8;
		uint32  :12;
	} Data;
} BME280_PressureReading;

/******************************************************************************
 * @union BME280_HumidityReading
 * @brief union that contain the humidity reading before calibrating
 *
 *******************************************************************************/
typedef union
{
	uint16 humidity;
	struct
	{
		uint32 lsb :8;
		uint32 msb :8;
	} Data;
} BME280_HumidityReading;

/******************************************************************************
 * @union BME280_CtrlMeasRegisterUnion
 * @brief Union used to create control measurements register byte
 *
 *******************************************************************************/
typedef union
{
	uint8 config;
	struct
	{
		uint8 mode :2;
		uint8 osrs_p :3;
		uint8 osrs_t :3;
	} Bits;
} BME280_CtrlMeasRegisterUnion	;

/******************************************************************************
 * @union BME280_CtrlHumRegisterUnion
 * @brief  Union used to create control humidity register byte
 *
 *******************************************************************************/
typedef union
{
	uint8 config;
	struct
	{
		uint8 ctrl_hum :3;
		uint8  :5; /* Padding */
	} Bits;
} BME280_CtrlHumRegisterUnion;


/******************************************************************************
 * @union BME280_StatusRegisterUnion
 * @brief Union used to create status register byte
 *******************************************************************************/
typedef union
{
	uint8 config;
	struct
	{
		uint8 im_update :1;
		uint8  :2;
		uint8 measuring :1;
		uint8  :4;
	} Bits;
} BME280_StatusRegisterUnion;


/******************************************************************************
 * @struct BME_280_Config
 * @brief  Structure to configure each individual sensor:
 *
 *  1. The protocol used either it is SPI or I2C
 *  2. structure that has first part of the calibration data
 *  3. structure that has second part of the calibration data
 *******************************************************************************/
struct BME_280_Configurations
{
	uint8							 occupied ;
	BME_280_CommunicationProtocol    ProtocolUsed;
	BME_280_Calib1 					 calib1;
	BME_280_Calib2 					 calib2;


	void (*SetSlaveSelect_ptr)(void);
	void (*ResetSlaveSelect_ptr)(void);
};



#endif /* BME_280_DRIVER_BME_280_PRIVATE_TYPES_H_ */
