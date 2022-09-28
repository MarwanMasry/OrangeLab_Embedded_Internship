#ifndef _BME280_REGISTERS_
#define _BME280_REGISTERS_

/*******************************************************************************
 *                          Definitions                                        *
 *******************************************************************************/

/*********** BME280 fixed point device limits ***********/
#define BME280_MAX_PRESSURE_FIXED_POINT			(110000)
#define BME280_MIN_PRESSURE_FIXED_POINT			(30000)

#define BME280_MAX_HUMIDITY_FIXED_POINT			(102400)
#define BME280_MIN_HUMIDITY_FIXED_POINT			(0)

#define BME280_MAX_TEMPERATURE_FIXED_POINT		(8500)
#define BME280_MIN_TEMPERATURE_FIXED_POINT		(-4000)

/*********** BME280 floating point device limits ***********/

#define BME280_MAX_PRESSURE_FLOATING_POINT		(110000.0)
#define BME280_MIN_PRESSURE_FLOATING_POINT		(30000.0)

#define BME280_MAX_HUMIDITY_FLOATING_POINT		(100.0)
#define BME280_MIN_HUMIDITY_FLOATING_POINT		(0.0)

#define BME280_MAX_TEMPERATURE_FLOATING_POINT	(85)
#define BME280_MIN_TEMPERATURE_FLOATING_POINT	(-40)

#define BME280_MAX_DISCOVERY_COUNT ((uint8)0x05)	/* Max discovery count of BME280 device on interface*/
#define BME280_IM_UPDATE_READY ((uint8)0x00)		/* IM UPDATE flag ready state */
#define BME280_MEASURING_DONE ((uint8)0x00)			/* Measuring flag ready state */
#define BME280_SPI_READ_MASK(reg) (reg|0x80)			/* Mask read register address in case of SPI*/
#define BME280_SPI_WRITE_MASK(reg) (reg&0x7F)			/* Mask write register address in case of SPI*/
#define BME280_SPI_TIMEOUT_MS (10)					/* SPI timeout in milliseconds, used in BME280_SPI_TransmitReceive function */

#define BME280_I2C_READ_MASK(reg) ((reg<<1)&0xFF)			/* Mask read register address in case of I2C*/
#define BME280_I2C_WRITE_MASK(reg) ((reg<<1)&0xFE)			/* Mask write register address in case of I2C*/
#define BME280_I2C_TIMEOUT_MS (100)					/* I2C timeout in milliseconds, used in BME280_I2C_Master_Transmit/Receive function */
#define BME280_LSBYTE_MASK (0x0F)

/*********** BME280 constants and configuration parameters ***********/

#define BME280_RESET_WORD (0xB6)					/* Reset word which will reset BME280 when written into reset register*/
#define BME280_READINGS_BYTES_LENGTH (8)			/* Temperature, humidity, and pressure readings length in bytes */
#define BME280_START_UP_TIME_MS ((uint8)0x02)		/* Sensor boot time according to data-sheet in milliseconds*/
#define BME280_CHIP_ID ((uint8)(0x60))				/* Sensor chip ID, always the same for BME280, non-editable*/

#define BME280_TEMP_PRESS_CALIB_BLOCK_SIZE (26)		/* Temperature and pressure calibration block size in bytes mapped to sensor */
#define BME280_HUM_CALIB_BLOCK_SIZE (7)				/* Humidity calibration block size in bytes mapped to sensor*/
#define BME280_CALIB_1_BLOCK_SIZE (13)				/* Calib 1 block size mapped to structures */
#define BME280_CALIB_2_BLOCK_SIZE (19)				/* Calib 2 block size mapped to structures */
#define BME280_MAX_SENSOR_POOL_SIZE ((4))			/* Maximum pool size of available sensor instances */
/*********** BME280 Register addresses ***********/
#define BME280_HUM_REGISTER_LSB (0xFE)
#define BME280_HUM_REGISTER_MSB (0xFD)

#define BME280_TEMP_REGISTER_XLSB (0xFC)
#define BME280_TEMP_REGISTER_LSB (0xFB)
#define BME280_TEMP_REGISTER_MSB (0xFA)

#define BME280_PRESS_REGISTER_XLSB (0xF9)
#define BME280_PRESS_REGISTER_LSB (0xF8)
#define BME280_PRESS_REGISTER_MSB (0xF7)

#define BME280_START_READINGS_ADDRESS (0xF7)
#define BME280_TEMP_PRESS_BLOCK_START_ADDRESS (0x88)
#define BME280_HUM_BLOCK_START_ADDRESS (0xE1)

#define BME280_CONFIG_REGISTER (0xF5)
#define BME280_CTRL_MEAS_REGISTER (0xF4)
#define BME280_STATUS_REGISTER (0xF3)

#define BME280_CTRL_HUM_REGISTER (0xF2)
#define BME280_RESET_REGISTER (0xE0)
#define BME280_ID_REGISTER (0xD0)

/*******************************************************************************
 *                          Macros                                             *
 *******************************************************************************/

#define BME280_CONFIG_IM_UPDATE_MASK(config_reg) (config_reg & (uint8)0x01)
#define BME280_CONFIG_MEAS_MASK(config_reg) ((config_reg>>3) & (uint8)0x01)

#endif
