#ifndef _BME280_REGISTERS_
#define _BME280_REGISTERS_

/*******************************************************************************
 *                          Definitions                                        *
 *******************************************************************************/

#define BME280_MAX_DISCOVERY_COUNT ((uint8)0x05)
#define BME280_IM_UPDATE_READY ((uint8)0x00)
#define BME280_MEASURING_DONE ((uint8)0x00)
#define BME280_READ_MASK(reg) (reg|0x80)
#define BME280_WRITE_MASK(reg) (reg&0x7F)
#define BME280_SPI_TIMEOUT_MS (10)

/*********** BME280 constants and configuration parameters ***********/

#define BME280_RESET_WORD (0xB6)
#define BME280_READINGS_BYTES_LENGTH (8)
#define BME280_START_UP_TIME_MS ((uint8)0x02)
#define BME280_CHIP_ID ((uint8)(0x60))
#define BME280_TEMP_PRESS_CALIB_BLOCK_SIZE (26)
#define BME280_HUM_CALIB_BLOCK_SIZE (7)
#define BME280_LSBYTE_MASK (0x0F)
#define BME280_MAX_SENSOR_POOL_SIZE ((4))
/*******************************************************************************
 *                          Macros                                             *
 *******************************************************************************/
#define BME280_NULL_PTR(ptr) (ptr!=((void*)(0)))
#define BME280_CONFIG_BYTE(t_sb,filter) ( ((uint8)((t_sb&0x07)<<5)| \
	(uint8)((filter&07)<<2)) & (0xFD) )\

#define BME280_CTRL_MEAS_BYTE(osrs_t,osrs_p,mode)	\
((uint8)((osrs_t&0x07)<<5)|\
 (uint8)((osrs_p&0x02)<<2)|\
 (uint8)(mode&0x03))

#define BME280_CONCAT_BYTES_TO_16(byte_1,byte_2)  ((uint16_t)( (((uint16_t)byte_1)<<8) |\
															   (((uint16_t)byte_2)&BME280_LSBYTE_MASK)))

#define BME280_CONFIG_IM_UPDATE_MASK(config_reg) (config_reg & (uint8)0x01)
#define BME280_CONFIG_MEAS_MASK(config_reg) ((config_reg>>3) & (uint8)0x01)

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

#endif
