 /******************************************************************************
 *
 * Module: BME_280
 *
 * File Name: BME_280_Registers.h
 *
 * Description: Header file for BME_280 Driver Registers
 *
 * Author: Marwan Abdelhakim Elmasry
 ******************************************************************************/

#ifndef BME_280_REGISTERS_H_
#define BME_280_REGISTERS_H_

/*******************************************************************************
 *                              Includes Needed                                *
 *******************************************************************************/      
#include "Std_Types.h"

/*******************************************************************************
 *                          Pre-Processors Definition                          *
 *******************************************************************************/

/* name Register Address */
#define BME280_CONFIG_ADDR                        (0xF5)
#define BME280_CTRL_MEAS_ADDR                     (0xF4)
#define BME280_STATUS_ADDR						  (0xF3)
#define BME280_CTRL_HUM_ADDR                      (0xF2)
#define BME280_RESET_ADDR                         (0xE0)
#define BME280_CHIP_ID_ADDR                       (0xD0)

#define BME280_TEMP_PRESS_CALIB_DATA_ADDR         (0x88)
#define BME280_HUMIDITY_CALIB_DATA_ADDR           (0xE1)

#define BME280_DATA_START_ADDR                    (0xF7)
#define BME280_PWR_CTRL_ADDR                      (0xF4)

#endif /* BME_280_REGISTERS_H_ */
