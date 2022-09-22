 /******************************************************************************
 *
 * Module: BME_280
 *
 * File Name: BME_280_Cfg.h
 *
 * Description: Pre-Compile Configuration Header file for BME_280 Driver
 *
 * Author: Marwan Abdelhakim Elmasry
 ******************************************************************************/
#ifndef BME_280_CFG_H_
#define BME_280_CFG_H_


/*******************************************************************************
 *                          Pre-Compile Configuration                          *
 *******************************************************************************/

/* The number of Sensors to be configured */
#define MAX_INSTANCE_OF_BME_280_SENSOR				10

/* Enable-Disable 32 bit value Compensating measurements functions */
#define BME280_32BIT_COMPENSATING_MEASURMENTS		_ENABLE_

/* Enable-Disable float value Compensating measurements functions */
#define BME280_FLOAT_COMPENSATING_MEASURMENTS		_ENABLE_

/* CSB Port and Pin location */
#define CSB_PIN_PORT						     	GPIOA
#define CSB_PIN										GPIO_PIN_4



#endif /* BME_280_CFG_H_ */
