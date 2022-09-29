/******************************************************************************
 *
 * Module: ModuleTest
 *
 * File Name: BME_280_unitTest.c
 *
 * Description: Source file for the BME280 Test Cases on STM32WB55xG MCU.
 *
 * Marwan Abdelhakim Elmasry
 ******************************************************************************/

/*******************************************************************************
 *                              Includes Needed                                *
 *******************************************************************************/
#include "BME_280_unitTest.h"
#include "main.h"
#include <assert.h>


/*******************************************************************************
 *                          Public Function Definitions	                       *
 *******************************************************************************/

/* these Callback functions is specific to STM32WB55xG */
static void set_CS_pin(void)
{
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,SET);
}

static void reset_CS_pin(void)
{
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin,RESET);
}


/*******************************************************************************
 * @fn void BME_280_unitTest1(void)
 * @brief Test 1
 *
 *******************************************************************************/
void BME_280_unitTest1
(
		void
)
{
	BME_280_Config  handler1 ={0};
	BME_280_Status result = {0};

	uint8 ID;
	uint8 Status1;
	uint8 Status2;

	/******************* Configurations *************************************/
	result = BME_280_getInstance(&handler1);

	result = BME_280_setInterfaceProtocol(&handler1, BME_280_INTERFACE_SPI);

	result = BME_280_SetAssertSlaveSelectCallback(&handler1,  set_CS_pin);
	result = BME_280_SetReleaseSlaveSelectCallback(&handler1, reset_CS_pin);

	result = BME_280_Init(&handler1);


	result = BME_280_getID(&handler1, &ID);

	result = BME_280_getMeasurmentStatus(&handler1, &Status1);

	result = BME_280_getUpdateStatus(&handler1, &Status2);

	/******************* Finishing *************************************/
	result = BME_280_DeInit(&handler1);
}

/*******************************************************************************
 * @fn void BME_280_unitTest2(void)
 * @brief Test2
 *
 *******************************************************************************/
void BME_280_unitTest2
(
		void
)
{
	BME_280_Config  handler1 ={0};
	BME_280_Status result = {0};

	BME_280_Mode mode;
	BME_280_FilterCoefficient filterCoef;
	BME_280_StandbyTimeInNormalMode t_sb;
	BME_280_OversamplingValue t_os;
	BME_280_OversamplingValue p_os;
	BME_280_OversamplingValue H_os;


	/******************* Configurations *************************************/
	result = BME_280_getInstance(&handler1);

	result = BME_280_setInterfaceProtocol(&handler1, BME_280_INTERFACE_SPI);

	result = BME_280_SetAssertSlaveSelectCallback(&handler1,  set_CS_pin);
	result = BME_280_SetReleaseSlaveSelectCallback(&handler1, reset_CS_pin);

	result = BME_280_Init(&handler1);


	result = BME_280_SetMode(&handler1, NORMAL_MODE);
	result = BME_280_GetMode(&handler1, &mode);

	result = BME_280_SetTempOverSamplingSetting(&handler1, OVERSAMPLING_2);
	result = BME_280_GetTempOverSamplingSetting(&handler1, &t_os);

	result = BME_280_SetPressureOverSamplingSetting(&handler1, OVERSAMPLING_4);
	result = BME_280_GetPressureOverSamplingSetting(&handler1, &p_os);

	result = BME_280_SetHumidityOverSamplingSetting(&handler1, OVERSAMPLING_8);
	result = BME_280_GetHumidityOverSamplingSetting(&handler1, &H_os);


	result = BME_280_SetFilterSettings(&handler1, _4);
	result = BME_280_GetFilterSettings(&handler1, &filterCoef);


	result = BME_280_SetStandbyTime(&handler1,_125_MS);
	result = BME_280_GetStandbyTime(&handler1, &t_sb);

	/******************* Finishing *************************************/
	result = BME_280_DeInit(&handler1);
}

/*******************************************************************************
 * @fn void BME_280_unitTest2(void)
 * @brief Test2
 *
 *******************************************************************************/
void BME_280_unitTest3
(
		void
)
{
	BME_280_Config  handler1 ={0};
	BME_280_Config  handler2 ={0};
	BME_280_Status  result = {0};

	BME_280_TempType_fixedPoint  temp_fixed = 0;
	BME_280_TempType_floatingPoint  temp_float = 0;

	BME_280_PressureType_fixedPoint  pressure_fixed = 0;
	BME_280_PressureType_floatingPoint pressure_float = 0;

	BME_280_HumidityType_fixedPoint  humidity_fixed = 0;
	BME_280_HumidityType_floatingPoint humidity_float = 0;

	BME_280_settings settings;

	/******************* Configurations *************************************/
	result = BME_280_getInstance(&handler1);

	result = BME_280_setInterfaceProtocol(&handler1, BME_280_INTERFACE_SPI);

	result = BME_280_SetAssertSlaveSelectCallback(&handler1,  set_CS_pin);
	result = BME_280_SetReleaseSlaveSelectCallback(&handler1, reset_CS_pin);

	result = BME_280_Init(&handler1);


	result = BME_280_SetTempOverSamplingSetting(&handler1, OVERSAMPLING_2);
	result = BME_280_SetPressureOverSamplingSetting(&handler1, OVERSAMPLING_4);
	result = BME_280_SetHumidityOverSamplingSetting(&handler1, OVERSAMPLING_8);
	result = BME_280_SetFilterSettings(&handler1, _4);

	result = BME_280_SetMode(&handler1, NORMAL_MODE);
	result = BME_280_SetStandbyTime(&handler1,_125_MS);

	result = BME_280_getSensorSettings(&handler1, &settings);

	/******************* Readings *************************************/
	result = BME_280_getTemperature_fixedPoint(&handler1, &temp_fixed);
	result = BME_280_getTemperature_floatingPoint(&handler1, &temp_float);

	result = BME_280_getPressure_fixedPoint(&handler1, &pressure_fixed);
	result = BME_280_getPressure_floatingPoint(&handler1, &pressure_float);

	result = BME_280_getHumidity_fixedPoint(&handler1, &humidity_fixed);
	result = BME_280_getHumidity_floatingPoint(&handler1, &humidity_float);

	/******************* Finishing *************************************/
	result = BME_280_DeInit(&handler1);

	temp_fixed = 0;
	temp_float = 0;
	pressure_fixed = 0;
	pressure_float = 0;
	humidity_fixed = 0;
	humidity_float = 0;

	/*  To have a suitable reading from the code bellow,
	 *  You must change change your wires layout if you have one sensor or
	 *  you must have 2 instances of the sensor hardware connected
	 */

	/******************* Configurations *************************************/
	result = BME_280_getInstance(&handler2);

	result = BME_280_setInterfaceProtocol(&handler2, BME_280_INTERFACE_I2C);

	result = BME_280_Init(&handler2);

	result = BME_280_SetTempOverSamplingSetting(&handler2, OVERSAMPLING_2);
	result = BME_280_SetPressureOverSamplingSetting(&handler2, OVERSAMPLING_4);
	result = BME_280_SetHumidityOverSamplingSetting(&handler2, OVERSAMPLING_8);
	result = BME_280_SetFilterSettings(&handler2, _4);

	result = BME_280_SetMode(&handler2, NORMAL_MODE);
	result = BME_280_SetStandbyTime(&handler2,_125_MS);

	result = BME_280_getSensorSettings(&handler2, &settings);

	/******************* Readings *************************************/
	result = BME_280_getTemperature_fixedPoint(&handler2, &temp_fixed);
	result = BME_280_getTemperature_floatingPoint(&handler2, &temp_float);

	result = BME_280_getPressure_fixedPoint(&handler2, &pressure_fixed);
	result = BME_280_getPressure_floatingPoint(&handler2, &pressure_float);

	result = BME_280_getHumidity_fixedPoint(&handler2, &humidity_fixed);
	result = BME_280_getHumidity_floatingPoint(&handler2, &humidity_float);

	/******************* Finishing *************************************/
	result = BME_280_DeInit(&handler2);

}

