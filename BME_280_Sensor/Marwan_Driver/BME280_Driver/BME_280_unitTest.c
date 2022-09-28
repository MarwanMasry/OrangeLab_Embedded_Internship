/******************************************************************************
 *
 * Module: ModuleTest
 *
 * File Name: BME_280_unitTest.c
 *
 * Description: Source file for the BME280 Test Cases.
 *
 * Marwan Abdelhakim Elmasry
 ******************************************************************************/

/*******************************************************************************
 *                              Includes Needed                                *
 *******************************************************************************/
#include "BME_280_unitTest.h"
#include <assert.h>


/*******************************************************************************
 *                          Public Function Definitions	                       *
 *******************************************************************************/

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


	result = BME_280_Init(&handler1, BME_280_INTERFACE_SPI);

	result = BME_280_getID(&handler1, &ID);

	result = BME_280_getMeasurmentStatus(&handler1, &Status1);

	result = BME_280_getUpdateStatus(&handler1, &Status2);

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

	result = BME_280_Init(&handler1, BME_280_INTERFACE_SPI);


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
	BME_280_Status result = {0};

	sint32  temp_fixed = 0;
	float64 temp_float = 0;

	uint32  pressure_fixed = 0;
	float64 pressure_float = 0;

	uint32  humidity_fixed = 0;
	float64 humidity_float = 0;

	BME_280_settings settings;

	result = BME_280_Init(&handler1, BME_280_INTERFACE_SPI);


	result = BME_280_SetTempOverSamplingSetting(&handler1, OVERSAMPLING_2);
	result = BME_280_SetPressureOverSamplingSetting(&handler1, OVERSAMPLING_4);
	result = BME_280_SetHumidityOverSamplingSetting(&handler1, OVERSAMPLING_8);
	result = BME_280_SetFilterSettings(&handler1, _4);

	result = BME_280_SetMode(&handler1, NORMAL_MODE);
	result = BME_280_SetStandbyTime(&handler1,_125_MS);


	result = BME_280_getSensorSettings(&handler1, &settings);

	result = BME_280_getTemperature_fixedPoint(&handler1, &temp_fixed);
	result = BME_280_getTemperature_floatingPoint(&handler1, &temp_float);

	result = BME_280_getPressure_fixedPoint(&handler1, &pressure_fixed);
	result = BME_280_getPressure_floatingPoint(&handler1, &pressure_float);

	result = BME_280_getHumidity_fixedPoint(&handler1, &humidity_fixed);
	result = BME_280_getHumidity_floatingPoint(&handler1, &humidity_float);

	result = BME_280_DeInit(&handler1);
}

