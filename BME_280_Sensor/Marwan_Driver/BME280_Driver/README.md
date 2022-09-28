# BME280 Driver (Marwan's version)

## Overview

This driver was developed for Bosch Sensortec BME280 Sensor. It measures temperature, pressure, and humidity. The driver allows interfacing with the sensor through I2C or SPI. Each sensor setting is configurable through API functions, and the driver supports both fixed and floating point arithmetic.

## How to use this driver

Before using the driver at all, several functions must be implemented by the user as they are target specific. These functions are in the bme280.h file at the very end. They are defined as weak functions to be implemented according to the user.
These functions are:
  - BME_280_SPI_TransmitReceive
  - BME_280_Delay
  - BME280_I2C_Master_Transmit
  - BME280_I2C_Master_Receive

Example of implemntations of the weak functions can be found in the main.c file on the Demo Project Directory that is based on  STM32WB55xG Processors / nucleo-64 MCU, also in this demo there is example of reading Temperature, pressure and humidity in Fixed/Floating point in both SPI and I2C configurations, I recommend to go through the demo project after reading the below steps.

### General Step 
#include "BME_280.h" in the file you want to use the driver

### Steps For SPI configurations 
1. Define a sensor handle like so, this is the instance which will be used to specifically communicate with the sensor if multiple exist.
``` C
	BME_280_Config  handler1 ={0};
```

2. Now, we must obtain an instance from the sensor pool which holds handles (up to 10). Note that the handle must be unoccupied or used before, if it is incorrectly occupied, you can de-init it using the ```BME_280_DeInit(&handler1);```

``` C

	BME_280_Status result =BME_280_getInstance(&handler1);
	if(result == BME280_OK)
	{
	  // success
	}
	else
	{
	  // see the type of error that the API returns, and you can understand the error from the error name returned
	}

```
3. After an instance has been obtained successfully, we must specify the interface type and the user must set call back functions for both setting and reseting the slave select pin, for example if we are using SPI:

```C
   /* Here we fitst supply teh interface type first */
   result = BME_280_setInterfaceProtocol(&handler1, BME_280_INTERFACE_SPI);
   
   /* Then in case of using SPI, the user must supply the functions [set_CS_pin,reset_CS_pin]  that control the slave select pin 
    * that is specific to the user MCU, see impelementation example for these call back in Demo project folder.
    */ 
   result = BME_280_SetAssertSlaveSelectCallback(&handler1,  set_CS_pin);
   result = BME_280_SetReleaseSlaveSelectCallback(&handler1, reset_CS_pin);
   
  
```
4. After these steps have been completed successfully without any errors, the user can init the sensor and check if communication is successful.

```C
	result = BME_280_Init(&handler1);
	if(result!=BME280_OK)
	  /* Error initializing */
  
```
5. Next, the user can set their specific device settings and obtain readings as necessary. It must be noted that functions which obtain readings do not set/check if valid settings are set on the sensor. Rather, they obtain readings and convert instantly; therefore, the user must set their settings before before Step 6.
```C
    
    /* IF the return of the these APIs is BME280_OK then every operation is done successfully */
    result = BME_280_SetTempOverSamplingSetting(&handler1, OVERSAMPLING_2);
    result = BME_280_SetPressureOverSamplingSetting(&handler1, OVERSAMPLING_4);
    result = BME_280_SetHumidityOverSamplingSetting(&handler1, OVERSAMPLING_8);   
    result = BME_280_SetFilterSettings(&handler1, _4);

    result = BME_280_SetMode(&handler1, NORMAL_MODE);
    result = BME_280_SetStandbyTime(&handler1,_125_MS);

    /* This API is helpful to make sure that all the setting set by the user is successfully set, also the user can rely on the status return of each of the above     
     * APIs 
     */
    BME_280_settings settings;
    result = BME_280_getSensorSettings(&handler1, &settings);
```
6. Finally, readings can be obtained and processed.

```C		
    /* Declare variables for Sensor Readings */
    BME_280_TempType_fixedPoint  temp_fixed = 0;
    BME_280_TempType_floatingPoint  temp_float = 0;

   BME_280_PressureType_fixedPoint  pressure_fixed = 0;
   BME_280_PressureType_floatingPoint pressure_float = 0;

   BME_280_HumidityType_fixedPoint  humidity_fixed = 0;
   BME_280_HumidityType_floatingPoint humidity_float = 0;

    /* Finally get the reading of the functions using these APIs and  If the return of the these APIs is BME280_OK then every operation is done successfully */
    result = BME_280_getTemperature_fixedPoint(&handler1, &temp_fixed);
    result = BME_280_getTemperature_floatingPoint(&handler1, &temp_float);

    result = BME_280_getPressure_fixedPoint(&handler1, &pressure_fixed);
    result = BME_280_getPressure_floatingPoint(&handler1, &pressure_float);

   result = BME_280_getHumidity_fixedPoint(&handler1, &humidity_fixed);
   result = BME_280_getHumidity_floatingPoint(&handler1, &humidity_float);

```
6. After finishing communicating with the sensor make sure to de-Init your handler to free your instance and free resouorces for others.

```C		
   result = BME_280_DeInit(&handler1);
```

### Steps For I2C configurations

0. The user is responsible for driving the CSB bit to high and Driving SDO to low in I2C configurations 

1. Define a sensor handle like so, this is the instance which will be used to specifically communicate with the sensor if multiple exist.
``` C
	BME_280_Config  handler2 ={0};
```

2. Now, we must obtain an instance from the sensor pool which holds handles (up to 10). Note that the handle must be unoccupied or used before, if it is incorrectly occupied, you can de-init it using the ```BME_280_DeInit(&handler1);```. Also NOTE THAT out of the pool of 10 instances their is only one instance can be used for I2C if you try to allocate another instance for I2C the driver won't allow you, this is becauese you the sensor can have only one address in the same I2C bus

``` C

	BME_280_Status result =BME_280_getInstance(&handler2);
	if(result == BME280_OK)
	{
	  // success
	}
	else
	{
	  // see the type of error that the API returns, and you can understand the error from the error name returned
	}

```
3. After an instance has been obtained successfully, we must specify the interface type. and in this step the driver will stop you and will de init you instance if  you try to allocate more than one instance for I2C.

```C
   /* Here we fitst supply teh interface type first */
   result = BME_280_setInterfaceProtocol(&handler2, BME_280_INTERFACE_SPI);
   
   /* The result will be BME280_CANNOT_HAVE_MORE_THAN_ONE_INSTANCE_OF_I2C if the user violate the above constrain about I2C */
  
```
4. After these steps have been completed successfully without any errors, the user can init the sensor and check if communication is successful.

```C
	result = BME_280_Init(&handler2);
	if(result!=BME280_OK)
	  /* Error initializing */
  
```
5. Next, the user can set their specific device settings and obtain readings as necessary. It must be noted that functions which obtain readings do not set/check if valid settings are set on the sensor. Rather, they obtain readings and convert instantly; therefore, the user must set their settings before before Step 6.
```C
    
    /* IF the return of the these APIs is BME280_OK then every operation is done successfully */
	result = BME_280_SetTempOverSamplingSetting(&handler2, OVERSAMPLING_2);
	result = BME_280_SetPressureOverSamplingSetting(&handler2, OVERSAMPLING_4);
	result = BME_280_SetHumidityOverSamplingSetting(&handler2, OVERSAMPLING_8);
	result = BME_280_SetFilterSettings(&handler2, _4);

	result = BME_280_SetMode(&handler2, NORMAL_MODE);
	result = BME_280_SetStandbyTime(&handler2,_125_MS);

	result = BME_280_getSensorSettings(&handler2, &settings);
    /* This API is helpful to make sure that all the setting set by the user is successfully set, also the user can rely on the status return of each of the above     
     * APIs 
     */
    BME_280_settings settings;
    result = BME_280_getSensorSettings(&handler2, &settings);
```
6. Finally, readings can be obtained and processed.

```C		
    /* Declare variables for Sensor Readings */
    BME_280_TempType_fixedPoint  temp_fixed = 0;
    BME_280_TempType_floatingPoint  temp_float = 0;

   BME_280_PressureType_fixedPoint  pressure_fixed = 0;
   BME_280_PressureType_floatingPoint pressure_float = 0;

   BME_280_HumidityType_fixedPoint  humidity_fixed = 0;
   BME_280_HumidityType_floatingPoint humidity_float = 0;

    /* Finally get the reading of the functions using these APIs and  If the return of the these APIs is BME280_OK then every operation is done successfully */
    result = BME_280_getTemperature_fixedPoint(&handler2, &temp_fixed);
    result = BME_280_getTemperature_floatingPoint(&handler2, &temp_float);

    result = BME_280_getPressure_fixedPoint(&handler2, &pressure_fixed);
    result = BME_280_getPressure_floatingPoint(&handler2, &pressure_float);

   result = BME_280_getHumidity_fixedPoint(&handler2, &humidity_fixed);
   result = BME_280_getHumidity_floatingPoint(&handler2, &humidity_float);

```
6. After finishing communicating with the sensor make sure to de-Init your handler to free your instance and free resouorces for others.

```C		
   result = BME_280_DeInit(&handler2);
```
