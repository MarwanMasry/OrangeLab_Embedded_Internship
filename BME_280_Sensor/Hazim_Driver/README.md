# BME280 Driver (Hazem's version)

## Overview

This driver was developed for Bosch Sensortec BME280 Sensor. It measures temperature, pressure, and humidity. The driver allows interfacing with the sensor through I2C or SPI. Each sensor setting is 
configurable through API functions, and the driver supports both fixed and floating point arithmetic.

## How to use this driver

Before using the driver at all, several functions must be implemented by the user as they are target specific. These functions are in the bme280.h file at the very end. They are defined as weak functions to be implemented according to the user.
These functions are:
  - BME280_delayMs
  - BME280_SPI_TransmitReceive
  - BME280_GPIO_WriteSlaveSelectPin
  - BME280_I2C_Transmit
  - BME280_I2C_Receive

Example implemntations can be found in the weak_impl_example.h file.
### Steps
1. Define a sensor handle like so, this is the instance which will be used to specifically communicate with the sensor if multiple exist.
``` C
BME280_Handle my_sensor;
```

2. Now, we must obtain an instance from the sensor pool which holds handles (up to 5). Note that the handle must be unoccupied or used before, if it is incorrectly occupied, you can de-init it using the ```BME280_DeInit(&my_sensor);```

``` C

BME280_Status result = BME280_getInstance(my_sensor);
if(result!=BME280_FOUND_EMPTY_INSTANCE)
  //Error initializing

```
3. After an instance has been obtained successfully, we must specify the interface type and setup specific options, for example if we are using SPI:

```C

BME280_setInterfaceType(&my_sensor, BME280_Interface_SPI);
BME280_setSlaveSelectSPin(&my_sensor,4);         // 4 should be mapped to user, for ex. in STM32 -> GPIO_PIN_4
BME280_setSlaveSelectPort(&my_sensor,PORTA_DEF); // PORTA_DEF  should be mapped by the user, for ex. in STM32 -> GPIOA
/* Note that the pin and port are just to identify the sensor if multiple exist,
and the user can map these stored variables according to their desire*/
```
4. After these steps have been completed successfully, the user can init the sensor and check if communication is successful.

```C
result = BME280_init(&my_sensor);
if(result!=BME280_OK)
  //Error initializing
```
5. Next, the user can set their specific device settings and obtain readings as necessary. It must be noted that functions which obtain readings do not set/check if valid settings are set on the sensor. Rather, they obtain readings and convert instantly; therefore, the user must set their settings beforehand.
```C
/* Status code is discarded but can be checked for each function call for success.*/
BME280_setFilterCoefficient(&my_sensor, BME280_FILTER_COEFF_2);
BME280_setTemperatureOversampling(&my_sensor, BME280_OVERSAMPLING_x16);
BME280_setHumidityOversampling(&my_sensor, BME280_OVERSAMPLING_x8);
BME280_setPressureOversampling(&my_sensor,BME280_OVERSAMPLING_x4);
BME280_setStandbyTime(&my_sensor, BME280_T_STDBY_125_MS);
BME280_setMode(&my_sensor, BME280_Mode_Normal);
```
6. Finally, readings can be obtained and processed.

```C		

BME280_float64 temp_float = 0;
BME280_float64 pressure_float = 0;
BME280_float64 humidity_float = 0;

BME280_getTemperature_floatingPoint(&a_cfgPtr, &temp_float);
BME280_getPressure_floatingPoint(&a_cfgPtr, &pressure_float);
BME280_getHumidity_floatingPoint(&a_cfgPtr, &humidity_float);

printf("Temp = %f c\n",&temp_float);
printf("Pressure = %f Pa\n",&pressure_float);
printf("RH = %f %%\n",&humidity_float);

```





		

