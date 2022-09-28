# BME280 Driver (Hazem's version)

## Overview

This driver was developed for Bosch Sensortec BME280 Sensor. It measures temperature, pressure, and humidity. The driver allows interfacing with the sensor through I2C or SPI. Each sensor setting is 
configurable through API functions, and the driver supports both fixed and floating point arithmetic.

## Driver features
- Generic driver which can support any target given the proper implementation of the weak functions.
- Multiple instances of the sensor if using SPI (up to 5)
- Individual setters and getters for all sensor settings:
	- Filter coefficient
	- Standby time
	- Temperature, pressure, and humidity oversampling settings
	- Sensor mode
- Fixed and floating point support. See functions post-fixed with ```  _fixedPoint``` and ``` _floatingPoint ```
- Pre-conversion measurement delay function to know the exact measurement time with chosen settings before getting a reading from the sensor. See ``` BME280_calculateMeasurementDelayMs```
- Status reporting functions for the sensor. Whether it is updating its registers or taking a measurement. See ``` BME280_getUpdateStatus and BME280_getMeasuringStatus```
- Unit testing functions for all functions included in __bme280_unitTests.h__

## File information

#### Header files
- __bme280.h__: Contains main API abstracted functions needed to use this driver as well as prototypes for weak functions which are target specific. 
- __bme280_private_types.h__: Contains several structures, unions, and the sensor main handle structure. Used internally within the driver.
- __bme280_private_defs.h__: Contains definitions for device register, limits, and several macros.
- __bme280_unitTests.h__: Contains prototypes for unit test functions. Used to test all functionalities of the driver. Has both I2C and SPI functions.
- __std_types.h__: Contains custom types and some definitions used internally.

#### Source files
- __bme280.c__: Source file for the driver
- __bme280_unitTests.c__: Contains definitions for unit test function implementations.

## How to use this driver

Before using the driver at all, several functions __*must*__ be implemented by the user as they are target specific. These functions are in the bme280.h file at the very end. They are defined as __weak__ functions to be implemented according to the user.

These functions are:
  - BME280_delayMs
  - BME280_SPI_TransmitReceive
  - BME280_I2C_Master_Transmit
  - BME280_I2C_Master_Receive

Example implemntations can be found in the bme280_weak_impl_example.h file.

## Supported interfaces
- I2C (1 sensor only)
- SPI (up to 5 sensors simultaneously)

### Steps
1. Define a sensor handle like so, this is the instance which will be used to specifically communicate with the sensor if multiple sensors exist.
``` C
BME280_Handle my_sensor;
```

2. Now, we must obtain an instance from the sensor pool which holds handles (up to 5 for SPI). Note that the handle must be unoccupied and unused before, if it is incorrectly occupied, you can de-init it using the ```BME280_DeInit(&my_sensor);```

``` C

BME280_Status result = BME280_getInstance(my_sensor);
if(result!=BME280_FOUND_EMPTY_INSTANCE)
  //Error initializing

```
3. After an instance has been obtained successfully, we must specify the interface type .

##### For SPI
- For SPI, these callbacks are called when driving the slave select pin low or high during SPI read/write.
```C
BME280_setInterfaceType(&my_sensor, BME280_Interface_SPI);
/* Set callbacks*/
BME280_setAssertNSSCallback(&my_sensor, set_nss_func);
BME280_setReleaseNSSCallback(&my_sensor, reset_nss_func);
```

#### For I2C

```C
BME280_setInterfaceType(&my_sensor, BME280_Interface_I2C);
```

_Note_:  For SPI, it is __required__ to have a GPIO pin configured to be controlled internally by the driver when needing to read/write. While in I2C, it is only required to set the interface and connect the CSB pin to VDDIO.

4. After these steps have been completed successfully, the user can init the sensor and check if communication is successful.

```C
result = BME280_init(&my_sensor);
if(result!=BME280_OK)
  //Error initializing
```

5. Next, the user can set their specific device settings and obtain readings as necessary. It must be noted that functions which obtain readings __do not set/check__ if valid settings are set on the sensor. Rather, they obtain readings and convert instantly; therefore, the user must set their settings beforehand.

```C
/* Status code is discarded but can be checked for each function call for success.*/

/* Setting filter coefficient*/
BME280_setFilterCoefficient(&my_sensor, BME280_FILTER_COEFF_2);
/* Setting over-sampling options */
BME280_setTemperatureOversampling(&my_sensor, BME280_OVERSAMPLING_x16);

BME280_setHumidityOversampling(&my_sensor, BME280_OVERSAMPLING_x8);

BME280_setPressureOversampling(&my_sensor,BME280_OVERSAMPLING_x4);

/* Setting standby time*/
BME280_setStandbyTime(&my_sensor, BME280_T_STDBY_125_MS);
/* Setting mode, sensor starts conversion */
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

