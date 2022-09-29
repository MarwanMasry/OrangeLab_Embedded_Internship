/******************************************************************************
 *
 * Module: Bosch Sensortec BME280
 *
 * File Name: bme280.h
 *
 * Description: Header file for BME280 Sensor example functions for weak functions on STM32 using
 * 			HAL drivers.
 *
 * Date Created: 29/9/2022
 *
 * Author: Hazem Montasser
 *
 *******************************************************************************/

#ifndef BME280_STM32_WEAK_EXAMPLE_IMPL_H_
#define BME280_STM32_WEAK_EXAMPLE_IMPL_H_

/*******************************************************************************
 *                          SPI function examples                              *
 *******************************************************************************/

/**
 * @fn BME280_Comm_Status BME280_SPI_TransmitReceive(BME280_uint8*, BME280_uint8*, BME280_uint16, BME280_uint32)
 * @brief
 *
 * @param txData	Data to transmit
 * @param rxData	Data to receive
 * @param size		Number of bytes to transmit
 * @param timeout	Timeout in ms
 * @return
 */
BME280_Comm_Status BME280_SPI_TransmitReceive(BME280_uint8 *txData,
		BME280_uint8 *rxData, BME280_uint16 size, BME280_uint32 timeout) {
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, txData, rxData,
			size, timeout);
	if (status != HAL_OK)
		return BME280_Comm_Error;
	else
		return BME280_Comm_OK;
}

/**
 * @fn void BME280_GPIO_setSlaveSelectPin(void)
 * @brief Sends high logic on slave select pin. Assumed to be connected to GPIOA, Pin 4
 *
 */
void BME280_GPIO_setSlaveSelectPin(void) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

/**
 * @fn void BME280_GPIO_resetSlaveSelectPin(void)
 * @brief Sends low logic on slave select pin. Assumed to be connected to GPIOA, Pin 4
 *
 */
void BME280_GPIO_resetSlaveSelectPin(void) {

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

/*******************************************************************************
 *                          I2C function examples                              *
 *******************************************************************************/

/**
 * @fn BME280_Status BME280_I2C_Master_Transmit(BME280_uint8, BME280_uint8*, BME280_uint16, BME280_uint32)
 * @brief
 *
 *  @param sensorAddr	Sensor address (assumed to be 0x76 if SDO is connected to GND)
 * @param rxData		Data to send
 * @param size			Number of bytes to receive
 * @param timeout		Timeout in ms
 * @return
 */
BME280_Status BME280_I2C_Master_Transmit(BME280_uint8 sensorAddr,
		BME280_uint8 *txData, BME280_uint16 size, BME280_uint32 timeout) {
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, sensorAddr,
			txData, size, timeout);
	if (status != HAL_OK)
		return BME280_Comm_Error;
	else
		return BME280_Comm_OK;
}

/**
 * @fn BME280_Status BME280_I2C_Master_Receive(BME280_uint8, BME280_uint8*, BME280_uint16, BME280_uint32)
 * @brief Receives data from sensor.
 *
 * @param sensorAddr	Sensor address (assumed to be 0x76 if SDO is connected to GND)
 * @param rxData		Buffer to receive data in
 * @param size			Number of bytes to receive
 * @param timeout		Timeout in ms
 * @return
 */
BME280_Status BME280_I2C_Master_Receive(BME280_uint8 sensorAddr,
		BME280_uint8 *rxData, BME280_uint16 size, BME280_uint32 timeout) {
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, sensorAddr,
			rxData, size, timeout);
	if (status != HAL_OK)
		return BME280_Comm_Error;
	else
		return BME280_Comm_OK;
}

/*******************************************************************************
 *                          General functions                                  *
 *******************************************************************************/

/**
 * @fn BME280_Status BME280_delayMs(BME280_uint32)
 * @brief Blocking delay for the specific milliseconds.
 *
 * @param a_milliseconds
 * @return	Unused.
 */
BME280_Status BME280_delayMs(BME280_uint32 a_milliseconds) {
	HAL_Delay(a_milliseconds);
	return 1;
}

#endif /* BME280_STM32_WEAK_EXAMPLE_IMPL_H_ */
