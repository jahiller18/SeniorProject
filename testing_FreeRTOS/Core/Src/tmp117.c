/*
 * tmp117.c
 *
 *  Created on: Jul 16, 2025
 *      Author: hillerj
 */

//
// Includes
//

#include "tmp117.h"

//
// Pack Config Function
//

uint16_t pack_config(config_bits_t *b) {
	return  (0 << 15) 								|	// High Alert Flag  -- Read Only
	        (0 << 14)								|	// Low Alert Flag   -- Read Only
	        (0 << 13) 								|	// Data Ready Flag  -- Read Only
	        (0 << 12) 								|	// EEPROM Busy Flag -- Read Only
	        ((b->CONVERSION_MODE      & 0x3) << 10) |
	        ((b->CONV                 & 0x7) << 7)  |
	        ((b->CONVERSION_AVG_MODE  & 0x3) << 5)  |
	        ((b->THERM_ALERT          & 0x1) << 4)  |
	        ((b->ALERT_POL            & 0x1) << 3)  |
	        ((b->DR_ALERT             & 0x1) << 2)  |
	        ((b->SOFT_RESET           & 0x1) << 1)  |
	        (0 << 0); // Read-only, usually set to 0 when writing
}

//
// Initialization
//

uint8_t TMP117_Initialization(TMP117 *dev, I2C_HandleTypeDef *i2cHandle) {

	// Store I2C handle with TMP117 device typedef
	dev->i2cHandle = i2cHandle;

	// Initialize Configuration Structs to hold Configuration Data
	// Updtae Bit Fields with Reset Values

	tmp117_configuration_t config;
	config.bits.CONVERSION_MODE = 0;
	config.bits.CONV = 0x4;
	config.bits.CONVERSION_AVG_MODE = 0x1;
	config.bits.THERM_ALERT = 0;
	config.bits.ALERT_POL = 0;
	config.bits.DR_ALERT = 0;
	config.bits.SOFT_RESET = 0;

	// Pack Bit Field in uint16_t byte
	config.reg = pack_config(&config.bits);

	// Initialize Necessary Variables
	HAL_StatusTypeDef status;

	uint8_t buffer[2];

	// Check Connnection and that Device is ready
	status = HAL_I2C_IsDeviceReady(dev->i2cHandle, TMP117_I2C_ADDR_GND << 1, 1, HAL_MAX_DELAY);
	if (status != HAL_OK) {
		return 255;
	}

	// Check Device ID and ensure matches correct value
	status = TMP117_ReadRegister(dev, TMP117_DEVICE_ID, buffer);
	if (status != HAL_OK) {
		return 225;
	}

	// Consolidate 12 bit Device ID
	uint16_t deviceID = (buffer[0] << 8) | buffer[1];

	// Check that Device ID matches
	if (deviceID != 0x117) {
		return 255;
	}

	return 0;
}

//
// Functions
//

uint8_t TMP117_GetTemperatureData(TMP117 *dev) {

	// Variable to hold the temp result register value
	uint16_t tempResult = 0;

	// Status Typedef variable
	HAL_StatusTypeDef status;

	// Data Buffer
	uint8_t buffer[2];
	uint16_t data;

	// Set Mode of Operating to One Shot Mode
	// After Conversion Conversion Mode will return to shutdown Mode
	// Data Sheet p.15
	dev->config.bits.CONVERSION_MODE = 0x3;
	dev->config.reg = pack_config(&dev->config.bits);

	// Update Config Register with One-Shot Mode
	status = TMP117_WriteRegister(dev, TMP117_CONFIGURATION, dev->config.reg);

	// Wait for Conversion to be done
	// Check Data ready flag in Configuration Register
	// If not ready call freeRTOS delay to free the processor
	do {
		// Get Aprox Conversion Time (TYP 15.5ms) as stated on datasheet p.6
		// USE FREERTOS DELAY HERE
		HAL_Delay(16);

		// Check new value of Configuration Register
		status = TMP117_ReadRegister(dev, TMP117_CONFIGURATION, buffer);
		data = (buffer[0] << 8) | buffer[0];



	} while ((data & (1 << 13)) != 0);

	// Once Temp Result Register is read, the Data Ready Flag will flip back to 0

	// Get register value
	status = TMP117_ReadRegister(dev, TMP117_TEMP_RESULT, buffer);

	// Store Register Value in tempResult
	tempResult = (buffer[0] << 8) + buffer[1];

	// Convert to celcius
	uint16_t cel_temp_value = TMP117_TwosCompToInt(tempResult);
	dev->temp_Celcius = cel_temp_value * TMP117_RESOLUTION;

	return 0;

}

uint16_t TMP117_TwosCompToInt(uint16_t temp) {

	// [0x0000; 0x7FFF] corresponds to [0; 32,767]
	// [0x8000; 0xFFFF] corresponds to [-32,768; -1]
	// int16_t has the range [-32,768; 32,767]

	uint16_t sign_mask = 0x8000;

	// if positive
	if ( (temp & sign_mask) == 0 ) {
		return temp;
	}
	// if negative
	else {
		// invert all bits and one and make negative
		return -(~temp + 1);
	}
}

//
// Configuration
//

uint8_t TMP117_GetConfiguration(TMP117 *dev) {

	// Init status and data buffer
	HAL_StatusTypeDef status;
	uint8_t dataBuffer[2];

	// Get configuration data
	status = TMP117_Readregister(dev, TMP117_CONFIGURATION, &dataBuffer);
	if ( status != HAL_OK ) {
		return 255;
	}

	// Shift data
	uint16_t config = (dataBuffer[0] << 8) + dataBuffer[1];


	// Update Configuration Data in dev typedef
	//dev->config.REG_0x01.byte = config;

	return 0;
}

uint8_t TMP117_SetConfiguration(TMP117 *dev) {

	// Init status and data buffer
	HAL_StatusTypeDef status;
	uint8_t dataBuffer[2];

	// Get New Configuration Data off TMP117 typedef
	//uint16_t config = dev->config.REG_0x01.byte;

	//Store in dataBuffer
	//dataBuffer[1] = config & 0xFF;
	//dataBuffer[0] = (config >> 8);

	// Update Sensor Configs
	status = TMP117_WriteRegister(dev, TMP117_CONFIGURATION, &dataBuffer);
	if ( status != HAL_OK ) {
		return 255;
	}

	return 0;
}

//
// Low Level Functions
//

HAL_StatusTypeDef TMP117_ReadRegister(TMP117 *dev, uint8_t register_pointer, uint8_t* receive_buffer) {

	// Define a status variable to check that transmission was successful
	HAL_StatusTypeDef status;
	uint8_t buffer[2];

	// Set the register pointer to the register wanted to be read
	status = HAL_I2C_Master_Transmit(dev->i2cHandle, TMP117_I2C_ADDR_GND << 1, &register_pointer, 1, HAL_MAX_DELAY);

		// Confirm no issues with transmission
		if ( status != HAL_OK) {
			return status;
		}

		// Receive the 2 x 8bit data into the receive buffer
		return HAL_I2C_Master_Receive(dev->i2cHandle, TMP117_I2C_ADDR_GND << 1, receive_buffer, 2, 100);



}
HAL_StatusTypeDef TMP117_WriteRegister(TMP117 *dev, uint8_t register_pointer, uint16_t register_value) {

	uint8_t data[3];

	data[0] = register_pointer;
	data[1] = register_value >> 8;
	data[2] = register_value;

	return HAL_I2C_Master_Transmit(dev->i2cHandle, TMP117_I2C_ADDR_GND << 1, data, 3, 100);
}
