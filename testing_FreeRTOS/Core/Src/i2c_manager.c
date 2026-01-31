/*
 * i2c_manager.c
 *
 *  Created on: Jul 24, 2025
 *      Author: hillerj
 */


//
// Includes
//

#include "i2c_manager.h"
#include "cmsis_os.h"
#include "semphr.h"

//
// Semaphores
//

HAL_StatusTypeDef I2C_LockAndReadRegister16Bit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t register_pointer, uint8_t* receive_buffer) {
	SemaphoreHandle_t mutex = getMutexForHandle_I2C(hi2c);
	if (!mutex) return HAL_ERROR;

	if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
		HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, devAddr, &register_pointer, 1, 100);
		if (status != HAL_OK) {
			xSemaphoreGive(mutex);
			return status;
		}

		status = HAL_I2C_Master_Receive(hi2c, devAddr, receive_buffer, 2, 100);
		xSemaphoreGive(mutex);
		return status;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef I2C_LockAndWriteRegister16Bit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t register_pointer, uint16_t register_value) {
	SemaphoreHandle_t mutex = getMutexForHandle_I2C(hi2c);
	if (!mutex) return HAL_ERROR;
	uint8_t data[3];
	data[0] = register_pointer;
	data[1] = register_value >> 8;
	data[2] = register_value;

	if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
		HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, devAddr, data, 3, 100);
		xSemaphoreGive(mutex);
		return status;
		}
		return HAL_ERROR;
}

HAL_StatusTypeDef I2C_LockAndReadRegister8Bit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t register_pointer, uint8_t* receive_buffer) {
	SemaphoreHandle_t mutex = getMutexForHandle_I2C(hi2c);
	if (!mutex) return HAL_ERROR;

	if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
		HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, devAddr, &register_pointer, 1, 100);
		if (status != HAL_OK) {
			xSemaphoreGive(mutex);
			return status;
		}

		status = HAL_I2C_Master_Receive(hi2c, devAddr, receive_buffer, 1, 100);
		xSemaphoreGive(mutex);
		return status;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef I2C_LockAndWriteRegister8Bit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t register_pointer, uint8_t register_value) {
	SemaphoreHandle_t mutex = getMutexForHandle_I2C(hi2c);
	if (!mutex) return HAL_ERROR;
	uint8_t data[2];
	data[0] = register_pointer;
	data[1] = register_value;

	if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
		HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, devAddr, data, 2, 100);
		xSemaphoreGive(mutex);
		return status;
		}
		return HAL_ERROR;
}

HAL_StatusTypeDef I2C_LockAndIsDeviceReady(I2C_HandleTypeDef *hi2c, uint8_t devAddr) {
	SemaphoreHandle_t mutex = getMutexForHandle_I2C(hi2c);
	if (!mutex) return HAL_ERROR;

	if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
		HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, devAddr, 1, HAL_MAX_DELAY);
		xSemaphoreGive(mutex);
		return status;
	}
	return HAL_ERROR;
}

