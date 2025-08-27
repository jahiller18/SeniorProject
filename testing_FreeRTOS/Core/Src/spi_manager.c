/*
 * spi_manager.c
 *
 *  Created on: Jul 25, 2025
 *      Author: hillerj
 */

#include "spi_manager.h"
#include "cmsis_os.h"
#include "semphr.h"

HAL_StatusTypeDef SPI_LockAndReadRegsiter16Bit(SPI_HandleTypeDef *hspi, uint8_t devAddr, uint8_t register_pointer, uint8_t* receive_buffer) {
	SemaphoreHandle_t mutex = getMutexForHandle_SPI(hspi);
	if (!mutex) return HAL_ERROR;

	if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
		// Define SPI communication here
		HAL_StatusTypeDef status = HAL_OK;

		// Return Mutex
		xSemaphoreGive(mutex);
		return status;
	}
	return HAL_ERROR;
}

HAL_StatusTypeDef SPI_LockAndIsDeviceReady(SPI_HandleTypeDef *hspi, uint8_t devAddr) {
	SemaphoreHandle_t mutex = getMutexForHandle_SPI(hspi);
	if (!mutex) return HAL_ERROR;

	if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
		// Define Communication here
		HAL_StatusTypeDef status = HAL_OK;

		// Return Mutex
		xSemaphoreGive(mutex);
		return status;
	}
	return HAL_ERROR;
}
