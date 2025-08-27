/*
 * spi_manager.h
 *
 *  Created on: Jul 25, 2025
 *      Author: hillerj
 */

#ifndef INC_SPI_MANAGER_H_
#define INC_SPI_MANAGER_H_

//
// Includes
//

#include "stm32h7xx_hal.h"
#include "spi_mutex_map.h"

//
// Functions
//

HAL_StatusTypeDef SPI_LockAndReadRegister16Bit(SPI_HandleTypeDef *hspi, uint8_t devAddr, uint8_t register_pointer, uint8_t* receive_buffer);
HAL_StatusTypeDef SPI_LockAndWriteRegister16Bit(SPI_HandleTypeDef *hspi, uint8_t devAddr, uint8_t register_pointer, uint16_t register_value);
HAL_StatusTypeDef SPI_LockAndisDeviceReady(SPI_HandleTypeDef *hspi, uint8_t devAddr);

#endif /* INC_SPI_MANAGER_H_ */
