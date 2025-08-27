/*
 * i2c_manager.h
 *
 *  Created on: Jul 24, 2025
 *      Author: hillerj
 */

#ifndef INC_I2C_MANAGER_H_
#define INC_I2C_MANAGER_H_

#include "stm32h7xx_hal.h"
#include "i2c_mutex_map.h"

void I2C_Manager_Init(void);

HAL_StatusTypeDef I2C_LockAndReadRegister16Bit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t register_pointer, uint8_t* receive_buffer);

HAL_StatusTypeDef I2C_LockAndWriteRegister16Bit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t register_pointer, uint16_t register_value);

HAL_StatusTypeDef I2C_LockAndReadRegister8Bit(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t register_pointer, uint8_t* receive_buffer);

HAL_StatusTypeDef I2C_LockAndWriteRegister8it(I2C_HandleTypeDef *hi2c, uint8_t devAddr, uint8_t register_pointer, uint16_t register_value);

HAL_StatusTypeDef I2C_LockAndIsDeviceReady(I2C_HandleTypeDef *hi2c, uint8_t devAddr);

#endif /* INC_I2C_MANAGER_H_ */
