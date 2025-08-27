/*
 * i2c_mutex_map.h
 *
 *  Created on: Jul 24, 2025
 *      Author: hillerj
 */

#ifndef INC_I2C_MUTEX_MAP_H_
#define INC_I2C_MUTEX_MAP_H_

//
// Includes
//

#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "semphr.h"

//
// Defines
//

#define MAX_I2C_HANDLES		 3

//
// Struct
//

typedef struct {
	I2C_HandleTypeDef *handle;
	SemaphoreHandle_t mutex;
} I2CHandleMutexPair;

//
// Functions
//

void registerI2CHandleWithMutex(I2C_HandleTypeDef *hi2c, SemaphoreHandle_t mutex);
SemaphoreHandle_t getMutexForHandle_I2C(I2C_HandleTypeDef *hi2c);

#endif /* INC_I2C_MUTEX_MAP_H_ */
