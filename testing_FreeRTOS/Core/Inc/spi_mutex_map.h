/*
 * spi_mutex_map.h
 *
 *  Created on: Jul 25, 2025
 *      Author: hillerj
 */

#ifndef INC_SPI_MUTEX_MAP_H_
#define INC_SPI_MUTEX_MAP_H_

//
// Includes
//

#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "semphr.h"

//
// Defines
//

#define MAX_SPI_HANDLES		2

//
// Struct
//

typedef struct {
	SPI_HandleTypeDef *handle;
	SemaphoreHandle_t mutex;
} SPIHandleMutexPair;

//
// Functions
//

void registerSPIHandleWithMutex(SPI_HandleTypeDef *hspi, SemaphoreHandle_t mutex);
SemaphoreHandle_t getMutexForHandle_SPI(SPI_HandleTypeDef *hspi);
#endif /* INC_SPI_MUTEX_MAP_H_ */
