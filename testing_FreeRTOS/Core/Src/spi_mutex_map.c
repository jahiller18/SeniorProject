/*
 * spi_mutex_map.c
 *
 *  Created on: Jul 25, 2025
 *      Author: hillerj
 */

//
// Includes
//

#include <stddef.h>
#include "spi_mutex_map.h"

//
// Defines
//

static SPIHandleMutexPair spiMutexMap[MAX_SPI_HANDLES];
static int numRegisteredHandles = 0;

//
// Functions
//

void registerSPIHandlesWithMutex(SPI_HandleTypeDef *hspi, SemaphoreHandle_t mutex) {
	if (numRegisteredHandles < MAX_SPI_HANDLES) {
		spiMutexMap[numRegisteredHandles].handle = hspi;
		spiMutexMap[numRegisteredHandles].mutex = mutex;
		numRegisteredHandles++;
	}
}

SemaphoreHandle_t getMutexForHandle_SPI(SPI_HandleTypeDef *hspi) {
	for (int i = 0; i < numRegisteredHandles; i++) {
		if (spiMutexMap[i].handle == hspi) {
			return spiMutexMap[i].mutex;
		}
	}
	return NULL;
}
