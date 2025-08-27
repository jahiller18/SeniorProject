/*
 * i2c_mutex_map.c
 *
 *  Created on: Jul 24, 2025
 *      Author: hillerj
 */

//
// Includes
//

#include <stddef.h>
#include "i2c_mutex_map.h"

//
// Defines
//

static I2CHandleMutexPair i2cMutexMap[MAX_I2C_HANDLES];
static int numRegisteredHandles = 0;

//
// Functions
//

void registerI2CHandleWithMutex(I2C_HandleTypeDef *hi2c, SemaphoreHandle_t mutex) {
	if (numRegisteredHandles < MAX_I2C_HANDLES) {
		i2cMutexMap[numRegisteredHandles].handle = hi2c;
		i2cMutexMap[numRegisteredHandles].mutex = mutex;
		numRegisteredHandles++;
	}
}

SemaphoreHandle_t getMutexForHandle_I2C(I2C_HandleTypeDef *hi2c) {
	for (int i = 0; i < numRegisteredHandles; i++) {
		if (i2cMutexMap[i].handle == hi2c) {
			return i2cMutexMap[i].mutex;
		}
	}
	return NULL;
}
