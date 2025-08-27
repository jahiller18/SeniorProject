/*
 * fault_manager.h
 *
 *  Created on: Jul 24, 2025
 *      Author: hillerj
 */

#ifndef INC_FAULT_MANAGER_H_
#define INC_FAULT_MANAGER_H_

//
// Includes
//

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

//
// Fault Sources (Subsystems)
//

typedef enum {
	FUALT_SOURCE_COMM,
	FUALT_SOURCE_POWER,
	FUALT_SOURCE_THERMAL,
	FUALT_SOURCE_ADCS,
	FUALT_SOURCE_SOFTWARE,
	FUALT_SOURCE_STORAGE,
	FUALT_SOURCE_PAYLOAD,
} fault_source_t;

//
// Fault Severity
//

typedef enum {
	FAULT_SEVERITY_INFO,
	FAULT_SEVERITY_WARN,
	FAULT_SEVERITY_CRITICAL
} fault_severity_t;

//
// Fault Event
//

typedef struct {
	fault_source_t source;
	uint8_t code;
	fault_severity_t severity;
	uint32_t timestamp;
	char description[64];
} fault_event_t;

extern QueueHandle_t xFaultQueue;
void vFaultManagerTask(void *pvParameters);
void log_event(const char *msg);
void enter_safe_mode(void);



#endif /* INC_FAULT_MANAGER_H_ */
