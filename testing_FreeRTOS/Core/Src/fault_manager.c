/*
 * fault_manager.c
 *
 *  Created on: Jul 24, 2025
 *      Author: hillerj
 */

#include "fault_manager.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

fault_event_t faultLog[50];
uint8_t faultLogIndex;
uint8_t faultCounts[8] = {0};
QueueHandle_t xFaultQueue;

static void store_fault(fault_event_t *event) {
	if (faultLogIndex < 50) {
		faultLog[faultLogIndex++] = *event;
	}
}

static void issue_warning(fault_source_t src) {
	printf("Warning: Fault from source %d\n", src);
}

static void handle_critical_fault(fault_source_t src) {
	printf("Critical fault from source %d\n", src);
	switch(src) {
		case FUALT_SOURCE_POWER:	log_event("Power fault - low power mode"); break;
		case FUALT_SOURCE_THERMAL:	log_event("Thermal fault - throttling"); break;
		case FUALT_SOURCE_ADCS:		log_event("ADCS fault - detumbling"); break;
		case FUALT_SOURCE_COMM:		log_event("Comm fault - beacon mode"); break;
		default: break;
	}

	//enter_safe_mode();

}

static void escalate_fault(void *pvParameters) {
	log_event("Escalating fault to safe mode");
	//enter_safe_mode();
}

void vFaultManagerTask(void *pvParameters) {
	fault_event_t event;
	while(1) {
		if (xQueueReceive(xFaultQueue, &event, portMAX_DELAY) == pdPASS) {
			store_fault(&event);
			faultCounts[event.source]++;

			switch(event.severity) {
				case FAULT_SEVERITY_INFO: break;
				case FAULT_SEVERITY_WARN: issue_warning(event.source); break;
				case FAULT_SEVERITY_CRITICAL: handle_critical_fault(event.source); break;
			}

			if (faultCounts[event.source] >= 3) {
				//escalate_fault(event.source);
			}
		}
	}
}

void log_event(const char *msg) {
	printf("LOG: %s\n", msg);
}

void enter_safe_mode(void) {
	printf("Entering Safe Mode");
}
