/*
 * task_manager.h
 *
 *  Created on: Jul 25, 2025
 *      Author: hillerj
 */

#ifndef INC_TASK_MANAGER_H_
#define INC_TASK_MANAGER_H_

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "power_manager.h"

#define MAX_TASKS		15
#define HEARTBEAT_TIMEOUT 		pdMS_TO_TICKS(4000)
#define MONITOR_PERIOD			pdMS_TO_TICKS(500)

typedef enum {
	TASK_TYPE_CRITICAL = 0,
	TASK_TYPE_NORMAL,
	TASK_TYPE_SUBSYSTEM,
	TASK_TYPE_COUNT
} TaskType_t;

typedef struct {
	TaskHandle_t handle;
	const char *name;
	TickType_t lastHeartbeat;
	UBaseType_t priority;
	bool isHealthy;
	TaskType_t type;
	Subsystem_t subsystem;
	bool isSuspended;
	bool wasRunningBeforePowerChange;

} MonitoredTask_t;

//
// Functions
//

void TaskManager_Init(void);
bool TaskManager_Register(TaskHandle_t handle, const char *name, UBaseType_t priority,
						 TaskType_t type, Subsystem_t subsystem);
void TaskManager_SendHeartbeat(TaskHandle_t handle);
void TaskManager_Task(void* pvParameters);

void TaskManager_SuspendAllTasks(void);
void TaskManager_SuspendAllNonCriticalTasks(void);
void TaskManager_ResumeAllTasks(void);
void TaskManager_ResumeCriticalTasks(void);
void TaskManager_ManageSubsystemTasks(Subsystem_t subsystem, bool enable);

bool TaskManager_IsTaskHealthy(TaskHandle_t handle);
uint32_t TaskManager_GetUnhealthyTaskCount(void);
void TaskManager_GetTaskStatus(TaskHandle_t handle, eTaskState *state, bool *isHealthy);

PowerResult_t TaskManager_PowerModeCallback(PowerMode_t fromMode, PowerMode_t toMode);

#endif /* INC_TASK_MANAGER_H_ */
