/*
 * task_manager.c
 *
 *  Created on: Jul 25, 2025
 *      Author: hillerj
 */

//
// Includes
//

#include "task_manager.h"
#include "fault_manager.h"
#include "cmsis_os.h"
#include <string.h>

//
// Defines
//

static MonitoredTask_t monitoredTasks[MAX_TASKS];
static uint8_t task_count = 0;
static bool isInitialized = false;
static TaskHandle_t taskManagerHandle = NULL;

static void suspendTaskSafely(MonitoredTask_t *task);
static void resumeTaskSafely(MonitoredTask_t *task);
static MonitoredTask_t* findTaskByHandle(TaskHandle_t handle);
static MonitoredTask_t* findTaskBySubsystem(Subsystem_t subsystem);


void TaskManager_Init(void) {

	if (isInitialized) {
		return;
	}

	memset(monitoredTasks, 0, sizeof(monitoredTasks));
	task_count = 0;

	osThreadId_t TaskManagerHandle;
	const osThreadAttr_t TaskManager_attributes = {
	  .name = "TaskMgr",
	  .stack_size = 128 * 4,
	  .priority = (osPriority_t) osPriorityHigh,
	};

	TaskManagerHandle = osThreadNew(TaskManager_Task, NULL, &TaskManager_attributes);

	TaskManager_Register(taskManagerHandle, "TaskMgr", configMAX_PRIORITIES - 1,
							TASK_TYPE_CRITICAL, SUBSYSTEM_COUNT);
	isInitialized = true;

}

bool TaskManager_Register(TaskHandle_t handle, const char* name, UBaseType_t priority,
						 TaskType_t type, Subsystem_t subsystem) {

	if (task_count >= MAX_TASKS || handle == NULL || name == NULL) return false;

	monitoredTasks[task_count].handle = handle;
	monitoredTasks[task_count].name = name;
	monitoredTasks[task_count].lastHeartbeat = xTaskGetTickCount();
	monitoredTasks[task_count].priority = priority;
	monitoredTasks[task_count].isHealthy = true;
	monitoredTasks[task_count].type = type;
	monitoredTasks[task_count].subsystem = subsystem;
	monitoredTasks[task_count].isSuspended = false;
	monitoredTasks[task_count].wasRunningBeforePowerChange = true;
	task_count++;
	return true;
}

void TaskManager_Task(void* pvParameters) {
	while (1) {
		TickType_t now = xTaskGetTickCount();
		for (int i = 0; i < task_count; i++) {
			eTaskState state = eTaskGetState(monitoredTasks[i].handle);
			if (state != eSuspended) {
				if (state != eBlocked) {
					if ((now - monitoredTasks[i].lastHeartbeat) > HEARTBEAT_TIMEOUT) {
						if (monitoredTasks[i].isHealthy) {
							monitoredTasks[i].isHealthy = false;
							// Raise Timeout Fault
						}
					}
				}
			}
		}
		vTaskDelay(MONITOR_PERIOD);
	}
}

void TaskManager_SendHeartbeat(TaskHandle_t handle) {
	eTaskState state = eTaskGetState(handle);
	if (state != eSuspended) {
		TickType_t now = xTaskGetTickCount();
		for (int i = 0; i < task_count; i++) {
			if (monitoredTasks[i].handle = handle) {
				monitoredTasks[i].lastHeartbeat = now;
				monitoredTasks[i].isHealthy = true;
				break;
			}
		}
	}
}


void TaskManager_SuspendAllTasks(void) {

    if (!isInitialized) return;

    for (int i = 0; i < task_count; i++) {
        MonitoredTask_t *task = &monitoredTasks[i];

        eTaskState state = eTaskGetState(task->handle);
        task->wasRunningBeforePowerChange = (state != eSuspended);

        if (task->name != "TaskMgr") {
            suspendTaskSafely(task);
        }
    }

}

void TaskManager_SuspendAllNonCriticalTasks(void) {
    if (!isInitialized) return;

    for (int i = 0; i < task_count; i++) {
        MonitoredTask_t *task = &monitoredTasks[i];

        eTaskState state = eTaskGetState(task->handle);
        task->wasRunningBeforePowerChange = (state != eSuspended);

        if (task->type != TASK_TYPE_CRITICAL) {
            suspendTaskSafely(task);
        }
    }
}

void TaskManager_ResumeAllTasks(void) {
    if (!isInitialized) return;

    for (int i = 0; i < task_count; i++) {
        MonitoredTask_t *task = &monitoredTasks[i];

        if (task->wasRunningBeforePowerChange && task->isSuspended) {
            resumeTaskSafely(task);
        }
    }
}

void TaskManager_ResumeCriticalTasks(void) {

    if (!isInitialized) return;

    for (int i = 0; i < task_count; i++) {
        MonitoredTask_t *task = &monitoredTasks[i];

        eTaskState state = eTaskGetState(task->handle);
        task->wasRunningBeforePowerChange = (state != eSuspended);

        if (task->type == TASK_TYPE_CRITICAL) {
            resumeTaskSafely(task);
        }
    }

}

void TaskManager_ManageSubsystemTasks(Subsystem_t subsystem, bool enable) {
    if (!isInitialized) return;

    for (int i = 0; i < task_count; i++) {
        MonitoredTask_t *task = &monitoredTasks[i];

        if (task->type == TASK_TYPE_SUBSYSTEM && task->subsystem == subsystem) {
            if (enable) {
                if (task->wasRunningBeforePowerChange && task->isSuspended) {
                    resumeTaskSafely(task);
                }
            } else {
                eTaskState state = eTaskGetState(task->handle);
                task->wasRunningBeforePowerChange = (state != eSuspended);
                suspendTaskSafely(task);
            }
        }
    }
}

bool TaskManager_IsTaskHealthy(TaskHandle_t handle) {
    MonitoredTask_t *task = findTaskByHandle(handle);
    return (task != NULL) ? task->isHealthy : false;
}

uint32_t TaskManager_GetUnhealthyTaskCount(void) {
    uint32_t count = 0;

    for (int i = 0; i < task_count; i++) {
        if (!monitoredTasks[i].isHealthy) {
            count++;
        }
    }

    return count;
}

void TaskManager_GetTaskStatus(TaskHandle_t handle, eTaskState *state, bool *isHealthy) {
    if (state != NULL) {
        *state = eTaskGetState(handle);
    }

    if (isHealthy != NULL) {
        MonitoredTask_t *task = findTaskByHandle(handle);
        *isHealthy = (task != NULL) ? task->isHealthy : false;
    }
}

PowerResult_t TaskManager_PowerModeCallback(PowerMode_t fromMode, PowerMode_t toMode) {
    // Perform any task manager specific actions during power mode transitions

    switch (toMode) {
        case POWERMODE_NORMAL:
            // Entering normal mode - all tasks should be available
        	TaskManager_ResumeAllTasks();
            break;

        case POWERMODE_LOW_POWER:
            // Entering low power mode - only critical tasks and subsystem tasks should run
        	if (fromMode == POWERMODE_SLEEP) {
        		TaskManager_ResumeCriticalTasks();
        	}
        	else {
        		TaskManager_SuspendNonCriticalTasks();
        	}
            break;

        case POWERMODE_SLEEP:
            // Entering sleep mode - system will enter STANDBY, all tasks will be suspended
        	TaskManager_SuspendAllTasks();
            break;

        default:
            return POWER_ERROR_INVALID_MODE;
    }

    return POWER_OK;
}

static void suspendTaskSafely(MonitoredTask_t *task) {
    if (task == NULL || task->handle == NULL) return;

    eTaskState state = eTaskGetState(task->handle);
    if (state != eSuspended) {
        vTaskSuspend(task->handle);
        task->isSuspended = true;

        // Update heartbeat to prevent timeout detection while suspended
        task->lastHeartbeat = xTaskGetTickCount();
    }
}

static void resumeTaskSafely(MonitoredTask_t *task) {
    if (task == NULL || task->handle == NULL) return;

    if (task->isSuspended) {
        vTaskResume(task->handle);
        task->isSuspended = false;

        // Update heartbeat to give task time to send first heartbeat
        task->lastHeartbeat = xTaskGetTickCount();
    }
}

static MonitoredTask_t* findTaskByHandle(TaskHandle_t handle) {
    for (int i = 0; i < task_count; i++) {
        if (monitoredTasks[i].handle == handle) {
            return &monitoredTasks[i];
        }
    }
    return NULL;
}

static MonitoredTask_t* findTaskBySubsystem(Subsystem_t subsystem) {
    for (int i = 0; i < task_count; i++) {
        if (monitoredTasks[i].subsystem == subsystem) {
            return &monitoredTasks[i];
        }
    }
    return NULL;
}
