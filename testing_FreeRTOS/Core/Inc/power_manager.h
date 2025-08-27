/*
 * power_manager.h
 *
 *  Created on: Aug 4, 2025
 *      Author: hillerj
 */

#ifndef POWER_MANAGER_H_
#define POWER_MANAGER_H_

#include "stm32h7xx_hal.h"
#include "FreeRTOS.h"
#include "timers.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Power modes enumeration
typedef enum {
    POWERMODE_NORMAL = 0,     // Full performance mode
    POWERMODE_LOW_POWER,      // Reduced performance, critical subsystems active
    POWERMODE_SLEEP,          // STANDBY mode, RTC wakeup only
    POWERMODE_COUNT
} PowerMode_t;

// Legacy mode definitions for backward compatibility
#define POWER_MODE_OPERATING    POWERMODE_NORMAL
#define POWER_MODE_STANDBY      POWERMODE_LOW_POWER
#define POWER_MODE_SHUTDOWN     POWERMODE_SLEEP

// Subsystem enumeration
typedef enum {
    SUBSYSTEM_UART_DEBUG = 0,
    SUBSYSTEM_UART_COMM,
    SUBSYSTEM_I2C_LUX,
	SUBSYSTEM_I2C_TMP,
    SUBSYSTEM_COUNT
} Subsystem_t;

// Power result enumeration
typedef enum {
    POWER_OK = 0,
    POWER_ERROR_INVALID_PARAM,
    POWER_ERROR_INVALID_MODE,
    POWER_ERROR_ALREADY_IN_MODE,
    POWER_ERROR_TIMER_FAIL,
    POWER_ERROR_CLOCK_CONFIG,
    POWER_ERROR_PERIPHERAL_CONFIG,
    POWER_ERROR_SUBSYSTEM_FAIL,
    POWER_ERROR_HEALTH_CHECK_FAIL,
    POWER_ERROR_BACKUP_SRAM_FAIL
} PowerResult_t;

// Wakeup source enumeration - RTC only system
typedef enum {
    WAKEUP_SOURCE_RESET = 0,      // Normal power-on or system reset
    WAKEUP_SOURCE_RTC,            // RTC wakeup timer (only wakeup from STANDBY)
    WAKEUP_SOURCE_UNKNOWN         // Unexpected wakeup condition
} WakeupSource_t;

// Forward declarations for callback function types
typedef void (*PowerModeChangeCallback_t)(PowerMode_t fromMode, PowerMode_t toMode);
typedef void (*WakeupCallback_t)(WakeupSource_t source);
typedef void (*PeripheralStateCallback_t)(Subsystem_t subsystem, bool enabled);
typedef PowerResult_t (*SystemHealthCheckCallback_t)(void);
typedef PowerResult_t (*SubsystemHealthCheckCallback_t)(void);

// Task management callback types
typedef void (*SuspendAllTasksCallback_t)(void);
typedef void (*SuspendNonCriticalTasksCallback_t)(void);
typedef void (*ResumeAllTasksCallback_t)(void);
typedef void (*ManageSubsystemTasksCallback_t)(Subsystem_t subsystem, bool enable);

// Hardware control callback types
typedef void (*EnableHardwareCallback_t)(void);
typedef void (*DisableHardwareCallback_t)(void);

// Clock configuration structure
typedef struct {
    // Operating mode clock settings
    struct {
        uint32_t pllM;
        uint32_t pllN;
        uint32_t pllP;
        uint32_t pllQ;
        uint32_t pllR;
        uint32_t ahbPrescaler;
        uint32_t apb1Prescaler;
        uint32_t apb2Prescaler;
        uint32_t apb3Prescaler;
        uint32_t apb4Prescaler;
    } operating;

    // Standby mode clock settings
    struct {
        bool usePLL;
        uint32_t ahbPrescaler;
        uint32_t apb1Prescaler;
        uint32_t apb2Prescaler;
        uint32_t apb3Prescaler;
        uint32_t apb4Prescaler;
    } standby;
} ClockConfig_t;

// Wakeup configuration structure - RTC only
typedef struct {
    bool enableTimer;           // Enable timer-based wakeup (legacy, not used in STANDBY)
    uint32_t timerWakeupMs;     // Timer wakeup interval in milliseconds (legacy)
    bool enableRtcWakeup;       // Enable RTC wakeup from STANDBY (always true for race to sleep)
    uint32_t rtcWakeupSeconds;  // RTC wakeup interval in seconds (used for STANDBY mode)
} WakeupConfig_t;

// Subsystem configuration structure
typedef struct {
    Subsystem_t id;                                    // Subsystem identifier
    bool isCritical;                                   // Is this subsystem critical?
    EnableHardwareCallback_t enableHardware;          // Enable hardware function
    DisableHardwareCallback_t disableHardware;        // Disable hardware function
    SubsystemHealthCheckCallback_t healthCheck;       // Health check function
    const char* name;                                  // Subsystem name for debugging
} SubsystemConfig_t;

// Race to sleep configuration structure
typedef struct {
    bool enabled;                          // Enable race to sleep functionality
    uint32_t systemCheckIntervalMs;        // Interval between system health checks
    uint32_t healthCheckTimeoutMs;         // Max time for health check in low power mode
    uint32_t maxConsecutiveErrors;         // Max errors before disabling race to sleep
    bool useBackupSram;                    // Use backup SRAM for state preservation
} RaceToSleepConfig_t;

// Main power manager configuration structure
typedef struct {
    // Subsystem configuration
    SubsystemConfig_t *subsystems;         // Array of subsystem configurations
    uint32_t numSubsystems;                // Number of configured subsystems

    // Clock configuration
    ClockConfig_t clockConfig;             // Clock settings for different modes

    // Wakeup configuration
    WakeupConfig_t wakeupConfig;           // Wakeup source configuration

    // Race to sleep configuration
    RaceToSleepConfig_t raceToSleepConfig; // Race to sleep settings

    // Task management callbacks (required)
    SuspendAllTasksCallback_t suspendAllTasks;
    SuspendNonCriticalTasksCallback_t suspendNonCriticalTasks;
    ResumeAllTasksCallback_t resumeAllTasks;
    ManageSubsystemTasksCallback_t manageSubsystemTasks;
} PowerManagerConfig_t;

// Backup SRAM structure for state preservation
typedef struct {
    uint32_t magicNumber;                  // Magic number to validate data
    bool raceToSleepEnabled;               // Race to sleep state
    uint32_t consecutiveErrors;            // Error counter
    uint32_t systemCheckInterval;          // System check interval
    PowerMode_t lastPowerMode;             // Last power mode before STANDBY
    uint32_t bootCount;                    // Boot counter for debugging
    uint32_t checksum;                     // Data integrity checksum
} BackupSramData_t;

// Public function declarations

/**
 * @brief Initialize the power manager
 * @param config Power manager configuration structure
 * @return PowerResult_t Result of initialization
 */
PowerResult_t PowerManager_Init(const PowerManagerConfig_t *config);

/**
 * @brief Deinitialize the power manager and cleanup resources
 * @return PowerResult_t Result of deinitialization
 */
PowerResult_t PowerManager_Deinit(void);

/**
 * @brief Set the power mode
 * @param mode Target power mode
 * @return PowerResult_t Result of mode change
 * @note When entering POWERMODE_SLEEP, system will enter STANDBY mode and reset on wakeup
 */
PowerResult_t PowerManager_SetMode(PowerMode_t mode);

/**
 * @brief Get the current power mode
 * @return PowerMode_t Current power mode
 */
PowerMode_t PowerManager_GetCurrentMode(void);

/**
 * @brief Handle wakeup from STANDBY mode - call at beginning of main()
 * @return PowerResult_t Result of wakeup handling
 * @note This function determines wakeup source and handles race to sleep logic
 */
PowerResult_t PowerManager_HandleWakeup(void);

/**
 * @brief Enable or disable a specific subsystem
 * @param subsystem Subsystem to control
 * @return PowerResult_t Result of operation
 */
PowerResult_t PowerManager_EnableSubsystem(Subsystem_t subsystem);
PowerResult_t PowerManager_DisableSubsystem(Subsystem_t subsystem);

/**
 * @brief Check if a subsystem is currently enabled
 * @param subsystem Subsystem to check
 * @return bool True if enabled, false if disabled
 */
bool PowerManager_IsSubsystemEnabled(Subsystem_t subsystem);

/**
 * @brief Enable or disable race to sleep functionality
 * @param enable True to enable, false to disable
 */
void PowerManager_EnableRaceToSleep(bool enable);

/**
 * @brief Check if race to sleep is currently active
 * @return bool True if race to sleep is active
 */
bool PowerManager_IsRaceToSleepActive(void);

/**
 * @brief Get the last wakeup source
 * @return WakeupSource_t Last wakeup source
 */
WakeupSource_t PowerManager_GetLastWakeupSource(void);

/**
 * @brief Set power mode change callback
 * @param callback Function to call when power mode changes
 */
void PowerManager_SetModeChangeCallback(PowerModeChangeCallback_t callback);

/**
 * @brief Set wakeup callback
 * @param callback Function to call on wakeup events
 */
void PowerManager_SetWakeupCallback(WakeupCallback_t callback);

/**
 * @brief Set peripheral state change callback
 * @param callback Function to call when peripheral state changes
 */
void PowerManager_SetPeripheralCallback(PeripheralStateCallback_t callback);

/**
 * @brief Set system health check callback
 * @param callback Function to call for system health checks
 */
void PowerManager_SetHealthCheckCallback(SystemHealthCheckCallback_t callback);

/**
 * @brief Perform immediate system health check
 * @return PowerResult_t Result of health check
 */
PowerResult_t PowerManager_PerformHealthCheck(void);

/**
 * @brief Get error count for race to sleep system
 * @return uint32_t Number of consecutive errors
 */
uint32_t PowerManager_GetErrorCount(void);

/**
 * @brief Reset error count
 */
void PowerManager_ResetErrorCount(void);

/**
 * @brief Force immediate transition to sleep mode
 * @return PowerResult_t Result of sleep transition
 * @note System will enter STANDBY mode and reset on wakeup
 */
PowerResult_t PowerManager_ForceSleep(void);

/**
 * @brief Get power mode string representation
 * @param mode Power mode
 * @return const char* Mode string
 */
const char* PowerManager_GetModeString(PowerMode_t mode);

/**
 * @brief Get subsystem string representation
 * @param subsystem Subsystem identifier
 * @return const char* Subsystem string
 */
const char* PowerManager_GetSubsystemString(Subsystem_t subsystem);

/**
 * @brief Get power manager statistics
 * @param stats Pointer to statistics structure to fill
 * @return PowerResult_t Result of operation
 */
typedef struct {
    uint32_t totalModeChanges;
    uint32_t totalWakeups;
    uint32_t rtcWakeups;            // Only RTC wakeups tracked
    uint32_t unknownWakeups;        // Unexpected wakeup conditions
    uint32_t healthChecksPassed;
    uint32_t healthChecksFailed;
    uint32_t timeInNormalMode;
    uint32_t timeInLowPowerMode;
    uint32_t timeInSleepMode;
    uint32_t currentBootCount;
} PowerManagerStats_t;

PowerResult_t PowerManager_GetStats(PowerManagerStats_t *stats);

/**
 * @brief Reset power manager statistics
 */
void PowerManager_ResetStats(void);

// Utility macros
#define POWER_MANAGER_VERSION_MAJOR    1
#define POWER_MANAGER_VERSION_MINOR    0
#define POWER_MANAGER_VERSION_PATCH    0

#define POWER_MANAGER_BACKUP_SRAM_MAGIC    0xDEADBEEF
#define POWER_MANAGER_MAX_SUBSYSTEMS       32

// Configuration validation macros
#define IS_VALID_POWER_MODE(mode)       ((mode) < POWERMODE_COUNT)
#define IS_VALID_SUBSYSTEM(subsystem)   ((subsystem) < SUBSYSTEM_COUNT)
#define IS_VALID_WAKEUP_SOURCE(source)  ((source) <= WAKEUP_SOURCE_UNKNOWN)

#ifdef __cplusplus
}
#endif

#endif /* POWER_MANAGER_H_ */
