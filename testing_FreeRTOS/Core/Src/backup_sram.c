/*
 * backup_sram.c
 *
 *  Created on: Aug 5, 2025
 *      Author: hillerj
 */


#include "power_manager.h"
#include "stm32h7xx_hal.h"

// Backup SRAM addresses for state storage
#define BACKUP_SRAM_BASE        0x38800000U
#define RACE_TO_SLEEP_STATE_ADDR (BACKUP_SRAM_BASE + 0x00)

typedef struct {
    uint32_t magic;           // Magic number to validate data
    bool raceToSleepEnabled;
    PowerMode_t lastMode;
    uint32_t timestamp;
} BackupState_t;

#define BACKUP_MAGIC 0xDEADBEEF

void saveRaceToSleepState(void) {
    // Enable backup SRAM clock
    __HAL_RCC_BKPRAM_CLK_ENABLE();

    // Enable write access to backup SRAM
    HAL_PWR_EnableBkUpAccess();

    BackupState_t *state = (BackupState_t*)RACE_TO_SLEEP_STATE_ADDR;
    state->magic = BACKUP_MAGIC;
    state->raceToSleepEnabled = true;  // This would come from power manager
    state->lastMode = POWERMODE_SLEEP;
    state->timestamp = HAL_GetTick();

    // Disable write access
    HAL_PWR_DisableBkUpAccess();
}

void restoreRaceToSleepState(void) {
    // Enable backup SRAM clock
    __HAL_RCC_BKPRAM_CLK_ENABLE();

    BackupState_t *state = (BackupState_t*)RACE_TO_SLEEP_STATE_ADDR;

    // Validate magic number
    if (state->magic == BACKUP_MAGIC) {
        // Restore state - this would integrate with power manager
        // PowerManager_EnableRaceToSleep(state->raceToSleepEnabled);
    }
}
