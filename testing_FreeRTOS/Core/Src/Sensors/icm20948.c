/*
 * icm20948.c
 *
 *  Created on: Sep 24, 2025
 *      Author: Jared Hiller
 */

//
// Includes
//

#include "icm20948.h"

// Pack User Control Register (0x03h, Bank 0)
uint8_t pack_user_ctrl(user_ctrl_bits *b) {
    return  ((b->DMP_EN      & 0x1) << 7) |
            ((b->FIFO_EN     & 0x1) << 6) |
            ((b->I2C_MST_EN  & 0x1) << 5) |
            ((b->I2C_IF_DIS  & 0x1) << 4) |
            ((b->DMP_RST     & 0x1) << 3) |
            ((b->SRAM_RST    & 0x1) << 2) |
            ((b->I2C_MST_RST & 0x1) << 1) |
            ((b->Reserved    & 0x1) << 0);
}

// Pack LP Config Register (0x05h, Bank 0)
uint8_t pack_lp_config(lp_config_bits *b) {
    return  ((b->I2C_MST_CYCLE & 0x1) << 6) |
            ((b->ACCEL_CYCLE   & 0x1) << 5) |
            ((b->GRYO_CYCLE    & 0x1) << 4);
}

// Pack PWR_MGMT_1 Register (0x06h, Bank 0)
uint8_t pack_pwr_mgmt_1(pwr_mgmt_1_bits *b) {
    return  ((b->DEVICE_RESET & 0x1) << 7) |
            ((b->SLEEP        & 0x1) << 6) |
            ((b->LP_EN        & 0x1) << 5) |
            ((b->RESERVED     & 0x1) << 4) |
            ((b->TEMP_DIS     & 0x1) << 3) |
            ((b->CLKSEL       & 0x7) << 0);  // 3 bits wide
}

// Pack PWR_MGMT_2 Register (0x07h, Bank 0)
uint8_t pack_pwr_mgmt_2(pwr_mgmt_2_bits *b) {
    return  ((b->DISABLE_ACCEL & 0x7) << 3) | // 3 bits wide
            ((b->DISBALE_GYRO & 0x7) << 0);  // 3 bits wide
}

uint8_t pack_accel_config_1(accel_config_1_bits *b) {
    return  ((b->RESERVED       & 0x3) << 6) |  // Bits 7-6
            ((b->ACCEL_DLPFCFG  & 0x7) << 3) |  // Bits 5-3
            ((b->ACCEL_FS_SEL   & 0x3) << 1) |  // Bits 2-1
            ((b->ACCEL_FCHOICE  & 0x1) << 0);   // Bit 0
}

uint8_t ICM20948_SelectRegisterBank(ICM20948 *dev, uint8_t bank) {

	uint8_t buffer;
	uint8_t retries = 3;
	HAL_StatusTypeDef status;

	if (dev->registerBank != bank) {
		status = I2C_LockAndWriteRegister8Bit(dev->i2cHandle, dev->i2cAddress, ICM20948_REG_BANK_SEL, bank);
		if (status != HAL_OK) {
			return status;
		}
	}

	status = I2C_LockAndReadRegister8Bit(dev->i2cHandle, dev->i2cAddress, ICM20948_REG_BANK_SEL, buffer);

	if (status != HAL_OK) {
		return status;
	}

	while (bank != buffer && retries > 0) {

		status = I2C_LockAndWriteRegister8Bit(dev->i2cHandle, dev->i2cAddress, ICM20948_REG_BANK_SEL, bank);

		if (status != HAL_OK) {
			return status;
		}

		reties--;

		status = I2C_LockAndReadRegister8Bit(dev->i2cHandle, dev->i2cAddress, ICM20948_REG_BANK_SEL, buffer);

		if (status != HAL_OK) {
			return status;
		}

	}

}

void ICM20948_SetSampleRate(ICM20948 *dev, uint8_t sampleRate) {

	uint8_t div = 1;
	HAL_StatusTypeDef status;

	// Write to Register Bank 2
	ICM20948_SelectRegisterBank(dev, ICM20948_USER_BANK_2);


	if (sampleRate == 0) {
		sampleRate = ICM20948_GYRO_DEFAULT_RATE;
	}

	div = 1100 / sampleRate;

	if (div > 200) { div = 200; }
	if (div < 1) { div = 1; }

	status = I2C_LockAndWriteRegister8Bit(dev->i2cHandle, dev->i2cAddress, ICM20948_REG_GYRO_SMPLRT_DIV, div - 1);
	if (status != HAL_OK) {
		return status;
	}
	status = I2C_LockAndWriteRegister8Bit(dev->i2cHandle, dev->i2cAddress, ICM20948_REG_ACCEL_SMPLRT_DIV_2, div - 2);
	if (status != HAL_OK) {
		return status;
	}

	// Update sample rate
	dev->sampleRate = div;

}

void ICM20948_SetAccelRange(ICM20948 *dev, icm20948_accel_fs_select_t sel) {

	// Update Device Bits and Byte
	dev->accel_1.bits.ACCEL_FS_SEL = sel;

	dev->accel_1.byte = pack_accel_config_1(dev->accel_1.bits);

	// Change Register Values
	ICM20948_SelectRegisterBank(dev, ICM20948_USER_BANK_2);
	status = I2C_LockAndWriteRegister8Bit(dev->i2cHandle, dev->i2cAddress, ICM20948_REG_ACCEL_CONFIG, dev->accel_1.byte);

	// Check that value updated

	// Use retry method

}


void ICM20948_Initialization(ICM20948 *dev, I2C_HandleTypeDef *i2cHandle) {

	// Store I2C Handle with the ICM20948 device struct
	dev->i2cHandle = i2cHandle;

	// Set 7-bit I2C Address
	dev->i2cAddress = ICM20948_I2C_ADDRESS_AD0_LOW;

	// Initialize Configuration with Reset Values
	icm20948_user_ctrl_configuration_t user_config;
	user_config.bits.DMP_EN = 0;
	user_config.bits.DMP_RST = 0;
	user_config.bits.FIFO_EN = 0;
	user_config.bits.I2C_IF_DIS = 0;
	user_config.bits.I2C_MST_EN = 0;
	user_config.bits.I2C_MST_RST = 0;
	user_config.bits.SRAM_RST = 0;

	config.reg = pack_user_ctrl(&user_config);

	dev->user = user_config;

	// Initialize Configuration of Low Power
	icm20948_lp_configuration_t lp_config;
	lp_config.bits.I2C_MST_CYCLE = 0;
	lp_config.bits.ACCEL_CYCLE = 0;
	lp_config.bits.GRYO_CYCLE = 0;

	lp_config.reg = pack_lp_config(&lp_config);

	// Initialize Configuration of Power 1
	icm20948_pwr_mgmt_1_configuration_t pwr1_config;
	pwr1_config.bits.DEVICE_RESET = 0;
	pwr1_config.bits.SLEEP = 0;
	pwr1_config.bits.LP_EN = 0;
	pwr1_config.bits.RESERVED = 0;
	pwr1_config.bits.TEMP_DIS = 0;
	pwr1_config.bits.CLKSEL = 1;

	pwr1_config.reg = pack_pwr_mgmt_1(&pwr1_config);

	// Initialize Configuration of Power 2
	icm20948_pwr_mgmt_2_configuration_t pwr2_config;

	pwr2_config.bits.DISABLE_ACCEL = 0;
	pwr2_config.bits.DISBALE_GYRO = 0;

	pwr2_config.reg = pack_pwr_mgmt2(&pwr2_config);

	HAL_StatusTypeDef status;
	uint8_t buffer;

	status = I2C_LockAndIsDeviceReady(dev->i2cHandle, dev->i2cAddress);
	if (status != HAL_OK) {
		// Exit and retry
	}

	status = I2C_LockAndReadRegister8Bit(dev->i2cHandle, dev->i2cAddress, ICM20948_REG_WHOAMI, buffer);
	if (status != HAL_OK) {
		// Exit and retry
	}

	// If Device ID is not correct
	if (buffer != 0xEA) {
		// Exit and retry
	}

}


