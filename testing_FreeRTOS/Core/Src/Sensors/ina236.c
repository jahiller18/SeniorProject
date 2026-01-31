/*
 * ina236.c
 *
 *  Created on: Jan 30, 2026
 *      Author: Jared Hiller
 */

#include "ina236.h"

//
// Register Pack Function
//

uint16_t pack_config(config_bits_t *b) {
	return  ((b->RST      & 0x1) << RST_SHIFT)      |
			((0b10  	  & 0x3) << RESERVED_SHIFT)	|
	        ((b->ADCRANGE & 0x1) << ADCRANGE_SHIFT) |
	        ((b->AVG      & 0x7) << AVG_SHIFT)      |
	        ((b->VBUSCT   & 0x7) << VBUSCT_SHIFT)   |
	        ((b->VSHCT    & 0x7) << VSHCT_SHIFT)    |
	        ((b->MODE     & 0x7) << MODE_SHIFT);
}


uint8_t INA236_Init(INA236 *dev, I2C_HandleTypeDef *i2cHandle) {

	dev->i2cHandle = i2cHandle;

	// Initialize Configuration

	ina236_configuration_t config = {0};
	config.bits.RST = 0;
	config.bits.ADCRANGE = 0;
	config.bits.AVG = INA236_ADC_AVG_16;
	config.bits.VBUSCT = INA236_VBUS_CT_1100us;
	config.bits.VSHCT = INA236_SHUNT_CT_1100us;
	config.bits.MODE = INA236_MODE_SHUTDOWN; // Not reset value, but want to be off

	config.reg = pack_config(config.bits);
	dev->config = config.reg;

	// Status Variable
	HAL_StatusTypeDef status;

	uint8_t data[2];

	// Check Communication Bus and that Device is ready
	status = I2C_LockAndIsDeviceReady(dev->i2cHandle, INA236_I2C_ADDR << 1);
	if (status != HAL_OK) return 255;

	// Get Device ID
	status = I2C_LockAndReadRegister16Bit(dev->i2cHandle, INA236_I2C_ADDR << 1, INA236_DEVICE_ID, data);
	uint16_t deviceID = (regData[0] << 8) | regData[1];
	if (deviceID != 0xA080) return 255;

	// Need to update Calibration Register with Shunt Resistor Value
	status = I2C_LockAndWriteRegister16Bit(dev->i2cHandle, INA236_I2C_ADDR << 1, INA236_REG_CALIBRATION, INA236_SHUNT_RESISTOR);
	if (status != HAL_OK) return 255;

	// Update Configuraiton Register and shutdown
	status = I2C_LockAndWriteRegister16Bit(dev->i2cHandle, INA236_I2C_ADDR << 1, INA236_REG_CONFIG, config.reg);
	if (status != HAL_OK) return 255;

	return 0;
}

uint8_t INA236_trigger_single_shot(INA236 *dev) {

	// Update Operating Mode
	dev->config.bits.MODE = INA236_SHUNT_AND_BUS_VOLTAGE_SINGLE;
	dev->config = pack_config(dev->config.bits);
	status = I2C_LockAndWriteRegister16Bit(dev->i2cHandle, INA236_I2C_ADDR << 1, INA236_REG_CONFIG, config.reg);
	if (status != HAL_OK) return 255;

	uint8_t data[2];
	uint16_t raw;
	uint8_t flag = 0;

	// Wait for conversion
	// (VBUSCT + VSHCT) x AVG = 35ms

	//Use FreeRTOS Delay Here when implementing to free processor
	HAL_Dealy(35);
	do {
		HAL_Delay(10);
		if (I2C_LockAndReadRegister16Bit(dev->i2cHandle, INA236_I2C_ADDR << 1, INA236_REG_MASK, data)) return 255;
		flag = (data[1] & INA236_CVRF_FLAG_MASK) >> INA236_CVRF_FLAG;
	} while (!flag);

	// Voltage Measurment
	if (I2C_LockAndReadRegister16Bit(dev->i2cHandle, INA236_I2C_ADDR << 1, INA236_REG_BUS_VOLTAGE, data)) return 255;
	raw = (data[0] << 8) | data [1];
	dev->voltage = raw * BUS_VOLTAGE_LSB;

	// Current Measurment
	if (I2C_LockAndReadRegister16Bit(dev->i2cHandle, INA236_I2C_ADDR << 1, INA236_REG_CURRENT, data)) return 255;
	raw = (data[0] << 8) | data [1];
	dev->current = raw * CURRENT_LSB;

	// Power Measuremnt
	if (I2C_LockAndReadRegister16Bit(dev->i2cHandle, INA236_I2C_ADDR << 1, INA236_REG_POWER, data)) return 255;
	raw = (data[0] << 8) | data [1];
	dev->power = raw * POWER_LSB;

	return 0;
}
