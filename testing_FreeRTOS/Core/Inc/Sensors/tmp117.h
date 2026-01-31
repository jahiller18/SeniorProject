/*
 * tmp117.h
 *
 *  Created on: Jul 16, 2025
 *      Author: hillerj
 */

#ifndef INC_TMP117_H_
#define INC_TMP117_H_

// Includes
#include "stm32h7xx_hal.h"
#include "i2c_manager.h"

#include <stdint.h>

//
// Defines
//

//
// Sensor Resolution
//

#define TMP117_RESOLUTION 			0.0078125f

//
// Device Addresses
//

#define TMP117_I2C_ADDR_GND			0x48
#define TMP117_I2C_ADDR_VDD			0x49
#define TMP117_I2C_ADDR_SDA			0x50
#define TMP117_I2C_ADDR_SCL			0x51

//
// Internal Register Addresses (16-bit)
//

#define TMP117_TEMP_RESULT			0x00	// Temperature Result Register, formated in 2s complement
#define TMP117_CONFIGURATION		0x01	// Configuration Register
#define TMP117_THIGH_LIMIT			0x02	// High Limit Register, store high limit value for comparison against temperature result
#define TMP117_TLOW_LIMIT			0x03	// Low Limit Register, stores low limit vlaue for comparison against temperature result
#define TMP117_EEPROM_UL			0x04	// EEPROM Unlock Register
#define TMP117_EEPROM1				0x05	// EEPROM Register 1, can be programmed to hold data
#define TMP117_EEPROM2				0x06	// EEPROM Register 2, can be programmed to hold data
#define TMP117_TEMP_OFFSET			0x07	// Temperature Offset Register, stores a offset value to be used during system calibration
#define TMP117_EEPROM3				0x08	// EEPROM Register 3, can be programmed to hold data
#define TMP117_DEVICE_ID			0x0F	// Device ID Register, stores the device ID (read only)


//
// Configuraiton Register 0x01h
//

/*
 * @brief holds the byte infomation for the configuration register
 *
 * Holds the data stored in configuration register 0x01h
 * Functionality of this register is store on page 27-28 of the datasheet
 *
 */

//
// Bit Position Constants
//

// TMP117 Register 0x01 Bit Positions (MSB = bit 15)
#define TMP117_HIGH_ALERT_POS           15
#define TMP117_LOW_ALERT_POS            14
#define TMP117_DATA_READY_POS           13
#define TMP117_EEPROM_BUSY_POS          12
#define TMP117_CONVERSION_MODE_POS      10  // 2 bits: 11–10
#define TMP117_CONV_POS                 7   // 3 bits: 9–7
#define TMP117_CONVERSION_AVG_MODE_POS  5   // 2 bits: 6–5
#define TMP117_THERM_ALERT_POS          4
#define TMP117_ALERT_POL_POS            3
#define TMP117_DR_ALERT_POS             2
#define TMP117_SOFT_RESET_POS           1
#define TMP117_ZERO_POS                 0

//
// Configuration Struct
//

typedef struct {
	uint8_t HIGH_ALERT : 1;				// Bit 15-15 High Alert Flag, Set when conversion result is higher than high limit
	uint8_t LOW_ALERT1 : 1;				// Bit 14-14 Low Alert Flag, Set when conversion result is lower than low limit
	uint8_t DATA_READY : 1;				// Bit 13-13 Data Ready Flag, Set when conversion is complete and temperature register can be read
	uint8_t EEPROM_BUSY : 1;			// Bit 12-12 EEPROM Busy Flag, Set when EPPROM is busy during programming or power-up
	uint8_t CONVERSION_MODE : 2;		// Bits 11-10 Conversion Mode
	uint8_t	CONV : 3;					// Bits 9-7 Conversion Cycle Bits
	uint8_t	CONVERSION_AVG_MODE : 2;	// Bits 6-5 Conversion Averaging Modes, determines number of conversion results that are collected and averaged before updating temperature register
	uint8_t THERM_ALERT : 1;			// Bit 4-4 Therm/Alert Mode Select
	uint8_t ALERT_POL : 1;				// Bit 3-3 Alert Pin Polarity
	uint8_t DR_ALERT : 1;				// Bit 2-2 Alert Pin Select
	uint8_t SOFT_RESET : 1;				// Bit 1-1 Software Reset bit
	uint8_t ZERO : 1;					// Bit 0-0 Zero, (read only)
} config_bits_t;

typedef struct {
	config_bits_t bits;
	uint16_t reg;
} tmp117_configuration_t;

//
// TMP117 Conversion Mode enum
//

/*
 * @brief Available Conversion Modes for the TMP117 sensor
 *
 * These control how the device handles conversion as described
 * on page 27 of the datasheet
 *
 */

typedef enum {
	TMP117_MODE_CONTINUOUS = 0,		// Continuous Conversion
	TMP117_MODE_SHUTDOWN = 1,		// Shutdown
	TMP117_MODE_CONTINUOUS_ = 2,	// Same as 00, Continuous Conversion (reads back 00)
	TMP117_MODE_ONE_SHOT = 3		// One-Shot Conversion
} tmp117_conversion_mode_t;

//
// TMP117 Conversion Averaging Modes
//

/*
 * @brief Available Conversion Averaging Modes
 *
 * Controls the number of conversion results that are collected and averaged
 * before updating the temperature register. The average is an accumulated average
 * Described in page 27 of the datasheet
 *
 */

typedef enum {
	TMP117_AVG_MODE_NONE = 0,	// No Averaging
	TMP117_AVG_MODE_8 = 1,		// 8 Averaged Conversions
	TMP117_AVG_MODE_32 = 2,		// 32 Averaged Conversions
	TMP117_AVG_MODE_64 = 3		// 64 Averaged Conversions
} tmp117_conversion_average_mode_t;


//
// TMP117 Struct
//

typedef struct {
	I2C_HandleTypeDef *i2cHandle;

	uint8_t i2cAddress;

	tmp117_configuration_t config;

	float temp_Celcius;
} TMP117;


//
// Functions
//

void TMP117_GetTemperatureData(TMP117 *dev);
uint16_t TMP117_TwosCompToInt(uint16_t temp);

//
// Configuration
//

uint8_t TMP117_SetConfiguration(TMP117 *dev);

//
// Initialization
//

uint8_t TMP117_Initialization(TMP117 *dev, I2C_HandleTypeDef *i2cHandle);

//
// Pack_Config_Function
//

#endif /* INC_TMP117_H_ */
