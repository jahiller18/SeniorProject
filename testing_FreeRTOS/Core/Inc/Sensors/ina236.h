/*
 * ina236.h
 *
 *  Created on: Jan 30, 2026
 *      Author: Jared Hiller
 */

#ifndef INC_SENSORS_INA236_H_
#define INC_SENSORS_INA236_H_


#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "i2c_manager.h"

//
// Defines
//

#define MAXIMUM_EXPECTED_CURRENT		5.0
#define CURRENT_LSB_MINIMUM				(MAXIMUM_EXPECTED_CURRENT / ((float)(1<<15)))
#define CURRENT_LSB						(CURRENT_LSB_MINIMUM * 1.0) // in A
#define BUS_VOLTAGE_LSB					0.0016 // in V
#define SHUNT_VOLTAGE_81_92mv_LSB		0.0025 // in mV
#define SHUNT_VOLTAGE_20_48mv_LSB		0.000625 // in mV
#define POWER_LSB						(CURRENT_LSB*32) // in W

#define INA236_I2C_ADDR			0x40	// A0 -> GND
//#define INA236_I2C_ADDR			0x41	// A0 -> VS
//#define INA236_I2C_ADDR			0x42	// A0 -> SDA
//#define INA236_I2C_ADDR			0x43	// A0 -> SCL

#define INA236_MANUFACTURER_ID	0x3E
#define INA236_DEVICE_ID		0x3F

#define INA236_SHUNT_RESISTOR	0x00

#define INA236_CVRF_FLAG_MASK		1 << 3
#define INA236_CVRF_FLAG			3

//
// Register Map
//

#define INA236_REG_CONFIG			0x00
#define INA236_REG_SHUNT_VOLTAGE 	0x01
#define INA236_REG_BUS_VOLTAGE		0x02
#define INA236_REG_POWER			0x03
#define INA236_REG_CURRENT			0x04
#define INA236_REG_CALIBRATION		0x05
#define INA236_REG_MASK				0x06
#define INA236_REG_ALERT			0x07

//
// Configuration Register
//

//
// Bit Position Constants
//

#define RST_SHIFT					15	// 1-bit, 15-15. Reset when set to 1
#define RESERVED_SHIFT				13	// 2-bit, 14-13, Reserved
#define ADCRANGE_SHIFT				12	// 1-bit, 12-12, Enables selection of the shunt full scale input across IN+ and IN-
#define AVG_SHIFT					9	// 3-bit, 11-9, Sets the number of ADC conversions results to be averaged (pg. 20)
#define VBUSCT_SHIFT				6	// 3-bit, 8-6, Sets the conversion time of the VBUS measurement
#define VSHCT_SHIFT					3	// 3-bit, 5-3, Sets the conversion time of the SHUNT measurement
#define MODE_SHIFT					0	// 3-bit, 2-0, Sets Operating Mode

/*
 *
 * @brief Configurations Struct for the Configuration Register
 *
 * Holds the data stored in the register 0x00, functionality is found on p.19-20 on the datasheet
 *
 */

typedef struct {
	uint8_t RST;
	uint8_t ADCRANGE;
	uint8_t AVG;
	uint8_t VBUSCT;
	uint8_t VSHCT;
	uint8_t MODE;
} config_bits_t;

/*
 * @brief Combined Configuration Struct Container
 *
 * Holds the bit information and uses it to contruct a byte that can be passed
 *
 */
typedef struct {
	config_bits_t bits;
	uint16_t reg;
} ina236_configuration_t;

//
// Pack Config Function
//

uint16_t pack_config(config_bits_t *b);

//
// INA236 ADC Range enum
//

/*
 * @brief Available ADC averages for the INA236
 *
 */

typedef enum {
	INA236_ADC_AVG_1    = 0,
	INA236_ADC_AVG_4    = 1,
	INA236_ADC_AVG_16   = 2,
	INA236_ADC_AVG_64   = 3,
	INA236_ADC_AVG_128  = 4,
	INA236_ADC_AVG_256  = 5,
	INA236_ADC_AVG_512  = 6,
	INA236_ADC_AVG_1024 = 7
} ina236_adc_avg_t;

//
// INA236 Available VBUS Conversion Times
//

/*
 * @brief Available VBUS Converison Times
 *
 */

typedef enum {
	INA236_VBUS_CT_120us  = 0,		// 140 microseconds
	INA236_VBUS_CT_204us  = 1,		// 204 microseconds
	INA236_VBUS_CT_332us  = 2,		// 332 microseconds
	INA236_VBUS_CT_588us  = 3, 		// 588 microseconds
	INA236_VBUS_CT_1100us = 4,		// 1100 microseconds
	INA236_VBUS_CT_2116us = 5,		// 2116 microseconds
	INA236_VBUS_CT_4156us = 6,		// 4156 microseconds
	INA236_VBUS_CT_8244us = 7		// 8244 microseconds
} ina236_vbus_ct_t;

//
// INA236 Available SHUNT Conversion Times
//

/*
 * @brief Available SHUNT Converison Times
 *
 */

typedef enum {
	INA236_SHUNT_CT_120us  = 0,		// 140 microseconds
	INA236_SHUNT_CT_204us  = 1,		// 204 microseconds
	INA236_SHUNT_CT_332us  = 2,		// 332 microseconds
	INA236_SHUNT_CT_588us  = 3, 	// 588 microseconds
	INA236_SHUNT_CT_1100us = 4,		// 1100 microseconds
	INA236_SHUNT_CT_2116us = 5,		// 2116 microseconds
	INA236_SHUNT_CT_4156us = 6,		// 4156 microseconds
	INA236_SHUNT_CT_8244us = 7		// 8244 microseconds
} ina236_shunt_ct_t;

//
// INA236 Available Operating Modes
//

/*
 * @brief Available Operating Modes for the INA236
 *
 */

typedef enum {
	INA236_MODE_SHUTDOWN = 0,
	INA236_MODE_SHUNT_VOLTAGE_SINGLE = 1,
	INA236_MODE_BUS_VOLTAGE_SINGLE = 2,
	INA236_MODE_SHUNT_AND_BUS_VOLTAGE_SINGLE = 3,
	INA236_MODE_SHUNT_VOLTAGE_CONT = 5,
	INA236_MODE_BUS_VOLTAGE_CONT = 6,
	INA236_MODE_SHUNT_AND_BUS_CONT = 7
} ina236_operating_mode_t;

//
// INA236 Struct
//

typedef struct {
	I2C_HandleTypeDef *i2cHandle;

	ina236_configuration_t config;

	uint16_t voltage;

	uint16_t current;

	uint16_t power;

} INA236;

//
// Initialization
//

uint8_t INA236_init(INA236 *dev, I2C_HandleTypeDef *i2cHandle);

//
// Capture Measurement
//

uint8_t INA236_trigger_single_shot(INA236 *dev);


#endif /* INC_SENSORS_INA236_H_ */
