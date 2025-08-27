/*
 * opt4048.h
 *
 *  Created on: Jun 23, 2025
 *      Author: Jared Hiller
 */

#ifndef INC_OPT4048_H_
#define INC_OPT4048_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "i2c_manager.h"

//
// Defines
//

#define OPT4048_I2C_ADDR		0x44

#define OPT4048_DEVICE_ID_LOW	0x08
#define OPT4048_DEVICE_ID_HIGH	0x21

//
// Registers (16 bit)
//

//
// Channel 0
//
#define OPT4048_REG_EXPONENT_CH0            0x00    // 4-bit, 15-12, exponent output CH0
#define OPT4048_REG_RESULT_MSB_CH0          0x00    // 12-bit, 11-0, result register MSB CH0,
#define OPT4048_REG_RESULT_LSB_CH0          0x01    // 8-bit, 15-8, result register LSB CH0
#define OPT4048_REG_COUNTER_CH0             0x01    // 4-bit, 7-4, sample counter CH0
#define OPT4048_REG_CRC_CH0                 0x01    // 4-bit, 3-0, CRC bits for CH0

//
// Channel 1
//

#define OPT4048_REG_EXPONENT_CH1            0x02    // 4-bit, 15-12, exponent output CH1
#define OPT4048_REG_RESULT_MSB_CH1          0x02    // 12-bit, 11-0, result register MSB CH1
#define OPT4048_REG_RESULT_LSB_CH1          0x03    // 8-bit, 15-8, result register LSB CH1
#define OPT4048_REG_COUNTER_CH1             0x03    // 4-bit, 7-4, sample counter CH1
#define OPT4048_REG_CRC_CH1                 0x03    // 4-bit, 3-0, CRC bits for CH1

//
// Channel 2
//

#define OPT4048_REG_EXPONENT_CH2            0x04    // 4-bit, 15-12, exponent output CH2
#define OPT4048_REG_RESULT_MSB_CH2          0x04    // 12-bit, 11-0, result register MSB CH2
#define OPT4048_REG_RESULT_LSB_CH2          0x05    // 8-bit, 15-8, result register LSB CH2
#define OPT4048_REG_COUNTER_CH2             0x05    // 4-bit, 7-4, sample counter CH2
#define OPT4048_REG_CRC_CH2                 0x05    // 4-bit, 3-0, CRC bits for CH2

//
// Channel 3
//

#define OPT4048_REG_EXPONENT_CH3            0x06    // 4-bit, 15-12, exponent output CH3
#define OPT4048_REG_RESULT_MSB_CH3          0x06    // 12-bit, 11-0, result register MSB CH3
#define OPT4048_REG_RESULT_LSB_CH3          0x07    // 8-bit, 15-8, result register LSB CH3
#define OPT4048_REG_COUNTER_CH3             0x07    // 4-bit, 7-4, sample counter CH3
#define OPT4048_REG_CRC_CH3                 0x07    // 4-bit, 3-0, CRC bits for CH3

//
// Threshold
//

#define OPT4048_REG_EXPONENT_THRESHOLD_L    0x08    // 4-bit, 15-12, exponent output threshold low
#define OPT4048_REG_RESULT_THRESHOLD_L      0x08    // 12-bit, 11-0, result threshold low
#define OPT4048_REG_EXPONENT_THRESHOLD_H    0x09    // 4-bit, 15-12, exponent output threshold high
#define OPT4048_REG_RESULT_THRESHOLD_H      0x09    // 12-bit, 11-0, result threshold high

//
// Configuration
//

#define OPT4048_REG_QUAKE                   0x0A    // 1-bit, 15-15, quick wake up from standby in one shot mode
#define OPT4048_REG_RANGE                   0x0A    // 4-bit, 13-10, controls the full scale light level range of the device (p.29)
#define OPT4048_REG_CONVERSION_TIME         0x0A    // 4-bit, 9-6, controls the device conversion time per channel (p.30)
#define OPT4048_REG_OPERATING_MODE          0x0A    // 2-bit, 5-4, controls device mode of operation (p.30)
#define OPT4048_REG_LATCH                   0x0A    // 1-bit, 3-3, controls functinality of the interrupt reporting mechanisms for INT pin for threshold detection logic
#define OPT4048_REG_INT_POL                 0x0A    // 1-bit, 2-2, controls the polarity or active state of the INT pin
#define OPT4048_REG_FAULT_COUNT             0x0A    // 2-bit, 0-1, fault count register insrtucts the device as to how many consecutive fault events are required to trigger the threshold mechanisms
#define OPT4048_REG_THRESHOLD_CH_SEL        0x0B    // 2-bit, 6-5, channel select for threshold logic
#define OPT4048_REG_INT_DIR                 0x0B    // 1-bit, 4-4, determines the direction of the INT pin
#define OPT4048_REG_INT_CFG                 0x0B    // 2-bit, 3-2, controls the output interrupt mechanisms after end of conversion
#define OPT4048_REG_I2C_BURST               0x0B    // 1-bit, 0-0, when set enables I2C burst mode minimizing I2C read cycles

//
// Status Register
//

#define OPT4048_REG_STATUS					0x0C	// Status Register

//
// Field Descriptions
//

#define OPT4080_REG_DIDL					0x11	// 2-bit, 13-12, device ID low, value of 8h
#define OPT4048_REG_DIDH					0x11	// 12 bit, 11-0, device ID high, value of 21h

//
// Statis Register (0x0C) bit flags
//

#define OPT4048_FLAG_L						0x01	// Flag Low - measurement smaller than threshold
#define OPT4048_FLAG_H						0x02	// Flag High - measurement larger then threshold
#define OPT4048_FLAG_CONVERSION_READY		0x04	// Converison Ready
#define OPT4048_FLAG_OVERLOAD				0x08	// Overflow condition

//
// Configuration Register 0x0A
//

//
// Bit Position Constants
//

#define QUAKE_SHIFT            15
#define ZERO_SHIFT             14
#define RANGE_SHIFT            10
#define CONVERSION_TIME_SHIFT  6
#define OPERATING_MODE_SHIFT   4
#define LATCH_SHIFT            3
#define INT_POL_SHIFT          2
#define FAULT_COUNT_SHIFT      0

#define HIGH_SHIFT             7
#define THRESHOLD_CH_SEL_SHIFT 5
#define INT_DIR_SHIFT          4
#define INT_CFG_SHIFT          2
#define ZERO_SHIFT             1
#define I2C_BURST_SHIFT        0

/*
 * @brief Configuration Struct for register 0x0A
 *
 * Holds the data stored in register 0x0A whos functionality is
 * found on page 29-30 of the datasheet
 */

typedef struct {
	uint8_t QUAKE;
	uint8_t ZERO;
	uint8_t RANGE;
	uint8_t CONVERSION_TIME;
	uint8_t OPERATING_MODE;
	uint8_t LATCH;
	uint8_t INT_POL;
	uint8_t FAULT_COUNT;
} config_bits_A_t;

/*
 * @brief Combined Config Struct Containter
 *
 * Holds the bit information and uses it to contruct a byte that can be passed to the sensor
 *
 */

typedef struct {
	config_bits_A_t bits;
	uint16_t regA;
} configA;

//
// Pack_Config_Function
//

uint16_t pack_config(config_bits_A_t *b);

//
// Configuration Register 0x0B
//

typedef struct {
	uint16_t HIGH;
	uint8_t THRESHOLD_CH_SEL;
	uint8_t INT_DIR;
	uint8_t INT_CFG;
	uint8_t ZERO;
	uint8_t I2C_BURST;
} config_bits_B_t;

typedef struct {
	config_bits_B_t bits;
	uint16_t regB;
} configB;

//
// Pack Config Function
//

uint16_t pack_config_B(config_bits_B_t *b);


typedef struct {
	// Configuration Register A
	configA configA;

	// Configuration Register B
	configB configB;
} opt4048_configuration_t;


//
// OPT4048 Light Level Range enum
//

/*
 * @brief Available range settings for the OPT4048 sensor
 *
 * Full-scale light ranges as described in datasheet page 29
 */

typedef enum {
	OPT4048_RANGE_2K_LUX = 0,	// 2.2klux
	OPT4048_RANGE_4K_LUX = 1,	// 4.5klux
	OPT4048_RANGE_9K_LUX = 2,	// 9 kkux
	OPT4048_RANGE_18K_LUX = 3,	// 18 klux
	OPT4048_RANGE_36K_LUX = 4,	// 36 klux
	OPT4048_RANGE_72K_LUX = 5,	// 72 klux
	OPT4048_RANGE_144K_LUX = 6,	// 114klux
	OPT4048_RANGE_AUTO = 12		// Auto-range
} opt4048_range_t;

//
// OPT4048 Operating Mode enum
//

/*
 * @brief Available converstion time settings for the OPT4048 sensor
 *
 * These control the device conversion time per channel as described
 * in datasheet page 29
 *
 */

typedef enum {
	OPT4048_MODE_POWERDOWN = 0,		// Power-Down mode
	OPT4048_MODE_AUTO_ONESHOT = 1,	// Forced auto-range one-shot mode
	OPT4048_MODE_ONESHOT = 2,		// One-Shot mode
	OPT4048_MODE_CONTINUOUS = 3		// Continuous mode
} opt4048_operating_mode_t;

//
// OPT4048 Conversion Time enum
//

/*
 * @brief Available operating mode settings for the OPT4048 sensor
 *
 * Controls the device mode of operation as described in datasheet page 30
 *
 */

typedef enum {
	OPT4048_CONVERSION_TIME_600us = 0, 	// 600 microseconds
	OPT4048_CONVERSION_TIME_1ms = 1,	// 1 millisecond
	OPT4048_CONVERSION_TIME_1_8ms = 2,	// 1.8 milliseconds
	OPT4048_CONVERSION_TIME_3_4ms = 3,	// 3.4 milliseconds
	OPT4048_CONVERSION_TIME_6_5ms = 4,	// 6.5 milliseconds
	OPT4048_CONVERSION_TIME_12_7ms = 5, // 12.7 milliseconds
	OPT4048_CONVERSION_TIME_25ms = 6,	// 25 milliseconds
	OPT4048_CONVERSION_TIME_50ms = 7,	// 50 milliseconds
	OPT4048_CONVERSION_TIME_100ms = 8,	// 100 milliseconds
	OPT4048_CONVERSION_TIME_200ms = 9,	// 200 milliseconds
	OPT4048_CONVERSION_TIME_400ms = 10,	// 400 milliseconds
	OPT4048_CONVERSION_TIME_800ms = 11  // 800 milliseconds
} opt4048_conversion_time_t;

//
// OPT4048 Fault Count
//

/*
 * @brief Available fault count settings for the OPT 4048 sensor
 *
 * Controls how many consecutive fault events are needed to trigger
 * an interrupt
 *
 */

typedef enum {
	OPT4048_FAULT_COUNT_1 = 0, // 1 fault count <default>
	OPT4048_FAULT_COUNT_2 = 1, // 2 consecutive fault counts
	OPT4048_FAULT_COUNT_4 = 2, // 4 consecutive fault counts
	OPT4048_FAULT_COUNT_8 = 3, // 8 consecutive fault counts
} opt4048_fault_count_t;

//
// OPT4048 Interrupt Configuration
//

/*
 * @brief Interrupt configuration settings for the OPT4048 sensor
 *
 * Controls the interrupt mechanism after end of conversion
 */

typedef enum {
	OPT4048_INT_CFG_SMBUS_ALERT = 0,		// SMBUS Alert
	OPT4048_INT_CFG_DATA_READY_NEXT = 1,    // INT pin data ready for next channel
	OPT4048_INT_CFG_DATA_READY_ALL = 3,     // INT pin data ready for all channels
} opt4048_int_cfg_t;

//
// Channel Results Struct
//

/*
 * @brief Stores the raw data from a specific channel
 *
 * Store the raw data from the different sensor channels
 */

typedef struct {
	// RAW_ADC_CODES_VALUE
	uint32_t result;

} opt4048_channel_t;

//
// OPT4048 Struct
//

typedef struct {
	I2C_HandleTypeDef *i2cHandle;

	opt4048_configuration_t config;

	double CIEx;

	double CIEy;

	double lux;

	opt4048_channel_t CH0;

	opt4048_channel_t CH1;

	opt4048_channel_t CH2;

	opt4048_channel_t CH3;
} OPT4048;


//
// Functions
//



//
// Initialization
//

uint8_t OPT4048_Init(OPT4048 *dev, I2C_HandleTypeDef *i2cHandle);

uint8_t OPT4048_GetConfiguration(OPT4048 *dev);

uint8_t OPT4048_UpdateConfig(OPT4048 *dev);

uint8_t OPT4048_GetCIE(OPT4048 *dev);

uint8_t OPT4048_GetMeasurement(OPT4048 *dev);




//
// Low Level Functions
//

/*
 * @brief Read data from OPT4048 register without the use of DMA
 * @param dev pointer to the OPT4048 structure that contains the pointer to the I2C_Handle
 * 				TypeDef
 * @param register_pointer pointer to the OPT4048 register that is wanted to be read
 * @param data buffer to receive the data stored the in OPT4048 register
 * @retval HAL_Status
 */
HAL_StatusTypeDef OPT4048_Read_Register(OPT4048 *opt, uint8_t register_pointer, uint8_t* receive_buffer);

/*
 * @brief Read data from OPT4048 register with the use of DMA
 * @param dev pointer to the OPT4048 structure that contains the pointer to the I2C_Handle
 * 				TypeDef
 * @param register_pointer pointer to the OPT4048 register that is wanted to be read
 * @param data buffer to receive the data stored the in OPT4048 register
 * @retval HAL_Status
 */
HAL_StatusTypeDef OPT4048_Read_Register_DMA(OPT4048 *dev, uint8_t register_pointer, uint8_t* receive_buffer);

/*
 * @brief Write data to a OPT4048 register without the use of DMA
 * @param dev pointer to the OPT4048 structure that contains the pointer to the I2C_Handle
 * 				TypeDef
 * @param register_pointer pointer to the OPT4048 register that is wanted to be written to
 * @param buffer holds the data to be written to the OPT4048 register
 * @retval HAL_Status
 */
HAL_StatusTypeDef OPT4048_Write_Register(OPT4048 *opt, uint8_t register_pointer, uint16_t register_value);

/*
 * @brief Write data to a OPT4048 register with the use of DMA
 * @param dev pointer to the OPT4048 structure that contains the pointer to the I2C_Handle
 * 				TypeDef
 * @param register_pointer pointer to the OPT4048 register that is wanted to be written to
 * @param buffer holds the data to be written to the OPT4048 register
 * @retval HAL_Status
 */
HAL_StatusTypeDef OPT4048_Write_Register_DMA(OPT4048 *dev, uint8_t register_pointer, uint8_t* buffer);
#endif /* INC_OPT4048_H_ */
