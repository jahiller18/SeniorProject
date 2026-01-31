/*
 * icm20948.h
 *
 *  Created on: Sep 24, 2025
 *      Author: Jared Hiller
 */

#ifndef INC_SENSORS_ICM20948_H_
#define INC_SENSORS_ICM20948_H_

//
// Includes
//

#include "i2c_manager.h"

//
// Definitions
//

#define ICM20948_GYRO_DEFAULT_RATE		1

//
// I2C Address
//

#define ICM20948_I2C_ADDRESS_AD0_LOW	0x68
#define ICM20948_I2C_ADDRESS_AD0_HIGH	0x69

//
// Register Addresses
//

//
// Bank Select
//

#define ICM20948_REG_BANK_SEL			0x7F

//
// Register Bank 0
//

#define ICM20948_REG_WHOAMI				0x00
#define ICM20948_REG_USER_CTRL			0x03
#define ICM20948_REG_LP_CONFIG			0x05
#define ICM20948_REG_PWR_MGMT_1			0x06
#define ICM20948_REG_PWR_MGMT_2			0x07
#define ICM20948_REG_ACCEL_XOUT_H		0x2D
#define ICM20948_REG_ACCEL_XOUT_L		0x2E
#define ICM20948_REG_ACCEL_YOUT_H		0x2F
#define ICM20948_REG_ACCEL_YOUT_L		0x30
#define ICM20948_REG_ACCEL_ZOUT_H		0x31
#define ICM20948_REG_ACCEL_ZOUT_L		0x32
#define ICM20948_REG_GYRO_XOUT_H		0x33
#define ICM20948_REG_GYRO_XOUT_L		0x34
#define ICM20948_REG_GYRO_YOUT_H		0x35
#define ICM20948_REG_GYRO_YOUT_L		0x36
#define ICM20948_REG_GYRO_ZOUT_H		0x37
#define ICM20948_REG_GYRO_ZOUT_L		0x38

//
// Register Bank 1
//

#define ICM20948_REG_XA_OFFS_H			0x14
#define ICM20948_REG_XA_OFFS_L			0x15
#define ICM20948_REG_YA_OFFS_H			0x17
#define ICM20948_REG_YA_OFFS_L			0x18
#define ICM20948_REG_ZA_OFFS_H			0x1A
#define ICM20948_REG_ZA_OFFS_L			0x1B

//
// Register Bank 2
//

#define ICM20948_REG_GYRO_SMPLRT_DIV	0x00
#define ICM20948_REG_GYRO_CONFIG_1		0x01
#define ICM20948_REG_GYRO_CONFIG_2		0x02
#define ICM20948_REG_XG_OFFS_USRH		0x03
#define ICM20948_REG_XG_OFFS_USRL		0x04
#define ICM20948_REG_YG_OFFS_USRH		0x05
#define ICM20948_REG_YG_OFFS_USRL		0x06
#define ICM20948_REG_ZG_OFFS_USRH		0x07
#define ICM20948_REG_ZG_OFFS_USRL		0x08
#define ICM20948_REG_ACCEL_SMPLRT_DIV_1	0x10
#define ICM20948_REG_ACCEL_SMPLRT_DIV_2	0x11
#define ICM20948_REG_ACCEL_CONFIG		0x14
#define ICM20948_REG_ACCEL_CONFIG_2		0x15

//
// Register Bank Select Enum
//

typedef enum {
	ICM20948_USER_BANK_0 = 0x00,
	ICM20948_USER_BANK_1 = 0x01,
	ICM20948_USER_BANK_2 = 0x02,
	ICM20948_USER_BANK_3 = 0x03
}icm20948_reg_bank_select_t;

//
// Gyro Full Scale Select
//

typedef enum {
	ICM20948_GYRO_250DPS = 0,
	ICM20948_GYRO_500DPS = 1,
	ICM20948_GYRO_1000DPS = 2,
	ICM20948_GYRO_2000DPS = 3
}icm20948_gyro_full_scale_select_t;

//
// Gyro Averaging Configurations
//

typedef enum {
	ICM20948_GYRO_AVERAGE_1 = 0,
	ICM20948_GYRO_AVERAGE_2 = 1,
	ICM20948_GYRO_AVERAGE_4 = 2,
	ICM20948_GYRO_AVERAGE_8 = 3,
	ICM20948_GYRO_AVERAGE_16 = 4,
	ICM20948_GYRO_AVERAGE_32 = 5,
	ICM20948_GYRO_AVERAGE_64 = 6,
	ICM20948_GYRO_AVERAGE_128 = 7
}icm20948_gyro_averaging_select_t;

//
// Accelerometer Full Scale
//

typedef enum {
	ICM20948_FS_SELECT_2G = 0,
	ICM20948_FS_SELECT_4G = 1,
	ICM20948_FS_SELECT_8G = 2,
	ICM20948_FS_SELECT_16G = 3
}icm20948_accel_fs_select_t;

//
// Accelerometer Averaging Configurations
//

typedef enum {
	ICM20948_ACCEL_AVERAGING_1 = 0,
	ICM20948_ACCEL_AVERAGING_8 = 1,
	ICM20948_ACCEL_AVERAGING_16 = 2,
	ICM20948_ACCEL_AVERAGING_32 = 3
}icm20948_accel_averaging_select_t;


// User Control Register Bitmap
// Address 0x03h, Bank 0
// Reset 0x00 (R/W)
typedef struct {
	uint8_t DMP_EN;			// Bit 7-7, Enables DMP features
	uint8_t FIFO_EN;		// Bit 6-6, Enables FIFO operation mode
	uint8_t I2C_MST_EN;		// Bit 5-5, Enables I2C Master Module
	uint8_t I2C_IF_DIS;		// Bit 4-4, Reset I2C slave module, puts serial interface into SPI mode
	uint8_t DMP_RST;		// Bit 3-3, Reset DMP Module
	uint8_t SRAM_RST;		// Bit 2-2, Reset SRAM Module
	uint8_t I2C_MST_RST;	// Bit 1-1, Reset I2C Master Module
	uint8_t Reserved;		// Reserved
}user_ctrl_bits;

typedef struct {
	user_ctrl_bits bits;
	uint8_t reg;
}icm20948_user_ctrl_configuration_t;

// LP Configuration Register Bitmap
// Address 0x05h, Bank 0
// Reset 0x40h (R/W)
typedef struct {
	uint8_t I2C_MST_CYCLE;	// Bit 6-6, Operate I2C master in duty cycled mode
	uint8_t ACCEL_CYCLE;	// Bit 5-5, Operate ACCEL in duty cycled mode
	uint8_t GRYO_CYCLE;		// Bit 4-4, Operate GYRO in duty cycled mode
}lp_config_bits;

typedef struct {
	lp_config_bits bits;
	uint8_t reg;
}icm20948_lp_configuration_t;

// PWR_MGMT_1 Register Bitmap
// Address 0x06h, Bank 0
// Reset 0x41h (R/W)
typedef struct {
	uint8_t DEVICE_RESET;	// Bit 7-7, Reset internal registers, restore default settings
	uint8_t SLEEP;			// Bit 6-6, Set chip to sleep mode
	uint8_t LP_EN;			// Bit 5-5, Enable Low Power feature (more info on p.37 of datasheet)
	uint8_t RESERVED;		// Bit 4-4, Reserved
	uint8_t TEMP_DIS;		// Bit 3-3, Disables temperature sensor
	uint8_t CLKSEL;			// Bit 2-0, Clock Source Select
}pwr_mgmt_1_bits;

typedef struct {
	pwr_mgmt_1_bits bits;
	uint8_t reg;
}icm20948_pwr_mgmt_1_configuration_t;

// PWR_MGMT_2 Register Bitmap
// Address 0x07h, Bank 0
// Reset 0x00 (R/W)
typedef struct {
	uint8_t DISABLE_ACCEL;	// Bit 5-3, Toggle Accelerometer Axes
	uint8_t DISBALE_GYRO;	// Bit 2-0, Toggle Gyro Axes
}pwr_mgmt_2_bits;

typedef struct {
	pwr_mgmt_2_bits bits;
	uint8_t reg;
}icm20948_pwr_mgmt_2_configuration_t;

typedef struct {
	uint8_t RESERVED;		// Bit 7-6, Reserved
	uint8_t ACCEL_DLPFCFG;	// Bit 5-3 Accelerometer low pass filter configuration
	uint8_t ACCEL_FS_SEL;	// Bit 2-1 Accelerometer Full Scale Select
	uint8_t ACCEL_FCHOICE;	// Bit 0-0, Enable DLPF bit
} accel_config_1_bits;

typedef struct {
	accel_config_1_bits bits;
	uint8_t byte;
} icm20948_accel_1_configuration_t;

typedef struct {
	uint8_t RESERVED;		// Bit 7-5, Reserved
	uint8_t AX_ST_EN_REG;	// Bit 4-4, X-Accel Self-Test enable
	uint8_t AY_ST_EN_REG;	// Bit 3-3, Y-Accel Self-Test enable
	uint8_t AZ_ST_EN_REG;	// Bit 2-2, Z-Accel Self-Test enable
	uint8_t DEC3_CFG;		// Bit 1-0, Controls number of samples averaged in the accelerometer
} accel_config_2_bits;

typedef struct {
	accel_config_2_bits bits;
	uint8_t byte;
}icm20948_accel_2_configuration_t;

typedef struct {
	uint8_t RESERVED; 		// Bit 7-6, Reserved
	uint8_t GRYO_DLPFCFG;	// Bit 5-3, Gyro Low Pass Filter Configuration
	uint8_t GYRO_FS_SEL;	// Bit 2-1, Gyro Full Scale Select
	uint8_t GYRO_FCHOICE;	// Bit 0-0, Enable DLPF bit
} gyro_config_1_bits;

typedef struct {
	gyro_config_1_bits bits;
	uint8_t byte;
}icm20948_gyro_1_configuration_t;

typedef struct {
	uint8_t RESERVED;		// Bit 7-6, Reserved
	uint8_t XGYRO_CTEN;		// Bit 5-5, X Gyro Self-Test enable
	uint8_t YGYRO_CTEN;		// Bit 4-4, Y Gyro Self-Test enable
	uint8_t ZGYRO_CTEN;		// Bit 3-3, Z Gyro Self-Test enable
	uint8_t GYRO_AVGCFG;	// Bit 2-0, Average Filter Congfiguration for LP Mode
} gyro_config_2_bits;

typedef struct {
	gyro_config_2_bits bits;
	uint8_t byte;
}icm20948_gyro_2_configuration_t
;
//
// ICM20948 Struct
//

typedef struct {

	I2C_HandleTypeDef *i2cHandle;

	uint8_t i2cAddress;

	uint8_t registerBank;

	// Configuration

	icm20948_user_ctrl_configuration_t user;

	lp_config_bits lp;

	pwr_mgmt_1_bits pw1;

	pwr_mgmt_2_bits pwr2;

	icm20948_accel_1_configuration_t accel_1;



	uint8_t sampleRate;

	// Data

	uint16_t accelXOut;

	uint16_t accelYOut;

	uint16_t accelZOut;

	uint16_t gyroXOut;

	uint16_t gyroYOut;

	uint16_t gyroZOut;
}ICM20948;

//
// Initialization
//

void ICM20948_Initialization(ICM20948 *dev, I2C_HandleTypeDef *i2cHandle);

//
// Configuration
//

void ICM20948_SetConfiguration(ICM20948 *dev);

void ICM20948_SetSampleRate(ICM20948 *dev, uint8_t sampleRate);

#endif /* INC_SENSORS_ICM20948_H_ */
