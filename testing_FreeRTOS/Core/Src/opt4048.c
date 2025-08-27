/*
 * opt4048.c
 *
 *  Created on: Jun 23, 2025
 *      Author: Jared Hiller
 */


#include "opt4048.h"

//
// Register Pack Functions
//

//
// Register 0x0A
//

uint16_t pack_config_A(config_bits_A_t *b) {
	return  ((b->QUAKE        & 0x1) << QUAKE_SHIFT)           |
	        ((b->ZERO         & 0x1) << ZERO_SHIFT)            |
	        ((b->RANGE        & 0xF) << RANGE_SHIFT)           |
	        ((b->CONVERSION_TIME & 0xF) << CONVERSION_TIME_SHIFT) |
	        ((b->OPERATING_MODE  & 0x3) << OPERATING_MODE_SHIFT)  |
	        ((b->LATCH        & 0x1) << LATCH_SHIFT)           |
	        ((b->INT_POL      & 0x1) << INT_POL_SHIFT)         |
	        ((b->FAULT_COUNT  & 0x3) << FAULT_COUNT_SHIFT);
}

uint16_t pack_config_B(config_bits_B_t *b) {
	return ((b->HIGH             & 0x1FF) << HIGH_SHIFT)         |
	       ((b->THRESHOLD_CH_SEL & 0x3)   << THRESHOLD_CH_SEL_SHIFT) |
	       ((b->INT_DIR          & 0x1)   << INT_DIR_SHIFT)          |
	       ((b->INT_CFG          & 0x3)   << INT_CFG_SHIFT)          |
	       ((b->ZERO             & 0x1)   << ZERO_SHIFT)             |
	       ((b->I2C_BURST        & 0x1)   << I2C_BURST_SHIFT);
}

//
// Initialization
//

uint8_t OPT4048_Init(OPT4048 *dev, I2C_HandleTypeDef *i2cHandle) {

	dev->i2cHandle = i2cHandle;

	// Initialize Configuration Structs to hold Configuration Data

	// Register 0x0A

	// Update Bit Field with Reset Values
	configA configA = {0};
	configA.bits.QUAKE				= 0;
	configA.bits.ZERO				= 0;
	configA.bits.RANGE				= 0xC;
	configA.bits.CONVERSION_TIME  	= 0x8;
	configA.bits.OPERATING_MODE 	= 0;
	configA.bits.LATCH				= 1;
	configA.bits.INT_POL			= 0;
	configA.bits.FAULT_COUNT		= 0;

	// Pack Bit Field into uint16_t byte
	configA.regA = pack_config_A(&configA.bits);

	// Update OPT4048 device
	dev->config.configA = configA;

	// Register 0x0B

	// Update Bit Field with Reset Values
	configB configB = {0};
	configB.bits.HIGH 				= 0x80;
	configB.bits.THRESHOLD_CH_SEL	= 0;
	configB.bits.INT_DIR			= 1;
	configB.bits.INT_CFG			= 1;
	configB.bits.ZERO				= 0;
	configB.bits.I2C_BURST			= 1;

	// Pack Bit Field into uint16_t byte
	configB.regB = pack_config_B(&configB.bits);

	// Update OPT4048 device
	dev->config.configB = configB;


	// Status Variable
	HAL_StatusTypeDef status;

	// Variable/Pointer to hold register values
	uint8_t regData[2];

	// Check Communication Bus and that Device is ready
	status = I2C_LockAndIsDeviceReady(dev->i2cHandle, OPT4048_I2C_ADDR << 1);
	if (status != HAL_OK) {
		return 255;
	}

	// Get Device ID and match with correct value
	status = I2C_LockAndReadRegister16Bit(dev->i2cHandle, OPT4048_I2C_ADDR << 1, OPT4048_REG_DIDH, regData);
	uint16_t data = (regData[0] << 8) | regData[1];

	// If part number does not match return error (255)
	if (data != 0x821) return 255;

	return 0;
}

//
// Functions
//

uint8_t OPT4048_GetConfiguration(OPT4048 *dev) {

	return 0;
}

uint8_t OPT4048_UpdateConfig(OPT4048 *dev) {

	return 0;
}

uint8_t OPT4048_GetCIE(OPT4048 *dev) {

	// Check and read channels
	if (OPT4048_GetMeasurement(dev) != 0) {
		// Bad Reading
		return 255;
	}

	 // Matrix multiplication coefficients (from datasheet)
	 const double m0x = 2.34892992e-04;
	 const double m0y = -1.89652390e-05;
	 const double m0z = 1.20811684e-05;
	 const double m0l = 0;

	 const double m1x = 4.07467441e-05;
	 const double m1y = 1.98958202e-04;
	 const double m1z = -1.58848115e-05;
	 const double m1l = 2.15e-3;

	 const double m2x = 9.28619404e-05;
	 const double m2y = -1.69739553e-05;
	 const double m2z = 6.74021520e-04;
	 const double m2l = 0;

	 const double m3x = 0;
	 const double m3y = 0;
	 const double m3z = 0;
	 const double m3l = 0;

	 // The equation from the datasheet is a matrix multiplication:
	 // [ch0 ch1 ch2 ch3] * [m0x m0y m0z m0l] = [X Y Z Lux]
	 //                     [m1x m1y m1z m1l]
	 //                     [m2x m2y m2z m2l]
	 //                     [m3x m3y m3z m3l]

	 double X = dev->CH0.result * m0x + dev->CH1.result * m1x + dev->CH2.result * m2x + dev->CH3.result * m3x;
	 double Y = dev->CH0.result * m0y + dev->CH1.result * m1y + dev->CH2.result * m2y + dev->CH3.result * m3y;
	 double Z = dev->CH0.result * m0z + dev->CH1.result * m1z + dev->CH2.result * m2z + dev->CH3.result * m3z;
	 double L = dev->CH0.result * m0l + dev->CH1.result * m1l + dev->CH2.result * m2l + dev->CH3.result * m3l;

	 double sum = X + Y + Z;

	 dev->lux = L;

	 if (sum <= 0) {
		 // Avoid div by 0 and other issues
		 dev->CIEx = 0;
		 dev->CIEy = 0;
		 dev->lux = 0;
		 return 255;
	 }

	 dev->CIEx = X / sum;
	 dev->CIEy = Y / sum;

	 return 0;

}

uint8_t OPT4048_GetMeasurement(OPT4048 *dev) {
	// Uses One-Shot Mode (Register Trigger) to get and update values of each channel

		// define necessary variables
		uint8_t buffer[2];
		uint8_t Reg1;
		uint8_t Reg2;
		uint16_t data;
		uint8_t exp;
		uint16_t msb;
		uint16_t lsb;
		uint8_t counter;
		uint8_t crc;
		uint32_t mant;

		//Status
		HAL_StatusTypeDef status;

		// Set Mode of operation to One-Shot Mode (Register Trigger)
		// Set Data
		dev->config.configA.bits.OPERATING_MODE = OPT4048_MODE_ONESHOT;

		// Update Register 0x0A byte
		dev->config.configA.regA = pack_config_A(&dev->config.configA.bits);

		// Update data variable with new configuration
		data = dev->config.configA.regA;

		// Write to Register
		status = I2C_LockAndWriteRegister16Bit(dev->i2cHandle, OPT4048_I2C_ADDR << 1, 0xA, data);
		if (status != HAL_OK) {
			return 255;
		}

		// Check Conversion Times and Wait (4 x conversion time)
		// Need to use freeRTOS delay to free processor
		HAL_Delay(400);

		// Get Data From Channels
		// Channel 0
		for (uint8_t ch = 0; ch < 3; ch++) {

			switch (ch) {
			case 0:
				Reg1 = OPT4048_REG_EXPONENT_CH0;
				Reg2 = OPT4048_REG_RESULT_LSB_CH0;
				break;
			case 1:
				Reg1 = OPT4048_REG_EXPONENT_CH1;
				Reg2 = OPT4048_REG_RESULT_LSB_CH1;
				break;
			case 2:
				Reg1 = OPT4048_REG_EXPONENT_CH2;
				Reg2 = OPT4048_REG_RESULT_LSB_CH2;
				break;
			case 3:
				Reg1 = OPT4048_REG_EXPONENT_CH3;
				Reg2 = OPT4048_REG_RESULT_LSB_CH3;
				break;
			}

			status = I2C_LockAndReadRegister16Bit(dev->i2cHandle, OPT4048_I2C_ADDR << 1, Reg1, buffer);
			if (status != HAL_OK) {
				return 255;
			}
			data = (buffer[0] << 8) | buffer [1];
			exp = (data & 0xF000) >> 12;
			msb = (data & 0x0FFF);
			status = I2C_LockAndReadRegister16Bit(dev->i2cHandle, OPT4048_I2C_ADDR << 1, Reg2, buffer);
			if (status != HAL_OK) {
				return 255;
			}
			data = (buffer[0] << 8) | buffer [1];
			lsb = (data & 0xFF00) >> 8;
			counter = (data & 0x00F0) >> 4;
			crc = (data & 0x000F);
			mant = lsb + (msb << 8);

			// CRC Check based on the formula on the datasheet

			// CRC bits for each channel:
				// R[19:0]=(RESULT_MSB_CH0[11:0]<<8)+RESULT_LSB_CH0[7:0]
				// X[0]=XOR(EXPONENT_CH0[3:0],R[19:0],COUNTER_CHx[3:0]) - XOR of all bits
				// X[1]=XOR(COUNTER_CHx[1],COUNTER_CHx[3],R[1],R[3],R[5],R[7],R[9],R[11],R[13],R[15],R[17],R[19],E[1],E[3])
				// X[2]=XOR(COUNTER_CHx[3],R[3],R[7],R[11],R[15],R[19],E[3])
				// X[3]=XOR(R[3],R[11],R[19])

				// Note: COUNTER_CHx[3:0] is the CRC itself, which creates a circular
				// reference We need to include it in our calculations to match the hardware
				// implementation

			// Init CRC variables
			uint8_t x0 = 0;
			uint8_t x1 = 0;
			uint8_t x2 = 0;
			uint8_t x3 = 0;

			// Calculate each CRC bit according to the datasheet formula:
			// Calculate bit 0 (x0):
			// X[0]=XOR(EXPONENT_CH0[3:0],R[19:0],COUNTER_CHx[3:0])
			x0 = 0;

			// XOR all exp bits
			for (uint8_t i = 0; i < 4; i++) {
				x0 ^= (exp >> i) & 1;
			}

			// XOR all mantissa bits
			for (uint8_t i = 0; i < 20; i++) {
				x0 ^= (mant >> 1) & 1;
			}

			// XOR all counter (CRC) bits
			for (uint8_t i = 0; i < 4; i++) {
				x0 ^= (counter >> 1) & 1;
			}

			// Calculate bit 1 (x1) per datasheet:
			// X[1]=XOR(COUNTER_CHx[1],COUNTER_CHx[3],R[1],R[3],R[5],R[7],R[9],R[11],R[13],R[15],R[17],R[19],E[1],E[3])
			x1 = 0;
			x1 ^= (counter >> 1) & 1; // COUNTER_CHx[1]
			x1 ^= (counter >> 3) & 1; // COUNTER_CHx[3]

		    // Include odd-indexed mantissa bits
		    for (uint8_t i = 1; i < 20; i += 2) {
		      x1 ^= (mant >> i) & 1;
		    }

		    // Include exponent bits 1 and 3
		    x1 ^= (exp >> 1) & 1; // E[1]
		    x1 ^= (exp >> 3) & 1; // E[3]

		    // Calculate bit 2 (x2) per datasheet:
		    // X[2]=XOR(COUNTER_CHx[3],R[3],R[7],R[11],R[15],R[19],E[3])
		    x2 = 0;
		    x2 ^= (counter >> 3) & 1; // COUNTER_CHx[3]

		    // Include mantissa bits at positions 3,7,11,15,19
		    for (uint8_t i = 3; i < 20; i += 4) {
		      x2 ^= (mant >> i) & 1;
		    }

		    // Include exponent bit 3
		    x2 ^= (exp >> 3) & 1; // E[3]

		    // Calculate bit 3 (x3) per datasheet:
		    // X[3]=XOR(R[3],R[11],R[19])
		    x3 = 0;
		    // XOR mantissa bits at positions 3, 11, 19
		    x3 ^= (mant >> 3) & 1;  // R[3]
		    x3 ^= (mant >> 11) & 1; // R[11]
		    x3 ^= (mant >> 19) & 1; // R[19]

		    // Combine bits to form the CRC
	        uint8_t calculated_crc = (x3 << 3) | (x2 << 2) | (x1 << 1) | x0;

	        // Verify
	        if (crc != calculated_crc) {
	        	return 255;
	        }

	        // Calculate CH_CODES Value using formula on data sheet
	        // mant << exp
	        uint32_t output = (uint32_t)mant << (uint32_t)exp;

	        switch(ch) {

	        	case 0:
	        		dev->CH0.result = output;
	        		break;
	        	case 1:
	        		dev->CH1.result = output;
	        		break;
	        	case 2:
	        		dev->CH2.result = output;
	        		break;
	        	case 3:
	        		dev->CH3.result = output;
	        		break;
	        }

		}
		return 0;

}
