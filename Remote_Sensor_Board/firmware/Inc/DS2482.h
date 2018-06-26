/*
 * DS2482.h
 *
 *  Created on: May 26, 2018
 *      Author: blake
 *
 *  @brief This is a driver for the DS2482 I2C to wire wire bridge IC
 */

#ifndef DS2482_H_
#define DS2482_H_

#include <stdint.h>
#include <stdbool.h>
#include "i2c.h"

#define DS2482_I2C_ADDR 0x18	// 7-bit I2C address of the DS2482. Change this to whatever I2C address your device is set to.


// Defines for commands the DS2482 understands
typedef enum ds2482_cmds
{
	DRST 	= 0xF0, 	// Device reset command
	SRP 	= 0xE1,		// Set read pointer command
	WCFG 	= 0xD2,		// Write configuration command
	OWRS 	= 0xB4, 	// 1-Wire reset command
	OWSB 	= 0x87, 	// 1-Wire single bit command
	OWWB 	= 0xA5, 	// 1-Wire write byte command
	OWRB	= 0x96,		// 1-Wire read byte command
	WT		= 0x78		// 1-Wire triplet command
}Ds2482_cmds;

// Define the pointers to the DS2482 registers
typedef enum ds2482_regs
{
	STATUS_REG		= 0xF0,
	DATA_READ_REG	= 0xE1,
	CONFIG_REG		= 0xC3
}Ds2482_regs;

// Define the individual bits of the status register
typedef union
{
	struct
	{
		uint8_t OneWB 	: 1;	// Set to 1 when the 1-wire bus is busy and 0 when it is idle
		uint8_t PPD		: 1;	// Presence-Pulse Detect
		uint8_t SD		: 1;	// Short Detect
		uint8_t LL		: 1;	// Logic Level. Indicates the current logic state of the 1-wire bus
		uint8_t RST 	: 1;	// Device Reset. Indicates that the DS2482 has been reset and needs to be configured again
		uint8_t	SBR		: 1;	// Single Bit Reset.
		uint8_t TSB		: 1; 	// Triplet Second Bit.
		uint8_t DIR		: 1;	// Branch Direction Taken.
	};
	uint8_t status;
}Ds2482_Status_Reg;


// Function prototypes
bool DS2482_Reset(void);
bool DS2482_Set_Read_Pointer(Ds2482_regs regAddr);
bool DS2482_Write_Config(bool OneWireSpeed, bool StrongPullup, bool ActivePullup);
bool DS2482_Write_Byte(uint8_t dataByte);
bool DS2482_Read_Byte(uint8_t *dataByte);


#endif /* DS2482_H_ */
