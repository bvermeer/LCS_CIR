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


// Defines for commands the DS2482 understands
typedef enum ds2482_cmds {
	DRST 	= 0xF0, 	// Device reset command
	SRP 	= 0xE1,		// Set read pointer command
	WCFG 	= 0xD2,		// Write configuration command
	OWRS 	= 0xB4, 	// 1-Wire reset command
	OWSB 	= 0x87, 	// 1-Wire single bit command
	OWWB 	= 0xA5, 	// 1-Wire write byte command
	OWRB	= 0x96,		// 1-Wire read byte command
	WT		= 0x78		// 1-Wire triplet command
};


#endif /* DS2482_H_ */
