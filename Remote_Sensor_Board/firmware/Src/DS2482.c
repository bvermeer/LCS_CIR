/*
 * DS2482.c
 *
 *  Created on: May 26, 2018
 *      Author: blake
 */
#include "DS2482.h"


bool DS2482_Reset(void)
{
	uint8_t cmd;
	cmd = DRST;

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, DS2482_I2C_ADDR << 1, &cmd, 1, 5);

	if(status == HAL_OK)
	{
		return true;
	}

	return false;
}

bool DS2482_Set_Read_Pointer(Ds2482_regs regAddr)
{
	uint8_t data[2];

	data[0] = SRP;
	data[1] = regAddr;

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, DS2482_I2C_ADDR << 1, data, 2, 5);

	if(status == HAL_OK)
	{
		return true;
	}

	return false;
}

bool DS2482_Write_Config(bool OneWireSpeed, bool StrongPullup, bool ActivePullup)
{
	// TODO - Implement function
	return false;

}

bool DS2482_Write_Byte(uint8_t dataByte)
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, DS2482_I2C_ADDR << 1, dataByte, 1, 5);

	if(status == HAL_OK)
	{
		return true;
	}

	return false;

}

bool DS2482_Read_Byte(uint8_t *dataByte)
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, DS2482_I2C_ADDR << 1, dataByte, 1, 5);

	if(status == HAL_OK)
	{
		return true;
	}

	return false;

}
