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
	uint8_t configByte = ((~OneWireSpeed) << 7) | ((~StrongPullup) << 6) | (1 << 5) | ((~ActivePullup) << 4) | (OneWireSpeed << 3) | (StrongPullup << 2) | (ActivePullup);

	uint8_t dataStream[2] = {WCFG, configByte};

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, DS2482_I2C_ADDR << 1, dataStream, sizeof(dataStream), 5);

	if(status != HAL_OK)
	{
		return false;
	}

	uint8_t receiveByte;

	status = HAL_I2C_Master_Receive(&hi2c1, DS2482_I2C_ADDR << 1, &receiveByte, 1, 5);

	if(status != HAL_OK)
	{
		return false;
	}


	if(receiveByte == (0xF & configByte))
	{
		return true;
	}

	return false;
}

bool DS2482_Write_Byte(uint8_t dataByte)
{
	uint8_t writeBytes[2] = {OWWB, dataByte};

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, DS2482_I2C_ADDR << 1, writeBytes, sizeof(writeBytes), 5);

	if(status != HAL_OK)
	{
		return false;
	}


	Ds2482_Status_Reg statusByte;

	do
	{
		// Wait for data to be written
		HAL_Delay(1);

		status = HAL_I2C_Master_Receive(&hi2c1, DS2482_I2C_ADDR << 1, (uint8_t*) &statusByte, 1, 5);

	} while(statusByte.OneWB == 1);


	return true;

}

bool DS2482_Read_Byte(uint8_t *dataByte)
{
	uint8_t readByteCmd = OWRB;

	// Send the one-wire read byte command
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, DS2482_I2C_ADDR << 1, &readByteCmd, 1, 5);

	if(status != HAL_OK)
	{
		return false;
	}

	// Change the read pointer to the read data register
	if( !DS2482_Set_Read_Pointer(DATA_READ_REG) )
	{
		return false;
	}

	// Wait for the data to be read
	HAL_Delay(1);

	status = HAL_I2C_Master_Receive(&hi2c1, DS2482_I2C_ADDR << 1, dataByte, 1, 5);

	if(status != HAL_OK)
	{
		return false;
	}

	return true;

}

bool DS2482_One_Wire_Reset()
{
	uint8_t resetCmd = OWRS;

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, DS2482_I2C_ADDR << 1, &resetCmd, 1, 5);

	if(status != HAL_OK)
	{
		return false;
	}

	// Wait for the one-wire reset command to be sent
	HAL_Delay(1);

	return true;
}
