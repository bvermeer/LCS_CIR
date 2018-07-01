/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "usart.h"
#include <string.h>
#include "mySemaphores.h"
#include "stm32f1xx_hal_uart.h"
#include "DS2482.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId LinTaskHandle;
osThreadId ds18S20TempSensorTaskHandle;

/* USER CODE BEGIN Variables */

struct {
	uint8_t 	xMin 	: 1;
	uint8_t 	xMax 	: 1;
	uint8_t 	unused 	: 2;
	uint16_t	gantryAlignmentSensor : 12;
} sensorData;

int16_t sensorTemp;

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartLinTask(void const * argument);
void StartDS18S20TempSensorTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void TriggerEstop(void);
void ResetEstop(void);
void SetupDS2482(void);

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	linTaskSemaphore = xSemaphoreCreateBinary();
	tempAccessSemaphore = xSemaphoreCreateMutex();

	//assert(linTaskSemaphore != NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  SetupDS2482();
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LinTask */
  osThreadDef(LinTask, StartLinTask, osPriorityAboveNormal, 0, 128);
  LinTaskHandle = osThreadCreate(osThread(LinTask), NULL);

  // definition and creation of ds18S20TempSensorTask
  osThreadDef(ds18S20TempSensorTask, StartDS18S20TempSensorTask, osPriorityNormal, 0, 128);
  ds18S20TempSensorTaskHandle = osThreadCreate(osThread(ds18S20TempSensorTask), NULL);
  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	//HAL_GPIO_WritePin(PC13_LED_GPIO_Port, PC13_LED_Pin, GPIO_PIN_RESET);
    //osDelay(250);
    //HAL_GPIO_WritePin(PC13_LED_GPIO_Port, PC13_LED_Pin, GPIO_PIN_SET);
    //osDelay(250);


	  // Get the latest sensor data
	  sensorData.xMin = (uint8_t)HAL_GPIO_ReadPin(XMIN_IN_GPIO_Port, XMIN_IN_Pin);
	  sensorData.xMax = (uint8_t)HAL_GPIO_ReadPin(XMAX_IN_GPIO_Port, XMAX_IN_Pin);

	  // TODO BV - get the data from the gantry alignment sensor


	  // Check if the sensor data is in the safe zone

	  // TODO BV - check the gantry alignment sensor against the safe limit

	  // Check the temperature reading
	  xSemaphoreTake(tempAccessSemaphore, 5);

	  // TODO BV - Change this to use a define
	  if(sensorTemp > 50)
	  {
		  TriggerEstop();
	  }

	  xSemaphoreGive(tempAccessSemaphore);


	  osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartLinTask function */
void StartLinTask(void const * argument)
{
  /* USER CODE BEGIN StartLinTask */
	char charBuf[30] = "LIN thread started.\r\n";

	HAL_UART_Transmit(&huart1, (uint8_t*)charBuf, (uint16_t)strlen(charBuf), 20);

	strcpy(charBuf, "    LIN loop ran.\r\n");


  /* Infinite loop */
	while(1)
	{
		// Wait until a LIN event occurs
		xSemaphoreTake(linTaskSemaphore, portMAX_DELAY);

		// DEBUG
		HAL_UART_Transmit(&huart1, (uint8_t*)charBuf, (uint16_t)strlen(charBuf), 20);

		// Check if a LIN break was detected
		if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_LBD))
		{
			// TODO - Set a bool true to say the next expected transmission is the ID packet

			// Clear the LIN break flag by writing a 0 to it
			__HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_LBD);
		}

	}
  /* USER CODE END StartLinTask */
}

/* USER CODE BEGIN Application */

void StartDS18S20TempSensorTask(void const * argument)
{
	while(1)
	{
		DS2482_One_Wire_Reset();
		DS2482_Write_Byte(0xCC); // Skip ROM command

		// Test reading the temperature from the DS18S20
		DS2482_Write_Config(false, true, true);  // Enable strong pullup for temp conversion
		DS2482_Write_Byte(0x44);  // Get the temp

		osDelay(750); // Wait for the temp conversion


		DS2482_One_Wire_Reset();
		DS2482_Write_Byte(0xCC); // Skip ROM command

		DS2482_Write_Byte(0xbe); 	// Read scratchpad command

		uint8_t tempBytes[2];

		DS2482_Read_Byte(&tempBytes[0]); 	// LSB temp byte
		DS2482_Read_Byte(&tempBytes[1]); 	// MSB temp byte

		// Convert the temp to C
		xSemaphoreTake(tempAccessSemaphore, 5);
		sensorTemp = (tempBytes[1] << 8) | tempBytes[0];
		sensorTemp = sensorTemp >> 1;
		xSemaphoreGive(tempAccessSemaphore);


		// Delay some time before checking the temp again
		osDelay(5000);
	}

}


void TriggerEstop()
{
	HAL_GPIO_WritePin(STOP_OUT_GPIO_Port, STOP_OUT_Pin, GPIO_PIN_SET);
}

void ResetEstop()
{
	HAL_GPIO_WritePin(STOP_OUT_GPIO_Port, STOP_OUT_Pin, GPIO_PIN_RESET);
}

void SetupDS2482()
{
	DS2482_Reset();

	DS2482_Write_Config(false, false, true);

	DS2482_Set_Read_Pointer(STATUS_REG);

	uint8_t byte;

	DS2482_Read_Byte(&byte);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
