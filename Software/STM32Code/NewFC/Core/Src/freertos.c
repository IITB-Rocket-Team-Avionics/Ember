/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern QueueHandle_t Pressure;
extern QueueHandle_t Time;
extern QueueHandle_t Velocity;
extern I2C_HandleTypeDef hi2c1;
extern RTC_TimeTypeDef sTime;
extern RTC_DateTypeDef sDate;
extern UART_HandleTypeDef huart1;
uint8_t state = 0;
float altbuf[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float velbuf[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float timebuf[10] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float events[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

uint16_t flash_page = 0;
uint8_t offset = 0;

const float min_liftoff_alt;
const float force_burnout_time;
const float touchdown_alt;
const float touchdown_vel_limit;
const float main_to_touchdown_lockout;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void BMP_Polling(void *argument)
{
	  uint8_t imu_addr = (0x76<<1);
	  uint8_t wakeup = 0x03;
	  uint8_t ph, pl, pxl;
      //uint8_t acc_config = 0x08;

	  HAL_I2C_Mem_Write(&hi2c1, imu_addr, 0xF4, I2C_MEMADD_SIZE_8BIT, &wakeup , 1, HAL_MAX_DELAY);

	  HAL_I2C_Mem_Write(&hi2c1, imu_addr, 0x1C, I2C_MEMADD_SIZE_8BIT, &acc_config , 1, HAL_MAX_DELAY);

	  for(;;)
	  {
		  HAL_I2C_Mem_Read(&hi2c1, imu_addr, 0xF7, I2C_MEMADD_SIZE_8BIT, &ph, 1, HAL_MAX_DELAY);
		  HAL_I2C_Mem_Read(&hi2c1, imu_addr, 0xF8, I2C_MEMADD_SIZE_8BIT, &pl, 1, HAL_MAX_DELAY);
		  HAL_I2C_Mem_Read(&hi2c1, imu_addr, 0xF9, I2C_MEMADD_SIZE_8BIT, &pxl, 1, HAL_MAX_DELAY);

		  int16_t p = (ph<<8) | pl;

		  if((xQueueSend(Pressure, (const void*)(&p), 0))==pdPASS)
		  {
			  for(int i = 0; i < 9; i++)
				  altbuf[i] = altbuf[i+1]; //Rolling buffer
			  altbuf[9] = 4947.19*(8.9611 - pow(p, 0.190255));
			  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); //To prevent locking of shadow registers or smth
			  float t = (sTime.Minutes*60) + sTime.Seconds + ((sTime.SecondFraction - sTime.SubSeconds) * 1000) / (sTime.SecondFraction + 1);
			  xQueueSend(Time, (const void*)(&t), 0);
			  for(int i = 0; i < 9; i++)
				  timebuf[i] = timebuf[i+1];
			  timebuf[9] = t;
			  for(int i = 0; i < 9; i++)
				  velbuf[i] = velbuf[i+1]; //Rolling buffers implementation
			  velbuf[9] = (altbuf[9] - altbuf[8])/(timebuf[9] - timebuf[8]);
			  xQueueSend(Velocity, (const void*)(velbuf + 9), 0);
		  }
		  else
			  xTaskNotifyGive(Log_DataHandle);
			  osDelay(10);
	  }
}
void Get_State(void *argument)
{
	if(state==0) // Pad to Boost
	{
		int i;
		for(i = 0; i<10; i++)
		{
			if(altbuf[i] > min_liftoff_alt)
				continue;
			else
				break;
		}
		if(i == 10)
		{
			state = 1;
			events[1] = timebuf[-1];
		}
	}

	if(state==1) //Boost to Coast
		{
			int i;
			for(i = 0; i<10; i++)
			{
				if(timebuf[-1]-events[1] > force_burnout_time)
					continue;
				else
					break;
			}
			if(i == 10)
			{
				state = 2;
				events[2] = timebuf[-1];
			}
		}

	if(state==2) //Coast to Drogue
		{
			int i;
			for(i = 0; i<10; i++)
			{
				if(altbuf[i] > min_liftoff_alt)
					continue;
				else
					break;
			}
			if(i == 10)
			{
				state = 3;
				events[3] = timebuf[-1];
			}
		}

	if(state==3) //Drogue to Main
		{
			int i;
			for(i = 0; i<10; i++)
			{
				if(velbuf[i] < 0)
					continue;
				else
					break;
			}
			if(((i == 10) && (timebuf[-1] - events[3] > lockout_drogue)) || timebuf[-1] - events[3] > force_drogue)
			{
				state = 4;
				events[4] = timebuf[-1];
			}
		}

	if(state==4) //Main to Landed
		{
			int i;
			float avgvel = 0;
			for(i = 0; i<10; i++)
			{
				if(altbuf[i] < touchdown_alt)
				{
					avg_vel += velbuf[i]/10;
					continue;
				}
				else
					break;
			}
			if((i == 10) && (avg_vel<touchdown_vel_limit) && (timebuf[-1] - events[4] > main_to_touchdown_lockout))
			{
				state = 5;
				events[5] = timebuf[-1];
			}
		}

}
void Log_Data(void *argument) //Logging the Pressure data from the BMP along with the timestamp
{
	int16_t pressure;
	float time;
	float vel;
	uint8_t wren = 0x06;
	while(1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Logging to TDU
		while(xQueueReceive(Pressure, &pressure, portMAX_DELAY)==pdPASS)
		{
			xQueueReceive(Time, &time, portMAX_DELAY);
			xQueueReceive(Velocity, &vel, portMAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*)(&time), 4, HAL_MAX_DELAY);
			HAL_UART_Transmit(&huart1, (uint8_t*)(&pressure), 2, HAL_MAX_DELAY);
		}


		//Logging to Flash

		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &wren, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

		// 2. Prepare the Page Program Header (Command + 24-bit Address)
		uint8_t header[4];
		header[0] = 0x02;                 // Page Program Command
		header[1] = (flash_page >> 8) & 0xFF; // Address bits 23-16
		header[2] = flash_page & 0xFF;  // Address bits 15-8
		header[3] =  offset;         // Address bits 7-0

		// 3. Send the Header and Data Payload
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, header, 4, HAL_MAX_DELAY);       // Send 4-byte header
		HAL_SPI_Transmit(&hspi1, &time, 4, HAL_MAX_DELAY);      // Send data payload (max 256 bytes)
		HAL_SPI_Transmit(&hspi1, &pressure, 2, HAL_MAX_DELAY);
		HAL_SPI_Transmit(&hspi1, &vel, 4, HAL_MAX_DELAY);
		offset += 10;
		if(offset<10)
			flash_page++;



		osDelay(portMAX_DELAY);
	}
}
/* USER CODE END Application */

