/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
int mcp_raw[MCP_NUM];
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */

  /* USER CODE END I2C1_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
  /* USER CODE BEGIN I2C1_MspInit 1 */

  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = I2C2_SCL_Pin|I2C2_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();

    /**I2C1 GPIO Configuration
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA
    */
    HAL_GPIO_DeInit(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin);

    HAL_GPIO_DeInit(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin);

  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin);

    HAL_GPIO_DeInit(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void setMCP3425A(void)
{
	uint8_t DevAddress = 0x68;

	// set configuration byte
	uint8_t buffer[4] = {0x18, };

	I2C_reset(&hi2c1);
	if(sip != SIP_EV)
	{
		I2C_reset(&hi2c2);
	}

	for(uint8_t slave_addr=0; slave_addr<2; slave_addr++)
	{
		HAL_I2C_Master_Transmit(&hi2c1, ((DevAddress | slave_addr) << 1), &buffer[0], 1, 100);
		if(sip != SIP_EV)
		{
			HAL_I2C_Master_Transmit(&hi2c2, ((DevAddress | slave_addr) << 1), &buffer[0], 1, 100);
		}
	}
}

void getMCP3425A(void)
{
	static uint8_t buffer[4];
	uint8_t DevAddress = 0x68;
	uint16_t tempU16;
	static uint8_t i2c1_err = 0, i2c2_err = 0;
	HAL_StatusTypeDef i2c1_ret = HAL_OK;
	HAL_StatusTypeDef i2c2_ret = HAL_OK;
//	uint32_t busy_tick;

	for(uint8_t slave_addr=0; slave_addr<2; slave_addr++)
    {
		i2c1_ret = HAL_I2C_Master_Receive(&hi2c1, ((DevAddress | slave_addr) << 1), buffer, 3, 100);

		if(buffer[2] != 0x18)  // check configuration bit
		{
			buffer[0] = 0x18;
			HAL_I2C_Master_Transmit(&hi2c1, ((DevAddress | slave_addr) << 1), &buffer[0], 1, 100);
			break;
		}

		if(i2c1_ret == HAL_OK)
		{
		 i2c1_err = 0;
		}
//		else if(i2c1_ret == HAL_ERROR)
		else
		{
		 i2c1_err++;
		 if(i2c1_err > 3)
		 {
			i2c1_err = 0;
			I2C_reset(&hi2c1);
		 }
		}
//		else if(i2c1_ret == HAL_BUSY)
//		{
//		 busy_tick = HAL_GetTick();
//		 do {
//			HAL_Delay(1);
//			ret = HAL_I2C_Master_Receive(&hi2c1, ((DevAddress | slave_addr) << 1), buffer, 3, 10000);
//			if(HAL_GetTick() - busy_tick >= 25) break;
//		 } while(ret == HAL_BUSY);
//		}

		if(i2c1_ret == HAL_OK)
		{
			tempU16 = (uint16_t)((uint16_t)buffer[0] <<8) | (uint16_t)buffer[1];
			if (!slave_addr) {
				mcp_raw[HASS_MAIN_50S] = tempU16;
			} else {
				mcp_raw[HASS_ALT_300S] = tempU16;
			}
//			if(slave_addr == 0) setCrntsens(HASS_MAIN_50S, tempU16, &mcp_raw[HASS_MAIN_50S]);
//			if(slave_addr == 1) setCrntsens(HASS_ALT_300S, tempU16, &mcp_raw[HASS_ALT_300S]);
		}

    }
	if(sip != SIP_EV)
	{
		for(uint8_t slave_addr=0; slave_addr<2; slave_addr++)
		{
			i2c2_ret = HAL_I2C_Master_Receive(&hi2c2, ((DevAddress | slave_addr) << 1), buffer, 3, 100);

			if(buffer[2] != 0x18)  // check configuration bit
			{
				buffer[0]=0x18;
				HAL_I2C_Master_Transmit(&hi2c2, ((DevAddress | slave_addr) << 1), &buffer[0], 1, 100);
				break;
			}

			if(i2c2_ret == HAL_OK)
			{
			 i2c2_err = 0;
			}
			else
			{
			 i2c2_err++;
			 if(i2c2_err > 3)
			 {
				i2c2_err = 0;
				I2C_reset(&hi2c2);
			 }
			}
	//		else if(i2c2_ret == HAL_BUSY)
	//		{
	//		 busy_tick = HAL_GetTick();
	//		 do {
	//			HAL_Delay(1);
	//			ret = HAL_I2C_Master_Receive(&hi2c2, ((DevAddress | slave_addr) << 1), buffer, 3, 10000);
	//			if(HAL_GetTick() - busy_tick >= 25)
	//			{
	//				break;
	//			}
	//		 } while(ret == HAL_BUSY);
	//		}

			if(i2c2_ret == HAL_OK)
			{
				tempU16 = (uint16_t)((uint16_t)buffer[0] <<8) | (uint16_t)buffer[1];
				if (!slave_addr) {
					mcp_raw[HASS_MAIN_300S] = tempU16;
				} else {
					mcp_raw[HASS_PV_100S] = tempU16;
				}
//				if(slave_addr == 0) setCrntsens(HASS_MAIN_300S, tempU16, &mcp_raw[HASS_MAIN_300S]);
//				if(slave_addr == 1) setCrntsens(HASS_PV_100S, tempU16, &mcp_raw[HASS_PV_100S]);
			}
		}
	}
}

void I2C_reset(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	HAL_I2C_MspDeInit(hi2c);

	if(hi2c->Instance == I2C1)
	{
		// gpio set for SCL pin Swing
	  GPIO_InitStruct.Pin = I2C1_SCL_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(I2C1_SCL_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = I2C1_SDA_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(I2C1_SDA_GPIO_Port, &GPIO_InitStruct);

		while(HAL_GPIO_ReadPin(I2C1_SDA_GPIO_Port, I2C1_SDA_Pin) == 0)
		{
			HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(I2C1_SCL_GPIO_Port, I2C1_SCL_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
		}
	}
	else if(hi2c->Instance == I2C2)
	{
		// gpio set for SCL pin Swing
	  GPIO_InitStruct.Pin = I2C2_SCL_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(I2C2_SCL_GPIO_Port, &GPIO_InitStruct);

	  GPIO_InitStruct.Pin = I2C2_SDA_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  HAL_GPIO_Init(I2C2_SDA_GPIO_Port, &GPIO_InitStruct);

		while(HAL_GPIO_ReadPin(I2C2_SDA_GPIO_Port, I2C2_SDA_Pin) == 0)
		{
			HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_RESET);
			HAL_Delay(1);
			HAL_GPIO_WritePin(I2C2_SCL_GPIO_Port, I2C2_SCL_Pin, GPIO_PIN_SET);
			HAL_Delay(1);
		}
	}
	HAL_I2C_MspInit(hi2c);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
