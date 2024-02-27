/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "BW_parser.h"
#include "BW_def.h"

CAN_RxHeaderTypeDef canHeader_g;
stCanBuffer rxCANBuffer_g[CAN_RXFRAME_MAX];
uint8_t rxcan_hdr_g, rxcan_tail_g;
uint8_t _blink = 0;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Init(void)
{
	//uint8_t retry_cnt;
	CAN_FilterTypeDef     sFilterConfig;

	MX_CAN1_Init();

	sFilterConfig.FilterMode  = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x000<<5;
	sFilterConfig.FilterIdLow = 0;
	sFilterConfig.FilterMaskIdHigh = 0x000<<5;
	sFilterConfig.FilterMaskIdLow = 0;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.FilterBank  = 1;
	sFilterConfig.SlaveStartFilterBank = 14;

	if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_CAN_Start(&hcan1) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		Error_Handler();
	}

	// can buffer index parameter init
	rxcan_hdr_g = 0;
	rxcan_tail_g = 0;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1)
	{
		if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &canHeader_g, rxCANBuffer_g[rxcan_hdr_g].buffer) == HAL_OK)
		{
			rxCANBuffer_g[rxcan_hdr_g].id = (uint32_t)canHeader_g.StdId;
			rxCANBuffer_g[rxcan_hdr_g].dlc = canHeader_g.DLC;

			rxcan_hdr_g++;
			rxcan_hdr_g %= CAN_RXFRAME_MAX;

			_blink = 1;
		}
	}
}

void checkCANBuffer(void)
{
	if(rxcan_hdr_g != rxcan_tail_g)
	{
		InternalDataParser(rxCANBuffer_g[rxcan_tail_g].id, rxCANBuffer_g[rxcan_tail_g].buffer, rxCANBuffer_g[rxcan_tail_g].dlc);

		rxcan_tail_g++;
		rxcan_tail_g %= CAN_RXFRAME_MAX;
	}
}

void sendCan(uint32_t canid, uint8_t *pBuff, uint8_t len)
{
	uint8_t i;
	uint32_t mailBox;
	CAN_TxHeaderTypeDef canHeader;
	static uint8_t can_tx_err = 0;

	canHeader.IDE = CAN_ID_STD;
	canHeader.StdId = canid;         // CAN ID
	canHeader.DLC = len;             // data length
	canHeader.RTR = CAN_RTR_DATA;

	for(i=0; i < 3; i++)
	{
		if(HAL_CAN_AddTxMessage(&hcan1, &canHeader, pBuff, &mailBox) == HAL_OK)
		{
			break;
		}
		HAL_Delay(1);
	}

	if(i == 3)
	{
		can_tx_err++;
		if(can_tx_err >= 10)
		{
			can_tx_err = 0;
			HAL_CAN_DeInit(&hcan1);
			CAN_Init();
		}
	}
	else
	{
		can_tx_err = 0;
	}

}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
