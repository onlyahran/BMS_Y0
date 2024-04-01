/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_dma.h"

#include "stm32f4xx_ll_exti.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int8_t FLASH_Write(uint8_t* buffer);
void FLASH_Read(uint8_t* buffer);
void enable_backup_sram(void);
void BackUp_SRAM_Write(uint32_t l_data, uint32_t offset);
uint32_t BackUp_SRAM_Read(uint32_t offset);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RELAY_DIODE1_Pin GPIO_PIN_2
#define RELAY_DIODE1_GPIO_Port GPIOE
#define RELAY_MAIN_Pin GPIO_PIN_3
#define RELAY_MAIN_GPIO_Port GPIOE
#define SIP3_Pin GPIO_PIN_4
#define SIP3_GPIO_Port GPIOE
#define SIP2_Pin GPIO_PIN_5
#define SIP2_GPIO_Port GPIOE
#define SIP1_Pin GPIO_PIN_6
#define SIP1_GPIO_Port GPIOE
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOE
#define LED0_NOTI_Pin GPIO_PIN_13
#define LED0_NOTI_GPIO_Port GPIOE
#define LED1_NOTI_Pin GPIO_PIN_14
#define LED1_NOTI_GPIO_Port GPIOE
#define I2C2_SCL_Pin GPIO_PIN_10
#define I2C2_SCL_GPIO_Port GPIOB
#define I2C2_SDA_Pin GPIO_PIN_11
#define I2C2_SDA_GPIO_Port GPIOB
#define SCS_Pin GPIO_PIN_8
#define SCS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_6
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_7
#define I2C1_SDA_GPIO_Port GPIOB
#define LED3_POWER_PUSH_Pin GPIO_PIN_0
#define LED3_POWER_PUSH_GPIO_Port GPIOE
#define RELAY_DIODE2_Pin GPIO_PIN_1
#define RELAY_DIODE2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

#define DEBUGGING
#define TEST_OTA

// add cantp, 2023-12-08
#define CANTP_ENABLE                           (0)
#ifdef CANTP_ENABLE
#define CANTP_TEST_VALUE                       (0xAABBCCDD)
#endif
#define MAJOR_FW_VER                           (57)

// Mode
#if (MAJOR_FW_VER % 2)
#define OVER_Y0_2_0_5                          1
#else
#define OVER_Y0_2_0_5                          0
#endif

#define BALANCING_MODE                         1

// Test Mode
#if OVER_Y0_2_0_5
#define SIP_BOARD_TEST                         1
#define SIP_EV                                 5 //2023-05-10
#define SIP_Y0_2_0_5                           7
#endif

#define COMM_VER                                4

// CAN
#define CAN_RXBUF_SIZE				       	      8
#define CAN_RXFRAME_MAX				            100

// USART
#define UART_BUFFER_LEN                          80

// ADC
#define ADC_CHANNEL_MAX                          (6)
// I2C
#define MCP_NUM                                  (4)
//LTC
#define LTCBMS_CELLNUM                           (4)

// ESP32
#define DEF_WRITE                                 0
#define DEF_READ                                  1
#define ESP_EEP_DATA_PACKET                      20

#define ESP_EEP_FWVER                             80

#define INIT_CR                                1000
// SRAM
#define BKUP_ADDR_SAC                             10
#define BKUP_ADDR_SAAC                            11

#define BKUP_ADDR_LOCAL_CAC                       22

#define BKUP_ADDR_MARK1                           0
#define BKUP_ADDR_MARK2                           256
#define BKUP_SET_FLAG                             11  // 11(dec)
#define BKUP_SET_FLAG_V2                          0x1A

#define CNT_RELAY_MODE0                           200
// SAC MCU 단위: 10mA/1h
#define SENS_AC_CONV_SEC_TO_H                       360 // 100mA/1sec -> 10mA/h 단위 환산
#define SENS_AC_CONV_10mA_TO_A                      100 //  10mA/h     ->   A/h 단위 환산
#define SENS_AC_CONV_Aper6Min_TO_A                  10  //  A/6Min    -> 10mA/h 단위 환산

#define SENS_ADC_DEFAULT        (0xFFFF)

#define SENS_NTC_FILTER_MAX     (10)
#define SENS_LTC_FILTER_MAX     (10)
#define SENS_HASS_FILTER_MAX    (5)

typedef enum{
	TYPE_AC_10mAh = 0,
	TYPE_AC_CONV_Ah,
	TYPE_AC_CONV_Ah_TO_10mAh,
	TYPE_AC_CONV_Aper6Min_TO_10mAh,
	TYPE_AC_CONV_INITIAL,
} enumTypeAC;

typedef enum{
	HASS_MAIN_50S     =    0U,
	HASS_MAIN_300S    =    1U,
	HASS_ALT_300S     =    2U,
	HASS_PV_100S      =    3U
} enumCrntsens;

typedef enum{
	TEMP_MAIN_A       =    0U,
	TEMP_MAIN_B       =    1U,
	TEMP_DIODE_1      =    2U,
	TEMP_DIODE_2      =    3U,
	TEMP_BAT_A        =    4U,
	TEMP_BAT_B        =    5U
} enumTemperature;

typedef struct
{
	uint32_t id;
	uint8_t dlc;
	uint8_t buffer[CAN_RXBUF_SIZE];
} stCanBuffer;

typedef struct
{
	uint16_t cellVoltage[LTCBMS_CELLNUM];
	uint16_t minVoltage;
	uint16_t maxVoltage;
	uint16_t packVoltage;
	int16_t avgCurrent;      // 100mA/1Sec
	int16_t crntSensor[MCP_NUM];
	int16_t Temperature[ADC_CHANNEL_MAX];
	float favgCurrent;
} stBatterySens;

typedef struct
{
	int32_t SAC;                 // Ah
	uint32_t SAAC;               // Ah
	float fSAC;                  // 10mAh
	float fSAAC;                 // 10mAh
	int32_t localCAC;            // 10mAh
} stDiagState;

typedef struct {
	bool ltc;
	bool hass;
	bool ntc;
} stcalculated;

extern stBatterySens batterySens_g;
extern stDiagState diagState_g;
extern stcalculated calculated_g;
extern uint8_t hw_ver;

// CAN
extern stCanBuffer rxCANBuffer_g[CAN_RXFRAME_MAX];
extern uint8_t rxcan_hdr_g, rxcan_tail_g;
extern uint8_t _blink;

// UART
extern uint8_t uartBuffer_g[];
extern uint8_t uart_hdr_g;
extern uint8_t uart_tail_g;

uint8_t getSip(void);
extern uint8_t sip;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
