/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "BW_fsm.h"
#include "ltcDriver.h"
#include "BW_proc.h"
#include "usart.h"
#include "BW_def.h"
#include "BW_parser.h"
#include "adc.h"
#include "hass.h"
#include "algo.h"
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

/* USER CODE BEGIN PV */
//IWDG_HandleTypeDef hiwdg;
stBatterySens batterySens_g;
stDiagState diagState_g;
stcalculated calculated_g;
uint8_t ver[3] = {COMM_VER, MAJOR_FW_VER, 0};
uint8_t sip = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  // periperal init
  /* DMA must set before ADC initialization*/
  ADC_Init();
  CAN_Init();
  USART_Init();
  HAL_TIM_Base_Start_IT(&htim6);
  __HAL_IWDG_START(&hiwdg);

#if OVER_Y0_2_0_5
  sip = getSip();
#endif

  /// LTC6811 : voltage sensor
  LTC_Init();
  /// MCP3425 : current sensor
  setMCP3425A();
  fsmInit();
  relayInit();
  device_Init();
  batst_Init();
  BackUpSRAMInit();

#if OVER_Y0_2_0_5
	if(sip == SIP_BOARD_TEST)
	{
		while(true)
		{
			test_FsmEvent();
			checkCANBuffer();
		}
	}
#endif
  esp32_eep_write(ESP_EEP_FWVER, ver, 3);

#ifdef DEBUGGING
  __HAL_DBGMCU_FREEZE_IWDG();
  __HAL_DBGMCU_FREEZE_TIM6();
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	fsmEvent();
	uartParser();
	checkCANBuffer();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int8_t FLASH_Write(uint8_t* buffer)
{
  uint16_t WriteData, MemData;
  uint32_t Address = 0x8008000;
  uint32_t SectorError ;
  FLASH_EraseInitTypeDef EraseInitStruct;
  HAL_StatusTypeDef status = HAL_OK;

	WriteData = *(uint16_t *)(buffer);

	/* Unlock to control */
	HAL_FLASH_Unlock();

	/* Clear FLASH error pending bits */
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR| FLASH_FLAG_PGSERR);

	/* Erase sectors */
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = 2; //0x8008000 ~ 0x800BFFF
	EraseInitStruct.NbSectors = 1;

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		return -1; //failure
	}

	/* Clear cache for flash */
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, Address, WriteData);
	if(status == HAL_OK)
	{
		MemData = (*(__IO uint16_t*) Address);
		if(WriteData !=  MemData)
		{
			status = HAL_ERROR;
		}
	}

	/* Lock flash control register */
	HAL_FLASH_Lock();

	if (status != HAL_OK)
	{
		return -1; //failure
	}
	return 0; //success
}

void FLASH_Read(uint8_t* buffer)
{
	uint32_t Address = 0x8008000;
	uint8_t i;

	for(i=0; i<2; i++)
	{
		buffer[i] = *((uint8_t *)Address);
		Address++;
	}
}

uint8_t getSip(void)
{
	uint8_t getsip = 0;
	getsip  = (uint8_t)(HAL_GPIO_ReadPin(GPIOE, SIP1_Pin) << 0);
	getsip |= (uint8_t)(HAL_GPIO_ReadPin(GPIOE, SIP2_Pin) << 1);
	getsip |= (uint8_t)(HAL_GPIO_ReadPin(GPIOE, SIP3_Pin) << 2);
	return getsip;
}

void enable_backup_sram(void)
{
    /*DBP : Enable access to Backup domain */
    HAL_PWR_EnableBkUpAccess();
    /*PWREN : Enable backup domain access  */
    __HAL_RCC_PWR_CLK_ENABLE();
    /*BRE : Enable backup regulator
      BRR : Wait for backup regulator to stabilize */
    //HAL_PWREx_EnableBkUpReg();
   /*DBP : Disable access to Backup domain */
    //HAL_PWR_DisableBkUpAccess();
}

void BackUp_SRAM_Write(uint32_t offset, uint32_t l_data)
{
	  enable_backup_sram();

   /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write on specific location of backup SRAM */
  *(uint32_t *) (BKPSRAM_BASE + offset*4) = l_data;

 // MemData = (*(__IO uint16_t*) Address);
 /* Disable clock to BKPSRAM */
 __HAL_RCC_BKPSRAM_CLK_DISABLE();
}

uint32_t BackUp_SRAM_Read(uint32_t offset)
{
   uint32_t i_retval;

   enable_backup_sram();

  /* Enable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_ENABLE();
  /* Pointer write from specific location of backup SRAM */
  i_retval =  *(uint32_t*) (BKPSRAM_BASE + offset*4);
  /* Disable clock to BKPSRAM */
  __HAL_RCC_BKPSRAM_CLK_DISABLE();
  return i_retval;
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
