/*
 * BW_fsm.c
 *
 *  Created on: Aug 20, 2021
 *      Author: User
 */

#include "BW_fsm.h"
#include "can.h"
#include "adc.h"
#include "i2c.h"
#include "BW_proc.h"
#include "BW_def.h"
#include "BW_parser.h"
#include "ltcBms.h"

#include "ntc.h"
#include "hass.h"
#include "algo.h"

static stTickcount tickcount_g;
uint8_t Serial_buff[UART_BUFFER_LEN];

void fsmInit(void) {
	//Initialize tickcount
	tickcount_g.sysfsm_tick = 0;
	tickcount_g.bms_50ms_tick = 0;
	tickcount_g.bms_100ms_tick = 0;
	tickcount_g.bms_200ms_tick = 0;
	tickcount_g.bms_1000ms_tick = 0;
	tickcount_g.bms_10000ms_tick = 0;
}

#if OVER_Y0_2_0_5
void fsmTestEvent(void)
{

}
#endif
void fsmEvent(void) {

	if ((HAL_GetTick() - tickcount_g.bms_50ms_tick) >= 50) {			//every 50mSec
		tickcount_g.bms_50ms_tick = HAL_GetTick();

		ltcbms_Fsm();
		ntc_Convert_temperature(SENS_NTC_FILTER_MAX, bat_sens_g.Temperature);
	}

	if ((HAL_GetTick() - tickcount_g.bms_100ms_tick) >= 100) {				//every 100mSec
		tickcount_g.bms_100ms_tick = HAL_GetTick();

		if (hass_Convert_current(SENS_HASS_FILTER_MAX, bat_sens_g.crntSensor)) {
			bat_sens_g.avgCurrent = (bat_sens_g.crntSensor[HASS_MAIN_300S] >= -550 || bat_sens_g.crntSensor[HASS_MAIN_300S] < 550) ? \
					bat_sens_g.crntSensor[HASS_MAIN_50S] : bat_sens_g.crntSensor[HASS_MAIN_300S];

			float fsac = ((float) bat_sens_g.avgCurrent) / 360;
			diagState_g.fSAC += fsac; // 10mA/1h (float)
			diagState_g.fSAAC += (fsac > 0) ? fsac : -fsac; // 10mA/1h (float)
			diagState_g.SAC = (int32_t)diagState_g.fSAC / 100; // 1A/1h
			diagState_g.SAAC = (uint32_t)diagState_g.fSAAC / 100; // 1A/1h
			BackUp_SRAM_Write(BKUP_ADDR_SAC, (uint32_t)diagState_g.fSAC);
			BackUp_SRAM_Write(BKUP_ADDR_SAAC, (uint32_t)diagState_g.fSAAC);
		}

	}

	if ((HAL_GetTick() - tickcount_g.bms_200ms_tick) >= 200) {				//every 200mSec
		tickcount_g.bms_200ms_tick = HAL_GetTick();

		HAL_GPIO_WritePin(LED3_POWER_PUSH_GPIO_Port, LED3_POWER_PUSH_Pin, _blink);
		if (_blink) _blink = 0;
	}

	if ((HAL_GetTick() - tickcount_g.bms_500ms_tick) >= 500) {				 //every 500mSec
		tickcount_g.bms_500ms_tick = HAL_GetTick();

		uint8_t ret=0;
		ret |= bms_safetycheck_cellV() << 0;
		ret |= bms_safetycheck_I() << 1;
		ret |= bms_safetycheck_T() << 2;

		if (!(ret & 0x7)) {
			bms_fet_control();
			bms_fan_control();
			bms_ltc_balancing();
		}
		sendDataToServer();
		HAL_GPIO_TogglePin(LED0_NOTI_GPIO_Port, LED0_NOTI_Pin); // keep alive signal
	}

	if ((HAL_GetTick() - tickcount_g.bms_1000ms_tick) >= 1000) {				//every 1000mSec
		tickcount_g.bms_1000ms_tick = HAL_GetTick();

		if (getSwitchMode() == MODE_7) HAL_GPIO_WritePin(LED1_NOTI_GPIO_Port, LED1_NOTI_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(LED1_NOTI_GPIO_Port, LED1_NOTI_Pin, GPIO_PIN_SET);
	}

	if ((HAL_GetTick() - tickcount_g.bms_10000ms_tick) >= 10000) {				//every 10000mSec
		tickcount_g.bms_10000ms_tick = HAL_GetTick();

		device_ver_reset();
		device_env();
		device_all();
	}
}

void uartParser(void) {
	static uint8_t step = 0;
	static uint8_t datasize = 0;
	static uint8_t checksum = 0;
	static uint8_t buffIndex = 0;
	uint8_t data = 0;

	if (uart_hdr_g != uart_tail_g)
	{
		data = uartBuffer_g[uart_tail_g];

		switch(step)
		{
		case 0:				//STX check
			if (data == 0x02)
			{
				checksum = 0;
				datasize = 0;
				buffIndex = 0;

				step++;
			}
			break;

		case 1:				//length
			datasize = data;
			checksum += data;

			step++;
			break;

		case 2:				//data
			Serial_buff[buffIndex++] = data;
			checksum += data;

			if (buffIndex >= datasize)
			{
				step++;
			}
			break;

		case 3:				     //check checksum
			if (checksum == data)
			{
				step++;
			}
			else
			{
				step = 0;
			}
			break;

		case 4:			           	//ETX check
			if (data == 0x03)
			{
				analysis_parser(Serial_buff, datasize);
			}
			step = 0;
			break;

		default:
			step = 0;
			break;
		}

		uart_tail_g++;
		uart_tail_g %= UART_BUFFER_LEN;
	}
}
