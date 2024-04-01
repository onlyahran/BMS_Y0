/*
 * ltcDriver.c
 *
 *  Created on: Aug 20, 2021
 *      Author: User
 */

#include "ltcDriver.h"
#include "spi.h"
#include "BW_def.h"

_stSlaveCfonfig SlaveConfig_g;
static int16_t pec15Table[256];
const int16_t CRC15_POLY = 0x4599;
stLtcInfo ltcInfo_g;

static void init_PEC15_Table(void);
static uint16_t PECCalculator(const uint8_t *buff, uint16_t length);

static void init_PEC15_Table(void)
{
	int16_t remainder;
	int16_t i, bit;

	for (i = 0; i < 256; i++)
	{
		remainder = i << 7;
		for (bit = 8; bit > 0; --bit)
		{
			if (remainder & 0x4000)
			{
				remainder = ((remainder << 1));
				remainder = (remainder ^ CRC15_POLY);
			}
			else
			{
				remainder = ((remainder << 1));
			}
		}

		pec15Table[i] = remainder & 0xFFFF;
	}
}

static uint16_t PECCalculator(const uint8_t *buff, uint16_t length)
{
	uint16_t remainder, addr;
	uint16_t i;

	remainder = 16;		//initialize the PEC
	for (i = 0; i < length; i++) // loops for each byte in data array
	{
		addr = ((remainder >> 7) ^ buff[i]) & 0xff; //calculate PEC table address
		remainder = (remainder << 8) ^ pec15Table[addr];
	}

	return (remainder * 2); //The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}

void LTCWakeUp(void)
{
	SlaveConfig_g.spiClearCS();
	delay_us(10);
	SlaveConfig_g.spiSetCS();
	delay_us(1);
}

uint32_t LTCPollAdc(void) // SDO polling using PLADC cmd
{
	uint8_t cmd_buff[4] = {0x07, 0x14, };
	uint8_t rcv_buff[8] = {0x00,};
	uint16_t tempData;
	uint8_t finished = 0;
	uint32_t counter = 0;

	tempData = PECCalculator(cmd_buff, 2);
	cmd_buff[2] = (uint8_t)(tempData>>8);
	cmd_buff[3] = (uint8_t)(tempData);

	SlaveConfig_g.spiClearCS();
	//err = HAL_SPI_Transmit(&hspi1, cmd_buff, 4, 100);
	SlaveConfig_g.spiWrite(cmd_buff, 4);

	while((counter<200000) && (finished == 0))
	{
		counter++;
		SlaveConfig_g.spiRead(rcv_buff, 8);
		if(rcv_buff[4] == 0xff)
		{
			finished = 1;
		}
	}
	SlaveConfig_g.spiSetCS();

	return counter;
}

void LTC_Init(void)
{
	uint32_t tickcnt =0;
	uint8_t tmp[12];

	init_PEC15_Table();

	SlaveConfig_g.spiReadWrite = rwFromLTC;
	SlaveConfig_g.spiWrite = writeLTC;
	SlaveConfig_g.spiRead = readLTC;
	SlaveConfig_g.spiClearCS = clearSpiCS;
	SlaveConfig_g.spiSetCS = setSpiCS;

	SlaveConfig_g.spiSetCS();

	LTC_Cfg();

	tickcnt = HAL_GetTick();

	// Timing issue handle : delay for LTC configuration
	while((HAL_GetTick() - tickcnt) < 5000)
	{
		LTCReadCommand(LTC_CMD_RDCFG, tmp, 6);

		// check command : if REFON bit is set to 1, get out of the loop.
		if((tmp[4] & 0x04) != 0)
		{
			break;
		}
		else LTC_Cfg();
	}
}

enumLtcError LTCWriteCommand(uint16_t cmd)
{
	enumLtcError err = LTC_OK;
	uint8_t cmd_buff[20];
	uint16_t tempData;

	cmd_buff[0] = (uint8_t)(cmd>>8);
	cmd_buff[1] = (uint8_t)(cmd);

	tempData = PECCalculator(cmd_buff, 2);
	cmd_buff[2] = (uint8_t)(tempData>>8);
	cmd_buff[3] = (uint8_t)(tempData);

	SlaveConfig_g.spiClearCS();
	err = SlaveConfig_g.spiWrite(cmd_buff, 4);
	SlaveConfig_g.spiSetCS();

	return err;
}

enumLtcError LTCSetCommand(uint16_t cmd, uint8_t len, uint8_t* pCfg)
{
	uint8_t cmd_buff[20];
	uint16_t tempData;
	HAL_StatusTypeDef err;

	cmd_buff[0] = (uint8_t)(cmd>>8);
	cmd_buff[1] = (uint8_t)(cmd);

	tempData = PECCalculator(cmd_buff, 2);
	cmd_buff[2] = (uint8_t)(tempData>>8);
	cmd_buff[3] = (uint8_t)(tempData);

	memcpy(cmd_buff+4, pCfg, len);
	tempData = PECCalculator(pCfg, (uint16_t)len);
	cmd_buff[4+len] = (uint8_t)(tempData>>8);
	cmd_buff[5+len] = (uint8_t)(tempData);

	SlaveConfig_g.spiClearCS();
	err = SlaveConfig_g.spiWrite(cmd_buff, 4+len+2);
	SlaveConfig_g.spiSetCS();

	return err;
}

enumLtcError LTCReadCommand(uint16_t cmd, uint8_t* pOutBuff, uint8_t len)
{
	uint8_t cmd_buff[20];
	uint16_t tempData;
	uint16_t rx_pec;
	HAL_StatusTypeDef err;

	cmd_buff[0] = (uint8_t)(cmd>>8);
	cmd_buff[1] = (uint8_t)(cmd);

	tempData = PECCalculator(cmd_buff, 2);
	cmd_buff[2] = (uint8_t)(tempData>>8);
	cmd_buff[3] = (uint8_t)(tempData);

	// 2 is pec code from ltc6811
	SlaveConfig_g.spiClearCS();
	err = SlaveConfig_g.spiReadWrite(cmd_buff, pOutBuff, 4 + len + 2);
	SlaveConfig_g.spiSetCS();

	// rx_pec check
	tempData = PECCalculator(pOutBuff+4, len);
	rx_pec = (uint16_t)((uint16_t)(pOutBuff[4+len] << 8) | (uint16_t)(pOutBuff[4+len+1] << 0));

	if(tempData != rx_pec){
		err = LTC_ERR_PEC;
	}

	return err;
}

void LTC_Cfg(void)
{
	ltcInfo_g.config[0] = 0x04;     	    // GPIO[7:3]:0, REFON:1, DTEN:0(READ ONLY), ADCOPT:0
	ltcInfo_g.config[1] = 0x00;
	ltcInfo_g.config[2] = 0x00;
	ltcInfo_g.config[3] = 0x00;
	ltcInfo_g.config[4] = 0x00;
	ltcInfo_g.config[5] = 0x80;             // DCTO:8(15 min)

	LTCSetCommand(LTC_CMD_WRCFGA, 6, ltcInfo_g.config);
}
