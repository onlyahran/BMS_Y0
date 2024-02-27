/*
 * hass.c
 *
 *  Created on: Feb 19, 2024
 *      Author: better.ahran
 */

#define _HASS
#include "hass.h"
#include "i2c.h"
#include "adcfilter.h"

static void Convert_to_current(uint8_t sensor, int16_t value, int16_t *raw);

static void Convert_to_current(uint8_t sensor, int16_t value, int16_t *raw) {
    double tempDouble = 0;
	int16_t tempShort = 0;

    if((uint8_t)sensor < 0) sensor = HASS_MAIN_50S;
    if((uint8_t)sensor > 3) sensor = HASS_PV_100S;

	tempDouble = (double)value * 0.0625;

	if(sensor == HASS_MAIN_50S)
	{
		tempShort = (int16_t)(tempDouble*0.799);         // Main 50S // 0.0625*0.799
	}
	else if(sensor == HASS_ALT_300S)
	{
		tempShort = (int16_t)(tempDouble*4.86);         // alter // 0.0625*4.86
	}
	else if(sensor == HASS_MAIN_300S)
	{
		tempShort = (int16_t)(tempDouble*4.81);         // Main 300S
	}
	else if(sensor == HASS_PV_100S)
	{
		tempShort = (int16_t)(tempDouble*1.6);         //  PV 100S
	}
	*raw = tempShort;
}

bool hass_Convert_current(int filter_max, int16_t* value) {
	int i;
	static stMedFilter filter[MCP_NUM]={\
		[0 ... MCP_NUM-1] = { .arr = {0}, .cnt = 0 }};
	int16_t med[MCP_NUM];
	uint8_t ret=0;

	getMCP3425A();

	// put raw value in array to calculate median
	for(i=0; i<MCP_NUM; i++) {
		if (adcFilter_Med(&filter[i], filter_max, (int)mcp_raw[i], (int*)&med[i])) {
			Convert_to_current(i, med[i], &value[i]);
			ret |= (1<<i);
			//SEGGER_RTT_printf(0, "tNew=%d\r\n", RawNTC_g[i].tNew);
		}
	}
	calculated_g.hass = ret & 0xF;
	return calculated_g.hass;
}

enumTypeAC BackUpSRAM_flag_check(void)
{
	enumTypeAC type = TYPE_AC_CONV_INITIAL;
	uint32_t mark = BackUp_SRAM_Read(BKUP_ADDR_MARK1);

	if(mark == BKUP_SET_FLAG_V2)
	{
		type = TYPE_AC_10mAh;
	}
	else if(mark == BKUP_SET_FLAG)
	{
		type = TYPE_AC_CONV_Ah_TO_10mAh;
	}
	else
	{
		type = TYPE_AC_CONV_INITIAL;
	}

	return type;
}

void BackUpSRAMInit(void)
{
	switch((enumTypeAC) BackUpSRAM_flag_check())
	{
	case TYPE_AC_10mAh:
		diagState_g.fSAC = (float)(int32_t)BackUp_SRAM_Read(BKUP_ADDR_SAC);
		diagState_g.fSAAC = (float)(uint32_t)BackUp_SRAM_Read(BKUP_ADDR_SAAC);
		diagState_g.localCAC = (int32_t)BackUp_SRAM_Read(BKUP_ADDR_LOCAL_CAC);
		diagState_g.SAC = (int32_t)(diagState_g.fSAC / 100);
		diagState_g.SAAC = (uint32_t)(diagState_g.fSAAC / 100);
		break;
	case TYPE_AC_CONV_Ah_TO_10mAh:
		diagState_g.SAC = (int32_t)BackUp_SRAM_Read(BKUP_ADDR_SAC);
		diagState_g.SAAC = (uint32_t)BackUp_SRAM_Read(BKUP_ADDR_SAAC);
		diagState_g.fSAC = ((float)diagState_g.SAC * SENS_AC_CONV_10mA_TO_A);
		diagState_g.fSAAC = ((float)diagState_g.SAAC * SENS_AC_CONV_10mA_TO_A);
		diagState_g.localCAC = (int32_t)(BackUp_SRAM_Read(BKUP_ADDR_LOCAL_CAC) * SENS_AC_CONV_10mA_TO_A);
		break;
	default:
		diagState_g.fSAC = 0;
		diagState_g.fSAAC = 0;
		diagState_g.SAC = 0;
		diagState_g.SAAC = 0;
		diagState_g.localCAC = 0;

		BackUp_SRAM_Write(BKUP_ADDR_SAC, 0);
		BackUp_SRAM_Write(BKUP_ADDR_SAAC, 0);
		BackUp_SRAM_Write(BKUP_ADDR_LOCAL_CAC, 0);
		break;
	}
	BackUp_SRAM_Write(BKUP_ADDR_MARK1, (uint32_t)BKUP_SET_FLAG_V2);
	BackUp_SRAM_Write(BKUP_ADDR_MARK2, (uint32_t)BKUP_SET_FLAG);

}
