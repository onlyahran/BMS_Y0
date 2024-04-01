/*
 * ltcBms.c
 *
 *  Created on: Feb 19, 2024
 *      Author: better.ahran
 */

#define _LTCBMS
#include "ltcBms.h"
#include "ltcDriver.h"
#include "adcFilter.h"

const uint16_t VREG_MIN=25000;
const uint16_t VREG_MAX=55000;
const uint16_t LTCBMS_WDG_TIMEOUT_CNT=40;

uint8_t REG[12] = {0, };
uint32_t t_converted = 0;

bool ltcBms_Status_bal = false; // off
bool ltcBms_Status_gpio = false; // off
uint8_t ltcBms_Count_bal = 0;
uint32_t ltcBms_Count_fault = 0;
bool bmsLtcStatus_g = true;

static bool ltcbms_MedCellVolt(int filter_max, uint16_t *raw, uint16_t *med);
static bool ltcBms_Wrcfg(void);

static bool ltcbms_MedCellVolt(int filter_max, uint16_t *raw, uint16_t *med) {
	int i;
	static stSensorFilter filter[LTCBMS_CELLNUM] = {\
			[0 ... LTCBMS_CELLNUM-1] = { .arr = {0}, .cnt = 0 }};

	uint8_t ret=0;

	for(i=0; i<LTCBMS_CELLNUM; i++) {
		if (adcFilter_Med(&filter[i], filter_max, (int)raw[i], (int*)&med[i])) {
			ret |= (1<<i);
		}
//		filter[i].arr[filter[i].cnt++] = (int)raw[i];
//	    if (filter[i].cnt == filter_max) {
//	    	filter[i].cnt = 0;
//	    	med[i] = findMedian(filter[i].arr, filter_max);
//
//	        //
//	        //SEGGER_RTT_printf(0, "med=%d\r\n", printmed);
//	    }
	}
	calculated_g.ltc = (ret & 0xF) ? true : false;
	return calculated_g.ltc;
}

static bool ltcBms_Wrcfg(void){

	if(LTCSetCommand(LTC_CMD_WRCFGA, 6, ltcInfo_g.config) != LTC_OK){
		return false;
	}
	t_converted = LTCPollAdc();
	if (LTCReadCommand(LTC_CMD_RDCFG, REG, 6) != LTC_OK){
		return false;
	}

	return true;
}

bool ltcbms_Fsm(stBatterySens *sens) {
    uint8_t err;

	if (bmsLtcStatus_g==true) {
        // adstat
        if ((err = LTCWriteCommand(LTC_CMD_ADSTAT)) != LTC_OK) {
            bmsLtcStatus_g = false;
            return bmsLtcStatus_g;
        }
    	t_converted = LTCPollAdc();
		if ((err = LTCReadCommand(LTC_CMD_RDSTATA, REG, 6)) != LTC_OK) {
            bmsLtcStatus_g = false;
            return bmsLtcStatus_g;
        }
        ltcInfo_g.vreg = (uint16_t)((uint16_t)(REG[9] << 8) + (uint16_t)REG[8]);
        if (ltcInfo_g.vreg < VREG_MIN && ltcInfo_g.vreg > VREG_MAX) {
            bmsLtcStatus_g = false;
            err = LTC_ERR_VREG;
            return bmsLtcStatus_g;
		}
        // adcv
        if ((err = LTCWriteCommand(LTC_CMD_ADCV)) != LTC_OK) {
            bmsLtcStatus_g = false;
            return bmsLtcStatus_g;
        }
        t_converted = LTCPollAdc();
        if ((err = LTCReadCommand(LTC_CMD_RDCVA, REG, 6)) != LTC_OK) {
            bmsLtcStatus_g = false;
            return bmsLtcStatus_g;
        }
        ltcInfo_g.cellvolt[0] = (uint16_t)REG[5] << 8 | (uint16_t)REG[4];
	    ltcInfo_g.cellvolt[1] = (uint16_t)REG[7] << 8 | (uint16_t)REG[6];
        if ((err = LTCReadCommand(LTC_CMD_RDCVC, REG, 6)) != LTC_OK) {
            bmsLtcStatus_g = false;
            return bmsLtcStatus_g;
        }
        ltcInfo_g.cellvolt[2] = (uint16_t)REG[5] << 8 | (uint16_t)REG[4];
	    ltcInfo_g.cellvolt[3] = (uint16_t)REG[7] << 8 | (uint16_t)REG[6];

        if (ltcbms_MedCellVolt(SENS_LTC_FILTER_MAX, ltcInfo_g.cellvolt, sens->cellVoltage)) {
            if (bmsCtrl_Gpio_g) { // 켜져라 얍!
                if (!ltcBms_Status_gpio) { // 안켜진 상태면
                    ltcInfo_g.config[0] = 0x0C;  // turn on
                    if (ltcBms_Wrcfg()){
                        ltcBms_Status_gpio = true; // on
                    } else {
                        bmsLtcStatus_g = false;
                        return bmsLtcStatus_g;
                    }
                }
            } else {
                if (ltcBms_Status_gpio) {
                    ltcInfo_g.config[0] = 0x04;  // turn off
                    if (ltcBms_Wrcfg()){
                        ltcBms_Status_gpio = true; // on
                    } else {
                        bmsLtcStatus_g = false;
                        return bmsLtcStatus_g;
                    }
                    ltcBms_Status_gpio = false; // off
                }
            }
            if (bmsCtrl_Bal_g > 0) { // 켜져야 하는 조건이면
                ltcBms_Count_bal += 50; // 50ms마다
                if (!ltcBms_Status_bal) {
                    ltcInfo_g.config[4] = bmsCtrl_Bal_g;// turn on;
                    if (!ltcBms_Wrcfg()){
                        bmsLtcStatus_g = false;
                        return bmsLtcStatus_g;
                    }
                    ltcBms_Count_bal =0;
                }
            } else { // 조건에 해당하지 않음
                if (ltcBms_Status_bal==true && ltcBms_Count_bal >= 1000) {//
                    ltcInfo_g.config[4] = bmsCtrl_Bal_g;// turn off
                    if (!ltcBms_Wrcfg()){
                        bmsLtcStatus_g = false;
                        return bmsLtcStatus_g;
                    }
                }
            }
        }
    } else {
        ltcBms_Count_fault++;
        if (ltcBms_Count_fault > LTCBMS_WDG_TIMEOUT_CNT) {
            ltcBms_Count_fault = 0;

            LTC_Cfg();
            t_converted = LTCPollAdc();

            if ((err = LTCReadCommand(LTC_CMD_RDCFG, REG, 6)) == LTC_OK) {
                bmsLtcStatus_g = true;
            }
        }
    }
    return bmsLtcStatus_g;
}

#if OVER_Y0_2_0_5

bool test_ltcBms_Fan(void) {
	static bool st = false;
	if (st) {
		ltcInfo_g.config[0] = 0x0C;  // turn on
		st = false;
	} else {
		ltcInfo_g.config[0] =0x04;  // turn off
		st = true;
	}

	if(LTCSetCommand(LTC_CMD_WRCFGA, 6, ltcInfo_g.config) != LTC_OK){
		return false;
	}
	t_converted = LTCPollAdc();
	if (LTCReadCommand(LTC_CMD_RDCFG, REG, 6) != LTC_OK){
		return false;
	}
//	if ((REG[4] & ltcInfo_g.config[0]) != 0x04) {
//		return true;
//	} else {
//		LTCSetCommand(LTC_CMD_WRCFGA, 6, ltcInfo_g.config);
//	}
	return true;
}

bool test_ltcBms_Balancing(void) {
	static bool st = false;
	if (st) {
		ltcInfo_g.config[4] = 0xC2;  // turn on
		st = false;
	} else {
		ltcInfo_g.config[4] =0x3C;  // turn off
		st = true;
	}

	if(LTCSetCommand(LTC_CMD_WRCFGA, 6, ltcInfo_g.config) != LTC_OK){
		return false;
	}
	t_converted = LTCPollAdc();
	if (LTCReadCommand(LTC_CMD_RDCFG, REG, 6) != LTC_OK){
		return false;
	}
	if ((REG[8] & ltcInfo_g.config[4]) != 0x3C) {
		return true;
	} else {
		LTCSetCommand(LTC_CMD_WRCFGA, 6, ltcInfo_g.config);
	}
	return true;
}
#endif
