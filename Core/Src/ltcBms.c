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

ltcbms_status ltc_stat_g = {
	.st = LTCBMS_STAT_ADC,
	.gpio = LTCBMS_DEF,
	.gpio_st = false,
	.bal = LTCBMS_DEF,
	.bal_st = false,
};
uint8_t REG[12] = {0, };
uint32_t t_converted = 0;
uint32_t delay_cnt = 0;

static bool ltcbms_MedCellVolt(int filter_max, uint16_t *raw, uint16_t *med);

static bool ltcbms_MedCellVolt(int filter_max, uint16_t *raw, uint16_t *med) {
	int i;
	static stMedFilter filter[LTCBMS_CELLNUM] = {\
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
	calculated_g.ltc = ret & 0xF;
	return calculated_g.ltc;
}

void ltcbms_Fsm(void) {
    int i;
    uint8_t err;
    uint32_t t_converted;
    uint32_t packVolt;
    uint16_t minVolt = 50000;
    uint16_t maxVolt = 0;
    static uint8_t old_gpio = LTCBMS_DEF;
    static uint8_t old_bal = LTCBMS_DEF;

	if (ltc_stat_g.st == LTCBMS_STAT_ADC) {
        // adstat
        if((err = LTCWriteCommand(LTC_CMD_ADSTAT)) != LTC_OK){
            ltc_stat_g.st = LTCBMS_STAT_FAULT;
            return;
        }
        t_converted = LTCPollAdc();
        if((err = LTCReadCommand(LTC_CMD_RDSTATA, REG, 6)) != LTC_OK){
            ltc_stat_g.st = LTCBMS_STAT_FAULT;
            return;
        }
        ltc_info_g.vreg = (uint16_t)((uint16_t)(REG[9] << 8) + (uint16_t)REG[8]);
        if(ltc_info_g.vreg < VREG_MIN && ltc_info_g.vreg > VREG_MAX) {
            ltc_stat_g.st = LTCBMS_STAT_FAULT;
            err = LTC_ERR_VREG;
            return;
		}
        // adcv
        if((err = LTCWriteCommand(LTC_CMD_ADCV)) != LTC_OK){
            ltc_stat_g.st = LTCBMS_STAT_FAULT;
            return;
        }
        t_converted = LTCPollAdc();
        if((err = LTCReadCommand(LTC_CMD_RDCVA, REG, 6)) != LTC_OK){
            ltc_stat_g.st = LTCBMS_STAT_FAULT;
            return;
        }
        ltc_info_g.cellvolt[0] = (uint16_t)REG[5] << 8 | (uint16_t)REG[4];
	    ltc_info_g.cellvolt[1] = (uint16_t)REG[7] << 8 | (uint16_t)REG[6];
        if((err = LTCReadCommand(LTC_CMD_RDCVC, REG, 6)) != LTC_OK){
            ltc_stat_g.st = LTCBMS_STAT_FAULT;
            return;
        }
        ltc_info_g.cellvolt[2] = (uint16_t)REG[5] << 8 | (uint16_t)REG[4];
	    ltc_info_g.cellvolt[3] = (uint16_t)REG[7] << 8 | (uint16_t)REG[6];

        if (ltcbms_MedCellVolt(SENS_LTC_FILTER_MAX, ltc_info_g.cellvolt, bat_sens_g.cellVoltage)) {
            /* Pack volt*/
        	packVolt = bat_sens_g.cellVoltage[0];
        	packVolt += bat_sens_g.cellVoltage[1];
        	packVolt += bat_sens_g.cellVoltage[2];
        	packVolt += bat_sens_g.cellVoltage[3];
        	packVolt /= 10;
        	bat_sens_g.packVoltage = packVolt;

        	/* Min, Max Cell Volt */
            for(i=0; i<LTCBMS_CELLNUM; i++) {
                if(minVolt > bat_sens_g.cellVoltage[i]) {
                	minVolt = bat_sens_g.cellVoltage[i];
                }
                if(maxVolt < bat_sens_g.cellVoltage[i]) {
                	maxVolt = bat_sens_g.cellVoltage[i];
                }
            }
            bat_sens_g.minVoltage = minVolt;
            bat_sens_g.maxVoltage = maxVolt;
            /* WDG & Discharge timer reset */
            if ((!ltc_stat_g.gpio_st) || (!ltc_stat_g.bal_st)) {
                LTC_Cfg();
                //LTCSetCommand(LTC_CMD_WRCFGA, 6, ltc_info_g.config);
            }
        }

        if (ltc_stat_g.gpio_st) {
        	if(old_gpio != ltc_stat_g.gpio) {
            	if(ltc_stat_g.gpio == LTCBMS_ON) {
            		ltc_info_g.config[0] =0x84;
            	} else if(ltc_stat_g.gpio == LTCBMS_OFF) {
            		ltc_info_g.config[0] =0x04;
            	}
            	old_gpio = ltc_stat_g.gpio;

                if((err = LTCSetCommand(LTC_CMD_WRCFGA, 6, ltc_info_g.config)) != LTC_OK){
                    ltc_stat_g.st = LTCBMS_STAT_FAULT;
                    return;
                }
                t_converted = LTCPollAdc();
                if ((err = LTCReadCommand(LTC_CMD_RDCFG, REG, 6)) != LTC_OK){
                    ltc_stat_g.st = LTCBMS_STAT_FAULT;
                    return;
                }

                if(ltc_stat_g.gpio == LTCBMS_OFF) {
                	ltc_stat_g.gpio_st = false;
                	old_gpio = LTCBMS_DEF;
                }
        	}
        }
		if (ltc_stat_g.bal_st && old_bal != ltc_stat_g.bal) {
			ltc_info_g.config[4] = ltc_stat_g.bal;
			old_bal = ltc_stat_g.bal;

			if(old_bal >0 && ltc_stat_g.bal==0) {
				ltc_stat_g.bal_st=false;
			}
			if((err = LTCSetCommand(LTC_CMD_WRCFGA, 6, ltc_info_g.config)) != LTC_OK){
				ltc_stat_g.st = LTCBMS_STAT_FAULT;
				return;
			}
			t_converted = LTCPollAdc();
			if ((err = LTCReadCommand(LTC_CMD_RDCFG, REG, 6)) != LTC_OK){
				ltc_stat_g.st = LTCBMS_STAT_FAULT;
				return;
			}
		}
    } else if (ltc_stat_g.st == LTCBMS_STAT_FAULT) {
        delay_cnt++;
        if(delay_cnt > LTCBMS_WDG_TIMEOUT_CNT) {
            delay_cnt = 0;

            LTC_Cfg();
            t_converted = LTCPollAdc();
            if((err = LTCReadCommand(LTC_CMD_RDCFG, REG, 6)) == LTC_OK){
                ltc_stat_g.st = LTCBMS_STAT_ADC;
            }
        }
    }
}
