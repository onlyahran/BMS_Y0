/*
 * BW_algo.c
 *
 *  CLeated on: Jan 11, 2022
 *      Author: User
 */
#define _ALGO
#include "algo.h"
#include "ltcBms.h"
#include "BW_parser.h"

const uint16_t UnderVolt=25500;
const uint16_t OverVolt=36500;
//const uint16_t VolTLimit1 = 31000;
//const uint16_t VoltLimit2 = 33400;
//const uint16_t VoltLimit3 = 35000;
//const uint16_t VoltLimit4 = 36000;
//const uint16_t VoltLimit5 = 36500;

const int16_t UnderCurrent=-4000;
const int16_t OverCurrent=4000;
//const int16_t CurrentLimit1=-2000;
//const int16_t CurrentLimit2=-200;
//const int16_t CurrentLimit3=-50;
//const int16_t CurrentLimit4=50;
//const int16_t CurrentLimit5=200;
//const int16_t CurrentLimit6=500;
//const int16_t CurrentLimit7=2000;

const int16_t UnderTemps=-200;
const int16_t CellOverTemps=700;
const int16_t BoardOverTemps=1200;
//const int16_t TempsLimit1=600;
const uint16_t ErrNTCDev=20;

const uint8_t NumOfCellCh = LTCBMS_CELLNUM;
const uint8_t NumOfHASS = MCP_NUM;
const uint8_t NumOfNTC = ADC_CHANNEL_MAX;

const uint8_t PeriodChange = 2;

bms_safety_flag safety_g;
bms_safety_flag safety_info_g;
bms_sensor_value sensor_val;

int16_t TempsOld[ADC_CHANNEL_MAX] = {0, };
int16_t TempsNew[ADC_CHANNEL_MAX] = {0, };

uint8_t m0_cnt=0;

static stRelayInfo relayInfo_g = {
	.state = MODE_7,
	.old_state = MODE_7,
};
static void switchMode(uint8_t target);
static void compareAdjacent(int num0, int num1);

static void switchMode(uint8_t target)
{
	relayInfo_g.old_state = relayInfo_g.state;
	if (relayInfo_g.state != target) relayInfo_g.state = target;

	switch(target) {
	case MODE_0:
		if (m0_cnt < CNT_RELAY_MODE0) {
			HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_RESET);
		break;

	case MODE_1:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_SET);
		break;

	case MODE_2:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_SET);
		break;

	case MODE_3:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_SET);
		break;

	case MODE_7:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_RESET);
		break;

	default:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_RESET);
		break;
	}
}

uint8_t getSwitchMode(void)
{
	return relayInfo_g.state;
}

void relayInit(void)
{
	relayInfo_g.state = MODE_7;
	relayInfo_g.old_state = MODE_7;
	switchMode(MODE_7);
}

bool bms_safetycheck_cellV(void) {
	int i;
	static uint8_t cnt = 0;

	if (!calculated_g.ltc || ltc_stat_g.st == LTCBMS_STAT_FAULT) {
		return false;
	}
    for (i=0; i<NumOfCellCh; i++) {
        if (bat_sens_g.cellVoltage[i] < UnderVolt) {
        	safety_g.cellV[i] = UV;
        }
        if (bat_sens_g.cellVoltage[i] > OverVolt) {
        	safety_g.cellV[i] = OV;
        }
    }
	for (i=0; i<NumOfCellCh; i++) {
    	if (safety_g.cellV[i] & (UV | OV)) {
            cnt++;
            break;
        }
    }

	if(cnt >= 2) { // over 2 counts (= 1sec)
		cnt = 0;
		switchMode(MODE_STOP);
		return false;
	} else if (cnt == 1) {
        return false;
    } else {
    	cnt = 0;
		memcpy(&sensor_val.cellV, &bat_sens_g.cellVoltage, NumOfCellCh);
		sensor_val.mincellV = bat_sens_g.minVoltage;
		sensor_val.maxcellV = bat_sens_g.maxVoltage;
	}

	return true;
}

bool bms_safetycheck_I(void) {
	int i;
	static uint8_t cnt = 0;

	if (!calculated_g.hass) {
		return false;
	}
    for ( i=0; i<NumOfHASS; i++) {
        if (bat_sens_g.crntSensor[i] < UnderCurrent) {
        	safety_g.I[i] = UC;
        }
        if (bat_sens_g.crntSensor[i] > OverCurrent) {
        	safety_g.I[i] = OC;
        }
    }
	for (i=0; i<NumOfHASS; i++) {
    	if (safety_g.I[i] & (UC | OC)) {
            cnt++;
            break;
        }
    }

	if(cnt >= 2) { // over 2 counts (= 1sec)
		cnt = 0;
		switchMode(MODE_STOP);
		return false;
	} else if (cnt == 1) {
        return false;
    } else {
    	cnt = 0;
		sensor_val.packI = bat_sens_g.avgCurrent;
	}

	return true;
}

static void compareAdjacent(int num0, int num1) {
	bool ret0 = true; // no fault
	bool ret1 = true; // no fault

	if (abs(TempsNew[num0] - TempsNew[num1]) > ErrNTCDev) { // compare adjacent values A,B
		if (abs(TempsOld[num0] - TempsNew[num0]) > ErrNTCDev) { // compare old and new of A
			ret0 = false;
		}
		if (abs(TempsOld[num1] - TempsNew[num1]) > ErrNTCDev) { // compare old and new of B
			ret1 = false;
		}

		if (ret0 && !ret1){
			sensor_val.T[num0] = TempsNew[num0];
			sensor_val.T[num1] = TempsNew[num0];
		} else if (!ret0 && ret1){
			sensor_val.T[num0] = TempsNew[num1];
			sensor_val.T[num1] = TempsNew[num1];
		} else {
			sensor_val.T[num0] = TempsNew[num0];
			sensor_val.T[num1] = TempsNew[num1];
		}
	} else {
		sensor_val.T[num0] = TempsNew[num0];
		sensor_val.T[num1] = TempsNew[num1];
	}
	TempsOld[num0] = sensor_val.T[num0];
	TempsOld[num1] = sensor_val.T[num1];
}

bool bms_safetycheck_T(void) {
    static uint8_t cnt = 0;
    static bool initialised = false;

    if (!calculated_g.ntc) return false;

    for (int i = 0; i < NumOfNTC; i++) {
        if (bat_sens_g.Temperature[i] < UnderTemps) safety_g.T[i] = UT;
        else if ((i <= 3 && bat_sens_g.Temperature[i] >= BoardOverTemps) ||
                 (i > 3 && bat_sens_g.Temperature[i] >= CellOverTemps)) safety_g.T[i] = (i <= 3) ? boardOT : cellOT;
    }

    for (int i = 0; i < NumOfNTC; i++) {
        if (safety_g.T[i] & (UT | boardOT | cellOT)) {
            cnt++;
            break;
        }
    }

    if (cnt >= 2) {
        cnt = 0;
        switchMode(MODE_STOP);
        return false;
    } else if (cnt == 1) {
        return false;
    } else {
        cnt = 0;
        memcpy(&TempsNew, &bat_sens_g.Temperature, NumOfNTC);
        if (!initialised) {
            initialised = true;
            memcpy(&TempsOld, &TempsNew, NumOfNTC);
        }

        compareAdjacent(TEMP_MAIN_A, TEMP_MAIN_B);
        compareAdjacent(TEMP_DIODE_1, TEMP_DIODE_2);
        compareAdjacent(TEMP_BAT_A, TEMP_BAT_B);
    }

    return true;
}

void bms_fet_control(void) {
	static uint8_t m7_r1=0;
	static uint8_t m7_r2=0;
	static uint8_t m7_r3=0;
	static uint8_t m0_r1=0;
	static uint8_t m0_r2=0;
	static uint8_t m1_r1=0;
	static uint8_t m1_r2=0;
	static bool first_mode_3 = true;
	static uint8_t m3_r1=0;
	static uint8_t m3_r2=0;
	static uint8_t m3_r3=0;
	static uint8_t m3_r4=0;
	static uint8_t m3_r5=0;

	switch (relayInfo_g.state) {
	case MODE_7:
		if ((sensor_val.mincellV < 31000) && (sensor_val.maxcellV < 36000)) {
			m7_r1++;
			m7_r2=0;
			m7_r3=0;
		} else if ((sensor_val.mincellV >= 31000) && (sensor_val.maxcellV < 35000)) {
			m7_r2++;
			m7_r1=0;
			m7_r3=0;
		} else if ((sensor_val.mincellV >= 35000) && (sensor_val.maxcellV < 36000)) {
			m7_r3++;
			m7_r1=0;
			m7_r2=0;
		}

		if (m7_r1 > PeriodChange) {
			m7_r1=0;
			switchMode(MODE_0);
		}
		if (m7_r2 > PeriodChange) {
			m7_r2=0;
			switchMode(MODE_1);
		}
		if (m7_r3 > PeriodChange) {
			m7_r3=0;
			switchMode(MODE_3);
		}
		break;
	case MODE_0:
		if (m0_cnt < CNT_RELAY_MODE0) {
			m0_cnt++;
		} else {
			m0_cnt = CNT_RELAY_MODE0;
		}

		if (((sensor_val.mincellV < 31000) && (sensor_val.maxcellV < 35000))\
			&& ((sensor_val.packI >= 500) && (sensor_val.packI < OverCurrent))) {
			m0_r1++;
			m0_r2=0;
		} else if (((sensor_val.mincellV >= 31000) && (sensor_val.maxcellV < 35000))\
			&& ((sensor_val.packI >= UnderCurrent) && (sensor_val.packI < OverCurrent))) {
			m0_r2++;
			m0_r1=0;
		} else {
			m0_r1=0;
			m0_r2=0;
		}

		if (m0_r1 > PeriodChange) {
			m0_r1=0;
			switchMode(MODE_1);
		}
		if (m0_r2 > PeriodChange) {
			m0_r2=0;
			switchMode(MODE_1);
		}
		break;
	case MODE_1:
		if ((sensor_val.mincellV < 31000) && (sensor_val.maxcellV < 36000)) {
			m1_r1++;
			m1_r2=0;
		} else if ((sensor_val.mincellV >= 36000) && (sensor_val.maxcellV < 36500)) {
			m1_r2++;
			m1_r1=0;
		} else {
			m1_r1=0;
			m1_r2=0;
		}

		if (m1_r1 > PeriodChange) {
			m1_r1=0;
			switchMode(MODE_0);
		}
		if (m1_r2 > PeriodChange) {
			m1_r2=0;
			switchMode(MODE_3);
		}
		break;
	case MODE_3:
		if (((relayInfo_g.old_state == MODE_1)
			&& (relayInfo_g.state == MODE_3)
			&& (!first_mode_3))
			|| ((relayInfo_g.old_state == MODE_7)
				&& (relayInfo_g.state == MODE_3)
				&& (!first_mode_3))) {
			first_mode_3 = true;
			BackUp_SRAM_Write(BKUP_ADDR_LOCAL_CAC, (uint32_t)diagState_g.fSAC);
			diagState_g.localCAC =(int32_t)BackUp_SRAM_Read(BKUP_ADDR_LOCAL_CAC);
		}
		if (sensor_val.mincellV < 31000) {
			m3_r1++;
			m3_r2=0;
			m3_r3=0;
			m3_r4=0;
			m3_r5=0;
		} else if ((sensor_val.mincellV >= 31000) && (sensor_val.maxcellV < 33000)) {
			m3_r2++;
			m3_r1=0;
			m3_r3=0;
			m3_r4=0;
			m3_r5=0;
		} else if ((sensor_val.mincellV >= 31000) && (sensor_val.maxcellV < 34000) && (getPackSOC() < 95)) {
			m3_r3++;
			m3_r1=0;
			m3_r2=0;
			m3_r4=0;
			m3_r5=0;
		} else if ((sensor_val.mincellV >= 31000) && (sensor_val.maxcellV < 34000) && ((diagState_g.localCAC - diagState_g.SAC) > 500)) {
			m3_r4++;
			m3_r1=0;
			m3_r2=0;
			m3_r3=0;
			m3_r5=0;
		} else if ((sensor_val.mincellV >= 31000) && (sensor_val.maxcellV < 36500) \
				&& (sensor_val.packI >= -4000) && (sensor_val.packI < 0)) {
			m3_r5++;
			m3_r1=0;
			m3_r2=0;
			m3_r3=0;
			m3_r4=0;
		}else {
			m3_r1=0;
			m3_r2=0;
			m3_r3=0;
			m3_r4=0;
			m3_r5=0;
		}

		if (m3_r1 > PeriodChange) {
			m3_r1=0;
			first_mode_3 = false;
			switchMode(MODE_0);
		}
		if (m3_r2 > PeriodChange) {
			m3_r2=0;
			first_mode_3 = false;
			switchMode(MODE_1);
		}
		if (m3_r3 > PeriodChange) {
			m3_r3=0;
			first_mode_3 = false;
			switchMode(MODE_1);
		}
		if (m3_r4 > PeriodChange) {
			m3_r4=0;
			first_mode_3 = false;
			switchMode(MODE_1);
		}
		if (m3_r5 > PeriodChange) {
			m3_r5=0;
			first_mode_3 = false;
			switchMode(MODE_1);
		}
		break;
	case MODE_STOP:

		break;
	}
}


void bms_fan_control(void) {
	int i;

	for (i=2; i<4; i++) { // DIODE
		if ((sensor_val.T[i] >= 600) && (sensor_val.T[i] < 1200)) {
			ltc_stat_g.gpio = LTCBMS_ON;
			ltc_stat_g.gpio_st = true;
			break;
		}
		else {
			ltc_stat_g.gpio = LTCBMS_OFF;
		}
	}
}

void bms_ltc_balancing(void) {
    int i;
    uint16_t thres_volt;
    uint8_t cmd;

    if (sensor_val.mincellV >= 33400 && sensor_val.maxcellV < 36500) {
        if (abs(sensor_val.packI) < 50){
        	thres_volt =  100;
        } else if (abs(sensor_val.packI) < 200){
        	thres_volt =  300;
        } else {
			ltc_stat_g.bal=0;
        	return;
        }

        for (i = 0; i < 4; i++) {
        	cmd = (i > 1) ? i + 6 : i;
        	if ((sensor_val.cellV[i] - sensor_val.mincellV) > thres_volt) {
				ltc_stat_g.bal |= (1 << cmd);
			} else {
				ltc_stat_g.bal &= (~(1 << cmd));
			}
        }
		ltc_stat_g.bal_st = (ltc_stat_g.bal > 0);

    } else{
    	ltc_stat_g.bal=0;
    }
}
