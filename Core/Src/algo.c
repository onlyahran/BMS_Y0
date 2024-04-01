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

const uint16_t UnderVoltLimit=25500;
const uint16_t OverVoltLimit=36500;
const int16_t UnderCurrentLimit_l=-2500;
const int16_t OverCurrentLimit_l=2500;
const int16_t UnderCurrentLimit_h=-3000;
const int16_t OverCurrentLimit_h=3000;
const int16_t UnderTempsLimit=-200;
const int16_t CellOverTempsLimit=700;
const int16_t BoardOverTempsLimit=1200;
const uint16_t DevTempsLimit = 5;
const uint8_t PeriodChange = 2;

const uint16_t THRESHOLD_VOLTAGE_1=100;
const uint16_t THRESHOLD_VOLTAGE_2=300;
const uint16_t MAX_ABS_CURRENT_1 =50;
const uint16_t MAX_ABS_CURRENT_2 =200;
const uint16_t MIN_VOLTAGE_THRESHOLD =33400;
const uint16_t MAX_VOLTAGE_THRESHOLD =36500;

#define WINDOW_FAULT_CELLV                        (2) // 1sec
#define WINDOW_FAULT_I_low                        (20) // 10sec
#define WINDOW_FAULT_I_high                       (6) // 3sec
#define WINDOW_FAULT_TEMPS                        (2) // 1sec

stBmsSafetyFlag safetyFlag;
stBatterySens batterySens_algo;
static uint8_t m0_cnt=0;

static stRelayInfo relayInfo_g = {
	.state = MODE_7,
	.old_state = MODE_7,
};

float avgCellV;
static uint16_t data_v[NumOfCellCh][WINDOW_FAULT_CELLV] = {0};
static uint8_t index_v[NumOfCellCh] = {0};
float avgCellI;
static uint16_t data_l[NumOfHASS][WINDOW_FAULT_I_low] = {0};
static uint16_t data_h[NumOfHASS][WINDOW_FAULT_I_high] = {0};
static uint8_t index_l[NumOfHASS] = {0};
static uint8_t index_h[NumOfHASS] = {0};
float avgTemps = 0;
static float avgTempsOld = 0;
static uint16_t data_t[NumOfNTC][WINDOW_FAULT_TEMPS] = {0};
static uint8_t index_t[NumOfNTC] = {0};
static uint8_t devTempsCounts = 0;

static void switchMode(uint8_t target);
static float calculateAverage(int data[], int size);

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
#ifndef TEST_OTA
		HAL_GPIO_WritePin(LED1_NOTI_GPIO_Port, LED1_NOTI_Pin, GPIO_PIN_SET);
#endif
		break;

	case MODE_1:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_SET);

#ifndef TEST_OTA
		HAL_GPIO_WritePin(LED1_NOTI_GPIO_Port, LED1_NOTI_Pin, GPIO_PIN_SET);
#endif
		break;

	case MODE_2:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_SET);

#ifndef TEST_OTA
		HAL_GPIO_WritePin(LED1_NOTI_GPIO_Port, LED1_NOTI_Pin, GPIO_PIN_SET);
#endif
		break;

	case MODE_3:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_SET);

#ifndef TEST_OTA
		HAL_GPIO_WritePin(LED1_NOTI_GPIO_Port, LED1_NOTI_Pin, GPIO_PIN_SET);
#endif
		break;

	case MODE_STOP:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_RESET);
#ifndef TEST_OTA
		HAL_GPIO_WritePin(LED1_NOTI_GPIO_Port, LED1_NOTI_Pin, GPIO_PIN_SET);
#endif
		break;

	default:
		HAL_GPIO_WritePin(RELAY_MAIN_GPIO_Port, RELAY_MAIN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE1_GPIO_Port, RELAY_DIODE1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(RELAY_DIODE2_GPIO_Port, RELAY_DIODE2_Pin, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(LED1_NOTI_GPIO_Port, LED1_NOTI_Pin, GPIO_PIN_RESET);
		break;
	}
}

uint8_t getSwitchMode(void) {
	return relayInfo_g.state;
}

void relayInit(void) {
	relayInfo_g.state = MODE_7;
	relayInfo_g.old_state = MODE_7;
	switchMode(MODE_7);
}

static float calculateAverage(int data[], int size) {
    int sum = 0;
    for (int i = 0; i < size; i++) {
        sum += data[i];
    }
    return (float)sum / size;
}

bool bmsSafety_Check_CellV(stBatterySens *sens) {
	int i;

	if (!bmsLtcStatus_g) {
		return false;
	}

    for (i=0; i<NumOfCellCh; i++) {
        if (sens->cellVoltage[i] < UnderVoltLimit) {
        	safetyFlag.cellV[i] = BMS_SAFETY_WARNING_UV;
        } else if (sens->cellVoltage[i] > OverVoltLimit) {
        	safetyFlag.cellV[i] = BMS_SAFETY_WARNING_OV;
        } else {
            continue;
        }

        data_v[i][index_v[i]] = sens->cellVoltage[i];
        index_v[i] = (index_v[i]+1) % WINDOW_FAULT_CELLV;

        if (index_v[i] >= (WINDOW_FAULT_CELLV-1)) {
            avgCellV = calculateAverage((int*)data_v[i], WINDOW_FAULT_CELLV);

            if (avgCellV < UnderVoltLimit) {
                safetyFlag.cellV[i] = BMS_SAFETY_FAULT_UV;
            } else if (avgCellV > OverVoltLimit) {
                safetyFlag.cellV[i] = BMS_SAFETY_FAULT_OV;
            } else {
                safetyFlag.cellV[i] = 0;
            }

            memset(data_v[i], 0, sizeof(data_v[i]));
            index_v[i] = 0;
        }
    }

    for (i=0; i<NumOfCellCh; i++) {
        if (safetyFlag.cellV[i] == BMS_SAFETY_FAULT_UV || safetyFlag.cellV[i] == BMS_SAFETY_FAULT_OV) {
            switchMode(MODE_STOP);
            return false;
        }
    }

    return true;
}


bool bmsSafety_Check_I(stBatterySens *sens) {
	int i;

	if (calculated_g.hass== false) {
		return false;
	}

    for (i=0; i<NumOfHASS; i++) {
        if (sens->crntSensor[i] < UnderCurrentLimit_l) {
        	safetyFlag.I[i] = BMS_SAFETY_WARNING_UC;
        } else if (sens->crntSensor[i] > OverCurrentLimit_l) {
        	safetyFlag.I[i] = BMS_SAFETY_WARNING_OC;
        } else {
            continue;
        }

        data_l[i][index_l[i]] = sens->crntSensor[i];
        data_h[i][index_h[i]] = sens->crntSensor[i];

        index_l[i] = (index_l[i]+1) % WINDOW_FAULT_I_low;
        index_h[i] = (index_h[i]+1) % WINDOW_FAULT_I_high;

        if (index_l[i] >= (WINDOW_FAULT_I_low-1)) {
            avgCellI = calculateAverage((int*)data_l[i], WINDOW_FAULT_I_low);

            if (avgCellI < UnderCurrentLimit_l) {
                safetyFlag.I[i] = BMS_SAFETY_FAULT_UC;
            } else if (avgCellI > OverCurrentLimit_l) {
                safetyFlag.I[i] = BMS_SAFETY_FAULT_OC;
            } else {
                safetyFlag.I[i] = 0;
            }

            memset(data_l[i], 0, sizeof(data_l[i]));
            index_l[i] = 0;
        }

        if (index_h[i] >= (WINDOW_FAULT_I_high-1)) {
            avgCellI = calculateAverage((int*)data_h[i], WINDOW_FAULT_I_high);

            if (avgCellI < UnderCurrentLimit_h) {
                safetyFlag.I[i] = BMS_SAFETY_FAULT_UC;
            } else if (avgCellI > OverCurrentLimit_h) {
                safetyFlag.I[i] = BMS_SAFETY_FAULT_OC;
            } else {
                safetyFlag.I[i] = 0;
            }

            memset(data_h[i], 0, sizeof(data_h[i]));
            index_h[i] = 0;
        }
    }

    for (i=0; i<NumOfHASS; i++) {
        if (safetyFlag.I[i] == BMS_SAFETY_FAULT_UC || safetyFlag.I[i] == BMS_SAFETY_FAULT_OC) {
        	switchMode(MODE_STOP);
            return false;
        }
    }

    return true;
}

bool bmsSafety_Check_Temps(stBatterySens *sens) {
	int i;
    static bool init =false;

	if (calculated_g.ntc==false) {
		return false;
	}

    for (i=0; i<NumOfNTC; i++) {
        data_t[i][index_t[i]] = sens->Temperature[i];
        index_t[i] = (index_t[i]+1) % WINDOW_FAULT_TEMPS;

        if (index_t[i] >= (WINDOW_FAULT_TEMPS-1)) {
            avgTemps = calculateAverage((int*)data_t[i], WINDOW_FAULT_TEMPS);
            if (!init) {
            	init=true;
            	avgTempsOld = avgTemps;
            }

            if (avgTemps < UnderTempsLimit) {
                safetyFlag.T[i] = BMS_SAFETY_FAULT_UT;
            } else {
                if(((i<=3 && avgTemps > BoardOverTempsLimit) ||
                    (i>3 && avgTemps > CellOverTempsLimit))){
                    safetyFlag.T[i] = BMS_SAFETY_FAULT_OT;
                } else {
                    safetyFlag.T[i] = 0;
                }
            }

            int avgDevTemps = (int)(avgTemps - avgTempsOld);
            if(abs(avgDevTemps) > DevTempsLimit) {
            	devTempsCounts++;
            	if(devTempsCounts > 2) {
                	safetyFlag.T[i] = BMS_SAFETY_FAULT_DEV_T;
                	devTempsCounts = 0;
            	}
            }
            avgTempsOld = avgTemps;

            memset(data_t[i], 0, sizeof(data_t[i]));
            index_t[i] = 0;
        }
    }

    for (i=0; i<NumOfNTC; i++) {
        if (safetyFlag.T[i] == BMS_SAFETY_FAULT_UT \
        	|| safetyFlag.T[i] == BMS_SAFETY_FAULT_OT \
			|| safetyFlag.T[i] == BMS_SAFETY_FAULT_DEV_T) {
        	switchMode(MODE_STOP);
            return false;
        }
    }

    return true;
}

void bmsCtrl_Fet(stBatterySens *sens) {
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
		if ((sens->minVoltage < 31000) && (sens->maxVoltage < 36000)) {
			m7_r1++;
			m7_r2=0;
			m7_r3=0;
		} else if ((sens->minVoltage >= 31000) && (sens->maxVoltage < 35000)) {
			m7_r2++;
			m7_r1=0;
			m7_r3=0;
		} else if ((sens->minVoltage >= 35000) && (sens->maxVoltage < 36000)) {
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

		if (((sens->minVoltage < 31000) && (sens->maxVoltage < 35000))\
			&& ((sens->avgCurrent >= 500) && (sens->avgCurrent < OverCurrentLimit_l))) {
			m0_r1++;
			m0_r2=0;
		} else if (((sens->minVoltage >= 31000) && (sens->maxVoltage < 35000))\
			&& ((sens->avgCurrent >= UnderCurrentLimit_l) && (sens->avgCurrent < OverCurrentLimit_l))) {
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
		if ((sens->minVoltage < 31000) && (sens->maxVoltage < 36000)) {
			m1_r1++;
			m1_r2=0;
		} else if ((sens->minVoltage >= 36000) && (sens->maxVoltage < 36500)) {
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
		if (sens->minVoltage < 31000) {
			m3_r1++;
			m3_r2=0;
			m3_r3=0;
			m3_r4=0;
			m3_r5=0;
		} else if ((sens->minVoltage >= 31000) && (sens->maxVoltage < 33000)) {
			m3_r2++;
			m3_r1=0;
			m3_r3=0;
			m3_r4=0;
			m3_r5=0;
		} else if ((sens->minVoltage >= 31000) && (sens->maxVoltage < 34000) && (getPackSOC() < 95)) {
			m3_r3++;
			m3_r1=0;
			m3_r2=0;
			m3_r4=0;
			m3_r5=0;
		} else if ((sens->minVoltage >= 31000) && (sens->maxVoltage < 34000) && ((diagState_g.localCAC - diagState_g.SAC) > 500)) {
			m3_r4++;
			m3_r1=0;
			m3_r2=0;
			m3_r3=0;
			m3_r5=0;
		} else if ((sens->minVoltage >= 31000) && (sens->maxVoltage < 36500) \
				&& (sens->avgCurrent >= -4000) && (sens->avgCurrent < 0)) {
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


void bmsCtrl_Fan(stBatterySens *sens) {
	int i;

	for (i=2; i<4; i++) { // DIODE
		if ((sens->Temperature[i] >= 600) && (sens->Temperature[i] < 1200)) {
			bmsCtrl_Gpio_g = true;
		} else {
			bmsCtrl_Gpio_g = false;
		}
	}
}

void bmsCtrl_Balancing(stBatterySens *sens) {
    uint16_t thres_volt;

    if (sens->minVoltage < MIN_VOLTAGE_THRESHOLD || sens->maxVoltage > MAX_VOLTAGE_THRESHOLD) {
        bmsCtrl_Bal_g = 0;
        return;
    }

    if (abs(sens->avgCurrent) < MAX_ABS_CURRENT_1) {
        thres_volt = THRESHOLD_VOLTAGE_1;
    } else if (abs(sens->avgCurrent) < MAX_ABS_CURRENT_2) {
        thres_volt = THRESHOLD_VOLTAGE_2;
    } else {
        bmsCtrl_Bal_g = 0;
        return;
    }

    for (int i = 0; i < 4; i++) {
        uint8_t cmd = (i > 1) ? i + 6 : i;
        if (sens->cellVoltage[i] == sens->minVoltage) {
            continue;
        }
        if ((sens->cellVoltage[i] - sens->minVoltage) > thres_volt) {
            bmsCtrl_Bal_g |= (1 << cmd);
        } else {
            bmsCtrl_Bal_g &= ~(1 << cmd);
        }
    }
}
