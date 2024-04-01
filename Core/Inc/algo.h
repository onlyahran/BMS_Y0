/*
 * BW_algo.h
 *
 *  Created on: Jan 11, 2022
 *      Author: User
 */

#ifndef INC_ALGO_H_
#define INC_ALGO_H_

#ifdef _ALGO
#define _ALGO_EXT
#else
#define _ALGO_EXT extern
#endif
#include "main.h"

typedef enum
{
	MODE_0 = 0x00,
	MODE_1,
	MODE_2,
	MODE_3,
	MODE_STOP,
	MODE_CHANGE,
	MODE_SYS,
	MODE_7,
} enumRelayMode;

#define NumOfCellCh      LTCBMS_CELLNUM
#define NumOfHASS        MCP_NUM
#define NumOfNTC         ADC_CHANNEL_MAX

#define BMS_SAFETY_WARNING_OV        (0x1001)
#define BMS_SAFETY_WARNING_UV        (0x1002)
#define BMS_SAFETY_WARNING_DEV       (0x1004)
#define BMS_SAFETY_WARNING_OC        (0x1010)
#define BMS_SAFETY_WARNING_UC        (0x1020)
#define BMS_SAFETY_WARNING_OT        (0x1100)
#define BMS_SAFETY_WARNING_UT        (0x1200)
#define BMS_SAFETY_WARNING_DEV_T     (0x1400)
#define BMS_SAFETY_FAULT_OV          (0x2001)
#define BMS_SAFETY_FAULT_UV          (0x2002)
#define BMS_SAFETY_FAULT_DEV         (0x2004)
#define BMS_SAFETY_FAULT_OC          (0x2010)
#define BMS_SAFETY_FAULT_UC          (0x2020)
#define BMS_SAFETY_FAULT_OT          (0x2100)
#define BMS_SAFETY_FAULT_UT          (0x2200)
#define BMS_SAFETY_FAULT_DEV_T       (0x2400)

typedef struct {
	int cellV[NumOfCellCh];
	int I[NumOfHASS];
	int T[NumOfNTC];
} stBmsSafetyFlag;

typedef struct {
  uint8_t state;
  uint8_t old_state;
} stRelayInfo;

_ALGO_EXT stBmsSafetyFlag safetyInfo_g;

_ALGO_EXT void relayInit(void);
_ALGO_EXT uint8_t getSwitchMode(void);
_ALGO_EXT bool bmsSafety_Check_CellV(stBatterySens *sens);
_ALGO_EXT bool bmsSafety_Check_I(stBatterySens *sens);
_ALGO_EXT bool bmsSafety_Check_Temps(stBatterySens *sens);
_ALGO_EXT void bmsCtrl_Fet(stBatterySens *sens);
_ALGO_EXT void bmsCtrl_Fan(stBatterySens *sens);
_ALGO_EXT void bmsCtrl_Balancing(stBatterySens *sens);
#endif /* INC_ALGO_H_ */
