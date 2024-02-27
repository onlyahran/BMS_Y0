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

typedef enum {
	NORMAL_V = 0,
	UV, // < 25500
	OV,     // >= 36500
//	VL1,  //  >= 25500, < 31000
//	VL2,  //  >= 31000, < 33400 // BAL O
//	VL3,  //  >= 33400, < 36000 // BAL O
//	VL4,  //  >= 36000, < 36500 // BAL O
} bms_safety_cellV;

typedef enum {
	NORMAL_I = 0,
	UC, // < -4000
	OC,     // >= 4000
//	CL1,  //  >= -4000, < -2000
//	CL2,  //  >= -2000, < -200
//	CL3,  //  >= -200, < -50 // BAL O
//	CL4,  //  >= -50, < 50 // BAL O
//	CL5,  //  >= -50, < 50 // BAL O
//	CL6,  //  >= 200, < 500
//    CL7,  //  >= 500, < 2000
//	CL8,  //  >= 2000, < 4000
} bms_safety_I;

typedef enum {
	NORMAL_T = 0,
	UT, // < -200
	cellOT,    // >= 700
	boardOT,     // >= 1200
//	cellTR1,   // >= -200, < 700
//	boardTR1,  //  >= -200, < 600
//	boardTR2,  //  >= 600, < 1200 // FAN O
} bms_safety_T;

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

typedef struct {
	uint8_t cellV[LTCBMS_CELLNUM];
	uint8_t I[MCP_NUM];
	uint8_t T[ADC_CHANNEL_MAX];
} bms_safety_flag;

typedef struct {
	uint16_t cellV[LTCBMS_CELLNUM];
	uint16_t mincellV;
	uint16_t maxcellV;
	uint16_t packV;
	int16_t packI;
	int16_t T[ADC_CHANNEL_MAX];
} bms_sensor_value;

typedef struct {
  uint8_t state;
  uint8_t old_state;
}stRelayInfo;

_ALGO_EXT bms_safety_flag safety_info_g;

_ALGO_EXT void relayInit(void);
_ALGO_EXT uint8_t getSwitchMode(void);
_ALGO_EXT bool bms_safetycheck_cellV(void);
_ALGO_EXT bool bms_safetycheck_I(void);
_ALGO_EXT bool bms_safetycheck_T(void);
_ALGO_EXT void bms_fet_control(void);
_ALGO_EXT void bms_fan_control(void);
_ALGO_EXT void bms_ltc_balancing(void);
#endif /* INC_ALGO_H_ */
