/*
 * ltcBms.h
 *
 *  Created on: Feb 19, 2024
 *      Author: better.ahran
 */

#ifndef INC_LTCBMS_H_
#define INC_LTCBMS_H_

#ifdef _LTCBMS
#define _LTCBMS_EXT
#else
#define _LTCBMS_EXT extern
#endif

#include "main.h"

enum {
    LTCBMS_STAT_ADC = 0,
    LTCBMS_STAT_FAULT,
};

enum {
	LTCBMS_DEF = 0,
	LTCBMS_ON,
	LTCBMS_OFF,
};

typedef struct {
	uint8_t st;
	uint8_t gpio;
	bool gpio_st;
	uint8_t bal;
	bool bal_st;
}ltcbms_status;

_LTCBMS_EXT ltcbms_status ltc_stat_g;
_LTCBMS_EXT void ltcbms_Fsm(void);

#endif /* INC_LTCBMS_H_ */
