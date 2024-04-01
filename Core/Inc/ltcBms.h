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

_LTCBMS_EXT bool bmsCtrl_Gpio_g;
_LTCBMS_EXT uint8_t bmsCtrl_Bal_g;

_LTCBMS_EXT bool bmsLtcStatus_g;
_LTCBMS_EXT bool ltcbms_Fsm(stBatterySens *sens);
_LTCBMS_EXT bool test_ltcBms_Fan(void);
_LTCBMS_EXT bool test_ltcBms_Balancing(void);
#endif /* INC_LTCBMS_H_ */
