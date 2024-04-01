/*
 * BW_fsm.h
 *
 *  Created on: Aug 20, 2021
 *      Author: User
 */

#ifndef BW_FSM_H_
#define BW_FSM_H_

#include "main.h"

typedef struct
{
  uint32_t sysfsm_tick;
  uint32_t bms_50ms_tick;
  uint32_t bms_100ms_tick;
  uint32_t bms_200ms_tick;
  uint32_t bms_500ms_tick;
  uint32_t bms_1000ms_tick;
  uint32_t bms_10000ms_tick;
} stTickcount;

extern void fsmInit(void);
extern void fsmEvent(void);
extern void test_FsmEvent(void);
void uartParser(void);
#endif /* BW_FSM_H_ */
