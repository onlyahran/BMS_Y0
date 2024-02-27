/*
 * BW_def.h
 *
 *  Created on: Aug 21, 2021
 *      Author: User
 */

#ifndef BW_DEF_H_
#define BW_DEF_H_

#include <stdint.h>

#define DWT_DELAY_NEWBIE 		0

#if 0
void DWT_Init(void);
void DWT_Delay(uint32_t us);
#endif
extern void delay_us(uint32_t nCount);
extern uint8_t checksum(uint8_t arr[], uint8_t length);

#endif /* BW_DEF_H_ */
