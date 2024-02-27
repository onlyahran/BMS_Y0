/*
 * ntc.h
 *
 *  Created on: Feb 19, 2024
 *      Author: better.ahran
 */

#ifndef INC_NTC_H_
#define INC_NTC_H_

#ifdef _NTC
#define _NTC_EXT
#else
#define _NTC_EXT extern
#endif

#include "main.h"
#include "adcFilter.h"
#include "adc.h"

_NTC_EXT bool ntc_Convert_temperature(int filter_max, int16_t* temps);

#endif /* INC_NTC_H_ */
