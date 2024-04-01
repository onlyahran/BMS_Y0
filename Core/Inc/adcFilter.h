/*
 * filter.h
 *
 *  Created on: Feb 19, 2024
 *      Author: better.ahran
 */

#ifndef INC_ADCFILTER_H_
#define INC_ADCFILTER_H_

#ifdef ADCFILTER
#define ADCFILTER_EXT
#else
#define ADCFILTER_EXT extern
#endif

#include "main.h"

#define WINDOWS_MAX_SIZE  (20)

typedef struct{
	int arr[WINDOWS_MAX_SIZE];
	int cnt;
}stSensorFilter;

ADCFILTER_EXT bool adcFilter_Med(stSensorFilter *filter, int filter_max, int adc_raw, int* med);
ADCFILTER_EXT int findMedian(int* values, int size);
#endif /* INC_ADCFILTER_H_ */
