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

#define ADCFILTER_LIMIT_SIZE  (50)

typedef struct{
	int arr[ADCFILTER_LIMIT_SIZE];
	int cnt;
}stMedFilter;

ADCFILTER_EXT int findMedian(int* values, int size);
ADCFILTER_EXT bool adcFilter_Med(stMedFilter *filter, int filter_max, int adc_raw, int* med);
ADCFILTER_EXT int findMedian(int* values, int size);
#endif /* INC_ADCFILTER_H_ */
