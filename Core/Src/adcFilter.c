
/*
 * filter.c
 *
 *  Created on: Feb 19, 2024
 *      Author: better.ahran
 */
#define ADCFILTER
#include <adcFilter.h>

static int compare(const void *a, const void *b);

static int compare(const void *a, const void *b){
    return (*(int*)a - *(int*)b);
}

int findMedian(int* values, int size) {
	int *copied = (int*)malloc(size * sizeof(int));

	memcpy(copied, values, sizeof(int)*size);
    qsort(copied, size, sizeof(int), compare);

    free(copied);
    return copied[size / 2];
}

bool adcFilter_Med(stSensorFilter *filter, int filter_max, int adc_raw, int* med) {
	filter->arr[filter->cnt++] = adc_raw;
    if (filter->cnt == filter_max) {
    	filter->cnt = 0;
    	*med = findMedian(filter->arr, filter_max);
    	return true;
        //
        //SEGGER_RTT_printf(0, "med=%d\r\n", printmed);
    }

	return false;
}
