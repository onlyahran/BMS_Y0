/*
 * ntc.c
 *
 *  Created on: Feb 19, 2024
 *      Author: better.ahran
 */
#define _NTC
#include "ntc.h"

#define NTC_LUP_NUM           20

const int ADC_NTCALUG01A103G_LUP[NTC_LUP_NUM]={ // 51K resistance
		2863,	2559,	2160,	1715,	1287,	926,	649,	450,	311,   218,
		153,	109,	79,	    58,	    43,	    32,	    25,	    19,	    15,      12
};
#if 0
const int ADC_NTCALUG01A103G_LUP[NTC_LUP_NUM]={ // 1K resistance
		3290,	3281,	3266,	3241,	3201,   3142,     3055,	  2935,	   2778,	2582,
		2352,   2099,   1834,	1573,	1328,	1108,	  916,	   753,	    617,	506
};
#endif
const int ADC_TEMPERATURE_CELCIUS[NTC_LUP_NUM]={
		-400,   -300,    -200,  -100,      0,     100,    200,     300,      400,    500,
		600,    700,    800,     900,   1000,   1100,     1200,    1300,    1400,    1500
};

static void Convert_adc_to_Celsius_LUP(int adcValue, int16_t* celsius);

static void Convert_adc_to_Celsius_LUP(int adcValue, int16_t* celsius) {
	int i;
	int adcVolt = (int)((float)adcValue * 0.8056640625);
	int currentDiff;
	int closest = 0xFFFF;
	float temperatureCelsius;

	if(adcVolt >= ADC_NTCALUG01A103G_LUP[0]){
		*celsius = -0x7FFF;
		return;
	}

	if(adcVolt <= ADC_NTCALUG01A103G_LUP[NTC_LUP_NUM-1]){
		*celsius = 0x7FFF;
		return;
	}

	for(i=0; i<NTC_LUP_NUM; i++){
		currentDiff = (int)(ADC_NTCALUG01A103G_LUP[i] - adcVolt);

		if(currentDiff <= (int)(closest - adcVolt)){
			closest = ADC_NTCALUG01A103G_LUP[i];
		}else{
			break;
		}
	}

	temperatureCelsius = (float)(closest - (int)adcVolt) / (float)(closest - ADC_NTCALUG01A103G_LUP[i]) * (float)(ADC_TEMPERATURE_CELCIUS[i] - ADC_TEMPERATURE_CELCIUS[i-1]);
	*celsius = (int)temperatureCelsius;
}

bool ntc_Convert_temperature(int filter_max, int16_t* temps) {
	int i;
	static stMedFilter ntcfilter[ADC_CHANNEL_MAX]={\
			[0 ... ADC_CHANNEL_MAX-1] = { .arr = {0}, .cnt = 0 }};
	int16_t med[ADC_CHANNEL_MAX];
	uint8_t ret=0;

	// put raw value in array to calculate median
	for(i=0; i<ADC_CHANNEL_MAX; i++) {
		if (adcFilter_Med(&ntcfilter[i], filter_max, (int)raw_adc[i], (int*)&med[i])) {
			Convert_adc_to_Celsius_LUP(med[i], &temps[i]);
			ret|=(1<<i);
		}
	}
	calculated_g.ntc = ret & 0x3F;
	return calculated_g.ntc;
}
