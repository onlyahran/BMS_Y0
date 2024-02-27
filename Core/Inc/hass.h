/*
 * hass.h
 *
 *  Created on: Feb 19, 2024
 *      Author: better.ahran
 */

#ifndef INC_HASS_H_
#define INC_HASS_H_

#ifdef _HASS
#define _HASS_EXT
#else
#define _HASS_EXT extern
#endif

#include "main.h"

_HASS_EXT bool hass_Convert_current(int filter_max, int16_t* value);
_HASS_EXT void hass_Calculate_current(void);
_HASS_EXT void BackUpSRAMInit(void);

#endif /* INC_HASS_H_ */
