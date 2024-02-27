/*
 * BW_parser.h
 *
 *  Created on: Aug 22, 2021
 *      Author: User
 */

#ifndef BW_DATAPARSER_H_
#define BW_DATAPARSER_H_

#include "main.h"

extern bool disp_debug_data;

typedef struct{
	uint8_t  portnum_A1;
	uint8_t  portnum_A2;
	uint8_t  portnum_A3;
	uint8_t  portnum_A4;
	uint8_t  portnum_A5;
	uint8_t  portnum_A6;
	uint8_t  portnum_A7;
	uint8_t  portnum_A8;

	uint8_t portStatus_1[8];
	uint8_t portStatus_2[8];
	uint8_t portStatus_3[8];
	uint8_t portStatus_4[8];
	uint8_t info_remotestart[2];
	uint8_t info_door;
	uint8_t info_maxx[6];
	uint8_t info_air[4];
	uint8_t info_heat;
}InteContData_t;

typedef struct{
	int8_t ref_soc;
	int16_t ref_sac;
	int16_t ref_cellcapacity;
	int8_t soc;
	uint16_t soh;
	int16_t cellcapacity;
}stbat_t;

#define BOARD_MAX                   (6u)
#define PORT_MAX                    (8u)

typedef struct
{
    uint8_t port[PORT_MAX];
    uint8_t env[PORT_MAX];
}stdevice_t;

typedef struct
{
	stdevice_t device_g[BOARD_MAX];
	int8_t tilt_roll;
	int8_t tilt_pitch;
}stInteg_t;

typedef struct
{
	uint8_t is_comm[10];
	uint8_t comm_len;
}stComm_t;

typedef enum
{
    PWM_RELAY_1 = 0U,
    PWM_RELAY_2 ,
    PWM_RELAY_3 ,
    PWM_RELAY_4 ,
    CONT_A,
    CONT_B,
}enumdevice_type;

extern stComm_t stcomm_g;
extern void device_Init(void);

extern void batst_Init(void);
extern void soc_calculation(void);


extern void setPackSOC(int8_t value);
extern int8_t getPackSOC(void);
extern void setcellcapacity(uint16_t value);
extern uint16_t getcellcapacity(void);

void PutdiagStateValues(int16_t *intArray, uint16_t *UintArray);
extern void analysis_parser(uint8_t* array, uint8_t data_size);
extern void InternalDataParser(uint32_t id, uint8_t *pBuff, uint8_t len);
uint8_t givebackTrue(uint8_t value);
uint16_t getPortNumber_1to4(void);
uint16_t getPortNumber_5to8(void);
#endif /* BW_DATAPARSER_H_ */
