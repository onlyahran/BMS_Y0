/*
 * BW_parser.c
 *
 *  Created on: Aug 22, 2021
 *      Author: User
 */

#include "BW_parser.h"
#include "BW_proc.h"
#include "can.h"
#include "cantp.h"
#include "algo.h"

stbat_t bat_g;
bool disp_debug_data;
stInteg_t Integ;
stComm_t stcomm_g;

uint8_t frame[CANTP_PACKET_LEN] = {0};
uint8_t cmd;

void device_Init(void)
{
	for(uint8_t i=0; i<BOARD_MAX; i++) memset(Integ.device_g[i].port, 0, PORT_MAX);
	for(uint8_t i=0; i<BOARD_MAX; i++) memset(Integ.device_g[i].env, 0, PORT_MAX);
	Integ.tilt_pitch = 0x80;
	Integ.tilt_roll = 0x80;
	memset(stcomm_g.is_comm, 0, stcomm_g.comm_len);
}

void analysis_parser(uint8_t* array, uint8_t data_size)
{
	int32_t temp32;
	uint32_t tempu32;

    switch(array[0])       // packet head
    {
        case 0xC0:
            sendCan(0x100, array+1, 2);         // array[1]: Data, array[2]: Port number
            break;

        case 0xF0:
        	// little endian
        	bat_g.ref_soc = (int8_t)array[1];
        	bat_g.ref_sac = (int16_t)((int16_t)(array[3] << 8) | (int16_t)array[2]);
        	bat_g.ref_cellcapacity = (int16_t)((int16_t)(array[5] << 8) | (int16_t)array[4]);
        	bat_g.cellcapacity = (bat_g.ref_cellcapacity == -1) ? (int16_t)INIT_CR : bat_g.ref_cellcapacity;
        	break;

        case 0xF1:
        	// little endian
        	temp32 = *((int32_t *)(array + 1));
        	temp32 *= SENS_AC_CONV_10mA_TO_A;
        	diagState_g.fSAC = (float)(temp32);
        	break;

        case 0xF2:
        	// little endian
        	tempu32 = *((uint32_t *)(array + 1));
        	tempu32 *= SENS_AC_CONV_10mA_TO_A;
        	diagState_g.fSAAC = (float)(tempu32);
        	break;

//        case 0xA1:
//        	if(array[1] == 0xA1)
//        	{
//        		if(array[2] == 0x0E)  // Enable
//        		{
//        			sensor_err_g.ve_enable = true;
//        		}
//        		else if(array[2] == 0x0F) // Disable
//        		{
//        			sensor_err_g.ve_enable = false;
//        		}
//        	}
//        	break;
//
		case 0xA2:
			if(array[1] == 0xAA)
			{
				stcomm_g.comm_len = sizeof(array)/ sizeof(char);
				memcpy(stcomm_g.is_comm, array+2, stcomm_g.comm_len);
			}
			break;

        default:
            break;
    }
}


void InternalDataParser(uint32_t id, uint8_t *pBuff, uint8_t len)
{

	switch(id)
	{
	case 0x000:
		if(pBuff[0] == 0x00)
		{
			disp_debug_data = true;
		}
	    else if(pBuff[0] == 0x01)
		{

		}
		else if(pBuff[0] == 0x02)
		{
			disp_debug_data = false;
		}
		break;

	case 0x001:
		// input 32bit data
		diagState_g.SAAC = (uint32_t)((float)((uint32_t)pBuff[7] << 24
									| (uint32_t)pBuff[6] << 16
									| (uint32_t)pBuff[5] << 8
									| (uint32_t)pBuff[4] << 0) * SENS_AC_CONV_10mA_TO_A);
		diagState_g.fSAAC = (float)(diagState_g.SAAC);

		diagState_g.SAC = (int32_t)((float)((uint32_t)pBuff[3] << 24
									| (uint32_t)pBuff[2] << 16
									| (uint32_t)pBuff[1] << 8
									| (uint32_t)pBuff[0] << 0)* SENS_AC_CONV_10mA_TO_A);
		diagState_g.fSAC = (float)(diagState_g.SAC);
		break;

	case 0x1A1: 			// Port number 0 ~ 4 : PWM, Port number 5 ~ 6 : ON/OFF
		device_cmp(Integ.device_g[PWM_RELAY_1].port, pBuff, PORT_MAX-1, 0U);
		break;

	case 0x1A2: 			// Port number 7 ~ 11 : PWM, Port number 12 ~ 13 : ON/OFF
        device_cmp(Integ.device_g[PWM_RELAY_2].port, pBuff, PORT_MAX-1, 7U);
		break;

	case 0x1A3: 			// Port number 14 ~ 18 : PWM, Port number 19 ~ 20 : ON/OFF
        device_cmp(Integ.device_g[PWM_RELAY_3].port, pBuff, PORT_MAX-1, 14U);
		break;

	case 0x1A4: 			// Port number 21 ~ 25 : PWM, Port number 26 ~ 27 : ON/OFF
      	device_cmp(Integ.device_g[PWM_RELAY_4].port, pBuff, PORT_MAX-1, 21U);
		break;

	case 0x1A5: 			// MaxxFan Status
        device_cmp(Integ.device_g[CONT_A].port, pBuff, PORT_MAX, 28U);
        break;

	case 0x1A6: 			// Aircon Status
        device_cmp(Integ.device_g[CONT_B].port, pBuff, PORT_MAX, 36U);
		break;


    case 0x1AB :
    	memcpy(Integ.device_g[PWM_RELAY_1].env, pBuff, PORT_MAX);
		break;

    case 0x1AC :
    	memcpy(Integ.device_g[PWM_RELAY_2].env, pBuff, PORT_MAX);
		break;

    case 0x1AD :
    	memcpy(Integ.device_g[PWM_RELAY_3].env, pBuff, PORT_MAX);
		break;

    case 0x1AE :
    	memcpy(Integ.device_g[PWM_RELAY_4].env, pBuff, PORT_MAX);
		break;

    case 0x1A7 :
		device_ver_check(1, pBuff, len);
    	break;

    case 0x1A8 :
		device_ver_check(2, pBuff, len);
    	break;

    case 0x1A9 :
		device_ver_check(3, pBuff, len);
    	break;

    case 0x1AA :
		Integ.tilt_pitch = pBuff[0];
		Integ.tilt_roll = pBuff[1];
    	break;
#if (CANTP_ENABLE == 1)
    case CANTP_ID_REQ :
    	uint8_t index=0;
    	uint8_t indicator = (pBuff[0] >> 4);
    	memset(frame, 0, CANTP_PACKET_LEN);

		if (indicator == CANTP_DEF_SF) {
			uint8_t sf_len = pBuff[0] & 0xF;
			index = 2;
			memcpy(frame+index, pBuff+1, sf_len);
			index += sf_len;
			frame[2] += 0x40; //identifier
			frame[0] = (CANTP_DEF_FF << 4);

			cmd = pBuff[sf_len];
			if (cmd & CANTP_TX_BMS_INFO) {
				frame[1] = 0x1A;
#ifdef CANTP_TEST_VALUE
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 0) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 8) & 0xFF;
#else
				frame[index++] = (uint8_t)(bat_sens_g.avgCurrent >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.avgCurrent >> 8) & 0xFF;
#endif
			} else if (cmd & CANTP_TX_CELL_100) {
				frame[1] = sizeof(bat_sens_g.cellVoltage)+2; //(4*2)+2=0xA
#ifdef CANTP_TEST_VALUE
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 0) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 8) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 16) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 24) & 0xFF;
#else
				frame[index++] = (uint8_t)(bat_sens_g.cellVoltage[0] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.cellVoltage[0] >> 8) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.cellVoltage[1] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.cellVoltage[1] >> 8) & 0xFF;
#endif
			} else if (cmd & CANTP_TX_TEMPS_100) {
				frame[1] = sizeof(bat_sens_g.Temperature)+2; //(6*2)+2=0xE
#ifdef CANTP_TEST_VALUE
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 0) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 8) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 16) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 24) & 0xFF;
#else
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[0] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[0] >> 8) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[1] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[1] >> 8) & 0xFF;
#endif
			}
			sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);
		} else if (indicator == CANTP_DEF_FC) {
			if (cmd & CANTP_TX_BMS_INFO) {
#ifdef CANTP_TEST_VALUE
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 0) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 8) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 0) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 8) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 16) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 24) & 0xFF;
				frame[index++] = 0xAA;
				for(int i=0; i<3; i++) {
					frame[0] = 0x21+i;
					sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);
				}
#else
				frame[index++] = 0x21;
				frame[index++] = (uint8_t)(bat_g.soc >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_g.soc >> 8) & 0xFF;
				frame[index++] = (uint8_t)(diagState_g.SAC >> 0) & 0xFF;
				frame[index++] = (uint8_t)(diagState_g.SAC >> 8) & 0xFF;
				frame[index++] = (uint8_t)(diagState_g.SAC >> 16) & 0xFF;
				frame[index++] = (uint8_t)(diagState_g.SAC >> 24) & 0xFF;
				sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);

				index = 0;
				memset(frame, 0, CANTP_PACKET_LEN);
				frame[index++] = 0x22;
				sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);

				index = 0;
				memset(frame, 0, CANTP_PACKET_LEN);
				frame[index++] = 0x23;
				frame[index++] = (uint8_t)(bat_sens_g.packVoltage >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.packVoltage >> 8) & 0xFF;
				frame[index++] = 0;
				frame[index++] = 0;
				frame[index++] = 0;
				frame[index++] = 0;
				frame[index++] = 0xAA;
				sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);
#endif
			} else if (cmd & CANTP_TX_CELL_100) {
#ifdef CANTP_TEST_VALUE
				frame[index++] = 0x21;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 0) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 8) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 0) & 0xFF;
				frame[index++] = (uint8_t)(CANTP_TEST_VALUE >> 8) & 0xFF;
				sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);
#else
				frame[0] = 0x21;
				frame[index++] = (uint8_t)(bat_sens_g.cellVoltage[2] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.cellVoltage[2] >> 8) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.cellVoltage[3] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.cellVoltage[3] >> 8) & 0xFF;
				sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);
#endif
			} else if (cmd & CANTP_TX_TEMPS_100) {
				frame[0] = 0x21;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[2] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[2] >> 8) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[3] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[3] >> 8) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[4] >> 0) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[4] >> 8) & 0xFF;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[5] >> 0) & 0xFF;
				sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);

				index = 0;
				memset(frame, 0, CANTP_PACKET_LEN);
				frame[0] = 0x22;
				frame[index++] = (uint8_t)(bat_sens_g.Temperature[5] >> 8) & 0xFF;
				sendCan(CANTP_ID_RES, frame, CANTP_PACKET_LEN);
			}
		}
		break;
#endif
	}
}

void batst_Init(void)
{
	bat_g.ref_soc = 0;
	bat_g.ref_sac = 0;
	bat_g.ref_cellcapacity = 0;
	bat_g.soc = -1;
	bat_g.soh = -1;
	bat_g.cellcapacity = -1;
}

void soc_calculation(void)
{
	int16_t delta_sac;
	int16_t delta_soc;

	if((bat_g.ref_cellcapacity == -1) || (bat_g.ref_soc == -1))
	{
		bat_g.soc = -1;           //not_correct_soc;
		return;
	}

	delta_sac = (int16_t)diagState_g.SAC - (int16_t)bat_g.ref_sac;

	if((delta_sac <= -bat_g.ref_cellcapacity) || (delta_sac >= bat_g.ref_cellcapacity))
	{
		bat_g.soc = -1;           //not_correct_soc;
		return;
	}
	else
	{
		delta_soc = (int8_t)((float)((float)delta_sac / (float)bat_g.cellcapacity)*100);
	}
	bat_g.soc = (int8_t)bat_g.ref_soc + (int8_t)delta_soc;

	uint8_t min = 1;
	uint8_t max = 99;

	if (getSwitchMode() == MODE_0) {
		min = 0;
	}

	if (getSwitchMode() == MODE_3) {
		max = 100;
	}

	if (bat_g.soc > max) bat_g.soc = max;
	if (bat_g.soc < min) bat_g.soc = min;
}

void setPackSOC(int8_t value)
{
	bat_g.soc = value;
}

int8_t getPackSOC(void)
{
	return bat_g.soc;
}

void setcellcapacity(uint16_t value)
{
	bat_g.cellcapacity = value;
}

uint16_t getcellcapacity(void)
{
	return bat_g.cellcapacity;
}
