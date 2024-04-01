/*
 * dataproc.c
 *
 *  Created on: Aug 22, 2021
 *      Author: User
 */

#include "BW_proc.h"
#include "usart.h"
#include "can.h"
#include "BW_parser.h"
#include "BW_def.h"
#include "BW_fsm.h"
#include "ltcBms.h"
#include "algo.h"

uint16_t d1_ver_cnt;
uint16_t d2_ver_cnt;
uint16_t d3_ver_cnt;

extern stInteg_t Integ;
extern stComm_t stcomm_g;

void esp32_eep_write(uint8_t target_address, uint8_t *buf, uint8_t buflen)
{
	unsigned char str[ESP_EEP_DATA_PACKET] = {0x02, };
	uint8_t i = 2;
	uint8_t chksum;

	str[i++] = '&';
	str[i++] = '&';
	str[i++] = DEF_WRITE;
	str[i++] = target_address;
	str[i++] = buflen;
	memcpy(str + i, buf, buflen);

	str[1] = i - 2 + buflen;
	i += buflen;
	chksum = checksum(str, i);
	str[i++] = chksum;

	UART_TX(USART2, str, i);
}

void send_GUI(void)
{
	uint16_t tempU16;
	uint8_t tx_buff[20];
	static uint16_t seq_num = 0;

	tempU16 = batterySens_g.packVoltage;
	memcpy(tx_buff, (uint8_t*)&tempU16, 2);
	if(getPackSOC() == -1) tempU16 = -1;
	else tempU16 = getPackSOC()*10; // VER3_5
	memcpy(tx_buff+2, (uint8_t*)&tempU16,2);
	memcpy(tx_buff+4, &stcomm_g.is_comm, stcomm_g.comm_len);
	sendCan(0x020, tx_buff, 6);

	tempU16 = batterySens_g.crntSensor[HASS_MAIN_50S];
	memcpy(tx_buff, (uint8_t*)&tempU16, 2);
	tempU16 = batterySens_g.crntSensor[HASS_ALT_300S];
	memcpy(tx_buff+2, (uint8_t*)&tempU16, 2);
	tempU16= batterySens_g.crntSensor[HASS_MAIN_300S];
	memcpy(tx_buff+4, (uint8_t*)&tempU16, 2);
	tempU16= batterySens_g.crntSensor[HASS_PV_100S];
	memcpy(tx_buff+6, (uint8_t*)&tempU16, 2);
	sendCan(0x024, tx_buff, 8);

	if(disp_debug_data)
	{
		tempU16 = batterySens_g.cellVoltage[0];
		memcpy(tx_buff, (uint8_t*)&tempU16, 2);
		tempU16 = batterySens_g.cellVoltage[1];
		memcpy(tx_buff+2, (uint8_t*)&tempU16, 2);
		tempU16 = batterySens_g.cellVoltage[2];
		memcpy(tx_buff+4, (uint8_t*)&tempU16, 2);
		tempU16 = batterySens_g.cellVoltage[3];
		memcpy(tx_buff+6, (uint8_t*)&tempU16, 2);
		sendCan(0x021, tx_buff, 8);

		tempU16 = batterySens_g.Temperature[TEMP_MAIN_A];
		memcpy(tx_buff, (uint8_t*)&tempU16, 2);
		tempU16 = batterySens_g.Temperature[TEMP_MAIN_B];
		memcpy(tx_buff+2, (uint8_t*)&tempU16, 2);
		tempU16 = batterySens_g.Temperature[TEMP_DIODE_1];
		memcpy(tx_buff+4, (uint8_t*)&tempU16, 2);
		tempU16 = batterySens_g.Temperature[TEMP_DIODE_2];
		memcpy(tx_buff+6, (uint8_t*)&tempU16, 2);
		sendCan(0x022, tx_buff, 8);

		tempU16 = batterySens_g.Temperature[TEMP_BAT_A];
		memcpy(tx_buff, (uint8_t*)&tempU16, 2);
		tempU16 = batterySens_g.Temperature[TEMP_BAT_B];
		memcpy(tx_buff+2, (uint8_t*)&tempU16, 2);
		sendCan(0x025, tx_buff, 8);

		tx_buff[0] = 0;
		tx_buff[1] = getSwitchMode();
		tempU16 = seq_num++;
		memcpy(tx_buff+2, (uint8_t*)&tempU16, 2);
		sendCan(0x023, tx_buff, 8);
	}
}

void mqtt_send(const char* topic, unsigned char* data, int datalen)
{
	int topiclen = strlen(topic);
	static uint8_t tmp[100] = {0x02, };  // set start-identifier

	int idx = 1;
	uint8_t chks = 0;
//	uint8_t tmp2[50];
//	int random = rand()%10 *5;

//	memset(tmp2, 0x03, sizeof(tmp2));

	if(topiclen > 0xff) return;
	if(datalen > 0xff) return;

	tmp[idx++] = topiclen;
	memcpy(tmp + idx, topic, topiclen);
	idx += topiclen;

	tmp[idx++] = datalen;
	memcpy(tmp + idx, data, datalen);
	idx += datalen;

	chks = checksum(tmp, idx);

	tmp[idx ++] = chks;
	tmp[idx ++] = 0x03;                              // set close-identifier

	UART_TX(USART2, tmp, idx);
//	UART_TX(USART2, tmp2, random);
}

void sendDataToServer(void)
{
	uint8_t strTemp[100];
	uint16_t tempU16;
	int16_t tempINT;
	static uint16_t packetIndex = 0;
	uint16_t i=0;

	tempU16 = packetIndex++;
	if(packetIndex > 0xfffe) packetIndex = 0;
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempU16 = batterySens_g.packVoltage;
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempINT = batterySens_g.crntSensor[HASS_MAIN_50S];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempINT = batterySens_g.crntSensor[HASS_ALT_300S];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempINT = batterySens_g.crntSensor[HASS_MAIN_300S];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempINT = batterySens_g.crntSensor[HASS_PV_100S];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempU16 = getPackSOC()*10; // VER3_5
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempINT = batterySens_g.Temperature[TEMP_MAIN_A];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempINT = batterySens_g.Temperature[TEMP_DIODE_1];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempINT = batterySens_g.Temperature[TEMP_DIODE_2];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempINT = batterySens_g.Temperature[TEMP_MAIN_B];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempINT = batterySens_g.Temperature[TEMP_BAT_A];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempINT = batterySens_g.Temperature[TEMP_BAT_B];
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempU16 = getSwitchMode();
	strTemp[i++] = (uint8_t)(tempU16>>0);
	tempU16 = batterySens_g.cellVoltage[0];
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempU16 = batterySens_g.cellVoltage[1];
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempU16 = batterySens_g.cellVoltage[2];
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempU16 = batterySens_g.cellVoltage[3];
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempU16 = getcellcapacity()*10;              // 10Ah
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempINT = (int16_t)diagState_g.SAC;
	strTemp[i++] = (uint8_t)(tempINT>>0);
	strTemp[i++] = (uint8_t)(tempINT>>8);
	tempU16 = (uint16_t)diagState_g.SAAC;         // 1Ah
	strTemp[i++] = (uint8_t)(tempU16>>0);
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempINT = Integ.tilt_pitch;
	strTemp[i++] = (uint8_t)(tempINT>>0);
	tempINT = Integ.tilt_roll;
	strTemp[i++] = (uint8_t)(tempINT>>0);

	uint8_t cmd=0;
	cmd |= (bmsCtrl_Bal_g & 0x03);   // 0x01과 0x02 비트를 합친다
	cmd |= (bmsCtrl_Bal_g & 0x20) >> 3; // 0x20 비트를 오른쪽으로 5비트 시프트하여 0x03 비트에 추가한다
	cmd |= (bmsCtrl_Bal_g & 0x80) >> 4; // 0x80 비트를 오른쪽으로 6비트 시프트하여 0x03 비트에 추가한다
	strTemp[i++] = (uint8_t)cmd;
	tempU16 = 0;
	strTemp[i++] = (uint8_t)(tempU16>>8);
	tempU16 = MAJOR_FW_VER;
	strTemp[i++] = (uint8_t)(tempU16>>0);

	mqtt_send("bs/0001/0003//dk", strTemp, i);
}

void device_ver_reset(void)
{
	char ver[20] = "0.0.0.0.0.0.";

	d1_ver_cnt++;
	d2_ver_cnt++;
	d3_ver_cnt++;

	if(d1_ver_cnt >= 66)
	{
		d1_ver_cnt = 0;
		mqtt_send("devices_ver_//b1", (unsigned char*)ver, 4);
		mqtt_send("devices_ver_//hm", (unsigned char*)ver, 8);
	}

	if(d2_ver_cnt >= 66)
	{
		d2_ver_cnt = 0;
		mqtt_send("devices_ver_//b2", (unsigned char*)ver, 4);
	}

	if(d3_ver_cnt >= 66)
	{
		d3_ver_cnt = 0;
		mqtt_send("devices_ver_//b3", (unsigned char*)ver, 4);
	}
}

void device_ver_check(uint16_t board_num, unsigned char* dat, uint8_t dat_len)
{
	static char store[20];
	uint8_t charlen;

	switch(board_num)
	{
	case 1:
		d1_ver_cnt = 0;

		memset(store, 0x00, sizeof(store));

		sprintf(store, "%d.%d.", dat[0], dat[1]);
		charlen = strlen(store);
		mqtt_send("devices_ver_//b1", (unsigned char*)store, charlen);

		sprintf(store+charlen, "%d.%d.%d.%d.", dat[2], dat[3], dat[4], dat[5]);
		mqtt_send("devices_ver_//hm", (unsigned char*)store + charlen, 8);
		break;

	case 2:
		d2_ver_cnt = 0;

		memset(store, 0x00, sizeof(store));

		sprintf(store, "%d.%d.", dat[0], dat[1]);
		charlen = strlen(store);
		mqtt_send("devices_ver_//b2", (unsigned char*)store, charlen);
		break;

	case 3:
		d3_ver_cnt = 0;

		memset(store, 0x00, sizeof(store));

		sprintf(store, "%d.%d.", dat[0], dat[1]);
		charlen = strlen(store);
		mqtt_send("devices_ver_//b3", (unsigned char*)store, charlen);
		break;

	default:
		break;
	}
}

int device_cmp(const void *ptr1, const void *ptr2, unsigned int len, unsigned int port_base)
{
    unsigned int i = 0;
    unsigned char *old;
    unsigned char *new;

    uint8_t di[2] = {0, 0};


    if(len > 8U)               return -1;        // CAN max length = 8
    if(port_base + len > 44U)  return -1;        // port max = port_base(36) + len(6) = 42;

    old = (unsigned char*) ptr1;
    new = (unsigned char*) ptr2;

    for(i=0; i<len; i++)
    {
        if(old[i] != new[i])
        {
            di[0] = port_base + i;               // port num
            di[1] = new[i];                      // data
            mqtt_send("bs/0001/0004//di", di, 2);
        }
        ((unsigned char*)ptr1)[i] = ((unsigned char*)ptr2)[i];
    }
    return 0;
}

void device_all(void)
{
	uint8_t dw[50] = {0, };
    uint16_t idx = 1;
    uint16_t i = 0;
    bool stop = false;

    memcpy(dw+idx, Integ.device_g[PWM_RELAY_1].port, PORT_MAX-1);
    idx += PORT_MAX-1;

    memcpy(dw+idx, Integ.device_g[PWM_RELAY_2].port, PORT_MAX-1);
    idx += PORT_MAX-1;

    memcpy(dw+idx, Integ.device_g[PWM_RELAY_3].port, PORT_MAX-1);
    idx += PORT_MAX-1;

    memcpy(dw+idx, Integ.device_g[PWM_RELAY_4].port, PORT_MAX-1);
    idx += PORT_MAX-1;

    memcpy(dw+idx, Integ.device_g[CONT_A].port, PORT_MAX);
    idx += PORT_MAX;

    memcpy(dw+idx, Integ.device_g[CONT_B].port, PORT_MAX);
    idx += PORT_MAX;

    for(i=idx; i>0; i--)
    {
    	if(dw[i] != 0)
    	{
    		stop = true;
    		break;
    	}
    }

    if(stop)
    {
    	dw[0] = i;                                      // V4 protocol
    	mqtt_send("bs/0001/0004//dw", dw, i+1);
    }

}

void device_env(void)
{
	uint8_t dr[50] = {0, };
    uint16_t idx = 1;
    uint16_t i = 0;
    bool stop = false;

	memcpy(dr+idx, Integ.device_g[PWM_RELAY_1].env, PORT_MAX);
	idx += PORT_MAX;

	memcpy(dr+idx, Integ.device_g[PWM_RELAY_2].env, PORT_MAX);
	idx += PORT_MAX;

	memcpy(dr+idx, Integ.device_g[PWM_RELAY_3].env, PORT_MAX);
	idx += PORT_MAX;

	memcpy(dr+idx, Integ.device_g[PWM_RELAY_4].env, PORT_MAX);
	idx += PORT_MAX;

    for(i=idx; i>0; i--)
    {
    	if(dr[i] != 0)
    	{
    		stop = true;
    		break;
    	}
    }

    if(stop)
	{
    	dr[0] = i;                                    // V4 protocol
	    mqtt_send("bs/0001/0004//dr", dr, i+1);
	}
}
