/*
 * BW_proc.h
 *
 *  Created on: Aug 22, 2021
 *      Author: User
 */

#ifndef BW_DATAPROC_H_
#define BW_DATAPROC_H_

#include "main.h"

extern void sendDataToServer(void);
extern void mqtt_send(const char* topic, unsigned char* data, int datalen);

extern void esp32_eep_write(uint8_t target_address, uint8_t* buf, uint8_t buflen);

extern void send_GUI(void);
extern void device_ver_reset(void);
extern void device_ver_check(uint16_t can_id, unsigned char* dat, uint8_t dat_len);
extern void device_all(void);
extern void device_env(void);
extern int device_cmp(const void *ptr1, const void *ptr2, unsigned int len, unsigned int port_base);

#endif /* BW_DATAPROC_H_ */
