/*
 * cantp.h
 *
 *  Created on: Feb 27, 2024
 *      Author: better.ahran
 */

#ifndef INC_CANTP_H_
#define INC_CANTP_H_

#ifdef _CANTP
#define _CANTP_EXT
#else
#define _CANTP_EXT extern
#endif

#include "main.h"

#define CANTP_PACKET_LEN         (8)

#define CANTP_ID_REQ             (0x03E0)
#define CANTP_ID_RES             (0x03E8)

#define CANTP_DEF_SF             (0)
#define CANTP_DEF_FF             (1)
#define CANTP_DEF_CF             (2)
#define CANTP_DEF_FC             (3)

// tx command list
#define CANTP_TX_BMS_INFO           (0x01)
#define CANTP_TX_CELL_100           (0x02)
#define CANTP_TX_TEMPS_100          (0x06)

#endif /* INC_CANTP_H_ */
