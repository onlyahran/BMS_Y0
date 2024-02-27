/*
 * ltcDriver.h
 *
 *  Created on: Aug 20, 2021
 *      Author: User
 */

#ifndef INC_LTCDRIVER_H_
#define INC_LTCDRIVER_H_

#include "main.h"
//Addressed protocol
#define LTC_CMD_WRCFGA   	0x0001  // WRITE CONFIGURATION Group A
#define LTC_CMD_WRCFGB		0x0024	// WRITE CONFIGURATION Group B
#define LTC_CMD_RDCFG   	0x0002  // READ CONFIGURATION
#define LTC_CMD_RDCVA    	0x0004  // READ CELL VOLTAGE
#define LTC_CMD_RDCVB    	0x0006  // READ CELL VOLTAGE
#define LTC_CMD_RDCVC    	0x0008  // READ CELL VOLTAGE
#define LTC_CMD_RDCVD    	0x000A  // READ CELL VOLTAGE
#define LTC_CMD_RDAUXA   	0x000C  // Read Auxiliary Group A, GPIO voltage
#define LTC_CMD_RDAUXB      0x000E  // Read Auxiliary Group B, GPIO voltage, REF voltage
#define LTC_CMD_RDSTATA  	0x0010  // Read Status Group A
#define LTC_CMD_RDSTATB  	0x0012  // Read Status Group B
#define LTC_CMD_WRSCTRL 	0x0014  // Write S Control Register
#define LTC_CMD_WRPWM		0x0020	// Write PWM Register Group
#define LTC_CMD_STSCTRL 	0x0019  // Start S Control Pulsing and Poll Status
#define LTC_CMD_CLRSCTRL	0x0018	// Clear S Control Register
#define LTC_CMD_ADCV        0x0260  // Start Cell Voltage ADC Conversion and Poll Status, DCP disabled, 422Hz mode[ADCOPT=0]/1kHz[ADCOPT=1]
//#define LTC_CMD_ADAX        0x0586  // Start GPIOs ADC Conversion and Poll Status, 26Hz mode/2kHz[ADCOPT=1], 2nd Reference Voltage
#define LTC_CMD_ADAX        0x0466  // Start GPIOs ADC Conversion and Poll Status, 422Hz mode[ADCOPT=0]/1kHz[ADCOPT=1], 2nd Reference Voltage
#define LTC_CMD_ADSTAT		0x0468   // Start Status Group ADC Conversion and Poll Status, 422Hz mode, all status
#define LTC_CMD_ADOW	    0x02E8  // Start Open Wire ADC Conversion and Poll Status
#define LTC_CMD_CVST	    0x02A7  // Start Cell Voltage ADC Conversion and Poll Status
#define LTC_CMD_PLADC		0x0714
#define LTC_CMD_WRCOMM		0x0721	// Write COMM
#define LTC_CMD_RDCOMM		0x0722	// Read COMM Register Group
#define LTC_CMD_STCOMM		0x0723	// Start I2C /SPI Communication

typedef   uint8_t (*pSpiWrite)(uint8_t* pOutBuff, uint8_t len);
typedef   uint8_t (*pSpiReadWrite)(uint8_t* pInBuff, uint8_t* pOutBuff, uint8_t len);
typedef   uint8_t (*pSpiRead)(uint8_t* pInBuff, uint8_t len);
typedef   void (*pSpiClearCS)(void);
typedef   void (*pSpiSetCS)(void);
typedef   void (*pdelay)(void);

typedef enum{
	LTC_OK              = ( 0U),
	LTC_SPI_ERROR       = ( 1U),
	LTC_SPI_BUSY        = ( 2U),
	LTC_SPI_TIMEOUT     = ( 3U),
	LTC_ERR_PEC         = ( 4U),
	LTC_CFG_TIMEOUT     = ( 5U),
	LTC_ERR_VREG        = ( 6U),
	LTC_ERR_VREGD       = ( 7U),
	LTC_ERR_CSUM        = ( 8U),
} ltc_err;

typedef struct stSlaveCfonfig
{
  //uint8_t NumberOfCell;                     //Number of Cell
  pSpiWrite spiWrite;                      //callback function to control SPI
  pSpiRead spiRead;
  pSpiReadWrite spiReadWrite;
  pSpiClearCS spiClearCS;
  pSpiSetCS spiSetCS;
  pdelay delay;
} _stSlaveCfonfig;

typedef struct
{
	uint16_t cellvolt[4];
	uint8_t config[8];
	uint16_t vreg;
} ltc_info;

extern ltc_info ltc_info_g;

extern void LTC_Init(void);
extern void LTCWakeUp(void);
extern uint32_t LTCPollAdc(void);
extern ltc_err LTCWriteCommand(uint16_t cmd);
extern ltc_err LTCSetCommand(uint16_t cmd, uint8_t len, uint8_t* pCfg);
extern ltc_err LTCReadCommand(uint16_t cmd, uint8_t* pOutBuff, uint8_t len);
extern void LTC_Cfg(void);
#endif /* INC_LTCDRIVER_H_ */
