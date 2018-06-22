#ifndef __ADC_H_
#define __ADC_H_

#include "stm32f10x.h"
#include <stdio.h>

#define CMD_EXT_BUF_MAX     10     
#define CMD_EXT_LEN_MAX     16

#define CMD_NULL_ACK    0x00
#define CMD_SET_TEMP    0x10
#define CMD_GET_TEMP    0x11
#define CMD_GET_WIDTH   0x21
#define CMD_SET_FAN     0x30
#define CMD_SET_LED     0x40
#define CMD_SET_SERVO   0x50
#define CMD_GET_LEVEL   0x61


void USART2_Config(void);
void USART2_TxByte(u8 ch);
void USART2_TxString(u8 *string);


#endif