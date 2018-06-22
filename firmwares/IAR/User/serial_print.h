#ifndef __SERIAL_PRINT_H__
#define __SERIAL_PRINT_H__

#include "stm32f10x.h"
#include "XZK_Rule.h"
#include "XZK_Configuration.h"
#include "stdbool.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "command.h"
#include "wifi.h"

#define UART_BUFF_SIZE 1280
#define UART3_BUFF_SIZE 200


void get_serial_command(void);
void fetch_next_command(void);
void Display_command(void);
void Input_File_Transmission(void);
void Serial_command_Analyze(void);
void get_serial_commandS(void);
void fetch_next_commandS(void);

#endif
