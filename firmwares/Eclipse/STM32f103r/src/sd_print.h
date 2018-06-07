#ifndef __SD_PRINT_H__
#define __SD_PRINT_H__

#include "stm32f10x.h"
#include "XZK_Rule.h"
#include "XZK_Configuration.h"
#include "stdbool.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "command.h"
#include "delay.h"
#include "usart1.h"
#include "fat.h"
#include "sd.h"
#include "mmc_sd.h"
#include "setting.h"
#include "adc.h"
#include "step_motor.h"
#include "LCD2004.h"
#include "rocker.h"
#include "stdlib.h"
#include "stdio.h"
#include "endstop.h"   //fdwong
#include "planner.h"


#define SD_PRINT_IDLE   0x00
#define SD_PRINT_START  0x01
#define SD_PRINTING     0x02
#define SD_PRINT_FINISH 0x03
#define SD_PRINT_PAUSE  0x04
#define SD_PRINT_RECOVERY  0x05
#define SD_PRINT_PAUSE_F  0x06
#define SERIAL_PRINTING           0x07

#define FILE_NUM 32
#define FILE_NAME_SIZE 50



extern u16  Sum_layers;
extern float layer_high;

void Get_command(void);
u8 Get_SD_Data_Buffer(void);
//void SD_Print_Processing(void);
void Send_Info_To_Serial(void);
void Send_SD_Dir(char *path);
void Clear_SD_File(void);
void SD_File_Detect(void);
void Send_SD_DirtoLCD(void);
u16 get_print_layer(void);
void GCODE_M27(void);
#endif
