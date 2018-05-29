#ifndef _COMMAND_H_
#define _COMMAND_H_

#include "stm32f10x.h"
#include "XZK_Rule.h"
#include "XZK_Configuration.h"
#include "stdbool.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

//The ASCII buffer for receiving from the serial:
#define MAX_CMD_SIZE            96//128//128//96


//#define BUFSIZE  64//223//10   //fdwong
#define BUFSIZE                 64//96//128//
#define BUFSIZE3               20//128//

//#define SERIAL_BUF_SIZE         512            //fdwong

#define CMDBUF_SIZE             128
#define SD_DATA_BUF_SIZE        512

extern bool Print_Start_Flag;
extern char SD_Data_Buffer[SD_DATA_BUF_SIZE];


float Command_value_float(char *ch_point);
s32 Command_value_long(char *ch_point);
u8 Command_is(char code,char** ch_point);
void Processing_command(void);
void Get_linear_move_Base_Parameter(void);
void Prepare_linear_move(void);
static void Set_axis_steps_per_unit(void);
static void Set_hotend_pid(void);
static void Set_bed_pid(void);
//extern void Plan_buffer_for_linear_move(void);
void Print_command(void);
void Send_machine_info(void);

void wait_heating(void);
void Send_printer_data(void);
u8 Add_Upload_data(void);
u8 Get_Motor_Status(void);
void  Return_M105_Command(void);
void Send_m105_ask(void);
char *Get_Gcode_name(char *path);
void Set_E0_Motor_Flag(u8 n);
void Set_Up_Data_Flag(void);
void Send_Printer_State(void);
void WIF_EXIST_TEST(void);
#endif
