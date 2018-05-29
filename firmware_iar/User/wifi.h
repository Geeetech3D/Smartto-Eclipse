#ifndef _WIFI_H_
#define _WIFI_H_
#include "command.h"







#ifdef WIFI_MODULE
#include "variable.h"
#include "gta.h"
#include "stdlib.h"
#include "usart1.h"
#include "stdio.h"
#include <string.h>
#include <stdbool.h>

#include <stdarg.h>
#include <stdio.h>
#include "stdlib.h"
#include "stmflash.h"
#include "Setting.h"
#define WIFI_CONNECT_SERVER 1  //Connect router
#define WIFI_SERVER_WAITING 2  //waited Connect router
#define WIFI_LOGIN_SERVER   3  //Login user
#define WIFI_LOGIN_WAITING  4  //waited Login user
#define WIFI_HANDLE_DATA    5  //data handle
#define WIFI_SET_AP         6  //wifi AP
#define WIFI_WAIT_AP        7  //wifi AP
#define WIFI_CONNECT_RESET  8  //
#define WIFI_APP_RESET  9  //




#define WIFIMAX_CMD_SIZE    128  //wifi max command len
#define WIFI_BUFSIZE        32   //wifi num of command
#define WIFI_GPIO           GPIOA     //wifi pin reset
#define WIFI_GPIO_PIN       GPIO_Pin_7


extern u8 UARST_REDIRECT;
extern u8 WIFI_MODE;
#ifndef BOARD_M301_Pro_S 
extern char uart2_buff[1280];
#endif
extern u16  usart2_num;
extern u8 serial2_complete_flag;
extern u16 serial2_complete_Times;
extern u16 wifi_buff_start ;
extern u16 wifi_buff_end ;
extern u8 Wifi_Using_Flag ;                 
extern u8 WIFI_ASK_COMMAND;
extern char Wifi_Server_message[128];//="connect to TP-LINK_3928:JIETAI007;printer_123456;162.144.205.148;\r\n";
extern char Wifi_Login_Message[128];//="printer_123456:00;login\r\n";

extern struct WIFI_SD_FILE Wifi_file_info;
extern char Printf_Buf[512]; 


typedef enum {
    WIFI_READY,
    WIFI_TX_DATA,
    WIFI_WAIT_ASK,
}WIFI_TX_STATA;

typedef struct {
  u16 WIFI_OUTTIME;
  u8  WIFI_TX_TIMES;
  u8  WIFI_COMMEND;
  u16 WIFI_FILE_SUM;
  u8  WIFI_CONNECT_FLAG; 
  u16 WIFI_RECONNECT_TIMES;  //
  u8 WIFI_CONNECT_TIMES;
  char *SP;
}WIFI_ASK_FACTOR;

typedef struct {
	char SSID[32];
        char Router_Password[64];
	char Router_IP[32];
	char WebServer_IP[32];
	u8 WIFI_WORK_STATUS;
	u8 Router_STATUS;
	u8 WEBSERVER_STATUS;
	u8 AUTO_CONNECT;
}WIFI_MESSAGE;



typedef struct {
       u16 App_RefreshSatus_Times;
	u8   Mask_SDPrintButton;
	u16 Mask_SDPrintButton_Times;
	u8   App_Sendfiles_FLAG;
	u8   App_sendfiles_dirmun;
	u16 App_Refresh_Times;
	u8  App_Refresh_FLAG;
	u8 wifi_Recovery_process_FLAG;
}APP_REFRESH;


extern FILE_DOWNLOAD File_DownST2;
extern WIFI_ASK_FACTOR Wifi_ASK_Factors ; 
extern WIFI_MESSAGE Wifi_Work_Message;
extern APP_REFRESH APP_Refresh_flag;

void Wifi_Init_Config(void);
void Wifi_Conf_Set(char *config_message);
void Wifi_Mode_Select(void);
void fetch_wifi_next_command(void);
void Wifi_Printf_Wait(void);
void Print_3D_SD(void);
void Wifi_Processing_command(void);
void my_printf(char *Data);
void Store_WIFI_Memory(void);
void Read_WIFI_Memory(void);
void WIFI_Extract_Info(void);
void WIFI_Connect(void);


u16 Get_RxData_Size(void);
 u16  crc16( char *d,  u16  len);
u8 Analysis_RxData(void);
u8 get_File_Data(void);
void SD_Delete_File(void);
void Init_Usart2_arg(void);
void clear_Wifi_SSID(void);
void get_M2101_status(void);
void Clear_WIFI_Info(void);
u8 Exist_Wifi_Det(void);
void Open_wifi(u8 p);
void Open_APP_Set(u8 s);
void WIFI_Write_AutoConnect_Flag(u8 flag);
void Refresh_WIFI_Data(void);
u8 WIFI_Read_AutoConnect_Flag(void);

#endif//WIFI_MODULE
void Motor_Unlock_Write_Timer(u16 tim);
u16 Motor_Unlock_Read_Timer(void);

#endif
