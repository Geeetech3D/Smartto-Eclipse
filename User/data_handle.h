#ifndef __DATA_HANDLE_H__
#define __DATA_HANDLE_H__
#include "stm32f10x_it.h"


#define  TEMPERATURE_INFO                                      1   //温度
#define  RATE_FAN_LAYER                                   2  //层数、进度、速度
#define  PRINTING_STATUS                                3 //GCODE
#define  PRINTER_SD_STATUS                            4//打印机 SD状态
#define  COORDINATE_XYZ                                 5
#define  PRINTER_MESSAGE                                6 //打印机信息
#define  PRINTER_START_MESSAGE                    8// //自动调平、耗材检测、WiFi上电自动开启、wifi状态

#define  WIFI_IP_MESSAGE                                9   //wifi  ip
#define  WIFI_STATUS_MESSAGE                       10   //wifi  status
#define  WIFI_SERVER_MESSAGE                       11   //wifi  SSID、  server ip
#define  WIFI_SET_SUCCEE                                12  //wifi  set  ok
#define  WIFI_EXIST_TEST                                13  //wifi  exist test
#define  WIFI_DETAIL_MESSAGE                        14
#define  WIFI_DEBUG_MESSAGE                        15



#define  PRINTER_RESET                                   16 //打印机复位
#define  SD_EXCEPTION                                      17 //SD卡异常
#define  RECOVER_STANDBY                                 18
#define  INPUT_STANDBY                                      19
#define  LCD_VERSION_INFO                               20 //LCD  屏版本信息
#define  TEMP_EXCEPTION                                  21  //温度异常
#define  FILAMENT_DETECTOR                            22 //耗材检测
#define  FILAMENT_DETECTOR_OFF_ON                            23 //耗材检测开关





#define CMD_FILAMAND_NO                  1   //耗材无


void Frash_Update_ToLCD(void);
void  Updata_To_LCD(u8 item) ;  //Add_Message(TEMP_EXCEPTION);
void ADD_Item_to_LCD(void);
void Main_Command_Handle(void);
void Add_MessageM(u8 item);

#endif
