#ifndef __DATA_HANDLE_H__
#define __DATA_HANDLE_H__
#include "stm32f10x_it.h"


#define  TEMPERATURE_INFO                                      1   //temperture
#define  RATE_FAN_LAYER                                   2  //Layers, progress, speed
#define  PRINTING_STATUS                                3 //GCODE
#define  PRINTER_SD_STATUS                            4//printer and SD status
#define  COORDINATE_XYZ                                 5
#define  PRINTER_MESSAGE                                6 //printer informention
#define  PRINTER_START_MESSAGE                    8// auto leveling ,filament Testing ,turn on wifi status

#define  WIFI_IP_MESSAGE                                9   //wifi  ip
#define  WIFI_STATUS_MESSAGE                       10   //wifi  status
#define  WIFI_SERVER_MESSAGE                       11   //wifi  SSID¡¢  server ip
#define  WIFI_SET_SUCCEE                                12  //wifi  set  ok
#define  WIFI_EXIST_TEST                                13  //wifi  exist test
#define  WIFI_DETAIL_MESSAGE                        14
#define  WIFI_DEBUG_MESSAGE                        15



#define  PRINTER_RESET                                   16 //printer reset
#define  SD_EXCEPTION                                      17 //SD exception
#define  RECOVER_STANDBY                                 18
#define  INPUT_STANDBY                                      19
#define  LCD_VERSION_INFO                               20 //LCD   version
#define  TEMP_EXCEPTION                                  21  // temperture exception
#define  FILAMENT_DETECTOR                            22 //
#define  FILAMENT_DETECTOR_OFF_ON                            23 //





#define CMD_FILAMAND_NO                             1   //
#define CMD_WIFI_DISCONNECTED                  2   //disconnected net


void Add_Message(u8 item);



void Frash_Update_ToLCD(void);
void  Updata_To_LCD(u8 item) ;  //Add_Message(TEMP_EXCEPTION);
void ADD_Item_to_LCD(void);
void Main_Command_Handle(void);
void Add_MessageM(u8 item);


#endif
