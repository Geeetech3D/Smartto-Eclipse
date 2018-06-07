#ifndef _VARIABLE_H_
#define _VARIABLE_H_


#include "stm32f10x_it.h"
#include "command.h"


#define SERIAL_NUMBER_ADDR          0X0807F5D0          /*Serial number save address*/  //16
#define HARDWARE_VERSION_ADDR  0X0807F5E0          /*Hardware version number save address*/ //16
#define AUTO_LEVELEFLAG_ADDR       0X0807F5F0          /**/   
#define AUTO_FILAMENT_DEV_ADDR  0X0807F5F4          /**/  

#define RECOVERY_SAVE_FLAG_ADDR             0X0807F600//Power failure continued printing to save parameters address
#define RECOVERY_SAVE_ADDR                      0X0807F602


#define USER_SETTINGS_STORE_FLAG_ADDR   0X0807F800	/*User parameter save flag address*/
#define USER_SETTINGS_PARAMETERS_ADDR  0X0807F802	/*User parameter save  address*/
  


  #define WIFI_AUTO_CONNECT_FLAG_ADDR    0X0807FC00	/*Boot WiFi automatic connection flag*/
  #define WIFI_SERVER_PARAMETERS_ADDR    0X0807FC02	/*WiFi server parameter addr*/
  #define WIFI_USER_STORE_FLAG_ADDR         0X0807FCA0	/*wifi */
  #define WIFI_USER_PARAMETERS_ADDR        0X0807FCA2	/*wifi */
  #define MOTOR_UNLOCK_TIME_FLAG_ADDR   0X0807FF00	 

  #define FILAMENT_DEFAULT_ADDR    0X0807FF02	/*save filament default vavue*/

  #define BED_LEVELING_HOMINGXYZ_ADDR  0X0807FF10	// auto leveling homed flag(x0,y0,z0)





typedef struct{
       u8      Motor_Lock;  // motor lock flag
	float   print_percent;  //Print progress percentage
	float   print_file_size;  //file size
	u8      Filament_Dev_Flag;//filament inspection mark
	u8     Auto_Levele_Flag; //auto leveling flag
	u32   Motor_Disable_Times; //motor unlock time
	u16   Beep_period ; //period in milliseconds
       int    Beep_duration ; //duration in milliseconds 
       u8    files_refresh_flag; //refresh file list
       u16    feed_tare;//print rate
       u8   system_status;//system ststus
	u8   sd_print_flag;//SD print flag
	u8   sd_print_status;//SD printing status
	u8   serial_ack_flag;//uasrt ask flag
	u8   serial_printf_flag;//usart printing flag
	u8   sd_file_cmp;//After performing M23 flag
	u8   serial_axis_move_cmd;//=DISABLE;Relative coordinates
	u8   selected_file_pos;//file position
	int   serial_print_time;//Calculate print progress by received print time
	u8   serial_print_flag;//Serial port printing flag
       u32 pause_time;  //G4 pause time
       u8   pause_flag;  //G4pause flag
       u8   stop_flag;    //stop printing 
       u16 fan_hotend_speed;//Blower speed
       u16 fan_controler_speed;//Main board fan speed
       u8 Unexpected_events;//Emergencies
       u8  sd_status;//sd card status
       u8 auto_leveling_calculate_enable;//Automatic Leveling Calculation Enable
       u8 sd_file_num;//gcode file num
       
}SystemInfor;

typedef struct{
	char  printer_file_path[512]; 
}Systembuf_Info;

typedef struct {
        char File_Data[2080];
	u16  RxData_Len;
	char  Save_Location[5];
	char Save_Path[128];
	char Save_FileName[50];
	long long    File_Offset;
        long long    File_Offset_re;
	u16 Crc_Data;
	u8 Send_timers;
	u16 Rx_File_Size;
        u8 File_Flag;
}FILE_DOWNLOAD;

extern SystemInfor system_infor;
extern Systembuf_Info  Systembuf_Infos;
extern float Current_Position[4];
extern float Destination[4];
extern char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
extern bool axis_known_position[3];//xyz homed flag
extern char Printf_Buf[512]; 
extern char WF_version[6];
extern FILE_DOWNLOAD File_DownST;

#define UARST1_REDIRECT     1  //printf()  Redirection
#define UARST2_REDIRECT     2
#define UARST12_REDIRECT   3  //printf() Redirection
#define UARST3_REDIRECT     4

extern u8 UARST_REDIRECT;

void system_data_init(void);
void SD_Printdata_init(void);
void my_printf(char *Data);


#endif

