#ifndef _VARIABLE_H_
#define _VARIABLE_H_


#include "stm32f10x_it.h"
#include "command.h"


#define SERIAL_NUMBER_ADDR          0X0807F5D0          /*序列号存放地址*/  //16
#define HARDWARE_VERSION_ADDR  0X0807F5E0  /*序列号存放地址*/ //16
#define AUTO_LEVELEFLAG_ADDR       0X0807F5F0      /**/  // 2
#define AUTO_FILAMENT_DEV_ADDR  0X0807F5F4  /**/ // 2 

#define RECOVERY_SAVE_FLAG_ADDR             0X0807F600//断电续打参数保存位置
#define RECOVERY_SAVE_ADDR                      0X0807F602

//#define FACTORY_SETTINGS_STORE_FLAG_ADDR   0X0807F600  	/*出厂设置存储标志存放地址*/
//#define FACTORY_SETTINGS_PARAMETERS_ADDR  0X0807F602	/*出厂设置参数存放地址*/

#define USER_SETTINGS_STORE_FLAG_ADDR   0X0807F800	/*用户设置存储标志存放地址*/
#define USER_SETTINGS_PARAMETERS_ADDR  0X0807F802	/*用户设置参数存放地址*/
  


  #define WIFI_AUTO_CONNECT_FLAG_ADDR    0X0807FC00	/*开机WiFi自动连接标志*/
  #define WIFI_SERVER_PARAMETERS_ADDR    0X0807FC02	/*WiFi server参数存放地址*/
  #define WIFI_USER_STORE_FLAG_ADDR         0X0807FCA0	/*wifi用户设置参数存放地址*/
  #define WIFI_USER_PARAMETERS_ADDR        0X0807FCA2	/*wifi用户设置参数存放地址*/
  #define MOTOR_UNLOCK_TIME_FLAG_ADDR   0X0807FF00	 

  #define FILAMENT_DEFAULT_ADDR    0X0807FF02	/*save filament default vavue*/

  #define BED_LEVELING_HOMINGXYZ_ADDR  0X0807FF10	//自动调平时，电机归位坐标保存(x0,y0,z0)





typedef struct{
       u8      Motor_Lock;  // 1:电机不解锁  0:电机可以解锁
	float   print_percent;  //打印进度
	float   print_file_size;  //文件大小
	u8      Filament_Dev_Flag;//耗材检测标志
	u8     Auto_Levele_Flag; //自动调平标志
	u32   Motor_Disable_Times; //电机解锁时间
	u16   Beep_period ; //period in milliseconds
       int    Beep_duration ; //duration in milliseconds 
       u8    files_refresh_flag; //刷新SD卡文件
       u8    feed_tare;//打印速率
       u8   system_status;//系统状态
	u8   sd_print_flag;//SD 卡打印使能
	u8   sd_print_status;//SD卡打印状态
	u8   serial_ack_flag;//串口应答标志
	u8   serial_printf_flag;//串口打印标志
	u8   sd_file_cmp;//执行过M23选择好gcode文件
	u8   serial_axis_move_cmd;//=DISABLE;相对坐标标志
	u8   selected_file_pos;//上位机选择的文件名在数组sd_file_name和sd_file_namebuf中的位置
	int   serial_print_time;//串口发送的打印时间计算打印进度
	u8   serial_print_flag;//串口打印标志
       u32 pause_time;  //G4 暂停时间
       u8   pause_flag;  //G4暂停标志
       u8   stop_flag;    //终止打印
       u16 fan_hotend_speed;//鼓风机速度
       u16 fan_controler_speed;//主板风扇速度
       u8 Unexpected_events;//突发事件
       u8  sd_status;//sd卡状态
       u8 auto_leveling_calculate_enable;//自动调平计算使能
       u8 sd_file_num;//gcode文件个数
       
}SystemInfor;

typedef struct{
	char  printer_file_path[512]; 
}Systembuf_Info;

extern SystemInfor system_infor;
extern Systembuf_Info  Systembuf_Infos;
extern float Current_Position[4];
extern float Destination[4];
extern char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
extern bool axis_known_position[3];//是否已经归位标志


void system_data_init(void);
void SD_Printdata_init(void);


#endif

