#include "variable.h"
#include "LCD2004.h"
#include "sd_print.h"




SystemInfor system_infor;
Systembuf_Info  Systembuf_Infos;
float Current_Position[4] = {0.0};
float Destination[4];
char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
bool axis_known_position[3] = {false, false, false};//是否已经归位标志

/*
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
       u8    percentage;//打印速率
       u8   system_status;//系统状态
	u8   sd_print_flag;//SD 卡打印使能
	u8   sd_print_status;//SD卡打印状态
	u8   serial_ack_flag;//串口应答标志
	u8   serial_printf_flag;//串口打印标志
	u8   sd_file_cmp;//执行过M23选择好gcode文件
	u8   serial_axis_move_cmd;//=DISABLE;相对坐标标志
       
}SystemInfor;
*/
/**********************************************************************
*****复位上电初始化数据
*****
*****
***********************************************************************/
void system_data_init(void)
{
	system_infor.serial_axis_move_cmd = DISABLE;
	system_infor.print_file_size=0;
	system_infor.system_status=IDLE;
	system_infor.sd_file_cmp=0;
	system_infor.fan_controler_speed =128;
	system_infor.Unexpected_events=DISABLE;//突发事件
	system_infor.auto_leveling_calculate_enable =ENABLE;
	system_infor.serial_printf_flag = DISABLE;
}

void SD_Printdata_init(void)
{
	system_infor.sd_print_status = SD_PRINTING;
	system_infor.print_percent=0.0;
      system_infor.system_status = PRINT; 
      system_infor.pause_flag = 0;
      system_infor.stop_flag = 0;

}



































