#ifndef _SETTING_H_
#define _SETTING_H_

#include "stm32f10x.h"
#include "XZK_Configuration.h"
#include "XZK_Rule.h"
#include "math.h"
#include "stdbool.h"
#include "vector_3.h"



typedef enum
{
	FACTORY_SETTINGS = 0,
	USER_SETTINGS	
}SettingMode;

typedef struct 
{
    float min_position[3];	       //机箱最小位置 12
    float max_position[3]; 	//机箱最大位置12
   u8   motor_direction[6];       //电机转动方向6
   float steps_per_mm[4];	//电机每走一毫米的步数16
   u16 max_feedrate[4];	//电机最大运行速度8
   float zz_offset;//Z轴偏差4
   float zprobe_zoffset;//自动调平Z轴偏差4
  u8    auto_leveling_flag;// 1

   char SN[16];
   char HV[6];
  u16 extrude_multiply;                                //出料倍数
  u32 min_segment_steps;                           //移动的最小步数
  u32 min_segment_time;                           //打印的最小频率
  u8   endstop_level[6];	     //限位点工作电平
  u8   endstop_status[6];    //限位点工作使能状态
  float retract_acceleration;                      //回抽加速度
  float acceleration;					//加速度
  u32 axis_steps_per_sqr_second[4];      //XYZE轴移动步数
  u8 min_travel_feedrate;                            //空打最小打印速度
  u8 min_feedrate;	                          //最小打印速度
  u8   home_position[3];	      //电机归位时的位置
  u16 home_speed[3];	      //电机归位的速度
  float max_jerk;             //设定默认的连接速度为最大突变速度
  float max_x_jerk;         //设定默认的连接速度为最大突变速度x
  float max_y_jerk;         //设定默认的连接速度为最大突变速度y
  float max_z_jerk;         //设定默认的连接速度为最大突变速度z
  float max_e_jerk;         //设定默认的连接速度为最大突变速度  e
   u16 max_acceleration[4];    //XYZE轴加速度
  u8 pid_adjust_range;   //进入PID调节的范围
  matrix_3x3 plan_bed_level_matrix;
  u8 locate_mode;	//坐标模式
  float Kp[HOTHEAD_NUMBER];					//PID调节参数P  20
  float Ki[HOTHEAD_NUMBER];					//PID调节参数i  20
  float Kd[HOTHEAD_NUMBER];					//PID调节参数d  20
  float targe_temperature[HOTHEAD_NUMBER];	//加热端目标温度20
  float min_temperature[HOTHEAD_NUMBER];	      //加热端最小温度20
  float max_temperature[HOTHEAD_NUMBER];	      //加热端最大温度20
  u8 hotend_num;//挤出头数目   1
  FunctionalState hotend_enable[HOTHEAD_NUMBER];	//加热端使能标志  5
  u8  temperature_sampling_period; //温度采集周期  1
  u8 unit;             //单位
  u8 wifi_exist_flag;
}setting_t;//

typedef struct 
{
	u8 a;
}setting_s;

extern setting_t Setting;

void Get_Printer_User_Settings(void);
void Store_Memory(SettingMode mode);
void Restore_Defaults(SettingMode mode);
void Store_SerialNumber(void);
void Get_SerialNumber(void);
void Store_Hardware_Version(void);
void Get_Hardware_Version(void);
void Store_AutoLevele_Flag(u8 flag);
u8 Get_AutoLevele_Flag(void);
void Store_Filament_Dev_Flag(u8 flag);
u8 Get_Filament_Dev_Flag(void);
#endif
