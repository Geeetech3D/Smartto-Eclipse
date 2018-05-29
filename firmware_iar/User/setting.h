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
    float min_position[3];	   //Chassis minimum position
    float max_position[3]; 	  //Chassis max position
   u8   motor_direction[6];       //Motor rotation direction
   float steps_per_mm[4];	  //Motor steps per millimeter
   u16 max_feedrate[4];	  //Motor max speed
   float zz_offset;                 //Z axis deviation
   float zprobe_zoffset;        //auto levele Z axis deviation
  u8    auto_leveling_flag;     // 1

   char SN[16];
   char HV[6];
#ifdef DELTA
  float delta_segments_per_sec;
  float delta_diagonal_rod;
  float delta_smooth_rod_offset;
  float delta_effector_offset;
  float delta_carriage_offset;
  float delta_radius;
  float delta_radius_error;  
  float delta_printable_radius;
  float endstop_adj[3];
  u16 Preheat_conf[2][3];
  u8 fanspeed;
  float z_offset;
  u8 mixer_ofp_min;    //percentage for mixer over fusion protect
  u8 mixer_ofp_max;
  u8 custom_conf_start_percent[6];
  u8 custom_conf_end_percent[6];
  float custom_conf_start_height[6];
  float custom_conf_end_height[6];
#endif 
  u16 extrude_multiply;                                //The multiple of discharge
  u32 min_segment_steps;                           //The minimum number of moves
  u32 min_segment_time;                           //The minimum frequency of printing
  u8   endstop_level[6];	                        //Limit switch operating level
  u8   endstop_status[6];                          //Limit switch enable status
  float retract_acceleration;                      //retract  Acceleration
  float acceleration;					//Acceleration
  u32 axis_steps_per_sqr_second[4];      //XYZE  number of moves
  u8 min_travel_feedrate;                            //Airspeed minimum speed
  u8 min_feedrate;	                                //minimum  print speed
  u8   home_position[3];	      //motor homed position
  u16 home_speed[3];	      //motor homed speed
  float max_jerk;             //Maximum mutation speed
  float max_x_jerk;         //Maximum mutation speed x
  float max_y_jerk;         //Maximum mutation speed y
  float max_z_jerk;         //Maximum mutation speed z
  float max_e_jerk;         //Maximum mutation speed e
   u16 max_acceleration[4];    //XYZE  Maximum Acceleration
  u8 pid_adjust_range;   //PID adjustment range
  matrix_3x3 plan_bed_level_matrix;
  u8 locate_mode;	//coordinate mode
  float Kp[HOTHEAD_NUMBER];					//PID adjustment parameters P  20
  float Ki[HOTHEAD_NUMBER];					//PIDadjustment parameters  i  20
  float Kd[HOTHEAD_NUMBER];					//PIDadjustment parameters  d  20
  float targe_temperature[HOTHEAD_NUMBER];	//targe temperture 20
  float min_temperature[HOTHEAD_NUMBER];	      //min targe temperture
  float max_temperature[HOTHEAD_NUMBER];	      //max targe temperture
  u8 hotend_num; 
  FunctionalState hotend_enable[HOTHEAD_NUMBER];	 
  u8  temperature_sampling_period;  //Temperature acquisition cycle
  u8 unit;             //unit
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
