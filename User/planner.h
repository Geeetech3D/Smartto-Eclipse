
#ifndef __PLANNER_H_
#define __PLANNER_H_

#include "stm32f10x.h"
#include "stdbool.h"
#include "XZK_Configuration.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "step_motor.h"
//#include "display.h"
#include "LCD2004.h"
#include "rocker.h"
#include "vector_3.h"

typedef struct {
  float point_x,point_y,point_z,point_e;
  float bed_ForCalulate_point_x,bed_ForCalulate_point_y,bed_ForCalulate_point_z;//保存未处理的xyz坐标
  s32 position_block[4];//保存当前位置值
  u32 steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  u32 step_event_count;           // The number of step events required to complete this block
  u32 accelerate_until;                    // The index of the step event on which to stop acceleration
  u32 decelerate_after;                    // The index of the step event on which to start decelerating
  float acceleration_rate;                   // The acceleration rate used for acceleration calculation
  u16 direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  u8 active_extruder;            // Selects the active extruder


  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float steps_per_mm;
  float nominal_speed;                               // The nominal speed for this block in mm/sec 
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float exit_speed;
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  float safe_speed;
  
  u8 recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  u8 nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  u32 nominal_rate;                        // The nominal step rate for this block in step_events/sec 
  u32 initial_rate;                        // The jerk-adjusted step rate at start of block  
  u32 final_rate;                          // The minimal rate at exit
  u32 acceleration_st;                     // acceleration steps/sec^2
  u32  fan_speed;
  u16 nominal_timer;
  u16 first_timer;
  u8 tools_chang_flag;
  u8 mix_rate[EXT_NUM];

  float feed_rate;
  DWORD sd_byte; 
  
  __IO bool busy;
} block_t;


typedef struct
{
  u8 flag[3];
  u16 timer;
  int count;
  int period[3];
  int step[3];
  int motor_steps[3];
  int move_distance[3];
}autohome_st;
extern autohome_st Autohome;
// Initialize the motion plan subsystem      
void plan_init();
void signal_axis_plan_init(u8 axis);


extern block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
extern __IO u8 block_buffer_head;           // Index of the next block to be pushed
extern __IO u8 block_buffer_tail; 
// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.    
#pragma inline=forced
void Discard_current_block()  
{
  u8 tail = block_buffer_tail;
  if (block_buffer_head != tail) 
  {
    block_buffer_tail = (block_buffer_tail + 1) & ( BLOCK_BUFFER_SIZE - 1);  
  }
}

// Gets the current block. Returns NULL if buffer empty
#pragma inline=forced
block_t *Get_current_block() 
{
  u8 tail = block_buffer_tail;
  if (block_buffer_head == tail)
  { 
    return(NULL); 
  }
   return  (&block_buffer[tail]);
}

// Gets the current block. Returns NULL if buffer empty
#pragma inline=forced
bool Blocks_queued_status() 
{
  uint8_t tail = block_buffer_tail;
  if (block_buffer_head == tail) 
  { 
    return false; 
  }
  else
    return true;
}

#pragma inline=forced
void Queue_wait(void)
{
  while(Blocks_queued_status());
}

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion.
void Plan_buffer_line(float x, float y, float z, const float e, float feed_rate, const uint8_t extruder); //&在形参作为引用是直接操作变量，而不是复制一份，提升读取速度
//void Plan_buffer_line(const float x, const float y, const float z, const float e, float feed_rate, const u8 extruder);
void Plan_mix_rate(mixer_t *m);
void Block_clean(void);

void reset_acceleration_rates();
void Signal_axis_go_origin(float origin_coordinate,u8 axis);
void All_axis_go_origin(void);
void Single_axis_reset_coordinate(float restcoordinate,char axis);
void All_axis_reset_coordinate(void);
void Axis_home_control(void);
void Axis_go_origin(u8 axis,u8 dir, u16 speed, u8 home, float origin_coordinate );//axis:轴（仅用于XY轴）；dir:方向；speed:速度，origin_coordinate:归位后位置值
void axis_move(u8 axis, u8 dir, u16 speed, u16 distance);
void  Filement_Constol_Function(u8 flag);
void st_set_position(const long x, const long y, const long z, const long e);
void st_set_e_position(const long e);
long st_get_position(u8 axis);
float st_get_position_mm(u8 axis);
void plan_get_position(vector_3 *position);
void plan_set_position(float x, float y, float z, const float e);
void st_set_z_position(const long e);
void plan_initXY(void) ;

#endif
