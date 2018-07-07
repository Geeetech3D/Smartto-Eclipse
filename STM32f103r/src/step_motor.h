#ifndef __STEP_MOTOR_CONTROL_H_
#define __STEP_MOTOR_CONTROL_H_

#include "stm32f10x.h"
#include "stdbool.h"
#include "fat.h"
#include "XZK_Configuration.h"

#define F_TIM1 8000000.0





  #define Enable_X_Axis()       GPIOA->BRR = GPIO_Pin_8
  #define Disable_X_Axis()      GPIOA->BSRR = GPIO_Pin_8
  #define X_DIR_L               GPIO_ResetBits(GPIOD, GPIO_Pin_13)
  #define X_DIR_H               GPIO_SetBits(GPIOD, GPIO_Pin_13)
  #define X_STEP_L              GPIO_ResetBits(GPIOC, GPIO_Pin_6)
  #define X_STEP_H              GPIO_SetBits(GPIOC, GPIO_Pin_6)

  #define Enable_Y_Axis()       GPIOA->BRR = GPIO_Pin_15
  #define Disable_Y_Axis()      GPIOA->BSRR = GPIO_Pin_15
  #define Y_DIR_L               GPIO_ResetBits(GPIOA, GPIO_Pin_11)
  #define Y_DIR_H               GPIO_SetBits(GPIOA, GPIO_Pin_11)
  #define Y_STEP_L              GPIO_ResetBits(GPIOA, GPIO_Pin_12)
  #define Y_STEP_H              GPIO_SetBits(GPIOA, GPIO_Pin_12)

  #define Enable_Z_Axis()       GPIOB->BRR = GPIO_Pin_3
  #define Disable_Z_Axis()      GPIOB->BSRR = GPIO_Pin_3
  #define Z_DIR_L               GPIO_ResetBits(GPIOD, GPIO_Pin_3)
  #define Z_DIR_H               GPIO_SetBits(GPIOD, GPIO_Pin_3)
  #define Z_STEP_L              GPIO_ResetBits(GPIOD, GPIO_Pin_6)
  #define Z_STEP_H              GPIO_SetBits(GPIOD, GPIO_Pin_6)

  #define Enable_E0_Axis()      GPIOC->BRR = GPIO_Pin_4
  #define Disable_E0_Axis()     GPIOC->BSRR = GPIO_Pin_4
  #define E0_DIR_L              GPIO_ResetBits(GPIOB, GPIO_Pin_11)
  #define E0_DIR_H              GPIO_SetBits(GPIOB, GPIO_Pin_11)
  #define E0_STEP_L             GPIO_ResetBits(GPIOB, GPIO_Pin_2)
  #define E0_STEP_H             GPIO_SetBits(GPIOB, GPIO_Pin_2)

  #define Enable_E1_Axis()      GPIOA->BRR = GPIO_Pin_1
  #define Disable_E1_Axis()     GPIOA->BSRR = GPIO_Pin_1
  #define E1_DIR_L              GPIO_ResetBits(GPIOB, GPIO_Pin_6)
  #define E1_DIR_H              GPIO_SetBits(GPIOB, GPIO_Pin_6)
  #define E1_STEP_L             GPIO_ResetBits(GPIOA, GPIO_Pin_0)
  #define E1_STEP_H             GPIO_SetBits(GPIOA, GPIO_Pin_0)

  #define Enable_E2_Axis()      GPIOC->BRR = GPIO_Pin_15
  #define Disable_E2_Axis()     GPIOC->BSRR = GPIO_Pin_15
  #define E2_DIR_L              GPIO_ResetBits(GPIOC, GPIO_Pin_13)
  #define E2_DIR_H              GPIO_SetBits(GPIOC, GPIO_Pin_13)
  #define E2_STEP_L             GPIO_ResetBits(GPIOC, GPIO_Pin_14)
  #define E2_STEP_H             GPIO_SetBits(GPIOC, GPIO_Pin_14)

  #define MS1_L                 
  #define MS1_H                 
  #define MS2_L                 
  #define MS2_H                 
  #define MS3_L                 
  #define MS3_H                 



#define TEMPLATE_1 1
#define TEMPLATE_2 2
#define TEMPLATE_3 3
#define TEMPLATE_4 4
#define TEMPLATE_5 5
#define TEMPLATE_6 6
#define TEMPLATE_7 7
#define TEMPLATE_8 8
#define CUSTOM_1   1
#define CUSTOM_2   2
#define CUSTOM_3   3
#define CUSTOM_4   4
#define CUSTOM_5   5
#define CUSTOM_6   6

typedef struct    
{
  u8 rate[EXT_NUM];
  u8 rate_buf[EXT_NUM];  //use as buffer when use T code
  int counts;
  u8 min,max;
  bool ofp;   //over fusion protect
}mixer_t;

extern __IO u8 stop_flag;


bool Read_endstop_Z_hit(void);
void Write_endstop_Z_hit(bool hit_state);
void enable_endstops(bool check);


void Step_Motor_GPIO_Config(void);
void Step_Motor_Timer_Enable(void);
void Step_Motor_Set_Position(sc32 x, sc32 y, sc32 z, sc32 e);
void Step_Motor_Set_E_Position(sc32 e);
void Step_Motor_Init(void);
void Step_Motor_Control(void);
void Disable_all_Axis(void);
void Enable_all_Axis(void);
void SetMotorEnableFlag(u8 flag);

void Mixer_Init(void);
void Color_change(u8 start_p,u8 end_p,float start_h, float end_h);
u8 GetMotorEnableFlag(void);
void Current_Block_Clean(void);
float get_current_position(u8 axis);
void color_control(void);
u8 Max_Divisor(u8 m, u8 n);
static void Nozzle_Select(void);
DWORD Save_line_num(void);
DWORD Save_sd_block(void);
DWORD Save_sd_byte(void);
float Save_feed_rate(void);
float Save_point_x(void);
float Save_point_y(void);
float Save_point_z(void);
float Save_point_e(void);
void Clear_Save_Parameter(void);	
void Motor_move_away(void);
void Set_Micro_step(u8 ms);
u8 Read_Z_MIN_ENDSTOP_INVERTING(void);
void color_control(void);
#endif
