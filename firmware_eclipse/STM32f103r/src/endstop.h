#ifndef __ENDSTOP_H_
#define __ENDSTOP_H_
#include "Configuration_Select_Printer.h"
#include "stm32f10x.h"
#include "XZK_Configuration.h"

#define Endstop GPIOE

#ifdef BOARD_A30_MINI_S
  #define Max_Z GPIO_Pin_1  //  #define Max_Z GPIO_Pin_0  
  #define Min_Z  GPIO_Pin_0  //  #define Min_Z GPIO_Pin_1
#else
  #define Max_Z GPIO_Pin_0
  #define Min_Z GPIO_Pin_1
#endif
  #define Max_Y GPIO_Pin_2
  #define Min_Y GPIO_Pin_3
  #define Max_X GPIO_Pin_4
  #define Min_X GPIO_Pin_5

void __Endstop_GPIO_config(void);
void Endstop_GPIO_config(void);
void Endstop_config_for_single_axis(u8 endstop);
void Read_endstop(void);
void Endstop_Exti_config(void);
void Endstop_NVIC_confing(void);
void Endstop_Handler(void);
void Endstop_Level_Init(u8 axis, u8 endstop);
void Endstop_Init(void);
u8 get_endstop_state(u8 axis, u8 end_position);

void Endstop_GPIO_config(void);  //fdwong
u8 get_endstop_state(u8 axis, u8 end_position); //fdwong

void Filament_Init(void);
u8 Get_FilamentStatus(void);
void Read_filamentstatus(void);
void BLTouch_StateSet(int servo_index, u8 servo_position);
u8 BLTouch_SelfCheck(int servo_index);
#endif
