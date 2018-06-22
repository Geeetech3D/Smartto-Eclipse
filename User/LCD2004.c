
#include "LCD2004.h"
#include "command.h"
#include "delay.h"
#include "XZK_Rule.h"
#include "mmc_sd.h"
#include "fat.h"
#include "setting.h"
#include "string.h"
#include "sd.h"
#include "adc.h"
#include "step_motor.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"
#include "planner.h"
#include "sd_print.h"

void Encoder_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;       
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE);
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);

}

void Encoder_EXTI_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure; 
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //开启IO口复用时钟
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource9); 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource13);  

    
    EXTI_InitStructure.EXTI_Line= EXTI_EC1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);       
          

    EXTI_InitStructure.EXTI_Line = EXTI_PRESS;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);  
  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
    
}

void Enable_Encoder_EXTI(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;   
    
    EXTI_InitStructure.EXTI_Line = EXTI_EC1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);  
    
    EXTI_InitStructure.EXTI_Line = EXTI_PRESS;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;   
    EXTI_Init(&EXTI_InitStructure);  

}




void Display_Update_Timer_Config(void)
{
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    
    TIM_TimeBaseStructure.TIM_Period = 9;//999;   //9999
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
    
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  
    TIM_Cmd(TIM5, DISABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM5, ENABLE);
}

static decoder_struct decoder = {0,0xff,0,1,0};

u8 Recovery_error(void)
{

  u8 i = 2;
  while(1)
  {
    if(decoder.decoder_turn_flag == 1)
    {
      if(i==2) i=13;
      else i=2;
      decoder.decoder_turn_flag = 0;
    }
    if(decoder.key_press)
    {
      decoder.key_press = 0;
      if(i==2) return ENABLE;
      else return DISABLE;   
    }
  }
}

