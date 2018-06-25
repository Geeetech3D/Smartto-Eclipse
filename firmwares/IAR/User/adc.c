/*
* Smartto, exclusively developed by Geeetech(http://www.geeetech.com/), is an open source firmware for desktop 3D printers. 
 * Smartto 3D Printer Firmware  
 * It adopts high-performance Cortex M3 core chip STM32F1XX, enabling users to make modifications on the basis of the source code.
 * Copyright (C) 2016, 2017 ,2018 Geeetech [https://github.com/Geeetech3D]
 *
 * Based on Sprinter and grbl.
 * Copyright (C)  2011 Camiel Gubbels / Erik van der Zalm /
 *
 * You should have received a copy of the GNU General Public License version 2 (GPL v2) and a commercial license
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Geeetech Smartto dual license offers users a protective and flexible way to maximize their innovation and creativity.  
 * Smartto aims to be applicable to as many control boards and configurations as possible,use it on your own risk.But to exclusively support Geeetech customers,we makes sure that the
 * releases here are stable and guaranted to work properly on all the printers and hardware sold by Geeetech. 
 * We encourage the community to be active and pursuing the spirits of sharing and mutual help. 
 * The GPL v2 license grants complete use of Smartto to common users. These users are not distributing proprietary modifications or derivatives of Smartto. 
 * If so then there is no need for them to acquire the legal protections of a commercial license.
 * For other users however, who want to use Smartto in their commercial products or have other requirements that are not compatible with the GPLv2, the GPLv2 is not 
 * applicable to them.Even if you want to do so then you must acquire written permission from Geeetech.
 * Only under written condition, Geeetech, the exclusive licensor of Smartto, offers Smartto commercial license to meet their needs. 
 * A Smartto commercial license gives customers legal permission to modify Smartto or incorporate it into their products without the obligation of sharing the final
 * code under the 
 * GPL v2 license. 
 * Fees vary with the application and the scale of its use. For more detailed information, please contact the Geeetech marketing department directly.
 *  
 * Geeetech commits itself to promoting the open source spirit.
*/


#include "XZK_Configuration.h"
#include "temperature_table.h"
#include "adc.h"
#include "usart1.h"
#include "math.h"
#include "setting.h"
#include "XZK_Rule.h"
#include "delay.h"
#include "LCD2004.h"
#include "rocker.h"
#include "step_motor.h"
#include "variable.h"

__IO u16 Current_ADC_Raw[5] ;

static u16 Temperature_Control_PWM[5];

static float Sum_Error[5] = {0,0,0,0,0};

static FlagStatus Pid_Reset[5];

static float PID_P[5],PID_I[5],PID_D[5];

float Current_Temperature[5];

float Current_Width[3];

u8 level_switch = 0;

float Previous_PID_temperature[5];

static float temperature_buffer[4][32]={0.0};

static u16 time_count=0;

static u16 temperature_count=0;

static float gt[5]= {0.0};  //温度返回数组


u8 power_supply = true;

extern void command_process(char* str);


static void ADC_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE); 
#if (defined BOARD_M301_Pro_S) || (defined BOARD_A30M_Pro_S)|| (defined BOARD_A30D_Pro_S)
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_5;
#else
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;// | GPIO_Pin_5
#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOC,&GPIO_InitStructure);           
}


static void ADC_Mode_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)Current_ADC_Raw;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 5;   //dxc 应该是5
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1,&DMA_InitStructure);

	DMA_Cmd(DMA1_Channel1,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);

        ADC_DeInit(ADC1);
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_NbrOfChannel = 5;   //DXC 应该是5
	ADC_Init(ADC1,&ADC_InitStructure);

	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
        


	ADC_RegularChannelConfig(ADC1,ADC_Channel_12,1,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_11,2,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_10,3,ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1,ADC_Channel_13,4,ADC_SampleTime_239Cycles5);
        ADC_RegularChannelConfig(ADC1,ADC_Channel_15,5,ADC_SampleTime_239Cycles5);
	ADC_DMACmd(ADC1,ENABLE);

	ADC_Cmd(ADC1,ENABLE);

	delay_ms(50);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));	
	
}


static void Temperature_PWM_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC, ENABLE); 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&GPIO_InitStructure);


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); 
  GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&GPIO_InitStructure);  
  
#if (defined BOARD_M301_Pro_S) || (defined BOARD_A30M_Pro_S)  || (defined BOARD_A30D_Pro_S)
  GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB,&GPIO_InitStructure);  
#else
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB,&GPIO_InitStructure);  
 #endif
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE);    //2016.6.16  LED
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD,&GPIO_InitStructure);
}
/**********************************************************
***Function:     Temperature_PWM_Mode_Config
***Description: Controlling heating through timer's PWM mode Init
***Input:  
***Output: 
***Return:
***********************************************************/
static void Temperature_PWM_Mode_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;


  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); 
  /* PWM */
  u16 Default_Val = 0;        

  TIM_DeInit(TIM2);
  TIM_DeInit(TIM3);
  TIM_DeInit(TIM4);
  
  TIM_TimeBaseStructure.TIM_Period = 4095;                               //Timer count period
  TIM_TimeBaseStructure.TIM_Prescaler = 35;	                           //Timer frequency division   --2MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//Set the frequency division factor
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //Counting mode
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
  /* PWM1 Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //PWM module 1
  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  

  TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 
  TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 
  TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
   
 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	   
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

  /* PWM1 Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);	  
  TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);	  
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  Default_Val=FULL_POWER;//0;//0x1fff;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  
  TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); 

  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);	  
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = Default_Val;	  
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);	  
  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);   
   
  TIM_ARRPreloadConfig(TIM2, ENABLE);			 
  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  
  /* TIM3 enable counter */
 
  TIM_Cmd(TIM2, ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}

void Temperature_Control_Enable(void)
{
    SET_T0_POWER(SHUT_OFF_POWER);
    SET_T1_POWER(SHUT_OFF_POWER);
    SET_T2_POWER(SHUT_OFF_POWER);
    SET_BED_POWER(SHUT_OFF_POWER);
    TIM_Cmd(TIM3, ENABLE);
}

void Temperature_Control_Disable(void)
{
    SET_T0_POWER(SHUT_OFF_POWER);
    SET_T1_POWER(SHUT_OFF_POWER);
    SET_T2_POWER(SHUT_OFF_POWER);
    SET_BED_POWER(SHUT_OFF_POWER);
    TIM_Cmd(TIM3, DISABLE);
}

void Fan_Control_Enable(void)
{
    SET_FAN0_SPEED(SHUT_OFF_POWER);
    SET_FAN1_SPEED(HALF_POWER);
    SET_FAN2_SPEED(HALF_POWER);
    TIM_Cmd(TIM4, ENABLE);
}

void Fan_Control_Disable(void)
{
    SET_FAN0_SPEED(SHUT_OFF_POWER);
    SET_FAN1_SPEED(SHUT_OFF_POWER);
    SET_FAN2_SPEED(SHUT_OFF_POWER);
    TIM_Cmd(TIM4, DISABLE);
}

void Temperature_Measure_Enable(void)
{
    for(u8 i=0;i<10;i++)  Current_ADC_Raw[i] = 0;
        ADC_SoftwareStartConvCmd(ADC1,ENABLE);
}

void Temperature_Measure_Disable(void)
{
    ADC_SoftwareStartConvCmd(ADC1,DISABLE);
    for(u8 i=0;i<10;i++)  Current_ADC_Raw[i] = 0;
}


/**********************************************************
***Function:     Temperature_Measure_Init
***Description: Temperature Measure GPIO Pins Init
***Input:  
***Output: 
***Return:
***********************************************************/
void Temperature_Measure_Init(void)
{
    ADC_GPIO_Config();
    ADC_Mode_Config();
}
/**********************************************************
***Function:     Temperature_Control_Init
***Description: Temperature Control GPIO Pins Init
***Input:  
***Output: 
***Return:
***********************************************************/
void Temperature_Control_Init(void)
{
	Temperature_PWM_GPIO_Config();
	Temperature_PWM_Mode_Config();
}
/**********************************************************
***Function:     Temperature_Sampling_Timer_Config
***Description: Temperature sampling cycle initialization --- 1ms
***Input:  
***Output: 
***Return:
***********************************************************/
void Temperature_Sampling_Timer_Config(void)
{

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    TIM_DeInit(TIM6);
    TIM_PrescalerConfig(TIM6,71,TIM_PSCReloadMode_Update);
    TIM_SetAutoreload(TIM6,999);
    TIM_UpdateDisableConfig(TIM6,DISABLE);
    TIM_UpdateRequestConfig(TIM6,TIM_UpdateSource_Regular);
    TIM_ARRPreloadConfig(TIM6, ENABLE);
    TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);  
    TIM_Cmd(TIM6, ENABLE);
}
/**********************************************************
***Function:     Temperature_Sampling_NVIC_Configuration
***Description: TIM6 Interrupt priority config
***Input:  
***Output: 
***Return:
***********************************************************/
void Temperature_Sampling_NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM3,ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC2 , ENABLE);
}

void Temperature_Sampling_Timer_for_PID_Autotune_Config(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_DeInit(TIM6);
	TIM_PrescalerConfig(TIM6,71,TIM_PSCReloadMode_Update);
	TIM_SetAutoreload(TIM6, 999);
	TIM_UpdateDisableConfig(TIM6,DISABLE);
	TIM_UpdateRequestConfig(TIM6,TIM_UpdateSource_Regular);
	TIM_ARRPreloadConfig(TIM6, ENABLE);
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);  
	TIM_Cmd(TIM6, ENABLE);
#ifdef BOARD_M301_Pro_S  
	  //PID_Autotune_Time = 0;
	  //PID_Autotune_flag = ENABLE;
#endif
}



void Temperature_Sampling_for_PID_Autotune_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
}

void Set_Hotend_Power(u8 e,u16 power)
{
  switch(e)
  {
    case NOZZLE0:SET_T0_POWER(power);break;//TIM_SetCompare1(TIM2,power);
    case NOZZLE1:SET_T1_POWER(power);break;
    case NOZZLE2:SET_T2_POWER(power);break;
    case BED:SET_BED_POWER(power);break;
    default:break;
  }
}

void Set_Fan_Power(u8 num, u16 power)
{
	

  switch(num)
  {
    case 0:SET_FAN0_SPEED(power);
            break;
    case 1:SET_FAN1_SPEED(power);
            break;
    case 2:SET_FAN2_SPEED(power);
            break;
  }
}

void Set_All_Fan_Power(u16 power)
{
  for(u8 i=0;i<3;i++)
  {
    Set_Fan_Power(i,power);
  }
}


static u16 PID_adjust(u8 e)
{
  float out_value;
  float error;
  u16 pwm_output;
  
  error = Setting.targe_temperature[e] - Current_Temperature[e];
#ifdef BOARD_M301_Pro_S
  if(error > Setting.pid_adjust_range)
#else
  if(error > (Setting.pid_adjust_range-5)&&e==3)//Hot bed heating
  {
    pwm_output = FULL_POWER;
    Pid_Reset[e] = SET; 
  }
  else if((error > Setting.pid_adjust_range)&&e!=3)//Setting.pid_adjust_range&&&&e!=3)  Extrusion head heating
#endif
  {
      pwm_output = FULL_POWER;
      Pid_Reset[e] = SET; 
  }
 else if(error < -Setting.pid_adjust_range)
  {
    pwm_output = SHUT_OFF_POWER;
    Pid_Reset[e] = SET;
  }
  else
  {
    if(Pid_Reset[e] == SET)
    {
      PID_D[e] = 0.0;
      Sum_Error[e] = 0.0;
    }
    Pid_Reset[e] = RESET;
    PID_P[e] = error*Setting.Kp[e];
    Sum_Error[e] += error;
    PID_I[e] = Sum_Error[e]*Setting.Ki[e];
    PID_D[e] = ((Current_Temperature[e] - Previous_PID_temperature[e])*Setting.Kd[e])*K1 + PID_D[e]*(1.0-K1);
    out_value = PID_P[e]+PID_I[e]-PID_D[e];
    Previous_PID_temperature[e] = Current_Temperature[e];
    pwm_output = (unsigned int)(Localization(out_value,SHUT_OFF_POWER,FULL_POWER));
  }
 // if(e==0)
 // 	printf("PWM:%d,Temp:%.1f,P:%.2f,I:%.2f,D:%.2f\r\n\r\n",pwm_output,Current_Temperature[e],Setting.Kp[e],Setting.Ki[e],Setting.Kd[e]);
  return pwm_output;
	
}


 extern u8 Temp_exception_flag;
 /**********************************************************
***Function:     Gloabl_temperature_control
***Description: Temperature control processing function  Called in the timer
***Input:  
***Output: 
***Return:
***********************************************************/
void Gloabl_temperature_control(void)
{
#ifndef BOARD_M301_Pro_S
    static u8 erro_flag_times=0;
    static u8 Temperature_error_flag = 0,Error_Times_flag=1;
    if(system_infor.sd_print_status == 2)//Temperature abnormal events
    {

        if((Setting.targe_temperature[NOZZLE0] - Current_Temperature[NOZZLE0] )<=10 && (Temperature_error_flag==0) &&(Setting.targe_temperature[NOZZLE0])!=0)
            Temperature_error_flag = 1;
        else if((Temperature_error_flag != 0) && (Setting.targe_temperature[NOZZLE0] - Current_Temperature[NOZZLE0] ) >=40)
        {
            erro_flag_times++;
            if(erro_flag_times>100)
            {
                command_process("M25\r\n");
                Temp_exception_flag=1;
                Temperature_error_flag = 0;
                erro_flag_times =0;
                printf("Setting.targe_temperature[NOZZLE0]:%f;Current_Temperature[NOZZLE0]:%f;\r\n",Setting.targe_temperature[NOZZLE0],Current_Temperature[NOZZLE0]);
            }
        }
        else
        {
        erro_flag_times=1;
        }
    }
    else
    {
        Temperature_error_flag=0;
        if(Current_Temperature[NOZZLE0]<1)
        {
            if(Error_Times_flag<220)
                Error_Times_flag++;
            if(Error_Times_flag%100==0)
            {
                Temp_exception_flag=1;
                Error_Times_flag=1;
            }
        }
        else
        {
            Error_Times_flag=1;
        }
    }
#endif
    for(unsigned char e=0;e<HOTHEAD_NUMBER;e++)
    {
        Current_Temperature[e] = Get_Temperature(e);
        if(Setting.hotend_enable[e] == ENABLE)
        {    
            if(Current_Temperature[e] < (-15))
            {    
                Current_Temperature[e]=0;
                Set_Hotend_Power(e,SHUT_OFF_POWER);         
            }
            else
            {
                if(Current_Temperature[e] > Setting.max_temperature[e]+10)  //Stop heating conditions
                {
                    Temperature_Control_PWM[e] = SHUT_OFF_POWER;
                }

                else if(Current_Temperature[e] < Setting.min_temperature[e])//Full power heating conditions
                {
                    Temperature_Control_PWM[e] = FULL_POWER;
                }

                else//Perform PID temperature adjustment
                {
                    Temperature_Control_PWM[e] = PID_adjust(e);
                }
                    Set_Hotend_Power(e,Temperature_Control_PWM[e]);
            }
        }
        else
            Set_Hotend_Power(e,SHUT_OFF_POWER);
    }
}


void Init_PID_Autotune(void)
{
    Temperature_Measure_Init();
    Temperature_Control_Init();
    Temperature_Sampling_Timer_for_PID_Autotune_Config();
    Temperature_Sampling_for_PID_Autotune_NVIC_Configuration();
    Temperature_Measure_Enable(); 
    Temperature_Control_Enable();
}

 /**********************************************************
***Function:     Get_Temperature
***Description: Get target current temperature
***Input:  
***Output: 
***Return:
***********************************************************/
float Get_Temperature(u8 hot_heat_num)
{
    u16 i = 1;
    u8 j=0,k=0;
    u16 temp=0;
    for(j=1;j<=31;j++) 
    { 
        for (k=0;k<32-j;k++) 
        {
            if (temperature_buffer[hot_heat_num][k]>temperature_buffer[hot_heat_num][k+1]) 
            { 
                temp = (u16)temperature_buffer[hot_heat_num][k]; 
                temperature_buffer[hot_heat_num][k]=temperature_buffer[hot_heat_num][k+1]; 
                temperature_buffer[hot_heat_num][k+1]=temp;
            } 
        }
    } 
    temp = 0;
    for(j=10;j<14;j++)
    {
        temp += temperature_buffer[hot_heat_num][j];
    }
    temp = temp>>2;
    i = 1;
#ifndef BOARD_M301_Pro_S
    if(hot_heat_num==NOZZLE0)
    {
        while((i<302)&&(temp< Temperature_Table_NOZZLE[i]))
        {
            i++;
        }
        gt[hot_heat_num] = (float)(Temperature_Table_NOZZLE[i] -Temperature_Table_NOZZLE[i+1]);
        gt[hot_heat_num] = 1.0/gt[hot_heat_num];
        gt[hot_heat_num] = (float)(Temperature_Table_NOZZLE[i] - temp)*gt[hot_heat_num];
        gt[hot_heat_num]+= (float)(i);
    }
    else
#endif
    {

        while((i<301)&&(temp< Temperature_Table_B3950[i]))
        {
            i++;
        }
        gt[hot_heat_num] = (float)(Temperature_Table_B3950[i] -Temperature_Table_B3950[i+1]);
        gt[hot_heat_num] = 1.0/gt[hot_heat_num];
        gt[hot_heat_num] = (float)(Temperature_Table_B3950[i] - temp)*gt[hot_heat_num];
        gt[hot_heat_num]+= (float)(i);
    }

    return gt[hot_heat_num];
}	


 /**********************************************************
***Function:     Temperature_Handler
***Description: Temperature control interval
***Input:  
***Output: 
***Return:
***********************************************************/
void Temperature_Handler(void)
{
    if(temperature_count<32)
    {    
        temperature_buffer[NOZZLE0][temperature_count]= Current_ADC_Raw[NOZZLE0];
        temperature_buffer[NOZZLE1][temperature_count]= Current_ADC_Raw[NOZZLE1];
        temperature_buffer[NOZZLE2][temperature_count]= Current_ADC_Raw[NOZZLE2];
        temperature_buffer[BED][temperature_count]= Current_ADC_Raw[BED];
        temperature_count++;
    }

    if(time_count >= Setting.temperature_sampling_period * 100)// 0.3s Detect temperature once
    {
        time_count=0;
        if(temperature_count==32)
        {
            Gloabl_temperature_control();
            temperature_count=0;						
        }
    }
    else time_count++;
}

void Close_all_hotend(void)
{
   Temperature_Control_Disable();
   SET_FAN0_SPEED(0);
   SET_FAN1_SPEED(0);
   SET_FAN2_SPEED(0);
   Disable_Y_Axis();
   Disable_Z_Axis();
#ifdef BOARD_A30M_Pro_S
        Disable_E2_Axis() ;
#endif
   Disable_E0_Axis();
}

int Voltage_get(void)
{
  int voltage = 0;
  voltage = (int)(ADC_REFV* 1000 * Current_ADC_Raw[4] / FULL_POWER);
  voltage = (int)(voltage/VOL_DIVIDER);
  return voltage;
}

void Voltage_detect(void)
{
  int voltage = 0;
  delay_ms(10);
  voltage = Voltage_get();
  if(voltage >= 20000) power_supply = USE24V;
  else power_supply = USE12V;
  
}

u8 Power_down_detect(void)
{
  u8 res = 0;
  int voltage;
  voltage = Voltage_get();
  if(voltage < power_supply*900) res = 1;
  return res;

}

u16 Beep_Delay=0;
void delay_beep(u16 i)
{
  u16 j=0,k=0;
  for(j=0;j<i;j++)
    for(k=0;k<10000;k++);
}
#ifdef BOARD_M301_Pro_S   
void Set_Beep(u16 period, u8 duty)
{
  int k = 0;
  k = 2000000/period;
  //TIM_SetAutoreload(TIM2,k); 
  SET_BEEP_PERIOD(k);
  k = duty * k/255;
  //TIM_SetCompare3(TIM2,k);
  SET_BEEP_DUTY(k);

}
#else
void Set_Beep(u16 period, u8 duty)     //20160813
{
  {
  	GPIO_SetBits(GPIOB, GPIO_Pin_10); 
  	delay_beep(50);
  	GPIO_ResetBits(GPIOB, GPIO_Pin_10); 
  }
}
#endif
extern u32 System_time;
void My_delay_ms(u32 num)
{
    static u32 Pre_time;
    Pre_time = System_time;
    while(1)
    {
        if(System_time >= Pre_time)
        {
            if((System_time - Pre_time) >= (num*2))
            {
                break;
            }
        }
        else
        {
                if((4294967295+System_time - Pre_time) >= (num*2))
                {
                    break;
                }
        }
    }

}

void Alarm_Function(u8 num)
{
    u8 i;
    for(i=0;i<num;i++)
    {
      	GPIO_SetBits(GPIOB, GPIO_Pin_10); 
  		My_delay_ms(6);
  		GPIO_ResetBits(GPIOB, GPIO_Pin_10); 
  		My_delay_ms(4);
    }
    My_delay_ms(10);
}

void Alarm_Handle(u8 num,u8 times)
{
    u8 i;
    for(i=0;i<times;i++)
    {
        Alarm_Function(num);
    }
}
extern u16 Beep_period; //period in milliseconds
extern int Beep_duration; //duration in milliseconds 
u16 Beep_count = 0;
void Beep_control(void)
{

    if(Beep_duration == 0) 
    {
      Beep_count = 0;
      BEEP_DIS;
    }
    else
    {
      SET_BEEP((10000/Beep_period), 127);
      Beep_duration--;
    }
}

