#include "endstop.h"
#include "XZK_Rule.h"
#include "planner.h"
#include "usart1.h"
#include "setting.h"
#include "sd_print.h"
#include "variable.h"
#include "data_handle.h"

u8 normal_end_signal[3]={0},touch_end_signal[3]={1,1,1},endstop_status[3]={0}, jitter_time[3] = {0};
extern setting_t Setting;
extern char Printf_Buf[512]; 
u8 normal_filament_signal=0, filament_status=0, filament_jitter_time = 0;

 extern void command_process(char* str);
void my_printf(char *Data);
void __Endstop_GPIO_config(void)
{
    Endstop_Level_Init(X_AXIS,X_MIN);
    Endstop_Level_Init(X_AXIS,X_MAX);
    Endstop_Level_Init(Y_AXIS,Y_MIN);
    Endstop_Level_Init(Y_AXIS,Y_MAX);
    Endstop_Level_Init(Z_AXIS,Z_MIN);
    Endstop_Level_Init(Z_AXIS,Z_MAX);
    for(u8 i=0;i<6;i++)
    {
      Endstop_config_for_single_axis(i);
    }
}

void Endstop_GPIO_config(void)
{
    u8 endstop;
    if(Setting.home_position[X_AXIS] == MINENDSTOP) endstop = X_MIN;
    else endstop = X_MAX;    
    Endstop_config_for_single_axis(endstop);
    Endstop_Level_Init(X_AXIS,endstop);
    
    if(Setting.home_position[Y_AXIS] == MINENDSTOP) endstop = Y_MIN;
    else endstop = Y_MAX;    
    Endstop_config_for_single_axis(endstop);
    Endstop_Level_Init(Y_AXIS,endstop);
    
    if(Setting.home_position[Z_AXIS] == MINENDSTOP) endstop = Z_MIN;
    else endstop = Z_MAX;
    Endstop_config_for_single_axis(endstop); 
    Endstop_Level_Init(Z_AXIS,endstop);
}


//axis: 0:x_axis  1:y_axis  2:z_axis
void Endstop_config_for_single_axis(u8 endstop)
{
    
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 
    
    switch(endstop)
    {
      case X_MAX:               
            if(((Setting.endstop_level[X_MAX] == HIGH)&&(Setting.endstop_status[X_MAX] == CLOSE)) || \
               ((Setting.endstop_level[X_MAX] == LOW)&&(Setting.endstop_status[X_MAX] == OPEN)) )
            {
                GPIO_InitStructure.GPIO_Pin =  Max_X;
#ifndef BOARD_M301_Pro_S
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	
#else
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
#endif	   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            else
            {
                GPIO_InitStructure.GPIO_Pin =  Max_X;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            break;
            
      case X_MIN:
        if(((Setting.endstop_level[X_MIN] == HIGH)&&(Setting.endstop_status[X_MIN] == CLOSE)) || \
               ((Setting.endstop_level[X_MIN] == LOW)&&(Setting.endstop_status[X_MIN] == OPEN)) )
            {
                GPIO_InitStructure.GPIO_Pin =  Min_X;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            else
            {
                GPIO_InitStructure.GPIO_Pin = Min_X;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }    
            
		
            break;
            
       case Y_MAX:
            if(((Setting.endstop_level[Y_MAX] == HIGH)&&(Setting.endstop_status[Y_MAX] == CLOSE)) || \
               ((Setting.endstop_level[Y_MAX] == LOW)&&(Setting.endstop_status[Y_MAX] == OPEN)) )
            {
                GPIO_InitStructure.GPIO_Pin =  Max_Y;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            else
            {
                GPIO_InitStructure.GPIO_Pin =  Max_Y;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            break;
            
       case Y_MIN:
            if(((Setting.endstop_level[Y_MIN] == HIGH)&&(Setting.endstop_status[Y_MIN] == CLOSE)) || \
               ((Setting.endstop_level[Y_MIN] == LOW)&&(Setting.endstop_status[Y_MIN] == OPEN)) )
            {
                GPIO_InitStructure.GPIO_Pin =  Min_Y;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            else
            {
                GPIO_InitStructure.GPIO_Pin = Min_Y;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            } 
            break;
            
       case Z_MAX:
            if(((Setting.endstop_level[Z_MAX] == HIGH)&&(Setting.endstop_status[Z_MAX] == CLOSE)) || \
               ((Setting.endstop_level[Z_MAX] == LOW)&&(Setting.endstop_status[Z_MAX] == OPEN)) )
            {
                GPIO_InitStructure.GPIO_Pin = Max_Z;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            else
            {
                GPIO_InitStructure.GPIO_Pin = Max_Z;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            break;
            
       case Z_MIN:  
            if(((Setting.endstop_level[Z_MIN] == HIGH)&&(Setting.endstop_status[Z_MIN] == CLOSE)) || \
               ((Setting.endstop_level[Z_MIN] == LOW)&&(Setting.endstop_status[Z_MIN] == OPEN)) )
            {
                GPIO_InitStructure.GPIO_Pin = Min_Z;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            else
            {
                GPIO_InitStructure.GPIO_Pin = Min_Z;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		   
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
                GPIO_Init(GPIOE, &GPIO_InitStructure);
            }
            break;
    }
    

}

void Read_endstop(void)
{
  u8 signal[3];  
  if(Setting.home_position[X_AXIS] == MINENDSTOP) signal[X_AXIS] = GPIO_ReadInputDataBit(Endstop,Min_X);
  else signal[X_AXIS] = GPIO_ReadInputDataBit(Endstop,Max_X);
  
  if(Setting.home_position[Y_AXIS] == MINENDSTOP) signal[Y_AXIS] = GPIO_ReadInputDataBit(Endstop,Min_Y);
  else signal[Y_AXIS] = GPIO_ReadInputDataBit(Endstop,Max_Y);

  if(Setting.home_position[Z_AXIS] == MINENDSTOP) signal[Z_AXIS] = GPIO_ReadInputDataBit(Endstop,Min_Z);
#if (defined BOARD_A30_MINI_S) || (defined BOARD_A30M_Pro_S)  || (defined BOARD_A30D_Pro_S)
	#ifdef CC_3D_Touch
		else signal[Z_AXIS] = GPIO_ReadInputDataBit(Endstop,Max_Z);
	#else
        else signal[Z_AXIS] = !GPIO_ReadInputDataBit(Endstop,Max_Z);
	#endif
#else
	else signal[Z_AXIS] = GPIO_ReadInputDataBit(Endstop,Max_Z);
#endif
  for(u8 i=0;i<3;i++)
  {
    if(endstop_status[i] != signal[i])
    {
      if(jitter_time[i]>=10) 
      {
          endstop_status[i] = signal[i];
          jitter_time[i] = 0;
      }
      else jitter_time[i]++;
    }
    else jitter_time[i] = 0;
  }   
}

void Endstop_Level_Init(u8 axis, u8 endstop)
{
    if(Setting.endstop_level[endstop] == HIGH)   
    {
        normal_end_signal[axis]=1;
        touch_end_signal[axis]=0;
    }
    else
    {
        normal_end_signal[axis]=0;
        touch_end_signal[axis]=1;
    }
}

/**********************************************************
***Function:     Endstop_Init
***Description: Limit switch pin initialization
***Input:  
***Output: 
***Return:
***********************************************************/
void Endstop_Init(void)
{
    __Endstop_GPIO_config();
    for(u8 i=0;i<100;i++)
    Read_endstop();
}

void Endstop_Exti_config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,  GPIO_PinSource0);
	EXTI_InitStructure.EXTI_Line = (EXTI_Line10);
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,  GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line = (EXTI_Line11);
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,  GPIO_PinSource2);
	EXTI_InitStructure.EXTI_Line = (EXTI_Line12);
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,  GPIO_PinSource3);
	EXTI_InitStructure.EXTI_Line = (EXTI_Line13);
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,  GPIO_PinSource4);
	EXTI_InitStructure.EXTI_Line = (EXTI_Line14);
	EXTI_Init(&EXTI_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,  GPIO_PinSource5);
	EXTI_InitStructure.EXTI_Line = (EXTI_Line15);
	EXTI_Init(&EXTI_InitStructure);
}
void Endstop_NVIC_confing(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure the NVIC Preemption Priority Bits */
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

void Endstop_Handler(void)
{
	//u16 temp;

	EXTI_ClearITPendingBit(EXTI_Line10|EXTI_Line11|EXTI_Line12|EXTI_Line13|EXTI_Line14|EXTI_Line15);
	//temp = (uint16_t)GPIOE->IDR & 0x003F;

}

u8 get_endstop_state(u8 axis, u8 end_position)
{
	u8 state;
#ifdef BOARD_E180_MINI_S
	if((axis == X_AXIS)&&(end_position == MINENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Min_X);
	else if((axis == X_AXIS)&&(end_position == MAXENDSTOP)) state = 1;//GPIO_ReadInputDataBit(Endstop,Max_X);
	else if((axis == Y_AXIS)&&(end_position == MINENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Min_Y);
	else if((axis == Y_AXIS)&&(end_position == MAXENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Max_Y);
	else if((axis == Z_AXIS)&&(end_position == MINENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Min_Z);
	else if((axis == Z_AXIS)&&(end_position == MAXENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Max_Z);   
       state =!state;
#else
	if((axis == X_AXIS)&&(end_position == MINENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Min_X);
	else if((axis == X_AXIS)&&(end_position == MAXENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Max_X);
	else if((axis == Y_AXIS)&&(end_position == MINENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Min_Y);
	else if((axis == Y_AXIS)&&(end_position == MAXENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Max_Y);
	else if((axis == Z_AXIS)&&(end_position == MINENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Min_Z);
	else if((axis == Z_AXIS)&&(end_position == MAXENDSTOP)) state = GPIO_ReadInputDataBit(Endstop,Max_Z);   
#endif
	return state;
}
#if (defined BOARD_A30_MINI_S) || (defined BOARD_A30M_Pro_S)  || (defined BOARD_A30D_Pro_S)
void BLTouch_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;          
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}    

void BLTouch_StateSet(int servo_index, u8 servo_position)//3
{
	if(servo_index==0)
		{
		if(servo_position==SERVO_Push_pin_Down)//Push-pin Down 700 us ( 10 )
			TIM_SetCompare1(TIM2,7);
		else if(servo_position==SERVO_Push_pin_Up)//Push-pin Up 1500 us ( 90)
			TIM_SetCompare1(TIM2,15);
		else if(servo_position==SERVO_Self_test)//Self-test 1800 us ( 120 )
			TIM_SetCompare1(TIM2,18);
		else if(servo_position==SERVO_Alarm_Release)//Alarm Release 2200 us ( 160 )
			TIM_SetCompare1(TIM2,22);
		else if(servo_position==SERVO_M119_Test_Mode)//M119 Test Mode 1200 us ( 60)
			TIM_SetCompare1(TIM2,12);
		else
			{TIM_SetCompare1(TIM2,0);
			  sprintf(Printf_Buf, "Servo %d: %d is not the available range\r\n",servo_index,servo_position);
			  my_printf(Printf_Buf);
			}
		}
}  

u8 BLTouch_SelfCheck(int servo_index)
{u8 i=10;//2
  u8 EndstopState=false,Old_EndstopState=false;
  
	sprintf(Printf_Buf, "Servo %d: self-checking...\r\n",servo_index);
	my_printf(Printf_Buf);
	BLTouch_StateSet(servo_index, SERVO_Push_pin_Down);
	while(i--)
	{
	    delay_ms(200);
          EndstopState=(GPIO_ReadInputDataBit(Endstop,Max_Z) == Read_Z_MIN_ENDSTOP_INVERTING());
          if(EndstopState && Old_EndstopState)
		{
		  BLTouch_StateSet(servo_index, SERVO_Push_pin_Up);
		  delay_ms(200);
		  sprintf(Printf_Buf, "Servo %d: error\r\n",servo_index);
		  my_printf(Printf_Buf);
		  if(system_infor.sd_print_status==SD_PRINTING)
		  {
			system_infor.sd_print_status=SD_PRINT_PAUSE_F;
		  }
		
		  return (EndstopState);
		  }
          Old_EndstopState = EndstopState;
      }
	BLTouch_StateSet(servo_index, SERVO_Push_pin_Up);
	delay_ms(200);
	sprintf(Printf_Buf, "Servo %d: self-checking is completed\r\n",servo_index);
	my_printf(Printf_Buf);
	return (EndstopState);
}  

void __Filament_Level_Init(void)
{
	normal_filament_signal = 0;
	
}

void __Filament_GPIO_config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;		   
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}
/**********************************************************
***Function:     Filament_Init
***Description: filament test pins initialization
***Input:  
***Output: 
***Return:
***********************************************************/
void Filament_Init(void)
{
	__Filament_Level_Init();
	__Filament_GPIO_config();
	for(u8 i=0;i<100;i++)
        Read_filamentstatus();
}

void Read_filamentstatus(void)
{
	u8 signal;  
	signal = GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
	
	  if(filament_status != signal)
	  {
	    if(filament_jitter_time>=200) 
	    {
	        filament_status = signal;
	        filament_jitter_time = 0;
	       //Get_FilamentStatus();
	    }
	    else filament_jitter_time++;
	  }
	  else filament_jitter_time = 0;
 
}

u8 Get_FilamentStatus(void)
{
        static u8 filament_statuss=0,filament_statuss2;
       if(filament_status==1 && filament_statuss<3)
       {

            if(system_infor.Filament_Dev_Flag==0)
		    return 0;
           // filament_status =0;
	    Add_Message(FILAMENT_DETECTOR);	
	    Add_MessageM(CMD_FILAMAND_NO);
	    filament_statuss2 =0;
	    filament_statuss++;
	    printf("filament:erro.....\r\n");
	    return 1;
       }
       else
       {
            if(filament_statuss2<3)
            {
                filament_statuss =0;
                filament_statuss2++;
                Add_Message(FILAMENT_DETECTOR);	
            }
            return 0;
       }
	
}
#endif
