#include "step_motor.h"
#include "XZK_Rule.h"
#include "delay.h"
#include "setting.h"
#include "planner.h"
#include "recovery_print.h"
#include "variable.h"

#define Set_Default_Step_Interrupt_Rate() //OCR1A = 2000;


static block_t *Current_Block = NULL; 

static s32 X_Error_Coefficient,Y_Error_Coefficient,Z_Error_Coefficient,E_Error_Coefficient;

static u32 Step_Event_Count_Completed = 0;

static u32 Acc_step_rate;
static u32 Acceleration_time;
static u32 Deceleration_time;
static u16 Nominal_Timer;

extern setting_t Setting;

extern DWORD Current_sd_block;
extern DWORD Current_sd_line_num;
extern DWORD Current_sd_byte;

extern float Current_feed_rate;
extern float Current_point_x;
extern float Current_point_y;
extern float Current_point_z;
extern float Current_point_e;

extern RecoveryParameterType recovery_parameter;

uint32_t countt;

__IO s32 Count_Position[NUM_AXIS] = { 0, 0, 0, 0};
__IO s8 Count_Direction[NUM_AXIS] = { 1, 1, 1, 1};

static bool check_endstops = false;
#define CHECK_ENDSTOPS  if(check_endstops)
static bool old_z_min_endstop=false;
volatile long endstops_trigsteps[3]={0,0,0};
static volatile bool endstop_z_hit=false;
const u8 Z_MIN_ENDSTOP_INVERTING = false;//false;//true; // 设定Zmin的逻辑方向
extern volatile long count_position[NUM_AXIS];
volatile s8 count_direction[NUM_AXIS] = { 1, 1, 1, 1};

static u8 Motor_Enable_Flag;
//u8 autohome_flag = 0;

//================================by Chilyn======================================//
//==============================================================================//

float basic_axis_position[3];
u8 color_change_flag = 0;



void Step_Motor_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE); 
        GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
       

        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15;
         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    // ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 
        

        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_11;//           

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    // ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
        

        GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; //
    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    // ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);       
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_13;//           
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    // ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
}

void Step_Motor_Timer_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE); 

    TIM_DeInit(TIM8);	 
	TIM_TimeBaseStructure.TIM_Period = 7999;       
	TIM_TimeBaseStructure.TIM_Prescaler = 8;	   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
  
	TIM_UpdateRequestConfig(TIM8,TIM_UpdateSource_Regular);//Ö»ÓÐÒç³ö»òÕßDMAÇëÇó²Å²úÉúÖÐ¶Ï
	TIM_ITConfig(TIM8,TIM_IT_Update,DISABLE);  
	TIM_Cmd(TIM8, DISABLE);
	
}

void Step_Motor_Timer_Enable(void)
{
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);  
	TIM_Cmd(TIM8, ENABLE);
}

void Step_Motor_NVIC_Config(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);

}
void Step_Motor_Init(void)
{
	Step_Motor_GPIO_Config();
	Step_Motor_Timer_Config();
	Step_Motor_NVIC_Config();
        Set_Micro_step(0x07);	
	Disable_all_Axis();  
            
}

void enable_endstops(bool check)
{
  check_endstops = check;
}
bool Read_endstop_Z_hit(void)
{
  return (endstop_z_hit);
}
u8 Read_Z_MIN_ENDSTOP_INVERTING(void)
{
  return (Z_MIN_ENDSTOP_INVERTING);
}
void Write_endstop_Z_hit(bool hit_state)
{
  endstop_z_hit = hit_state;
}

#pragma inline=forced
void Trapezoid_Generator_Reset()
{
 
  Nominal_Timer = Current_Block->nominal_timer;//(u16)ceil(F_TIM1 / (float)Current_Block->nominal_rate);
  Acc_step_rate = Current_Block->initial_rate;
  Acceleration_time = Current_Block->first_timer;//(u16)ceil(F_TIM1 / (float)Acc_step_rate);
  Deceleration_time = 0;//Acceleration_time;

}



void Step_Motor_Control(void)
{  

  u16 Axis_Dir_Step_PORT =0;
  if(Current_Block == NULL)
  {
    Current_Block = Get_current_block();
    if(Current_Block != NULL)
    {
      if(Current_Block->tools_chang_flag)
      {
        Current_Block->tools_chang_flag = false;
        Current_Block = NULL;
        Discard_current_block();
      }
      else
      {
      		Current_Block->busy = true;
      		Trapezoid_Generator_Reset();
      		X_Error_Coefficient = -(Current_Block->step_event_count >> 1);
      		Y_Error_Coefficient = X_Error_Coefficient;
      		Z_Error_Coefficient = X_Error_Coefficient;
      		E_Error_Coefficient = X_Error_Coefficient;
      		Step_Event_Count_Completed = 0;

			   
		if(system_infor.Auto_Levele_Flag==1)
		{

		     recovery_parameter.bed_calulate_Current_point_x = Current_Block->bed_ForCalulate_point_x;//st_get_position_mm(X_AXIS);//
		     recovery_parameter.bed_calulate_Current_point_y = Current_Block->bed_ForCalulate_point_y;//st_get_position_mm(Y_AXIS);//
		     recovery_parameter.bed_calulate_Current_point_z = Current_Block->bed_ForCalulate_point_z;		
		     recovery_parameter.Current_sd_byte =Current_Block->sd_byte;
		     recovery_parameter.Current_point_e = Current_Block->point_e;
		    
	       }

      
      }
    }
    else
    {
      TIM8->ARR = 7999;//1
    }
  }

  else//if(Current_Block != NULL)
  {
    if(Current_Block->busy)//&&(Current_Block->tools_chang_flag == false))
    {
        Axis_Dir_Step_PORT = Current_Block->direction_bits;
        //GPIO_Write(GPIOB,Axis_Dir_Step_PORT);
              
        if((Axis_Dir_Step_PORT & 0x0100) == 0x0100)
        {
            if(Setting.motor_direction[X_AXIS] == POSITIVE)
                {X_DIR_H;
		    count_direction[X_AXIS]=-1;//往0方向上传动计数量减少，所以方向为负
		}
            else
                {X_DIR_L;
		    count_direction[X_AXIS]=1;//没有写上挤出机相关的函数，看需要编写
		}
        }
        else
        {
            if(Setting.motor_direction[X_AXIS] == POSITIVE)
                {X_DIR_L;
		    count_direction[X_AXIS]=1;
            	}
            else
                {X_DIR_H;
		    count_direction[X_AXIS]=-1;
            	}
        }
        
        if((Axis_Dir_Step_PORT & 0x0200) == 0x0200)
        {
            if(Setting.motor_direction[Y_AXIS] == POSITIVE)
                {Y_DIR_H;
			count_direction[Y_AXIS]=-1;
            	}
            else
                {Y_DIR_L;
			count_direction[Y_AXIS]=1;
            	}
        }
        else
        {
            if(Setting.motor_direction[Y_AXIS] == POSITIVE)
                {Y_DIR_L;
			count_direction[Y_AXIS]=1;
            	}
            else
                {Y_DIR_H;
			count_direction[Y_AXIS]=-1;
            	}
        }
        
        if((Axis_Dir_Step_PORT & 0x0400) == 0x0400)
        {
            if(Setting.motor_direction[Z_AXIS] == POSITIVE)
                {Z_DIR_L;
			count_direction[Z_AXIS]=-1;
            	}
            else
                {Z_DIR_H;
			count_direction[Z_AXIS]=1;
            	}
        }
        else
        {
            if(Setting.motor_direction[Z_AXIS] == POSITIVE)
                {Z_DIR_H;
			count_direction[Z_AXIS]=1;
            	}
            else
                {Z_DIR_L;
			count_direction[Z_AXIS]=-1;
            	}
        }
        
        if((Axis_Dir_Step_PORT & 0x0800) == 0x0800)
        {         
         
                if(Setting.motor_direction[E_AXIS] == POSITIVE)
                    E0_DIR_H;
                else
                    E0_DIR_L;
            
        }
        else
        {
           
                if(Setting.motor_direction[E_AXIS] == POSITIVE)
                    E0_DIR_L;
                else
                    E0_DIR_H;
           
        }                                     
		//限位开关检查，目前只做为Zmin检查
   if((Axis_Dir_Step_PORT & 0x0400) == 0x0400){   // -direction
 
      CHECK_ENDSTOPS
      {
        //#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
          bool z_min_endstop=(GPIO_ReadInputDataBit(Endstop,Max_Z) == Z_MIN_ENDSTOP_INVERTING);
          if(z_min_endstop && old_z_min_endstop && (Current_Block->steps_z > 0)) {
            endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
            endstop_z_hit=true;
            Step_Event_Count_Completed = Current_Block->step_event_count;
          }
          old_z_min_endstop = z_min_endstop;
       // #endif
      }
    }
				 
      X_Error_Coefficient += Current_Block->steps_x;
      if(X_Error_Coefficient > 0)
      {
        //Axis_Do |= 1<<X_AXIS;
        X_STEP_H;
        X_Error_Coefficient -= Current_Block->step_event_count;
	 count_position[X_AXIS]+=count_direction[X_AXIS]; 
      }

      Y_Error_Coefficient += Current_Block->steps_y;
      if(Y_Error_Coefficient > 0)
      {
        //Axis_Do |= 1<<Y_AXIS;
        Y_STEP_H;
        Y_Error_Coefficient -= Current_Block->step_event_count;
	  count_position[Y_AXIS]+=count_direction[Y_AXIS];
      }

      Z_Error_Coefficient += Current_Block->steps_z;
      if(Z_Error_Coefficient > 0)
      {
        //Axis_Do |= 1<<Z_AXIS;
        Z_STEP_H;
        Z_Error_Coefficient -= Current_Block->step_event_count;
	 count_position[Z_AXIS]+=count_direction[Z_AXIS];
      }

      E_Error_Coefficient += Current_Block->steps_e;
      if(E_Error_Coefficient > 0)
      {


		
	        E0_STEP_H;
	   

        E_Error_Coefficient -= Current_Block->step_event_count;
      }
          
      Step_Event_Count_Completed++;
      
      X_STEP_L;
      Y_STEP_L;
      Z_STEP_L;
      
           
          E0_STEP_L;
      
      
      u32 dec_step_rate = 0;
      u16 timer = 0;

      if(Step_Event_Count_Completed < Current_Block->step_event_count)
      {
        if(Step_Event_Count_Completed <= (u32)Current_Block->accelerate_until)
        {
          Acc_step_rate = (u32)floor(Current_Block->acceleration_rate * (float)Acceleration_time);
          Acc_step_rate += Current_Block->initial_rate;

          if(Acc_step_rate > Current_Block->nominal_rate)
          Acc_step_rate = Current_Block->nominal_rate;

          timer =(u16)lround(F_TIM1 / (float)Acc_step_rate);
                  

          TIM8->ARR = timer;
          Acceleration_time += timer;
        }

        else if(Step_Event_Count_Completed > (u32)Current_Block->decelerate_after)
        { 
          dec_step_rate = (u32)floor(Current_Block->acceleration_rate * (float)Deceleration_time);
          if(dec_step_rate > Acc_step_rate)
          dec_step_rate = Current_Block->final_rate;
          else
          dec_step_rate = Acc_step_rate - dec_step_rate;

          if(dec_step_rate < Current_Block->final_rate)
          dec_step_rate = Current_Block->final_rate;

          timer = (u16)lround(F_TIM1 / (float)dec_step_rate);
          
          
          TIM8->ARR = timer;

          Deceleration_time += timer;
        }
        else
        {
          TIM8->ARR = Nominal_Timer;
          Acc_step_rate = Current_Block->nominal_rate;
        
        }
      }
      else
      {
      	if(system_infor.sd_print_status == SD_PRINTING)
      	{
		   
		/*if(system_infor.Auto_Levele_Flag==1)
		{

		     recovery_parameter.bed_calulate_Current_point_x = Current_Block->bed_ForCalulate_point_x;//st_get_position_mm(X_AXIS);//
		     recovery_parameter.bed_calulate_Current_point_y = Current_Block->bed_ForCalulate_point_y;//st_get_position_mm(Y_AXIS);//
		     recovery_parameter.bed_calulate_Current_point_z = Current_Block->bed_ForCalulate_point_z;		
		     recovery_parameter.Current_sd_byte =Current_Block->sd_byte;
		     recovery_parameter.Current_point_e = Current_Block->point_e;
		    
	       }
	       else*/
	       {
			recovery_parameter.Current_sd_byte = Current_Block->sd_byte;
		    	recovery_parameter.Current_point_x = Current_Block->point_x;
		    	recovery_parameter.Current_point_y = Current_Block->point_y;
		    	recovery_parameter.Current_point_z = Current_Block->point_z;
		    	recovery_parameter.Current_point_e = Current_Block->point_e;
	       }
			recovery_parameter.Current_feed_rate = Current_Block->feed_rate;
      	}
		
        Current_Block = NULL;
        Discard_current_block();
      }
    }  
  }
  
}

void Disable_all_Axis(void)
{
   Disable_X_Axis();
   Disable_Y_Axis();
   Disable_Z_Axis();
   Disable_E0_Axis();
   Disable_E1_Axis();
   Disable_E2_Axis();

   Block_clean();
   Current_Block_Clean();
}

void Enable_all_Axis(void)
{
   Enable_X_Axis();
   Enable_Y_Axis();
   Enable_Z_Axis();
   Enable_E0_Axis();
   Enable_E1_Axis();
   Enable_E2_Axis();
}

void SetMotorEnableFlag(u8 flag)
{
	Motor_Enable_Flag = flag;
}

u8 GetMotorEnableFlag(void)
{
	return Motor_Enable_Flag;
}





float get_current_position(u8 axis)
{
  basic_axis_position[axis] = Current_Position[axis];
}


u8 Max_Divisor(u8 a, u8 b)
{
  while ( a!=b)   
  {
   if (a>b)  a=a-b;      
   else  b=b-a;
  }
  return a;
}



void Current_Block_Clean(void)
{
  Current_Block = NULL;
}

void Motor_move_away(void)
{
	int i,distanceX=0;
	
	Enable_X_Axis();
	
	if(Current_Position[X_AXIS] > Setting.max_position[X_AXIS]/2 )  
	{
		distanceX = Setting.max_position[X_AXIS] - Current_Position[X_AXIS];
		X_DIR_L;
	}
	else
	{
		distanceX = Current_Position[X_AXIS];
		X_DIR_H;
	}
	distanceX *= Setting.steps_per_mm[X_AXIS];
       for(i=0;i<distanceX;i++)
	{
		X_STEP_L;
		X_STEP_H;
		//printf("away move  X!\r\n");
			
		delay_us(150);	
	}

}


void Set_Micro_step(u8 ms)
{
  if(ms&0x01) MS1_H;
  else  MS1_L;
  if((ms>>1)&0x01) MS2_H;
  else  MS2_L;
  if((ms>>2)&0x01) MS3_H;
  else  MS3_L;

}


