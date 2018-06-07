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
const u8 Z_MIN_ENDSTOP_INVERTING = false;
extern volatile long count_position[NUM_AXIS];
volatile s8 count_direction[NUM_AXIS] = { 1, 1, 1, 1};

static u8 Motor_Enable_Flag;
//u8 autohome_flag = 0;

//================================by Chilyn======================================//
u8 nozzle_select = NOZZLE0 ;
//==============================================================================//

float basic_axis_position[3];
u8 color_change_flag = 0;

mixer_t mixer;
extern u8 power_supply;
static void Nozzle_Select(void);
/*******************step motor gpio pins init*************************/
void Step_Motor_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE); 
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_11;//           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15; //
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    //
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);       
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_13;//           
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;		    // 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*************step motor Tim8  init*****************/
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
    TIM_UpdateRequestConfig(TIM8,TIM_UpdateSource_Regular);
    TIM_ITConfig(TIM8,TIM_IT_Update,DISABLE);  
    TIM_Cmd(TIM8, DISABLE);
}

void Step_Motor_Timer_Enable(void)
{
	TIM_ITConfig(TIM8,TIM_IT_Update,ENABLE);  
	TIM_Cmd(TIM8, ENABLE);
}

/***************Timer priority configuration*****************/
void Step_Motor_NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}
/**********************************************************
***Function:     Step_Motor_Init
***Description: The motor GPIO pin configuration initialization
***Input:  
***Output: 
***Return:
***********************************************************/
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

/**********************************************************
***Function:     Step_Motor_Control
***Description:  Stepper motor print control one step in TIM8
***Input:  
***Output: 
***Return:
***********************************************************/

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

#ifdef ENABLE_AUTO_BED_LEVELING
                if(system_infor.Auto_Levele_Flag==1)
                {
                    recovery_parameter.bed_calulate_Current_point_x = Current_Block->bed_ForCalulate_point_x;//st_get_position_mm(X_AXIS);//
                    recovery_parameter.bed_calulate_Current_point_y = Current_Block->bed_ForCalulate_point_y;//st_get_position_mm(Y_AXIS);//
                    recovery_parameter.bed_calulate_Current_point_z = Current_Block->bed_ForCalulate_point_z;		
                    recovery_parameter.Current_sd_byte =Current_Block->sd_byte;
                    recovery_parameter.Current_point_e = Current_Block->point_e;
                }
#endif
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
            if((Axis_Dir_Step_PORT & 0x0100) == 0x0100)
            {
                if(Setting.motor_direction[X_AXIS] == POSITIVE)
                {
                    X_DIR_H;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[X_AXIS]=-1;
#endif
                }
                else
                {
                    X_DIR_L;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[X_AXIS]=1;
#endif
                }
            }
            else
            {
                if(Setting.motor_direction[X_AXIS] == POSITIVE)
                {
                    X_DIR_L;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[X_AXIS]=1;
#endif
                }
                else
                {
                    X_DIR_H;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[X_AXIS]=-1;
#endif
                }
            }

            if((Axis_Dir_Step_PORT & 0x0200) == 0x0200)
            {
                if(Setting.motor_direction[Y_AXIS] == POSITIVE)
                {
                    Y_DIR_H;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[Y_AXIS]=-1;
#endif
                }
                else
                {
                    Y_DIR_L;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[Y_AXIS]=1;
#endif
                }
            }
            else
            {
                if(Setting.motor_direction[Y_AXIS] == POSITIVE)
                {
                    Y_DIR_L;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[Y_AXIS]=1;
#endif
                }
                else
                {
                    Y_DIR_H;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[Y_AXIS]=-1;
#endif
                }
            }

            if((Axis_Dir_Step_PORT & 0x0400) == 0x0400)
            {
                if(Setting.motor_direction[Z_AXIS] == POSITIVE)
                {
                    Z_DIR_L;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[Z_AXIS]=-1;
#endif
                }
                else
                {
                    Z_DIR_H;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[Z_AXIS]=1;
#endif
                }
            }
            else
            {
                if(Setting.motor_direction[Z_AXIS] == POSITIVE)
                {
                    Z_DIR_H;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[Z_AXIS]=1;
#endif
                }
                else
                {
                    Z_DIR_L;
#ifdef ENABLE_AUTO_BED_LEVELING
                    count_direction[Z_AXIS]=-1;
#endif
                }
            }

            if((Axis_Dir_Step_PORT & 0x0800) == 0x0800)
            {         
#ifdef BOARD_M301_Pro_S 
          if(nozzle_select == NOZZLE0)
            {
                if(Setting.motor_direction[E_AXIS] == POSITIVE)
                    E0_DIR_H;
                else
                    E0_DIR_L;
            }
            else if(nozzle_select == NOZZLE1)
            {
                if(Setting.motor_direction[E1_AXIS] == POSITIVE)    
                    E1_DIR_H;
                else
                    E1_DIR_L;
            }
            else if(nozzle_select == NOZZLE2)
            {
                if(Setting.motor_direction[E2_AXIS] == POSITIVE)
                    E2_DIR_H;
                else
                    E2_DIR_L;
            }
#else
                if(Setting.motor_direction[E_AXIS] == POSITIVE)
                    E0_DIR_H;
                else
                    E0_DIR_L;
#endif
            }
            else
            {
#ifdef BOARD_M301_Pro_S 
if(nozzle_select == NOZZLE0)
            {
                if(Setting.motor_direction[E_AXIS] == POSITIVE)
                    E0_DIR_L;
                else
                    E0_DIR_H;
            }
            else if(nozzle_select == NOZZLE1)
            {
                if(Setting.motor_direction[E1_AXIS] == POSITIVE)  
                    E1_DIR_L;
                else
                    E1_DIR_H;
            }
            else if(nozzle_select == NOZZLE2)
            {
                if(Setting.motor_direction[E2_AXIS] == POSITIVE)
                    E2_DIR_L;
                else
                    E2_DIR_H;
            }        
#else
                if(Setting.motor_direction[E_AXIS] == POSITIVE)
                    E0_DIR_L;
                else
                    E0_DIR_H;
#endif
            }          
#ifdef ENABLE_AUTO_BED_LEVELING
            if((Axis_Dir_Step_PORT & 0x0400) == 0x0400)
            {   // -direction
                CHECK_ENDSTOPS
                {
                    bool z_min_endstop=(GPIO_ReadInputDataBit(Endstop,Max_Z) == Z_MIN_ENDSTOP_INVERTING);
                    if(z_min_endstop && old_z_min_endstop && (Current_Block->steps_z > 0)) 
                    {
                        endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
                        endstop_z_hit=true;
                        Step_Event_Count_Completed = Current_Block->step_event_count;
                    }
                    old_z_min_endstop = z_min_endstop;
                }
            }
#endif			 
            X_Error_Coefficient += Current_Block->steps_x;
            if(X_Error_Coefficient > 0)
            {
                X_STEP_H;
                X_Error_Coefficient -= Current_Block->step_event_count;
#ifdef ENABLE_AUTO_BED_LEVELING
                count_position[X_AXIS]+=count_direction[X_AXIS]; 
#endif
            }
            Y_Error_Coefficient += Current_Block->steps_y;
            if(Y_Error_Coefficient > 0)
            {
                Y_STEP_H;
                Y_Error_Coefficient -= Current_Block->step_event_count;
#ifdef ENABLE_AUTO_BED_LEVELING
                count_position[Y_AXIS]+=count_direction[Y_AXIS];
#endif
            }
            Z_Error_Coefficient += Current_Block->steps_z;
            if(Z_Error_Coefficient > 0)
            {
                Z_STEP_H;
                Z_Error_Coefficient -= Current_Block->step_event_count;
#ifdef ENABLE_AUTO_BED_LEVELING
                count_position[Z_AXIS]+=count_direction[Z_AXIS];
#endif
            }
            E_Error_Coefficient += Current_Block->steps_e;
            if(E_Error_Coefficient > 0)
            {
#ifdef BOARD_M301_Pro_S 
        color_control();
        Nozzle_Select();  
        
        if(nozzle_select == NOZZLE0)
            E0_STEP_H;
        else if(nozzle_select == NOZZLE1)
            E1_STEP_H;
        else if(nozzle_select == NOZZLE2)
            E2_STEP_H;

		
        E_Error_Coefficient -= Current_Block->step_event_count;
#else
                E0_STEP_H;
                E_Error_Coefficient -= Current_Block->step_event_count;
#endif
            }
            Step_Event_Count_Completed++;
            X_STEP_L;
            Y_STEP_L;
            Z_STEP_L;
#ifdef BOARD_M301_Pro_S 
		if(nozzle_select == NOZZLE0)       
          E0_STEP_L;
      else if(nozzle_select == NOZZLE1)
	  E1_STEP_L;
      else if(nozzle_select == NOZZLE2)
          E2_STEP_L;
#else
            E0_STEP_L;
#endif
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
                    recovery_parameter.Current_sd_byte = Current_Block->sd_byte;
                    recovery_parameter.Current_point_x = Current_Block->point_x;
                    recovery_parameter.Current_point_y = Current_Block->point_y;
                    recovery_parameter.Current_point_z = Current_Block->point_z;
                    recovery_parameter.Current_point_e = Current_Block->point_e;
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
#ifdef BOARD_M301_Pro_S   
void Mixer_Init(void)
{
  mixer.max = Setting.mixer_ofp_max;
  mixer.min = Setting.mixer_ofp_min;

  mixer.rate[NOZZLE0] = mixer.max;   
  mixer.rate[NOZZLE1] = mixer.min;
  
#if defined(EXTRUDER_3IN1)  
  mixer.rate[NOZZLE2] = mixer.min;
#endif
  
  mixer.counts = 0;

}
void Color_change(u8 start_p,u8 end_p,float start_h, float end_h)
{
  float thickness;
  thickness = end_h - start_h;
  

  
  if(Current_Position[Z_AXIS] >= start_h)
  {
    if(end_p > start_p)  
    {
      mixer.rate[NOZZLE0] = start_p + (end_p - start_p)*((Current_Position[Z_AXIS] - start_h)/thickness);    
    }
    else if(end_p < start_p)
    {
      mixer.rate[NOZZLE0] = start_p - (start_p - end_p)*((Current_Position[Z_AXIS] - start_h)/thickness);
    }
                              
    if(mixer.rate[NOZZLE0] >= mixer.max) mixer.rate[NOZZLE0] = mixer.max;
    if(mixer.rate[NOZZLE0] <= mixer.min) mixer.rate[NOZZLE0] = mixer.min;
    mixer.rate[NOZZLE1] = 100 - mixer.rate[NOZZLE0];  
  }
  
  if(Current_Position[Z_AXIS] > end_h)
  {
    color_change_flag = 0; 
    return;
  }
}

float get_current_position(u8 axis)
{
  basic_axis_position[axis] = Current_Position[axis];
}

void color_control(void)
{
  u8 start_p,end_p;
  float start_h,end_h;
  switch(color_change_flag)
    {
      case 0:return;
      case TEMPLATE_1:
        start_p = (u8)mixer.max;
        end_p = (u8)mixer.min;
        start_h = basic_axis_position[Z_AXIS];
        end_h = start_h + 2;
        break;
      case TEMPLATE_2: 
        start_p = (u8)mixer.min;
        end_p = (u8)mixer.max;
        start_h = basic_axis_position[Z_AXIS];
        end_h = start_h + 2;
        break;  
      case TEMPLATE_3: 
        start_p = (u8)mixer.max;
        end_p = (u8)mixer.min;
        start_h = basic_axis_position[Z_AXIS];
        end_h = start_h + 10;
        break;          
      case TEMPLATE_4:
        start_p = (u8)mixer.min;
        end_p = (u8)mixer.max;
        start_h = basic_axis_position[Z_AXIS];
        end_h = start_h + 10;
        break; 
      case TEMPLATE_5: 
        start_p = (u8)mixer.max;
        end_p = (u8)mixer.min;
        start_h = basic_axis_position[Z_AXIS];
        end_h = start_h + 20;
        break;
      case TEMPLATE_6: 
        start_p = (u8)mixer.min;
        end_p = (u8)mixer.max;
        start_h = basic_axis_position[Z_AXIS];
        end_h = start_h + 20;
        break;   
      case TEMPLATE_7: 
        start_p = (u8)mixer.max;
        end_p = (u8)mixer.min;
        start_h = basic_axis_position[Z_AXIS];
        end_h = start_h + 50;
        break;
      case TEMPLATE_8: 
        start_p = (u8)mixer.min;
        end_p = (u8)mixer.max;
        start_h = basic_axis_position[Z_AXIS];
        end_h = start_h + 50;
        break;                     
      case CUSTOM_1 + 8: 
        start_p = (u8)Setting.custom_conf_start_percent[CUSTOM_1-1];
        end_p   = (u8)Setting.custom_conf_end_percent[CUSTOM_1-1];
        start_h =     Setting.custom_conf_start_height[CUSTOM_1-1];
        end_h   =     Setting.custom_conf_end_height[CUSTOM_1-1]; 
        break;
      case CUSTOM_2 + 8: 
        start_p = (u8)Setting.custom_conf_start_percent[CUSTOM_2-1];
        end_p   = (u8)Setting.custom_conf_end_percent[CUSTOM_2-1];
        start_h =     Setting.custom_conf_start_height[CUSTOM_2-1];
        end_h   =     Setting.custom_conf_end_height[CUSTOM_2-1];
        break;
      case CUSTOM_3 + 8: 
        start_p = (u8)Setting.custom_conf_start_percent[CUSTOM_3-1];
        end_p   = (u8)Setting.custom_conf_end_percent[CUSTOM_3-1];
        start_h =     Setting.custom_conf_start_height[CUSTOM_3-1];
        end_h   =     Setting.custom_conf_end_height[CUSTOM_3-1];
        break;
      case CUSTOM_4 + 8: 
        start_p = (u8)Setting.custom_conf_start_percent[CUSTOM_4-1];
        end_p   = (u8)Setting.custom_conf_end_percent[CUSTOM_4-1];
        start_h =     Setting.custom_conf_start_height[CUSTOM_4-1];
        end_h   =     Setting.custom_conf_end_height[CUSTOM_4-1];
        break;
      case CUSTOM_5 + 8: 
        start_p = (u8)Setting.custom_conf_start_percent[CUSTOM_5-1];
        end_p   = (u8)Setting.custom_conf_end_percent[CUSTOM_5-1];
        start_h =     Setting.custom_conf_start_height[CUSTOM_5-1];
        end_h   =     Setting.custom_conf_end_height[CUSTOM_5-1];
        break;
      case CUSTOM_6 + 8: 
        start_p = (u8)Setting.custom_conf_start_percent[CUSTOM_6-1];
        end_p   = (u8)Setting.custom_conf_end_percent[CUSTOM_6-1];
        start_h =     Setting.custom_conf_start_height[CUSTOM_6-1];
        end_h   =     Setting.custom_conf_end_height[CUSTOM_6-1];
        break;
    default: return;
    }
    Color_change(start_p, end_p, start_h, end_h);
}
#endif
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







u8 Max_Divisor(u8 a, u8 b)
{
  while ( a!=b)   
  {
   if (a>b)  a=a-b;      
   else  b=b-a;
  }
  return a;
}
#ifdef BOARD_M301_Pro_S   
static void Nozzle_Select(void)
{
  u8 div;
  if(mixer.rate[NOZZLE0] >= 100) 
  {
    nozzle_select = NOZZLE0;
    return;
  }
  else if(mixer.rate[NOZZLE1] >= 100) 
  {
    nozzle_select = NOZZLE1;
    return;
  }
  else if(mixer.rate[NOZZLE2] >= 100) 
  {
    nozzle_select = NOZZLE2;
    return;
  }  
  else 
  {
    if(mixer.rate[NOZZLE0] == 0) div = Max_Divisor(mixer.rate[NOZZLE1], mixer.rate[NOZZLE2]);
    else if(mixer.rate[NOZZLE1] == 0) div = Max_Divisor(mixer.rate[NOZZLE0], mixer.rate[NOZZLE2]);
    else if(mixer.rate[NOZZLE2] == 0) div = Max_Divisor(mixer.rate[NOZZLE0], mixer.rate[NOZZLE1]);
    else div = Max_Divisor(Max_Divisor(mixer.rate[NOZZLE0], mixer.rate[NOZZLE1]),mixer.rate[NOZZLE2]);          
    if(mixer.counts < mixer.rate[NOZZLE0]/div) 
    {
      nozzle_select = NOZZLE0;
      mixer.counts ++;
    }
    else if((mixer.rate[NOZZLE0]/div<=mixer.counts)&&(mixer.counts < (mixer.rate[NOZZLE0]+mixer.rate[NOZZLE1])/div))
    {
      nozzle_select = NOZZLE1; 
      mixer.counts ++;
    }
    else if(mixer.counts < 100/div) 
    {
      nozzle_select = NOZZLE2; 
      mixer.counts ++;
    }
    if(mixer.counts >= 100/div) mixer.counts = 0;
  }
}

DWORD Save_line_num(void)
{
  long res;
  res = Current_sd_line_num;
  return res;
}

DWORD Save_sd_block(void)
{
  long res;
  res = Current_sd_block;
  return res;
}
#endif

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
		distanceX = (int)(Setting.max_position[X_AXIS] - Current_Position[X_AXIS]);
		X_DIR_L;
	}
	else
	{
		distanceX =(int)Current_Position[X_AXIS];
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


