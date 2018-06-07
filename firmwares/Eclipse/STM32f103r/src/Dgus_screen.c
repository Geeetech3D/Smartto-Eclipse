
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
#include "Dgus_screen.h"
#include "fat.h"
#include "sd_print.h"
#include "wifi.h"
#include "planner.h"
#include "variable.h"

static char Command_Buffer_befor[200];	//Gcode command buffer
char com_cpy[200];	//Gcode
extern char Command_Buffer[CMDBUF_SIZE];	//Gcode  command

u16 fan_current_speed;	//Current fan speed
#define ROUND_TO_UINT16(x)   ((u16)(x)+0.5)>(x)? ((u16)(x)):((u16)(x)+1) //Floating point rounding




float new_Z_Max_Position;//Z axis print range

#define SEND_BUFF_SIZE 0x28

u8 Leveling_SetMotor_flag = DISABLE;

extern float Current_Temperature[5];	
void Read_Filament_Default(void);


#ifdef BOARD_M301_Pro_S    






#endif

/**********************************************************
***Function:     DGUS_Display_Timer_Config
***Description:  
***Input:  
***Output: 
***Return:
***********************************************************/
void DGUS_Display_Timer_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	TIM_DeInit(TIM7);
	
	TIM_PrescalerConfig(TIM7,999,TIM_PSCReloadMode_Update);
	TIM_SetAutoreload(TIM7,3599);
	TIM_UpdateDisableConfig(TIM7,DISABLE);
	TIM_UpdateRequestConfig(TIM7,TIM_UpdateSource_Regular);
	TIM_ARRPreloadConfig(TIM7, ENABLE);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);  
	TIM_Cmd(TIM7, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


 void command_process(char* str)
{
    memset(Command_Buffer_befor,0,200);
    strncpy(Command_Buffer_befor, Command_Buffer,200);
    memset(Command_Buffer,0,200);
    strcpy(Command_Buffer,str);
    Processing_command();
    memset(Command_Buffer,0,200);
    strncpy(Command_Buffer,Command_Buffer_befor,200);
}


static void Motor_Move(char axis, char sign, u8 interval, u16 speed)
{
	u8 setting_locate_mode;
	setting_locate_mode = Setting.locate_mode;
    if(setting_locate_mode == ABSOLUTE_COORDINATE)
    {	strcpy(com_cpy,"G91\r\n");
    	 command_process(com_cpy);
    }
	 
    if(interval == 0) sprintf(com_cpy,"G1 %c%c0.05 F%d\r\n", axis, sign, speed);
    else if(interval == 1) sprintf(com_cpy,"G1 %c%c0.1 F%d\r\n", axis, sign, speed);
    else if(interval == 2) sprintf(com_cpy,"G1 %c%c1 F%d\r\n", axis, sign, speed);
    else if(interval == 3) sprintf(com_cpy,"G1 %c%c10 F%d\r\n", axis, sign, speed); 
    else if(interval == 4) sprintf(com_cpy,"G1 %c%c50 F%d\r\n", axis, sign, speed);
    else if(interval == 5) sprintf(com_cpy,"G1 %c%c100 F%d\r\n", axis, sign, speed);        
    command_process(com_cpy);
    //Processing_command();

	if(setting_locate_mode == ABSOLUTE_COORDINATE)
    {	strcpy(com_cpy,"G90\r\n");
    	 command_process(com_cpy);
    }
}




void Filament_Change(u8 i)
{
        if(Get_Motor_Status() == 0)
	{
		  command_process("M17\r\n");
               command_process("G28\r\n");
	}
         if(i==1)
             Motor_Move('E', '+', 3, 300);
         else
            Motor_Move('E', '-', 3, 900);
}

float  Input_Leveling_Page(void)
{
        if(system_infor.sd_print_status == SD_PRINT_IDLE && system_infor.serial_printf_flag != 1)
	{
#ifdef BOARD_A30_MINI_S
                new_Z_Max_Position = Setting.zz_offset;
                Setting.zz_offset=0.0;
                command_process("G28\r\n");
                Setting.zz_offset=new_Z_Max_Position;
		  sprintf(com_cpy,"G1 Z%.2f F800\r\n",Setting.zz_offset);         
            	  command_process(com_cpy);
#elif BOARD_E180_MINI_S
                new_Z_Max_Position = Setting.max_position[Z_AXIS];
                command_process("G28\r\n");
		  sprintf(com_cpy,"G1 Z0 F%d\r\n", Setting.max_feedrate[Z_AXIS]*60);         
            	  command_process(com_cpy);
		  Queue_wait();
#endif
		  Queue_wait();
		if(Get_Motor_Status() == 0)
		{
                        command_process("M17\r\n");
		}
                return new_Z_Max_Position ;
	}
	else
	{
	        return 0;    
	}
}
float space_values[6] = {10.0,1.0,0.1,0.05,0.05,0.5};
float Leveling_Page_Zadjust(u8 tt)
{
    if(tt==1)
    {
            Set_Beep(1000,127);
	    strcpy(com_cpy,"G92 Z0.5\r\n");         
            command_process(com_cpy);
	     strcpy(com_cpy,"G1 Z0\r\n");         
            command_process(com_cpy);
#ifdef BOARD_A30_MINI_S
            new_Z_Max_Position -= 0.5;
#elif BOARD_E180_MINI_S
            new_Z_Max_Position += 0.5;
#endif
	    
	    
        }
    else if(tt==0)
    {
            Set_Beep(1000,127);
	    strcpy(com_cpy,"G92 Z0\r\n");         
            command_process(com_cpy);
	    strcpy(com_cpy,"G1 Z0.5\r\n");         
            command_process(com_cpy);
	    strcpy(com_cpy,"G92 Z0\r\n");         
            command_process(com_cpy);
#ifdef BOARD_A30_MINI_S
            new_Z_Max_Position += 0.5;
#elif BOARD_E180_MINI_S
            new_Z_Max_Position -= 0.5;
#endif

        }
     else if(tt==2)
     {
	    Set_Beep(1000,127);
	    strcpy(com_cpy,"G92 Z0.05\r\n");         
            command_process(com_cpy);
	     strcpy(com_cpy,"G1 Z0\r\n");         
            command_process(com_cpy);
#ifdef BOARD_A30_MINI_S
	    new_Z_Max_Position -= 0.05;
#elif BOARD_E180_MINI_S
	    new_Z_Max_Position += 0.05;
#endif

	    
     }
     else
     {
	    Set_Beep(1000,127);
	    strcpy(com_cpy,"G92 Z0\r\n");         
            command_process(com_cpy);
	    strcpy(com_cpy,"G1 Z0.05\r\n");         
            command_process(com_cpy);
	    strcpy(com_cpy,"G92 Z0\r\n");         
            command_process(com_cpy);
#ifdef BOARD_A30_MINI_S
            new_Z_Max_Position += 0.05;
#elif BOARD_E180_MINI_S
            new_Z_Max_Position -= 0.05;
#endif

     }
    
    return new_Z_Max_Position;
}
float AutoLeveling_Page_Zadjust(u8 tt,u8 spaces )
{
	u8 temps= system_infor.Auto_Levele_Flag;
	system_infor.Auto_Levele_Flag=0;
    if(tt==1)
    {
            Set_Beep(1000,127);
	    sprintf(com_cpy,"G92 Z%.2f\r\n",space_values[spaces]);         
            command_process(com_cpy);
	     strcpy(com_cpy,"G1 Z0 F800\r\n");         
            command_process(com_cpy);
	    new_Z_Max_Position -= space_values[spaces];
	    
        }
    else if(tt==0)
    {
            Set_Beep(1000,127);
	    strcpy(com_cpy,"G92 Z0\r\n");         
            command_process(com_cpy);
	    sprintf(com_cpy,"G1 Z%2f F800\r\n",space_values[spaces]);         
            command_process(com_cpy);
	    strcpy(com_cpy,"G92 Z0\r\n");         
            command_process(com_cpy);
            new_Z_Max_Position += space_values[spaces];
        }
    system_infor.Auto_Levele_Flag = temps;
    return new_Z_Max_Position;
}
u8 Leveling_Page_ChoosePi(u8 num)
{
    float x=0.0,y=0.0;
    switch(num)
    {
#ifdef BOARD_A30_MINI_S
        case 1:
            x=0.8;y=0.8;
            break;
       case 2:
            x=0.2;y=0.8;
            break;
      case 3:
         x=0.2;y=0.2;
            break;
      case 4:
         x=0.8;y=0.2;
            break;
      case 5:
         x=0.5;y=0.5;
            break;
#elif BOARD_E180_MINI_S
        case 1:
            x=0.9;y=0.7;
            break;
       case 2:
            x=0.1;y=0.7;
            break;
      case 3:
         x=0.1;y=0.0;
            break;
      case 4:
         x=0.9;y=0.0;
            break;
      case 5:
         x=0.5;y=0.4;
            break;
#endif
        default :return -1 ;break;
    }
            Set_Beep(1000,127);
#ifdef BOARD_A30_MINI_S
	     sprintf(com_cpy,"G1 Z10 F500\r\n");         
            command_process(com_cpy);
            sprintf(com_cpy,"G1 X%.2f Y%.2f F1800\r\n", Setting.max_position[X_AXIS]*x, Setting.max_position[Y_AXIS]*y);       
            command_process(com_cpy);
	     sprintf(com_cpy,"G1 Z%.2f F500\r\n",new_Z_Max_Position); 
            command_process(com_cpy);
#elif BOARD_E180_MINI_S
	     sprintf(com_cpy,"G1 Z10 F%d\r\n", Setting.max_feedrate[Z_AXIS]*60);         
            command_process(com_cpy);
            sprintf(com_cpy,"G1 X%.2f Y%.2f F%d\r\n", Setting.max_position[X_AXIS]*x, Setting.max_position[Y_AXIS]*y, Setting.max_feedrate[X_AXIS]*60/4);       
            command_process(com_cpy);
	    sprintf(com_cpy,"G1 Z0 F%d\r\n", Setting.max_feedrate[Z_AXIS]*60); 
            command_process(com_cpy);
#endif
            Queue_wait();
            Set_Beep(1000,127);
            delay_ms(100);
            Set_Beep(1000,127);
            return 0;
}
void Leveling_Page_SaveZ(void)
{
#ifdef BOARD_A30_MINI_S
       Setting.zz_offset = new_Z_Max_Position;
       printf("Setting.zz_offset = %f\r\n",Setting.zz_offset);
#elif BOARD_E180_MINI_S
       Setting.max_position[Z_AXIS] = new_Z_Max_Position;
#endif
       strcpy(com_cpy,"M500\r\n");         
       command_process(com_cpy);
       Set_Beep(1000,127);
}
void Filament_Detector(u8 i)
{
   
}

void Motor_Disable_IT(void)
{
    if((Get_Motor_Status() !=0) && (system_infor.sd_print_flag != 2 && system_infor.serial_printf_flag != 1)&&(Leveling_SetMotor_flag == DISABLE))//如果sd卡, 串口打印标志和调平状态不成立，电机到时间就禁能
    {
        if(system_infor.system_status != PRINT&&(system_infor.sd_print_status!=SD_PRINT_PAUSE_F||system_infor.sd_print_status!=SD_PRINT_PAUSE))//&& Setting.hotend_enable[NOZZLE0] != ENABLE&&Setting.hotend_enable[BED] != ENABLE
        {
            strcpy(com_cpy,"M18\r\n");         
            command_process(com_cpy);
        }
    }

}
void Set_Motor_Flag(u8 ff)
{
	if(ff == 0)
		Leveling_SetMotor_flag =  DISABLE;
	else
		Leveling_SetMotor_flag =  ENABLE;
}




u8 get_filament_status(void)
{
    return 0;
}


