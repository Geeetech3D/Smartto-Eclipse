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


#include "recovery_print.h"
#include "command.h"
#include "delay.h"
#include "usart1.h"
#include "fat.h"
#include "sd.h"
#include "mmc_sd.h"
#include "setting.h"
#include "adc.h"
#include "step_motor.h"
#include "LCD2004.h"
#include "rocker.h"
#include "stdlib.h"
#include "stdio.h"
#include "endstop.h"   //fdwong
#include "planner.h"
#include "wifi.h"
#include "variable.h"
#include "vector_3.h"
#include "data_handle.h"

#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)  



  

extern FATFS fats;
extern char SD_Driver_Number[5];
extern char Command_Buffer[CMDBUF_SIZE];
extern DWORD fptr_buf;
extern char SD_Path[512]; 
extern FIL Print_File;
RecoveryParameterType recovery_parameter;


typedef struct {
	float print_file_sz;
       DWORD  sd_byte; 
       float        xx;//4
       float      yy;
       float       zz;
       float      ee;
       float      feed_rate;
       float      targe_temp;
       float      targe_tempBED;
       char      file_path[70];
       
}Recovery_Data;

typedef struct {
	float print_file_sz;
       DWORD  sd_byte; 
       float        xx;//4
       float      yy;
       float       zz;
       float      ee;
       float      feed_rate;
       float      targe_temp;
       float      targe_tempBED;
       float     matrix[9];
       char      file_path[70];

}Recovery_AUTOData;

void Recovery_GPIO_Config(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;          
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure; 
#if (defined BOARD_A30_MINI_S) ||(defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S)   //#if (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode =    GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE,&GPIO_InitStructure);    
    EXTI_ClearITPendingBit(EXTI_Line4); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource4);   
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);  
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;// EXTI4_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
#elif BOARD_E180_MINI_S
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode =    GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC,&GPIO_InitStructure);    
    EXTI_ClearITPendingBit(EXTI_Line5); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource5);   
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);  
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;// EXTI4_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
#elif BOARD_M301_Pro_S
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE,&GPIO_InitStructure);    
     EXTI_ClearITPendingBit(EXTI_Line5); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource5);   
    EXTI_InitStructure.EXTI_Line = EXTI_Line5;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);  
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;// EXTI4_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
#endif
    NVIC_Init(&NVIC_InitStructure);

                       
}    


extern s32 position[4]; 
extern u8 SD_detec_flag;
extern u8 test_times;
extern u8  select_update_flag1,select_update_flag2;


void Recovery_sdprint(void)
{
	Recovery_Data Recovery_Save_Data;
	Recovery_AUTOData Recovery_SaveAUTOData;
	float feed_rate;
	float pos[4];
	u8 file_status ;
	//FIL fp;
	u8 res,i=0;
#ifdef ENABLE_AUTO_BED_LEVELING
   	if(system_infor.Auto_Levele_Flag ==1 )
   	{
   		system_infor.auto_leveling_calculate_enable=DISABLE;
   		matrix_3x3_set_to_identity(&Setting.plan_bed_level_matrix);
		STMFLASH_Read(RECOVERY_SAVE_ADDR,(u16*)(&Recovery_SaveAUTOData),sizeof(Recovery_SaveAUTOData)/2);  
		strcpy(Systembuf_Infos.printer_file_path, Recovery_SaveAUTOData.file_path);
		system_infor.print_file_size = Recovery_SaveAUTOData.print_file_sz;
		recovery_parameter.Current_sd_byte = Recovery_SaveAUTOData.sd_byte;
            if(system_infor.sd_print_status == 5) 
            {
		  Setting.targe_temperature[NOZZLE0] = Recovery_SaveAUTOData.targe_temp;
               Setting.targe_temperature[BED] =Recovery_SaveAUTOData.targe_tempBED;
            }
              feed_rate = Recovery_SaveAUTOData.feed_rate;
              pos[X_AXIS] = Recovery_SaveAUTOData.xx;
      		 pos[Y_AXIS] = Recovery_SaveAUTOData.yy;  
      		 pos[Z_AXIS] = Recovery_SaveAUTOData.zz;
      		 pos[E_AXIS] = Recovery_SaveAUTOData.ee;

      		recovery_parameter.Current_feed_rate = feed_rate;
      		recovery_parameter.Current_point_x = pos[X_AXIS];
      		recovery_parameter.Current_point_y = pos[Y_AXIS];  
      		recovery_parameter.Current_point_z = pos[Z_AXIS];
      		recovery_parameter.Current_point_e = pos[E_AXIS];
      		for(i=0;i<9;i++)
      		{
			Setting.plan_bed_level_matrix.matrix[i] = Recovery_SaveAUTOData.matrix[i];
      		}
      		system_infor.auto_leveling_calculate_enable=ENABLE;
      		 printf("new:path:%s;SZ:%f;bt:%d;nozz:%f;BED:%f;FR:%f;X:%f;Y:%f;Z:%f;E:%f\r\n",Recovery_SaveAUTOData.file_path,Recovery_Save_Data.print_file_sz,Recovery_SaveAUTOData.sd_byte,\
            Recovery_SaveAUTOData.targe_temp,Recovery_SaveAUTOData.targe_tempBED,Recovery_SaveAUTOData.feed_rate,Recovery_SaveAUTOData.xx,Recovery_SaveAUTOData.yy,\
            Recovery_SaveAUTOData.zz,Recovery_SaveAUTOData.ee);
   	}
   	else
#endif
   	{
		STMFLASH_Read(RECOVERY_SAVE_ADDR,(u16*)(&Recovery_Save_Data),sizeof(Recovery_Save_Data)/2);  
		strcpy(Systembuf_Infos.printer_file_path, Recovery_Save_Data.file_path);
		system_infor.print_file_size = Recovery_Save_Data.print_file_sz;
		recovery_parameter.Current_sd_byte = Recovery_Save_Data.sd_byte;
              if(system_infor.sd_print_status == 5) 
              {
		        Setting.targe_temperature[NOZZLE0] = Recovery_Save_Data.targe_temp;
                    Setting.targe_temperature[BED] =Recovery_Save_Data.targe_tempBED;
              }
              feed_rate = Recovery_Save_Data.feed_rate;
              pos[X_AXIS] = Recovery_Save_Data.xx;
      		 pos[Y_AXIS] = Recovery_Save_Data.yy;  
      		 pos[Z_AXIS] = Recovery_Save_Data.zz;
      		 pos[E_AXIS] = Recovery_Save_Data.ee;
          
      		recovery_parameter.Current_feed_rate = feed_rate;
      		recovery_parameter.Current_point_x = pos[X_AXIS];
      		recovery_parameter.Current_point_y = pos[Y_AXIS];  
      		recovery_parameter.Current_point_z = pos[Z_AXIS];
      		recovery_parameter.Current_point_e = pos[E_AXIS];
      		   printf("old:path:%s;SZ:%f;bt:%d;nozz:%f;BED:%f;FR:%f;X:%f;Y:%f;Z:%f;E:%f\r\n",Recovery_Save_Data.file_path,Recovery_Save_Data.print_file_sz,Recovery_Save_Data.sd_byte,\
            Recovery_Save_Data.targe_temp,Recovery_Save_Data.targe_tempBED,Recovery_Save_Data.feed_rate,Recovery_Save_Data.xx,Recovery_Save_Data.yy,\
            Recovery_Save_Data.zz,Recovery_Save_Data.ee);
      		
   	}
      	fptr_buf = recovery_parameter.Current_sd_byte;
      if(system_infor.sd_print_status != 5) 
        return;
      if(Print_File.fs!=NULL)
		f_close(&Print_File);
      do
      {
      		file_status = Recovery_file_search(Systembuf_Infos.printer_file_path, system_infor.print_file_size);
             if(file_status != 0)
             {
                    res = Recovery_error(); 
                    if(res == DISABLE)  
    	  	      {
    			      Recovery_remove();
                          system_infor.sd_print_status = SD_PRINT_FINISH;    //20160628
                          printf("SD_PRINT_FINISH 2\r\n");

    			      return;
          	      }
             } 
      }while(file_status);
      
       system_infor.sd_print_flag = ENABLE;         
      system_infor.system_status = PRINT;
      TIM_Cmd(TIM5, ENABLE); 
      plan_init();
      if(recovery_parameter.Current_sd_byte != 0)
      {
     		system_infor.sd_print_status = SD_PRINT_IDLE;
  		
  		memset(Command_Buffer,0,CMDBUF_SIZE);
  		system_infor.sd_print_status = SD_PRINT_PAUSE;
	       sprintf(Command_Buffer,"M104 S%2f\r\n",Setting.targe_temperature[NOZZLE0]); 
		Processing_command();
  		sprintf(Command_Buffer,"M190 S%2f\r\n",Setting.targe_temperature[BED]);  
  		Processing_command();
             system_infor.stop_flag = 0;
		sprintf(Command_Buffer,"M109 S%2f\r\n",Setting.targe_temperature[NOZZLE0]); 
		Processing_command();
		if(system_infor.stop_flag==1)
			return;
  		strcpy(Command_Buffer,"G90\r\n");
  		Processing_command();
#if (defined BOARD_A30_MINI_S) || (defined BOARD_A30M_Pro_S) || (defined BOARD_A30D_Pro_S) 
  		if(recovery_parameter.Current_point_z <10 && system_infor.Auto_Levele_Flag !=1)
  		{

  			Current_Position[Z_AXIS]=0;
  			strcpy(Command_Buffer,"G1 F500 Z5\r\n");
  			Processing_command();
			strcpy(Command_Buffer,"G28\r\n");
  			Processing_command();
  			strcpy(Command_Buffer,"G1 F500 Z10\r\n");
  			Processing_command();
  		}
  		else
  		{
  			strcpy(Command_Buffer,"G28 XY\r\n");
  			Processing_command();
#ifdef ENABLE_AUTO_BED_LEVELING
  			if(system_infor.Auto_Levele_Flag ==1 )
			{
			        Current_Position[Z_AXIS] =recovery_parameter.Current_point_z;//pos[Z_AXIS];
		  	       position[Z_AXIS]= (s32)(recovery_parameter.Current_point_z*Setting.steps_per_mm[Z_AXIS]) ;
			}
			else
#endif
			{
			  	test_times=3;
		  	  	Current_Position[Z_AXIS] =pos[Z_AXIS];
		  	       position[Z_AXIS]= (s32)( pos[Z_AXIS]*Setting.steps_per_mm[Z_AXIS]) ;
		  	  
			}
  		}
#elif BOARD_E180_MINI_S
            strcpy(Command_Buffer,"G28\r\n");
            Processing_command();
            sprintf(Command_Buffer,"G1 F%d Z%2f\r\n",Setting.max_feedrate[Z_AXIS]*40,pos[Z_AXIS]);//Setting.max_feedrate[Z_AXIS]*60,
            Processing_command();
#endif		
		Enable_all_Axis();
  
		sprintf(Command_Buffer,"G92 E0\r\n");
		Processing_command();
		sprintf(Command_Buffer,"G1 E8 F350\r\n");
		Processing_command();
		sprintf(Command_Buffer,"G1 E6 F350\r\n");
		Processing_command();
		sprintf(Command_Buffer,"G92 E0\r\n");
		Processing_command();
		
	     
#ifdef ENABLE_AUTO_BED_LEVELING
		if(system_infor.Auto_Levele_Flag ==1 )
		{
	  		system_infor.Auto_Levele_Flag=0;
	  		sprintf(Command_Buffer,"G1 F800 X%2f Y%2f E3\r\n", pos[X_AXIS], pos[Y_AXIS]);
	  		Processing_command();

	  		system_infor.Auto_Levele_Flag=1;
	  		test_times=3;
		}
		else
#endif
		{
	  		sprintf(Command_Buffer,"G1 F1200 X%2f Y%2f Z%2f E3\r\n", pos[X_AXIS], pos[Y_AXIS], pos[Z_AXIS]);
	  		Processing_command();
	  		
	 	 }
		sprintf(Command_Buffer,"G92 E%2f\r\n",pos[E_AXIS]);
		Processing_command();

      }
      memset(Command_Buffer,0,CMDBUF_SIZE);  

      
#ifdef ENABLE_AUTO_BED_LEVELING
    if(system_infor.Auto_Levele_Flag ==1 )
    {
        Current_Position[Z_AXIS] =recovery_parameter.Current_point_z;//pos[Z_AXIS];
        apply_rotation_xyz(Setting.plan_bed_level_matrix, &recovery_parameter.Current_point_x, &recovery_parameter.Current_point_y, &recovery_parameter.Current_point_z);
        position[Z_AXIS]= (s32)(recovery_parameter.Current_point_z*Setting.steps_per_mm[Z_AXIS]) ;
    }
    else
#endif
    {
        test_times=3;
        Current_Position[Z_AXIS] =pos[Z_AXIS];
        position[Z_AXIS]= (s32)( pos[Z_AXIS]*Setting.steps_per_mm[Z_AXIS]) ;

    }

      sprintf(Command_Buffer,"G1 F3000\r\n");
      Processing_command();

      	if(system_infor.stop_flag==1)
		return;
      
      system_infor.sd_print_status = SD_PRINT_START;    //20160628sd_print_status
      Print_3D_SD();
      
      Add_Message(PRINTING_STATUS);
      Recovery_remove();
      STMFLASH_Erase(RECOVERY_SAVE_FLAG_ADDR,150);
      delay_ms(500);
}


u8 Recovery_file_search(char *path, float file_size)
{
  FIL fp;
  u8 sd_st,r;
  SD_Select_Init();
  for(u8 i=0;i<10;i++)
  {
    sd_st = f_mount(&fats,SD_Driver_Number,1);
    if(sd_st == FR_OK) break;
  }
  if(sd_st == FR_OK)
  {
    r = f_open(&fp,path,FA_OPEN_EXISTING | FA_READ);    
	if(fabs(file_size - (float)f_size(&fp)) < 1)
    {
        f_close(&fp);
        return r;
    }    
    f_close(&fp);
  }
  return 0xFF; 
}


u8 Recovery_detect(void)
{
  u16 r;
  STMFLASH_Read(RECOVERY_SAVE_FLAG_ADDR,&r,1);   
  return r;
}
void Recovery_remove(void)
{
  //u16 write_flag = 0xFF;
  STMFLASH_Erase(RECOVERY_SAVE_FLAG_ADDR,256);
  //STMFLASH_Write(RECOVERY_SAVE_FLAG_ADDR,&write_flag,1);
   
}

void Recovery_process(void)
{
  u8 res;
  u16 i=0,temp=0;
  res = Recovery_detect();
  if(res == FR_OK)
  {
        Recovery_sdprint();
        system_infor.sd_print_status=SD_PRINT_PAUSE;
        Add_Message(PRINTER_SD_STATUS);
       
      
  }
  else
  {
  	for(i=0;i<256;i++)
  	{
		STMFLASH_Read(RECOVERY_SAVE_FLAG_ADDR+i*2, &temp,1);
		if(temp!=0xFFFF)
			break;
		if(i==255)
			return;
  	}
  	STMFLASH_Erase(RECOVERY_SAVE_FLAG_ADDR,256);
  	printf("Recovery_detect:%d \r\n",res);
  }
 
}
void Recovery_create(void)
{

  u16 write_flag = 0;
  u8 i=0;
  Recovery_Data Recovery_Save_Data;
  Recovery_AUTOData Recovery_SaveAUTOData;
  CLI();
   if(system_infor.Auto_Levele_Flag ==1 )
  {
		  Recovery_SaveAUTOData.print_file_sz = system_infor.print_file_size;
               Recovery_SaveAUTOData.sd_byte = recovery_parameter.Current_sd_byte;
               Recovery_SaveAUTOData.xx = recovery_parameter.bed_calulate_Current_point_x;
               Recovery_SaveAUTOData.yy = recovery_parameter.bed_calulate_Current_point_y;
               Recovery_SaveAUTOData.zz = recovery_parameter.bed_calulate_Current_point_z;
               Recovery_SaveAUTOData.ee = recovery_parameter.Current_point_e;
               Recovery_SaveAUTOData.feed_rate = recovery_parameter.Current_feed_rate;
               Recovery_SaveAUTOData.targe_temp = Setting.targe_temperature[NOZZLE0];
               Recovery_SaveAUTOData.targe_tempBED = Setting.targe_temperature[BED];
               for(i=0;i<9;i++)
               {
			Recovery_SaveAUTOData.matrix[i] = Setting.plan_bed_level_matrix.matrix[i];
               }
               strcpy(Recovery_SaveAUTOData.file_path,Systembuf_Infos.printer_file_path);
              STMFLASH_Write2(RECOVERY_SAVE_ADDR,(u16*)(&Recovery_SaveAUTOData),sizeof(Recovery_SaveAUTOData)/2);
  }
  else
  {
               Recovery_Save_Data.print_file_sz = system_infor.print_file_size;
               Recovery_Save_Data.sd_byte = recovery_parameter.Current_sd_byte;
               Recovery_Save_Data.xx = recovery_parameter.Current_point_x;
               Recovery_Save_Data.yy = recovery_parameter.Current_point_y;
               Recovery_Save_Data.zz = recovery_parameter.Current_point_z;
               Recovery_Save_Data.ee = recovery_parameter.Current_point_e;
               Recovery_Save_Data.feed_rate = recovery_parameter.Current_feed_rate;
               Recovery_Save_Data.targe_temp = Setting.targe_temperature[NOZZLE0];
               Recovery_Save_Data.targe_tempBED = Setting.targe_temperature[BED];
               strcpy(Recovery_Save_Data.file_path,Systembuf_Infos.printer_file_path);
              STMFLASH_Write2(RECOVERY_SAVE_ADDR,(u16*)(&Recovery_Save_Data),sizeof(Recovery_Save_Data)/2); 
            

      }
      write_flag=0;
      STMFLASH_Write2(RECOVERY_SAVE_FLAG_ADDR,&write_flag,1);
      SEI() ;
}


void Clear_Save_Parameter(void)
{
	recovery_parameter.Current_sd_byte = 0;
	recovery_parameter.Current_feed_rate = 0.0;
	recovery_parameter.Current_point_x = 0.0;
	recovery_parameter.Current_point_y = 0.0;
	recovery_parameter.Current_point_z = 0.0;
	recovery_parameter.Current_point_e = 0.0;
	recovery_parameter.bed_calulate_Current_point_x=0.0;
	recovery_parameter.bed_calulate_Current_point_y=0.0;
	recovery_parameter.bed_calulate_Current_point_z=0.0;
	memset(recovery_parameter.Current_position_block,0,sizeof(recovery_parameter.Current_position_block));
}
void get_file_disname(void)
{
}
void PVD_Init(void)
{
  /* Enable PWR and BKP clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);//

  /* Configure EXTI Line to generate an interrupt on falling edge */
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Configure EXTI Line16(PVD Output) to generate an interrupt on rising and
     falling edges */
  EXTI_ClearITPendingBit(EXTI_Line16); 
  EXTI_InitStructure.EXTI_Line = EXTI_Line16;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* NVIC configuration */
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
  
  /* Enable the PVD Interrupt */ 
  NVIC_InitStructure.NVIC_IRQChannel = PVD_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  /* Configure the PVD Level to 2.9V */
  PWR_PVDLevelConfig(PWR_PVDLevel_2V9);

  /* Enable the PVD Output */
  PWR_PVDCmd(ENABLE);
}

