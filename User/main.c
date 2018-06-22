
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "sd.h"
#include "adc.h"
#include "mmc_sd.h"
#include "fat.h"
#include "usart1.h"
#include "diskio.h"
#include "fat_config.h"
#include "Dgus_screen.h"
#include "delay.h"
#include "XZK_Rule.h"
#include "setting.h"
#include "step_motor.h"
#include "command.h"
#include "endstop.h"
#include "rocker.h"
#include "LCD2004.h"
#include "string.h"
#include "stmflash.h"
#include "gta.h"
#include "wifi.h"
#include "sd_print.h"
#include "rxfile.h"
#include "serial_print.h"
#include "variable.h"
#include "data_handle.h"


u8    serial_connect_flag=DISABLE;


extern u8 serial_connect_flag3,WIFI_Bebug_Flag;

void Port_to_PC_init(void)
{
	USART1_Config();    //串口1初始化
	USART1_NVIC_Configuration();
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	Send_Info_To_Serial();		  //加挂SD卡。
}
void Port_to_Slave_init(void)
{
	NewUSART3_Config();  
	NewUSART3_NVIC_Configuration();
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
}

void Get_Store_Date(void)
{

	if(Exist_Wifi_Det()==0)
	{
        	if(WIFI_Read_AutoConnect_Flag()==1)
        	{
             		WIFI_Connect();
             		WIFI_MODE=WIFI_CONNECT_SERVER;
             		Wifi_ASK_Factors.WIFI_CONNECT_FLAG=1;
             		Wifi_Work_Message.WIFI_WORK_STATUS=1;
			Wifi_ASK_Factors.WIFI_RECONNECT_TIMES=4;
			Add_Message(WIFI_STATUS_MESSAGE);
              	sprintf(Printf_Buf,"<AUTO_UD:WFET:0;*>\r\n");
              	USART3_printf(Printf_Buf);
        	}
        	Setting.wifi_exist_flag =0;
        	//printf("wifi modle exist\r\n");
        }
        else
        {
		Setting.wifi_exist_flag =1;
		sprintf(Printf_Buf,"<AUTO_UD:WFET:1;*>\r\n");
              USART3_printf(Printf_Buf);
              delay_ms(100);
              sprintf(Printf_Buf,"<AUTO_UD:WFET:1;*>\r\n");
              USART3_printf(Printf_Buf);
		 //printf("wifi modle no exist\r\n");
        }
	 system_infor.Auto_Levele_Flag=Get_AutoLevele_Flag();
        system_infor.Filament_Dev_Flag=Get_Filament_Dev_Flag();
}
/*******************************************************
*系统基地址初始化2000
*
*
*******************************************************/
void System_baseaddr_Init(void)
{
	SystemInit();		     //系统时钟等初始化
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x2000);//app base addr
	asm("CPSIE  I");  //开中断
}
/*******************************************************
*硬件初始化
*
*
*******************************************************/
void  Hardware_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//只在此位置分一次组，其他初始化分组都屏蔽
	Get_Printer_User_Settings();
	Step_Motor_Init();  
	Step_Motor_Timer_Enable();
	Port_to_PC_init();
	Port_to_Slave_init();

	Filament_Init();

	Endstop_Init();
	Temperature_Measure_Init();
	Temperature_Control_Init();
	Temperature_Sampling_Timer_Config();
	Temperature_Sampling_NVIC_Configuration();
	Temperature_Measure_Enable();      
	Temperature_Control_Enable();


	Encoder_GPIO_Config();
	Encoder_EXTI_Config();
	Enable_Encoder_EXTI(); 

	Fan_Control_Enable();

	Voltage_detect(); 
	
	
	BLTouch_Init();

	Display_Update_Timer_Config();
        DGUS_Display_Timer_Config();

        SD_LowLevel_Init();
	
	Recovery_GPIO_Config();
	system_data_init();
}
/*******************************************************
*系统数据初始化
*
*
*******************************************************/
void SystemData_Init(void)
{

	
        Wifi_Init_Config();  
        NVIC_EnableIRQ(USART1_IRQn);
        NVIC_EnableIRQ(USART3_IRQn);
        NVIC_EnableIRQ(USART2_IRQn);

	Get_Store_Date();
	
	Add_Message(PRINTER_RESET);
       Set_Beep(1000,127);
       Set_Beep(1000,127);
       //Set_Beep(1000,127);
       delay_ms(100);
       Add_Message(PRINTER_RESET);
       Set_Beep(1000,127);
       Set_Beep(1000,127);
      // Set_Beep(1000,127);
       delay_ms(600);
      
      WIF_EXIST_TEST();
	
      Recovery_process();
        
}

void System_Init_Action(void)
{
	system_infor.feed_tare=100;
        Add_Message(PRINTER_MESSAGE);
}

void main(void)
{      
	System_baseaddr_Init();       //系统初始化
	Hardware_Init();              //硬件初始化 
	SystemData_Init();            //系统数据初始化
       Add_Message(PRINTER_START_MESSAGE);
       Sent_Boot_Binfile();
       System_Init_Action();
       printf("Setting.zprobe_zoffset = %f\r\n",Setting.zprobe_zoffset);

     
    while(1)
    {   
		
		if(Wifi_ASK_Factors.WIFI_CONNECT_FLAG!=0)
		{//taicvb
                  
			Wifi_Mode_Select();
			if(Wifi_ASK_Factors.WIFI_RECONNECT_TIMES==0)
		      {
                                 UARST_REDIRECT=UARST1_REDIRECT;
				      Wifi_ASK_Factors.WIFI_CONNECT_FLAG=0;
				      WIFI_Bebug_Flag=2;
                    }
		}
		if(serial_connect_flag == ENABLE)
		{
		    fetch_next_command();   
		}
		if(system_infor.sd_print_flag == ENABLE)
		{
			Print_3D_SD();
		}	
		SD_File_Detect();
		Main_Command_Handle();
               

    }       
                          
}





