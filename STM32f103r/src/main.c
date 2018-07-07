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
#include "recovery_print.h"

/*

#ifdef BOARD_M301_Pro_S

#else

    #ifdef BOARD_A30_MINI_S

    #elif BOARD_E180_MINI_S

    #endif

#endif
*/


#ifdef BOARD_M301_Pro_S
u8 System_message=0;	
const char *machine_name = "GTM32 3D Controller";

#else

    #ifdef BOARD_A30_MINI_S

    #elif BOARD_E180_MINI_S

    #endif

#endif

u8    serial_connect_flag=DISABLE;
extern u8 serial_connect_flag3,WIFI_Bebug_Flag;
extern  char Command_Buffer[CMDBUF_SIZE];
char dir_name[10][50]={""};//each directory layer's name
extern u8 Firmware_Updata_Flag;
	

/**********************************************************
***Function:     Port_to_PC_init
***Description: USART1 initialization
***Output: 
***Return:
***********************************************************/
void Port_to_PC_init(void)
{
    USART1_Config();    
    USART1_NVIC_Configuration();
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    Send_Info_To_Serial();		  
}
/**********************************************************
***Function:     Port_to_Slave_init
***Description: USART3 initialization
***Input:  

***Output: 
***Return:
***********************************************************/
void Port_to_Slave_init(void)
{
    NewUSART3_Config();  
    NewUSART3_NVIC_Configuration();
    USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
}

/**********************************************************
***Function:     LCD_Init
***Description: LCD Init  M301,LCD2004
***Input:  
***Output: 
***Return:
***********************************************************/
void LCD_Init(void)
{
	LCD_2004_Init();  
	Display_Update_Timer_Config();
}

/**********************************************************
***Function:     Get_Store_Date
***Description:  WiFi/auto leveling/filament testing is turned on?
***Input:  
***Output: 
***Return:
***********************************************************/
void Get_Store_Date(void)
{
#ifdef WIFI_MODULE
    if(Exist_Wifi_Det()==0)  //Detect whether the WiFi module exists
    {
        if(WIFI_Read_AutoConnect_Flag()==1)
        {
            WIFI_Connect();
            WIFI_MODE=WIFI_CONNECT_SERVER;
            Wifi_ASK_Factors.WIFI_CONNECT_FLAG=1;
            Wifi_Work_Message.WIFI_WORK_STATUS=1;
            Wifi_ASK_Factors.WIFI_RECONNECT_TIMES=7;
            WIFI_Bebug_Flag=1;
            Add_Message(WIFI_DEBUG_MESSAGE);
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
#endif
    
#ifdef BOARD_A30_MINI_S
    system_infor.Auto_Levele_Flag=Get_AutoLevele_Flag();
    system_infor.Filament_Dev_Flag=Get_Filament_Dev_Flag();
#endif
}
/*******************************************************
*set app base addr 2000
*
*
*******************************************************/
void System_baseaddr_Init(void)
{
    SystemInit();		     //The system clock initialization
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x2000);//app base addr
    asm("CPSIE  I");  
}

/**********************************************************
***Function:     Hardware_Init
***Description: Peripheral initialization
***Input:  
***Output: 
***Return:
***********************************************************/
void  Hardware_Init(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Get_Printer_User_Settings();   //The initialization 3D printer parameter
    Step_Motor_Init();                  //Stepper motor control pin initialization  //
    Step_Motor_Timer_Enable();   //Control motor motion timer initialization
    Port_to_PC_init();                  //Serial port 1 initialization  //
#ifdef BOARD_M301_Pro_S    
    LCD_Init();
#else
    Port_to_Slave_init();             //Serial port 3 initialization   LCD
#endif

#ifdef BOARD_A30_MINI_S
    Filament_Init();                     //
#endif

    Endstop_Init();        //Limit switch pin initialization
    Temperature_Measure_Init();  // Temperature detection pin initialization
    Temperature_Control_Init();   //Temperature control pin initialization
    Temperature_Sampling_Timer_Config();   //Timing reading temperature timer initialization
    Temperature_Sampling_NVIC_Configuration();
    Temperature_Measure_Enable();      //Enable temperature acquisition
    Temperature_Control_Enable();     //Enable temperature control
#ifdef BOARD_M301_Pro_S   
    Delta_Init();  //Delta parameter initialization
    Mixer_Init();  //Three input and one parameter initialization
    	Encoder_GPIO_Config(); //LCD2004 knob pin initialization
	Encoder_EXTI_Config();  //
	Enable_Encoder_EXTI(); 
	page_init();                     //LCD2004
       PVD_Init();
#endif
    Fan_Control_Enable();    //Fan control pin initialization
    Voltage_detect();   


#ifndef BOARD_M301_Pro_S    
    Display_Update_Timer_Config();  //Timer initialization
    DGUS_Display_Timer_Config();   //Timer initialization
#endif


    SD_LowLevel_Init();  //SD card initialization

    Recovery_GPIO_Config();  //Power off detection pin initialization
    system_data_init();
}
/**********************************************************
***Function:     SystemData_Init
***Description: system data Init
***Input:  
***Output: 
***Return:
***********************************************************/
void SystemData_Init(void)
{
#ifdef WIFI_MODULE
    Wifi_Init_Config();      //wifi init
    NVIC_EnableIRQ(USART2_IRQn);
#endif

#ifndef BOARD_M301_Pro_S
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
    Get_Store_Date();

    Add_Message(PRINTER_RESET);

    Set_Beep(1000,127);
    Set_Beep(1000,127);
    delay_ms(100);
    Add_Message(PRINTER_RESET);
    Set_Beep(1000,127);
    Set_Beep(1000,127);
    delay_ms(600);
    
#ifdef WIFI_MODULE
    WIF_EXIST_TEST();   //Get WiFi version
#endif
    Recovery_process();  //Continue printing gcode file processing
#else
    NVIC_EnableIRQ(USART1_IRQn);
#endif
}

void System_Init_Action(void)
{
    system_infor.feed_tare=100;
    Add_Message(PRINTER_MESSAGE);
}
/**********************************************************
***Function:     main
***Description: Main task
***Input:  
***Output: 
***Return:
***********************************************************/
void main(void)
{      
    System_baseaddr_Init();       //The system clock initialization
    Hardware_Init();                  //Peripheral initialization
    SystemData_Init();               //Initialize the system data
      Add_Message(PRINTER_START_MESSAGE);  //  Upload printer information to LCD screen
    // Sent_Boot_Binfile();   //Firmware upgrade handle function
   System_Init_Action();
   // printf("Setting.zprobe_zoffset = %f\r\n",Setting.zprobe_zoffset);
   SD_File_Detect();
    while(1)
    {   

#ifdef WIFI_MODULE  
        if(Wifi_ASK_Factors.WIFI_CONNECT_FLAG!=0 && Firmware_Updata_Flag!=1) //WiFi processing
        {

            Wifi_Mode_Select();    //wifi module Data processing function
            if(Wifi_ASK_Factors.WIFI_RECONNECT_TIMES==0)  //Connection failure
            {
                UARST_REDIRECT=UARST1_REDIRECT;
                Wifi_ASK_Factors.WIFI_CONNECT_FLAG=0;
                WIFI_Bebug_Flag=2;
                Add_Message(WIFI_DEBUG_MESSAGE);
            }
        }
#endif

#ifdef BOARD_M301_Pro_S
      if(System_message == 1) //Home command
      {
        strcpy(Command_Buffer,"G28\r\n");
        Processing_command();
        System_message = 0;
      }
#endif

        if(serial_connect_flag == ENABLE)  //Serial command analysis function
        {
#ifndef BOARD_M301_Pro_S
            Serial_command_Analyze();
#else
            fetch_next_command();    
#endif
        }
        
        if(system_infor.sd_print_flag == ENABLE)  //SD card gcode file  print processing function
        {
            Print_3D_SD();
        }	

#ifndef BOARD_M301_Pro_S
          SD_File_Detect();    //Support SD card hot swap
         Main_Command_Handle(); //Call gcode command
#endif
    }       
}





