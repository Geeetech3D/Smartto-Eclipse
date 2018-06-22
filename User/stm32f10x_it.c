/**
  ******************************************************************************
  * @file    stm32_it.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "adc.h"
#include "sd.h"
#include "endstop.h"
#include "step_motor.h"
#include "LCD2004.h"
#include "delay.h"
#include "stm32f10x_gpio.h"
#include "rocker.h"
#include "XZK_Rule.h"
#include "planner.h"
#include "serial_print.h"
#include "wifi.h"
#include "serial_print.h"
#include "sd_print.h"
#include "recovery_print.h"
#include "Dgus_screen.h"
#include "variable.h"

int page_refresh_count;
u8 TransferError_flag=0;//传输出错时退出处理
static u8 E0_Moter_Constrol_Flag=0;

extern __IO SD_Error TransferError;     
extern u8 serial_connect_flag ,serial_connect_flag3 , serial_print_flag;
extern autohome_st Autohome;

static u32 System_time = 0;

extern u16 SDRead_TimeOut;//,SDRead_TimeOut1;//SD卡打印中暂停时间过长退出
extern u8 wifi_status;
static u8 Usart_Times=0;
static u8 Usart2_Times=0;
extern u16 uart_buff_start,uart_buff_start3;
extern u16 uart_buff_end,uart_buff_end3;
extern char uart_buff3[UART3_BUFF_SIZE];
extern u8 SD_detec_flag;
extern u8 Firmware_Updata_Flag;
void Set_E0_Motor_Flag(u8 n)
{
	E0_Moter_Constrol_Flag = n;
}
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                         */
/******************************************************************************/    
/*******************************************************************************
* Function Name  : NMI_Handler
* Description    : This function handles NMI exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NMI_Handler(void)
{
}

/*******************************************************************************
* Function Name  : HardFault_Handler
* Description    : This function handles Hard Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  	if(system_infor.sd_print_status== SD_PRINTING)
  	{
		Recovery_create();
	}
  	NVIC_SystemReset();
  }
}

/*******************************************************************************
* Function Name  : MemManage_Handler
* Description    : This function handles Memory Manage exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : BusFault_Handler
* Description    : This function handles Bus Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : UsageFault_Handler
* Description    : This function handles Usage Fault exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/*******************************************************************************
* Function Name  : SVC_Handler
* Description    : This function handles SVCall exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVC_Handler(void)
{
}

/*******************************************************************************
* Function Name  : DebugMon_Handler
* Description    : This function handles Debug Monitor exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DebugMon_Handler(void)
{
}

/*******************************************************************************
* Function Name  : PendSV_Handler
* Description    : This function handles PendSVC exception.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PendSV_Handler(void)
{
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*            STM32 Peripherals Interrupt Handlers                        */
/******************************************************************************/

/*******************************************************************************
* Function Name  : USB_HP_CAN1_TX_IRQHandler
* Description    : This function handles USB High Priority or CAN TX interrupts requests
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
void USB_HP_CAN1_TX_IRQHandler(void)
{
  //CTR_HP();
}
*/
/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB Low Priority interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  //printf("\r\n enter usb lp irq!");
 // USB_Istr();
}*/
/*******************************************************************************
* Function Name  : USB_FS_WKUP_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*

void USBWakeUp_IRQHandler(void)
{
//printf("\r\n enter usb wake up irq!");
 // EXTI_ClearITPendingBit(EXTI_Line18);
}
*/
/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                            */
/******************************************************************************/

/*******************************************************************************
* Function Name  : PPP_IRQHandler
* Description    : This function handles PPP interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*void PPP_IRQHandler(void)
{
}*/
extern u16 REV_FILE_TIME;
extern u16 Delay_Times_p;
extern int command_time;
u8  Rev_File_Flag=0;
/*1ms中断*/
void TIM6_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
  	/*温度控制处理*/
  	Temperature_Handler();
	REV_FILE_TIME++;
	Delay_Times++;
	SDRead_TimeOut++;
	//SDRead_TimeOut1++;
	APP_Refresh_flag.App_RefreshSatus_Times++;//app显示状态更新间隔
	APP_Refresh_flag.Mask_SDPrintButton_Times++;
	APP_Refresh_flag.App_Refresh_Times++;
       Delay_Times_p++;

	if(APP_Refresh_flag.Mask_SDPrintButton==ENABLE )//如果app传输完毕正准备打印,不执行打印操作
	{	
		if(APP_Refresh_flag.Mask_SDPrintButton_Times>20000)
		APP_Refresh_flag.Mask_SDPrintButton=DISABLE;	
	} 
	else APP_Refresh_flag.Mask_SDPrintButton=DISABLE;	

        if(File_DownST.File_Flag==1)
        {
          Usart_Times++;
          if(Usart_Times==5&&(uart_buff_end!=uart_buff_start))
             serial_connect_flag = ENABLE;
        }
        if(Rev_File_Flag==1)
        {
          Usart2_Times++;
          if(Usart2_Times==5&&(wifi_buff_end!=wifi_buff_start))
             serial2_complete_flag = ENABLE;
        }
}

/*50ms中断*/
extern u8 Uoload_LCDData_Flag;
extern u8 select_update_flag1,select_update_flag2;
void TIM7_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	
       System_time++;
       system_infor.Motor_Disable_Times++;
        if(system_infor.Motor_Disable_Times>2500)
       {
       	if(system_infor.Motor_Lock ==0)
       	{
          		 system_infor.Motor_Disable_Times=0;
          		 Motor_Disable_IT();
           	}
        }
     //Filement_Constol_Function(1);
	/*串口命令等待超时处理*/
  	if((command_time == 0) && (!Blocks_queued_status()))
  		serial_command_timeout();
  	else if((command_time > 0) && (command_time != 1200))//修正串口等待超时为1min,之前为54min
  		command_time--;
	if(Firmware_Updata_Flag==1)
	   return;
	
	Frash_Update_ToLCD();
	if(serial_connect_flag3== ENABLE)
	{
		 Display_command();
	}

	 if((System_time % 10) == 0 && Uoload_LCDData_Flag != 1)
	{
		Add_Upload_data();
		ADD_Item_to_LCD();
	}
	if((System_time % 100) == 0&&WIFI_MODE == 5)
	{
		Send_m105_ask();
		Send_Printer_State();
		
	}
}

void TIM5_IRQHandler(void)     //20160627
{    
    TIM_ClearITPendingBit(TIM5,TIM_IT_Update);

    
    page_refresh_count++;
    if(page_refresh_count >= 5000)
    {  
       page_refresh_count = 0;
       SD_Card_detect();
    
    }
   if(system_infor.sd_print_status !=SD_PRINTING ) 
   	Filement_Constol_Function(E0_Moter_Constrol_Flag);
   	
    Read_endstop();
    Read_filamentstatus();
}





void TIM1_UP_IRQHandler(void)   //传输文件死机时初始化SD卡，4s
{                      
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)//检查指定的TIM中断发生与否:TIM 中断源   
    {  	
    		SD_detec_flag = 1;
		SD_File_Detect();
		TransferError_flag++;
		if(TransferError_flag>=4)
		{TransferError_flag=0;
		  TransferError = SD_DATA_TIMEOUT;//防止死循环而作的超时处理,SD处理超时
			Wifi_Conf_Set("M2110 fail SD:Err\r\n");
			}
		
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);//清除TIMx的中断待处理位:TIM 中断源   
  
    }          
}  

/*基本1ms定时中断*/
void TIM8_UP_IRQHandler(void) 
{
	if((Autohome.flag[0]>0) || (Autohome.flag[1]>0) || (Autohome.flag[2]>0))
	{
	  Autohome.count++; 
	  All_axis_go_origin();
	}
	else if(system_infor.pause_flag == 1)
	{
	  TIM_ClearITPendingBit(TIM8,TIM_IT_Update);
	   if(system_infor.pause_time!=0)
              	system_infor.pause_time--;
	}     
	else Step_Motor_Control();
        
	TIM_ClearITPendingBit(TIM8,TIM_IT_Update);    //20160627
}

void GPIO_Disable(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;     
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |RCC_APB2Periph_GPIOE| RCC_APB2Periph_GPIOF| RCC_APB2Periph_GPIOG, ENABLE);
        
       GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	

}


void EXTI4_IRQHandler(void)
{ 
 	if(EXTI_GetITStatus(EXTI_Line4) != RESET)
	{ 
	    delay_us(2); 
	    if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)==1 && system_infor.sd_print_status == SD_PRINTING)
	    {
	    	      
		      Close_all_hotend();
		 	Recovery_create();
			//printf("EXTI4_IRQHandler\r\n");
			//Motor_move_away();
			NVIC_SystemReset();
	    }
		
	}

	EXTI_ClearITPendingBit(EXTI_Line4);
}

void EXTI15_10_IRQHandler(void)
{    
	
}

extern char uart_buff[UART_BUFF_SIZE];


void USART1_IRQHandler(void)
{
	char uart_ch;
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)
	{  
          
		uart_ch = USART_ReceiveData(USART1);
		if(((uart_buff_end + 1) % UART_BUFF_SIZE) == uart_buff_start)
			return;
               
                
		uart_buff[uart_buff_end] = uart_ch;
		uart_buff_end = (uart_buff_end + 1) % UART_BUFF_SIZE;
		if(File_DownST.File_Flag==1)
                  Usart_Times=0;
                else
                {
		   if(uart_ch == '\n' || uart_ch == '\r')
                   {
		 	serial_connect_flag = ENABLE;
                   }
                }
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);  
	}
     if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET)//注意！不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断  
    {  
       USART_ReceiveData(USART1);
    }  
}

#ifdef BOARD_GTM32_MINI_S
//void Deal_Command_Function(void)
//{
//	if()
//}

void USART3_IRQHandler(void)
{
	char uart_ch;
	if(USART_GetITStatus(USART3,USART_IT_RXNE) != RESET)
	{  
          
		uart_ch = USART_ReceiveData(USART3);
		if(((uart_buff_end3 + 1) % UART3_BUFF_SIZE) == uart_buff_start3)
			return;
               
         USART_SendData(USART1, (unsigned char) uart_ch);
         while (!(USART1->SR & USART_FLAG_TXE));
         
		uart_buff3[uart_buff_end3] = uart_ch;
		uart_buff_end3 = (uart_buff_end3 + 1) % UART3_BUFF_SIZE;

		   if(uart_ch == '\n' || uart_ch == '\r')
                   {
		 	     serial_connect_flag3 = ENABLE;
		 	     
                   }
		   
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);  
	}
     if (USART_GetFlagStatus(USART3, USART_FLAG_ORE) != RESET)//注意！不能使用if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)来判断  
    {  
       USART_ReceiveData(USART3);
    }  
}
#endif


void USART2_IRQHandler(void)
{
  char uart_ch;
    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET)
    {  
        USART_ReceiveData(USART2);  
    } 
   if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
   {
         uart_ch = USART_ReceiveData(USART2);
	 USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	 if(wifi_status==1)
	   return; 
	 
	  //USART_SendData(USART1, (unsigned char) uart_ch);
        //while (!(USART1->SR & USART_FLAG_TXE));
		 
        if(WIFI_MODE==WIFI_HANDLE_DATA)
        {
             if(((wifi_buff_end + 1) % 1280) == wifi_buff_start)
      	      {
      		    USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	 		return;
      	       }
         }
         uart2_buff[wifi_buff_end]=uart_ch;
         wifi_buff_end = (wifi_buff_end + 1) % 1280;
         if(Rev_File_Flag==1)
        {
              Usart2_Times=0;
         }
         else
         {
               if(uart_ch == '\n' || uart_ch == '\r' || uart_ch == 0)
    	           serial2_complete_flag = 1;
          } 	
     }
}
//#endif

void PVD_IRQHandler(void)   //PVD?D??×??a±?ó?±￡′?·?ê?￡?è·±￡′òó??D????3?±￡′?3é1|
{
  {
    if(EXTI_GetITStatus(EXTI_Line16) != RESET)
    {
      Close_all_hotend();
      Recovery_create();
      sprintf(Printf_Buf, "Save ok...\n");my_printf(Printf_Buf);
      //SET_BEEP(1000,255);
      Motor_move_away();    
      while(1);
    }
  }
  EXTI_ClearITPendingBit(EXTI_Line16);
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/





