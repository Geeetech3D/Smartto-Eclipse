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
 * Geeetech?¡¥s Smartto dual license offers users a protective and flexible way to maximize their innovation and creativity.  
 * Smartto aims to be applicable to as many control boards and configurations as possible. 
 * Meanwhile we encourage the community to be active and pursuing the spirits of sharing and mutual help. 
 * The GPL v2 license grants complete use of Smartto to common users. These users are not distributing proprietary modifications or derivatives of Smartto. 
 * There is no need for them to acquire the legal protections of a commercial license.
 * For other users however, who want to use Smartto in their commercial products or have other requirements that are not compatible with the GPLv2, the GPLv2 is not applicable to them.
 * Under this condition, Geeetech, the exclusive licensor of Smartto, offers Smartto commercial license to meet these needs. 
 * A Smartto commercial license gives customers legal permission to modify Smartto or incorporate it into their products without the obligation of sharing the final code under the 
 * GPL v2 license. 
 * Fees vary with the application and the scale of its use. For more detailed information, please contact the Geeetech marketing department directly.
 * Geeetech commits itself to promoting the open source spirit. 
*/

/************************************************************
***GEEETECH 3D printer Firmware
***wifi.c  for wifi module
***
***
***
***
***
***********/


#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include "sd_print.h"
#include "variable.h"
#include "data_handle.h"
#include "rxfile.h"
#include "wifi.h"


FILE_DOWNLOAD File_DownST2;
extern FILE_DOWNLOAD File_DownST;
#ifdef WIFI_MODULE

u8 WIFI_Bebug_Flag=0;
u8 WIFI_MODE=0;
char Wifi_Server_message[128]="connect to TP-LINK_3928:JIETAI007;17S626D2000000;www.geeetech.net;\r\n";
char Wifi_Login_Message[128]="17S626D2000000;login\r\n";
#ifndef BOARD_M301_Pro_S 
char uart2_buff[1280]={0};
//
u16 usart2_num=0;
u16 Wifi_REV_num=0;
u8 serial2_complete_flag=0;
u16 serial2_complete_Times=0;


u16 wifi_buff_start = 0;
u16 wifi_buff_end = 0;
static int wifi_count = 0;

static int wifi_bufwrite_count = 0;
static int wifi_bufread_count = 0;
static char wifi_char;
static bool wifi_comment_mode = false;

char wifi_cmdbuffer[WIFI_BUFSIZE][WIFIMAX_CMD_SIZE];
int wifi_bufindr = 0;    //read buffer 
int wifi_bufindw = 0;   //write buffer 
extern char Command_Buffer[CMDBUF_SIZE];

u8 Wifi_Using_Flag=0;                 //

extern char  SD_Path[512];   
extern u8 dir_index;
extern char  sd_file_name[FILE_NUM][FILE_NAME_SIZE];//sd_file_name[56][50];
extern char Printf_Buf[128];
char Webserver_name[22];//={"www.geeetech.net\r\n"};
WIFI_ASK_FACTOR Wifi_ASK_Factors ={0,0,0,0,0,5,NULL}; 
WIFI_MESSAGE Wifi_Work_Message;

FILE_DOWNLOAD File_DownST2;
APP_REFRESH APP_Refresh_flag ={
	.App_RefreshSatus_Times=0 ,
	.Mask_SDPrintButton=DISABLE,
	.Mask_SDPrintButton_Times=0,
	.App_Sendfiles_FLAG=DISABLE,
	.App_sendfiles_dirmun=0,
	.App_Refresh_Times=0,
	.App_Refresh_FLAG=DISABLE,
	.wifi_Recovery_process_FLAG=0
};


u8 wifi_status=0;/*0:idle;1:reset */
extern u8  Rev_File_Flag;
 char WIFIDATA_Buffer[1280];
u8 Printer_3d_Statu=0;
extern char WF_version[6];



  void Init_Usart2_arg(void);
  FRESULT set_timestamp (
    char *obj,     /* Pointer to the file name */
    int year,
    int month,
    int mday,
    int hour,
    int min,
    int sec
);


void WDGOpen_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

	TIM_TimeBaseStructure.TIM_Period = 49999;      // 4s Interruption occurs   40000*7200/72000000
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;	   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	//Set the priority
  	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;    
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   //pre-emption priority 
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;             //subpriority level
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  	NVIC_Init(&NVIC_InitStructure);   
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);// Clears the TIMx's pending flags.
	TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);
	TIM_Cmd(TIM1, DISABLE);
	
}


/*****wifi Init**************************/
void Wifi_Init_Config(void)
{

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);
    GPIO_InitStructure.GPIO_Pin = WIFI_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(WIFI_GPIO, &GPIO_InitStructure);    
    GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN);
	
    USART2_Config();
    WDGOpen_Init();
}

/********************wifi server connect*************************************/
void Wifi_Conf_Set(char *config_message)
{
	USART2_TxString((u8*)config_message);
}
/********************wifi module reset************************************************************/
u8 Wifi_Reset(void)
{
        static u8 select_flag=0;
        static u16 Save_Delay_Time;
        switch(select_flag)
        {
            case 0:
                  Save_Delay_Time=Get_CDelay();
                  GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN);
                  select_flag=1;
            break;
            case 1:
                  if(Get_Delay_ms(Save_Delay_Time)>2000)
                  {
                       select_flag=2;
                       Save_Delay_Time=Get_CDelay();
                       GPIO_SetBits(WIFI_GPIO,WIFI_GPIO_PIN);
                  }
            break;
            case 2:
                  if(Get_Delay_ms(Save_Delay_Time)>1000)
                  {
                      select_flag=0;
                       Init_Usart2_arg();
                      return 1;
                  }
            break;
        }
        return 0;
}



/*******************************get ip addr**************************************************/

void Get_Locality_ip(void)
{
#ifndef BOARD_M301_Pro_S 
  u8 i=0;
  u8 num_sp=0;
        char *sp=strstr(uart2_buff,"get ip:");
        if(sp!=NULL)
        {
              num_sp=7;
        }
        if(sp!=NULL)
        {
          sp+=num_sp;
          
          while((*sp!='\n')&&(*sp!=',')&&(*sp!=';')&& i<19)
          {
              
              Wifi_Work_Message.Router_IP[i++]=*sp;
              sp++;
          }
          Wifi_Work_Message.Router_IP[i++]='\0';
        }
        else
       {
          sprintf(Wifi_Work_Message.Router_IP,"%s","x.x.x.x");
       }
	Add_Message(WIFI_IP_MESSAGE);
	Add_Message(WIFI_IP_MESSAGE);
#endif  
}

/*********************Execute wifi command**************************/
void wifi_CommandExecute(void)
{	
    UARST_REDIRECT=UARST2_REDIRECT;

    if(wifi_bufwrite_count>wifi_bufread_count)
    {         
        wifi_bufwrite_count=wifi_bufwrite_count-wifi_bufread_count;
        wifi_bufread_count=0;
        memset(Command_Buffer,0,sizeof(Command_Buffer));
        for (int i=0; i<WIFIMAX_CMD_SIZE; i++)
        {
            Command_Buffer[i] = wifi_cmdbuffer[wifi_bufindr][i];
        }
        wifi_bufindr = (wifi_bufindr + 1)%WIFI_BUFSIZE;
        wifi_bufread_count++;
        if((system_infor.sd_print_status== SD_PRINTING||system_infor.sd_print_status==SD_PRINT_RECOVERY||system_infor.serial_printf_flag == 1)&&strstr(Command_Buffer,"G")!=NULL)
            return;
        Processing_command();
    }

}

/**********************************************************
***Function:     Wifi_Mode_Select
***Description: wifi set ap,connect to server,command processing
***Input:  
***Output: 
***Return:
***********************************************************/
void Wifi_Mode_Select(void)
{
#ifndef BOARD_M301_Pro_S 
    static u8 login_auto=DISABLE;//login by auto or by "connect to... and ...login in"
    static u16 Save_Delay_Time;
    static u8 send_times=0;
    switch(WIFI_MODE)
    {          
        case WIFI_CONNECT_SERVER:   //reset wifi module  , Connect router
            if(Wifi_Reset()==1)  //reset wifi module
            {
                Add_Message(WIFI_DEBUG_MESSAGE);
                send_times=0;
                sprintf(Wifi_Login_Message,"%s;login\r\n",Setting.SN);
                if(login_auto==ENABLE)  //select login mode
                {
                      Wifi_Conf_Set("set auto\r\n");
                }
                else
                {
                    Wifi_Conf_Set(Wifi_Server_message);
                }
                WIFI_MODE=WIFI_SERVER_WAITING;        
                Save_Delay_Time=Get_CDelay();
            }
        break;
        case WIFI_SERVER_WAITING:  //Waiting for the connection to the router to succeed
            if(Get_Delay_ms(Save_Delay_Time)>1000)
            {
                if(strstr(uart2_buff,"connect succe")!=NULL)
                {
                    Wifi_Work_Message.WIFI_WORK_STATUS=2;
                    Add_Message(WIFI_STATUS_MESSAGE);
                    if(login_auto==ENABLE)
                    {
                        WIFI_MODE=WIFI_LOGIN_WAITING;
                    }
                    else
                    {
                        WIFI_MODE=WIFI_LOGIN_SERVER;
                    }
                    Get_Locality_ip();
                    Save_Delay_Time=Get_CDelay();
                    Init_Usart2_arg();
                    break;
                }
                else if(strstr(uart2_buff,"connect to server fail")!=NULL)
                {
                    my_printf("erro1\r\n");
                    Save_Delay_Time=Get_CDelay();
                    WIFI_MODE=WIFI_CONNECT_SERVER;
                    break;
                }
                else if(strstr(uart2_buff,"connect to router fail")!=NULL)
                {
                    my_printf("erro2\r\n");
                    Save_Delay_Time=Get_CDelay();
                    WIFI_MODE=WIFI_CONNECT_SERVER;
                    break;
                }
                else
                {
                    if(send_times>5)
                    {
                        send_times=0;
                        my_printf("erro3\r\n");
                        Save_Delay_Time=Get_CDelay();
                        WIFI_MODE=WIFI_CONNECT_SERVER;
                        Wifi_ASK_Factors.WIFI_RECONNECT_TIMES--;
                    }
                    else
                    {
                        send_times++;
                        Save_Delay_Time=Get_CDelay();
                        //Init_Usart2_arg();
                    }
                }
            }
            break;
            case WIFI_LOGIN_SERVER:  //Login server
            if(Get_Delay_ms(Save_Delay_Time)>100)
            {
                Init_Usart2_arg();
                send_times=0;
                Wifi_Conf_Set(Wifi_Login_Message);
                WIFI_MODE=WIFI_LOGIN_WAITING;
            }
            break; 
            case WIFI_LOGIN_WAITING:   //Wait for login server to succeed
                if(Get_Delay_ms(Save_Delay_Time)>1000)
                {
                    if(strstr(uart2_buff,"login success")!=NULL)
                    {
                        Wifi_Work_Message.WIFI_WORK_STATUS=3;
                        Add_Message(WIFI_STATUS_MESSAGE);
                        Add_Message(WIFI_SERVER_MESSAGE);
                        Add_Message(WIFI_DETAIL_MESSAGE);
                        WIFI_MODE=WIFI_HANDLE_DATA;
                        Rev_File_Flag=0;
                        WIFI_Bebug_Flag =3;
                        Init_Usart2_arg();
	                 Add_Message(WIFI_DEBUG_MESSAGE);
                    }
                    else  //connect fail
                    {
                        if(send_times>5)
                        {
                            if(login_auto==ENABLE)
                                WIFI_MODE=WIFI_LOGIN_SERVER;
                            else 
                            {
                                Wifi_ASK_Factors.WIFI_CONNECT_TIMES++;
                                if( Wifi_ASK_Factors.WIFI_CONNECT_TIMES>3)
                                {
                                    WIFI_MODE=WIFI_CONNECT_SERVER;
                                    GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN);  
                                }
                                else
                                {
                                    WIFI_MODE=WIFI_LOGIN_SERVER;
                                }
												
                            }
                            Wifi_ASK_Factors.WIFI_RECONNECT_TIMES--;
                        }
                        else
                        {
                            Save_Delay_Time=Get_CDelay();
                            send_times++;
                        }
                    }
                }
        break;
        case WIFI_HANDLE_DATA:   //WiFi data processing
            if(serial2_complete_flag==1)
            {
                Save_Delay_Time=Get_CDelay();
                fetch_wifi_next_command();    
            } 
            if(Rev_File_Flag==1)
            {
                if(Get_Delay_ms(Save_Delay_Time)>6000)
                {
                    Save_Delay_Time=Get_CDelay();
                    File_DownST2.Send_timers++;
                    if(File_DownST2.Send_timers>7)
                    {  
                        Wifi_Conf_Set("M2110 send stop\r\n");//
                        File_DownST2.Send_timers=0;
                        Rev_File_Flag=0;
                        delay_ms(500);
                        sprintf(Printf_Buf, "M2110 %d fail SD:Err\r\n",File_DownST2.File_Offset_re); 
                        Wifi_Conf_Set(Printf_Buf);
                        delay_ms(500);
                        return;
                    }
                    sprintf(Printf_Buf, "M2110 %d fail 4\r\n",File_DownST2.File_Offset_re); 
                    Wifi_Conf_Set(Printf_Buf);
                    delay_ms(100);
                }
                                    
            }
            else
            {
                wifi_CommandExecute();                       //Execute the acquired command
                
                if(Wifi_ASK_Factors.WIFI_COMMEND!=0)
                {
                    Wifi_Processing_command();
                }
            }
            UARST_REDIRECT=UARST1_REDIRECT;
        break;
        case WIFI_SET_AP:  //Open WiFi hotspot
            if(Wifi_Reset()==1)
            {
                Wifi_Conf_Set("set ap\r\n");
                WIFI_MODE=WIFI_WAIT_AP;
            }
            break;
            case WIFI_WAIT_AP:      //Waiting for WiFi configuration to succeed
                if(serial2_complete_flag==1)
                {
                    WIFI_Extract_Info();
                }
            break;
        default :  break;
		
    }
	//UARST_REDIRECT=UARST1_REDIRECT;	
#endif
}

/******************Scheduled upload data*****************************/
void Refresh_WIFI_Data(void)
{
    static u16 Save_Delay_Time;
    static u8 Data=0,ref=0;
    if(Rev_File_Flag == 1 ||Wifi_ASK_Factors.WIFI_COMMEND!=0 ||WIFI_MODE !=WIFI_HANDLE_DATA)
        return;
    switch(Data)
    {
        case 0:
              Save_Delay_Time=Get_CDelay();
              Data=1;
              ref =0;
        break;
        case 1:
        {
            if(Get_Delay_ms(Save_Delay_Time)>10000)
            {

                if(ref==4 || ref==8 || ref ==12)
                {
                    Send_Printer_State();
                    Save_Delay_Time=Get_CDelay();
                    ref++;
                }
                else if(ref >12)
                {
                    Send_machine_info();
                    Data=0;
                }
                else
                {
                    Send_m105_ask();
                    Save_Delay_Time=Get_CDelay();
                    ref++;
                }
            }
        }
            break;
    }
}
/********************************************************************
****function :
****
****
*********************************************************************/
void Network_Detection(void)
{
    sprintf(Printf_Buf,"M2999\r\n");
    my_printf1(Printf_Buf);
}

/**********************ON/OFF WIFI ***************************/
void Open_wifi(u8 p)
{
    if(p)
    {
        Read_WIFI_Memory();
    
        WIFI_MODE=WIFI_CONNECT_SERVER;
        Wifi_ASK_Factors.WIFI_CONNECT_FLAG=1;
        Wifi_Work_Message.WIFI_WORK_STATUS=1;
        Wifi_ASK_Factors.WIFI_RECONNECT_TIMES=7;
        WIFI_Bebug_Flag=1;
        WIFI_Connect();
        Add_Message(WIFI_DEBUG_MESSAGE);
        Add_Message(WIFI_STATUS_MESSAGE);
    }
    else
    {
        Wifi_Conf_Set("esp8266_disconnect\r\n");	
        delay_ms(200);
        WIFI_MODE=0;
        GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN); 
        Wifi_ASK_Factors.WIFI_CONNECT_FLAG=0;
        Wifi_Work_Message.WEBSERVER_STATUS=0;
        Add_Message(WIFI_STATUS_MESSAGE);
    }
}
/**********************ON/OFF wifi ap***************************/
void Open_APP_Set(u8 s)
{
    if(s)
    {
        WIFI_MODE=WIFI_SET_AP;
        Wifi_ASK_Factors.WIFI_CONNECT_FLAG=1;
        Wifi_Work_Message.WIFI_WORK_STATUS=1;
    }
    else
    {
        if(WIFI_MODE==WIFI_WAIT_AP)
        {
            Open_wifi(0);
            Open_wifi(1);
        }
        else
        {
            Open_wifi(0);
        }
    }
}
/*****************get information about router and server*****************************/
void WIFI_Connect(void)
{
    u8 j=0,num=0;
    Read_WIFI_Memory();
    char *Start_Addr=  strstr(Wifi_Server_message,"connect to");
    if(Start_Addr!=NULL)
    {
        Start_Addr+=10;
        while(*++Start_Addr!=':'&& num<32)
        {
            Wifi_Work_Message.SSID[j++]=*Start_Addr;
            num++;
        }
        while(*++Start_Addr!=';');
        while(*++Start_Addr!=';');
        num=0;
        while(*++Start_Addr!=';'&& num<31)
        {
            Wifi_Work_Message.WebServer_IP[num++]=*Start_Addr;
        }
        Wifi_Work_Message.WebServer_IP[num++]='\0';

    }
    Add_Message(WIFI_SERVER_MESSAGE); 
}



/********************************wifi ap set  get SSID,password,server ip*********************************************************/
void WIFI_Extract_Info(void)
{
#ifndef BOARD_M301_Pro_S 
    u8 i=11,j=0,k=0,num=0;
    char *Start_Addr=  strstr(uart2_buff,"ssid:"); 
    memset(Wifi_Server_message,0,128);
    sprintf(Wifi_Server_message,"connect to ");
    char ssid[26];
    serial2_complete_flag=0;
    if(Start_Addr!=NULL)
    {
        memset(Wifi_Work_Message.SSID,0,sizeof(Wifi_Work_Message.SSID));
        memset(Wifi_Work_Message.Router_Password,0,sizeof(Wifi_Work_Message.Router_Password));
        memset(Wifi_Work_Message.Router_IP,0,sizeof(Wifi_Work_Message.Router_IP));
        Start_Addr+=4;
        while(*++Start_Addr!=';'  &&  num<31)
        {
            num++;
            Wifi_Work_Message.SSID[j++]=*Start_Addr;
            Wifi_Server_message[i++]=*Start_Addr;
        }
        if(strstr(Wifi_Work_Message.SSID,"%20") !=NULL)
        {
            for(num=0;num<j;num++)
            {
                if(Wifi_Work_Message.SSID[num]=='%'&&Wifi_Work_Message.SSID[num+1]=='2'&&Wifi_Work_Message.SSID[num+2]=='0'&&(num<j-2))
                {
                    ssid[k++]=' ';
                    num=num+3;
                }

                ssid[k++]=Wifi_Work_Message.SSID[num];
            }
            i=11;
            for(num=0;num<k;num++)                 
            Wifi_Server_message[i++]=ssid[num];
            memset(Wifi_Work_Message.SSID,0,26);

        }
        Wifi_Server_message[i++]=':';
        Start_Addr=  strstr(uart2_buff,"password:"); 
        if(Start_Addr!=NULL)
        {
            Start_Addr+=8;num=0;
            while(*++Start_Addr!=';'  &&  num<50)
            {
            num++;
            Wifi_Server_message[i++]=*Start_Addr;
            }
            Wifi_Server_message[i++]=';';

        }
        else
        {
            Wifi_Conf_Set("config fail\r\n");
            memset(uart2_buff,0,1280);
            return;
        }
        sprintf(&Wifi_Server_message[i],"%s;",Setting.SN);
        i=i+strlen(Setting.SN)+1;
        Start_Addr=  strstr(uart2_buff,"server:"); 
        if(Start_Addr!=NULL)
        {
            Start_Addr+=6;num=0;j=0;
            while(*++Start_Addr!=';'  &&  num<31)//
            {
                num++;
                Wifi_Server_message[i++]=*Start_Addr;
                Wifi_Work_Message.WebServer_IP[j++]=*Start_Addr;

            }
            Wifi_Server_message[i++]=';';
            Wifi_Server_message[i++]='\r';
            Wifi_Server_message[i++]='\n';
            WIFI_MODE=0;
            Store_WIFI_Memory();
            Add_Message(WIFI_SET_SUCCEE);
            Wifi_Conf_Set("config ok\r\n");
            Add_Message(WIFI_SET_SUCCEE);
            Wifi_Conf_Set("config ok\r\n");
            my_printf("config ok\r\n");
            return;
        }
        else
        {
            Wifi_Conf_Set("config fail\r\n");
            memset(uart2_buff,0,1280);
            return;
        }
    }
    else
    {
        Wifi_Conf_Set("config fail\r\n");
        return;
    }
#endif
}

void clear_Wifi_SSID(void)
{
    
   
}
/****************wifi data processing**************************/
void get_wifi_command(void)
{
#ifndef BOARD_M301_Pro_S 
    char GetWifi_cmdbuffer[WIFIMAX_CMD_SIZE];
    static long wifiGcode_N = 0;//, wifiGcode_LastN = 0;
    char *strchr_pointer;
    wifi_count = 0; //clear buffer
    memset(GetWifi_cmdbuffer,0,WIFIMAX_CMD_SIZE);
    while(wifi_buff_start != wifi_buff_end && wifi_bufwrite_count<(WIFI_BUFSIZE-1))
    {            		
        wifi_char = uart2_buff[wifi_buff_start];
        wifi_buff_start = (wifi_buff_start + 1) % 1280;
		
        if(wifi_char == '\n' || wifi_char == '\r' || (wifi_char == ':' && wifi_comment_mode == false) ||wifi_count >= (WIFIMAX_CMD_SIZE - 1) )
        {
            if(!wifi_count)    //if empty line
            {
                wifi_comment_mode = false;   //for new command
                continue;//
            }
            GetWifi_cmdbuffer[wifi_count] = 0;  //terminate string
            if(strchr(GetWifi_cmdbuffer, 'N') != NULL)
            {
                strchr_pointer = strchr(GetWifi_cmdbuffer, 'N');
                wifiGcode_N = (strtol(strchr_pointer + 1, NULL, 10));//get number
                if(strchr(GetWifi_cmdbuffer, '*') != NULL)
                {
                    u16 checksum = 0;
                    u16 count = 0;
                    memset(wifi_cmdbuffer[wifi_bufindw],0,WIFIMAX_CMD_SIZE);//clear buff
                    while(GetWifi_cmdbuffer[strchr_pointer-GetWifi_cmdbuffer+count] != '*')
                    {
                        wifi_cmdbuffer[wifi_bufindw][count]=GetWifi_cmdbuffer[strchr_pointer-GetWifi_cmdbuffer+count];
                        checksum = checksum^GetWifi_cmdbuffer[strchr_pointer-GetWifi_cmdbuffer+count];
                        count++;
                    }
                    wifi_cmdbuffer[wifi_bufindw][count]=GetWifi_cmdbuffer[strchr_pointer-GetWifi_cmdbuffer+count];
                    strchr_pointer = strchr(GetWifi_cmdbuffer, '*');
		  
                    if( (u16)(strtod(strchr_pointer + 1, NULL)) != checksum) //check error
                    {
                        if(wifiGcode_N>=0)
                        {
                            sprintf(Printf_Buf,"Err_MisMatch:%d;\r\n",wifiGcode_N);
                            Wifi_Conf_Set(Printf_Buf);
                            my_printf("%s",Printf_Buf);
                        }
                        memset(wifi_cmdbuffer[wifi_bufindw],0,WIFIMAX_CMD_SIZE);
                    }
                    else
                    {
                        wifi_bufindw = (wifi_bufindw + 1)%WIFI_BUFSIZE;
                        wifi_bufwrite_count++;
                    }
			
                }
	   
            }
            wifi_count = 0; //clear buffer
            memset(GetWifi_cmdbuffer,0,WIFIMAX_CMD_SIZE);
            wifi_comment_mode = false;   //for new command
        }
        else
        {
            if(wifi_char == ';') 
                wifi_comment_mode = true;  //for new command
            if(!wifi_comment_mode)
                GetWifi_cmdbuffer[wifi_count++] = wifi_char;              
        }
    }
    Init_Usart2_arg();
#endif
}



extern u16 Rx_File_Len;

/**************get wifi gcode file data*******************/
u8 getWifi_File_Data(void)
{
#ifndef BOARD_M301_Pro_S 
    char serial_char;
    u16 i=0;
    Rx_File_Len = 0; //clear buffer
    for(i=0;i<1220;i++)
    {            		
        serial_char = uart2_buff[wifi_buff_start];
        wifi_buff_start = (wifi_buff_start + 1) % 1280;
        if(Rx_File_Len > 1200 )
        {
            WIFIDATA_Buffer[Rx_File_Len] = '\0';  //terminate string
            return 0;
        }
        else
        {
            WIFIDATA_Buffer[Rx_File_Len++] = serial_char;             
        }

    }
 #endif
    return 1;
}

/**************Initialize wifi  data accept area****************/
void Init_Usart2_arg(void)
{
#ifndef BOARD_M301_Pro_S 
    serial2_complete_flag=0;
    wifi_buff_start  = 0;
    wifi_buff_end=0;
    memset(uart2_buff,0,1280);
#endif
}


/*************Analytical wifi gcode file data***************/
u8 Analysis2_RxData(void)
{
    u16 i=0;
    char str[10];
    Rx_File_Len=11;
    char *Start_Addr=  strstr(WIFIDATA_Buffer,"SD0");
    if(Start_Addr!=NULL)
    {
        sprintf(File_DownST2.Save_Location,"SD0");
    }
    Start_Addr+=4;
    memset(File_DownST2.Save_FileName,0,sizeof(File_DownST2.Save_FileName));
    for(i=0;i<50;i++)
    {
        if(*Start_Addr!=';')
            File_DownST2.Save_FileName[i]=*Start_Addr;
        else
        {
            File_DownST2.Save_FileName[i]='\0';
            Start_Addr++;
            break;
        }
        Start_Addr++;
    }
    Rx_File_Len+=strlen(File_DownST2.Save_FileName);
    sprintf(SD_Path, "SD1:/%s",File_DownST2.Save_FileName);
    strcpy(File_DownST2.Save_FileName,SD_Path);
    for(i=0;i<15;i++)
    {
        if(*Start_Addr!=';')
        {
            str[i]=*Start_Addr;
            Rx_File_Len++;
        }
        else
        {
            Rx_File_Len++;
            str[i]='\0';

            File_DownST2.File_Offset=  strtol(str, NULL, 10);
            Start_Addr++;
            break;
        }
        Start_Addr++;
    }
    for(i=0;i<5;i++)
    {
        if(*Start_Addr!=';')
        {
            Rx_File_Len++;
            str[i]=*Start_Addr;
        }
        else
        {
            str[i]='\0';
            Rx_File_Len++;
            File_DownST2.Rx_File_Size=  strtol(str, NULL, 10);
            Start_Addr++;
            break;
        }
        Start_Addr++;
    }
    for(i=0;i<File_DownST2.Rx_File_Size;i++)
    {
        File_DownST2.File_Data[i] = *(Start_Addr+i);
    }     
    File_DownST2.File_Data[i] = '\0' ;

    Start_Addr+=File_DownST2.Rx_File_Size;
    File_DownST2.Crc_Data=(*Start_Addr)<<8|(*(Start_Addr+1));
    Rx_File_Len+=(File_DownST2.Rx_File_Size+4);
}


/********************Get file modification, acceptance time*******************/
void Get_File_Times(char * Times)
{
  u16 year,month,mday,hour,min;//,sec;
  u8 i=0,time_out=0;
  char str[6]={0};
  Times=Times+14;
  while(1)
  {
	if((*Times =='Y')&&(*(Times+1) ==':'))
	{
           for(i=0;i<5;i++)
	        {
            	   if(*(Times+2)==';') {year=strtol(str, NULL, 10);memset(str,0,6);break;}
	    	   else
		    {
		  	str[i]=*(Times+2);
	        	Times++; 
		     }	   
	   	 }	  
        }
	if((*Times =='M')&&(*(Times+1) ==':'))
	{
          for(i=0;i<4;i++)
	      {
            	   if(*(Times+2)==';') {month=strtol(str, NULL, 10);memset(str,0,6);break;}
	    	   else
		    {
		  	str[i]=*(Times+2);
	        	Times++;
		     }	   
	   	}	  
        }
      if((*Times =='D')&&(*(Times+1) ==':'))
	{
          for(i=0;i<4;i++)
	      {
            	   if(*(Times+2)==';') {mday=strtol(str, NULL, 10);memset(str,0,6);break;}
	    	   else
		    {
		  	str[i]=*(Times+2);
	        	Times++;
		     }	   
	   	}	  
        }
     if((*Times =='H')&&(*(Times+1) ==':'))
	{
          for(i=0;i<4;i++)
	      {
            	   if(*(Times+2)==';') {hour=strtol(str, NULL, 10);memset(str,0,6);break;}
	    	   else
		    {
		  	str[i]=*(Times+2);
	        	Times++;
		     }	   
	   	}	  
        }
	if((*Times =='F')&&(*(Times+1) ==':'))
	{
          for(i=0;i<4;i++)
	      {
            	   if(*(Times+2)==';') {min=strtol(str, NULL, 10);i=88;}
	    	   else
		    {
		  	str[i]=*(Times+2);
	        	Times++;
		     }	   
	   	}	  
        }
		   
  Times++;
  if(i>60||time_out>50)
  	break;
  time_out++;
	
 }
  set_timestamp(File_DownST2.Save_FileName,year,month,mday,hour,min,0);

}

u16 crc12,crc13,crc14;
//u8 Mask_SDPrintButton=DISABLE;//
/***************************************************************************
*******wifi command  processing
****************************************************************************/
void fetch_wifi_next_command(void)
{
#ifndef BOARD_M301_Pro_S 
    char *Times_pointer;
    static u8 SD_Times=0;//u8 printf_flag = 1,SD_Times=0;
    char ret=0;
    UARST_REDIRECT=UARST2_REDIRECT;
    if(strstr(uart2_buff,"M2110 send end")!=NULL||strstr(uart2_buff,"M2110 send stop")!=NULL)
    {
        Times_pointer=strstr(uart2_buff,"M2110 send end");
        if(strstr(uart2_buff,"M2110 send end")!=NULL)
        {  	
            Get_File_Times(Times_pointer);
            Init_Usart2_arg();
            delay_ms(100);
            Wifi_Conf_Set("M2110 send completion\r\n");
            delay_ms(200);
            system_infor.files_refresh_flag=1;
            SD_File_Detect();
            Rev_File_Flag=0;
            Queue_wait();
            memset(Command_Buffer,0,CMDBUF_SIZE);
            sprintf(Command_Buffer,"M23 %s\r\n",&File_DownST2.Save_FileName[5]);//SD1:/%s
            Processing_command();
            memset(Command_Buffer,0,CMDBUF_SIZE);
            strcpy(Command_Buffer,"M24\r\n");
            Processing_command();
            return;
        }
        else
        {  
            Init_Usart2_arg();
            delay_ms(100);
            Wifi_Conf_Set("M2110 send stop\r\n");
            delay_ms(200);
            Rev_File_Flag=0;
            return;
        }
    }
     
    if(Rev_File_Flag==1) 
    {
        if(strstr(uart2_buff,"M2110 start")!=NULL)
        {            
            Init_Usart2_arg();
            delay_ms(100);
            File_DownST2.File_Offset_re=0;
            SD_Times=0;
            Wifi_Conf_Set("M2110 start\r\n");
            delay_ms(200);
            return;
        }
        else if(strstr(uart2_buff,"M2110")!=NULL)
        {	
            wifi_buff_start=0;
            File_DownST2.Send_timers=0;
            while(1)
            {
                if(uart2_buff[wifi_buff_start]=='M'&&uart2_buff[wifi_buff_start+1]=='2'&&uart2_buff[wifi_buff_start+2]=='1'&&uart2_buff[wifi_buff_start+3]=='1'&&uart2_buff[wifi_buff_start+4]=='0')
                {
                    getWifi_File_Data();
                    Init_Usart2_arg();
                    Analysis2_RxData();
                    if(crc16(WIFIDATA_Buffer,Rx_File_Len-3)==(WIFIDATA_Buffer[Rx_File_Len-3]<<8|WIFIDATA_Buffer[Rx_File_Len-2]))
                    {
                        ret=AddData_To_File(File_DownST2.File_Data,File_DownST2.Rx_File_Size,File_DownST2.Save_FileName,File_DownST2.File_Offset);
                    }
                    if(ret==1)
                    {
                        sprintf(Printf_Buf, "M2110 %d ok\r\n",File_DownST2.File_Offset); 
                        Wifi_Conf_Set(Printf_Buf);
                        my_printf("%s\r\n",Printf_Buf);
                        File_DownST2.File_Offset_re =File_DownST2.File_Offset+1024; 
                        SD_Times=0;
                        break;
                    }
                    else
                    {
                        if(SD_Times>10)
                        {
                            sprintf(Printf_Buf, "M2110 %d fail SD:Err\r\n",File_DownST2.File_Offset_re); 
                        }
                        else
                        {
                            SD_Times++;
                            sprintf(Printf_Buf, "M2110 %d fail\r\n",File_DownST2.File_Offset_re);
                        }
                        Wifi_Conf_Set(Printf_Buf);break;
                        my_printf("%s\r\n",Printf_Buf);
                    }
                }
                else
                { 
                    wifi_buff_start++;
                    if(wifi_buff_start>=2200)
                    {
                        Init_Usart2_arg();
                        sprintf(Printf_Buf, "M2110 %d fail\r\n",File_DownST2.File_Offset_re); 
                        Wifi_Conf_Set(Printf_Buf);break;
                        my_printf("%s",Printf_Buf);
                    }
                }
            }
       
        }
        else
        {
            Init_Usart2_arg();
            Wifi_ASK_Factors.WIFI_COMMEND=0;
            return;
        }
    }
    else
    {
        if(strstr(uart2_buff,"M2110 start") != NULL)
        {  
            Init_Usart2_arg();
            if(system_infor.sd_status != SD_OK)
            {
                delay_ms(100);
                Wifi_Conf_Set("SD: fail\r\n");
                delay_ms(200);
            }
            else
            {
                Rev_File_Flag=1;
                delay_ms(100);
                File_DownST2.File_Offset_re=0;
                SD_Times=0;
                Wifi_Conf_Set("M2110 start\r\n");
                my_printf("M2110 start\r\n");
                delay_ms(200);
            }
            return;
        } 
        if(strstr(uart2_buff,"M2110 Continue") != NULL)
        {
            Init_Usart2_arg();
            Rev_File_Flag=1;
            delay_ms(100);
            SD_Times=0;
            Wifi_Conf_Set("M2110 ContinueStart\r\n");
            delay_ms(200);
            return;
        } 
        get_wifi_command();
    } 
#endif
}

static u8 Pre_SDPrint_Status=0;
static u8 Pre_Motor_Status=0;
static u8 Pre_SD_Status=0;

extern float Current_Temperature[5];
static  float  Pre_print_percent=0.1;  //print_percent
void get_M2101_status(void)
{
   static u16 num_T=0,wifirefresh_status=0;
   
   if(Rev_File_Flag==1|| APP_Refresh_flag.App_RefreshSatus_Times<2500)
   	{
      	 return;
   	}
       APP_Refresh_flag.App_RefreshSatus_Times=0;
    UARST_REDIRECT=UARST2_REDIRECT;
if(system_infor.sd_print_status== SD_PRINT_IDLE||system_infor.sd_print_status== SD_PRINT_PAUSE)
{
   wifirefresh_status++;
   	if(wifirefresh_status==3)
   	{Send_m105_ask();
	return;
	}
	else if(wifirefresh_status>=4)
	{Send_Printer_State(); 
	wifirefresh_status=0;
	return;
	}
}
else{
    if(num_T==0)
    {
    if(Pre_SDPrint_Status!=system_infor.sd_print_status||Pre_Motor_Status!=Get_Motor_Status()||Pre_print_percent!=system_infor.print_percent||Pre_SD_Status!=system_infor.sd_status)
    {
         Pre_SDPrint_Status=system_infor.sd_print_status;
         Pre_Motor_Status=Get_Motor_Status();
         Pre_print_percent=system_infor.print_percent;
         Pre_SD_Status=system_infor.sd_status;
         Send_Printer_State();
     }
    num_T=1;
        }
    else
    { 
          Send_m105_ask();	
          num_T=0;
        }  
}
}
volatile WIFI_TX_STATA Wifi_ASK_State=WIFI_READY;

/*******************Wifi Processing command*****************************/
void Wifi_Processing_command(void)
{
    static u8  dir_num=0;//
    static u16 Save_Delay_Time;

    switch(Wifi_ASK_Factors.WIFI_COMMEND)//
    {
        case 20: 
        {
            switch(Wifi_ASK_State)	
            {
                case WIFI_READY:
                    if(system_infor.sd_status != SD_OK)
                    {
                        Wifi_ASK_Factors.WIFI_COMMEND=0;
                        sprintf(Printf_Buf,"M20 SD Card Removed\r\n");
                        USART2_TxString((u8 *)Printf_Buf);
                        break;
                    }		
                    Wifi_ASK_State=WIFI_TX_DATA;
                    dir_num=0;
                break;
                case WIFI_TX_DATA:
                    if( dir_num<Wifi_ASK_Factors.WIFI_FILE_SUM)
                    {
                        sprintf(Printf_Buf,"%s---%d\r\n",sd_file_name[dir_num],dir_num);
                        USART2_TxString((u8 *)Printf_Buf);
                        Wifi_ASK_State=WIFI_WAIT_ASK;
                        Save_Delay_Time=Get_CDelay();
                    }
                    else
                    {
                        Wifi_ASK_Factors.WIFI_COMMEND=0;
                        Wifi_ASK_State=WIFI_READY;
                    }
                break;
                case WIFI_WAIT_ASK:

                    if(APP_Refresh_flag.App_Sendfiles_FLAG==ENABLE)
                    {
                        APP_Refresh_flag.App_Sendfiles_FLAG=DISABLE;
                        dir_num=APP_Refresh_flag.App_sendfiles_dirmun+1;
                        Wifi_ASK_State=WIFI_TX_DATA;
                        Save_Delay_Time=Get_CDelay();
                        Wifi_ASK_Factors.WIFI_TX_TIMES=0; //Wifi_Send_Times=0;
                    }
                    else if(Get_Delay_ms(Save_Delay_Time)>2500)
                    {
                        Save_Delay_Time=Get_CDelay();
                        Wifi_ASK_Factors.WIFI_TX_TIMES++;
                        if(Wifi_ASK_Factors.WIFI_TX_TIMES>2)
                        {
                            Wifi_ASK_Factors.WIFI_TX_TIMES=0;
                            Wifi_ASK_State=WIFI_TX_DATA;
                        }
                    }       
                break;
                default :break;
            }                                               
        }
        break;

        default :break;

    }


}





//void SD_filePrint_select(void)

/******************
****Boot automatically connect flag
****
****
*******************/
void Wifi_Auto_Connect(u8 flag)
{
    Wifi_Work_Message.AUTO_CONNECT=flag;
    STMFLASH_Write(WIFI_AUTO_CONNECT_FLAG_ADDR, (u16*)&flag,1);
   
}
void WIFI_Write_AutoConnect_Flag(u8 flag)
{
    Wifi_Work_Message.AUTO_CONNECT=flag;
    STMFLASH_Write(WIFI_AUTO_CONNECT_FLAG_ADDR, (u16*)&flag,1);
}
u8 WIFI_Read_AutoConnect_Flag(void)
{
    u8 flag;
    STMFLASH_Read(WIFI_AUTO_CONNECT_FLAG_ADDR, (u16*)&flag,1);
    Wifi_Work_Message.AUTO_CONNECT=flag;
    return flag;
}





void Store_WIFI_Memory(void)
{
	STMFLASH_Write(WIFI_SERVER_PARAMETERS_ADDR, (u16*)Wifi_Server_message,  64);
}
void Read_WIFI_Memory(void)
{
	STMFLASH_Read(WIFI_SERVER_PARAMETERS_ADDR, (u16*)Wifi_Server_message,  64);
}
void Clear_WIFI_Info(void)
{
	memset(Wifi_Server_message,0,128);
	STMFLASH_Write(WIFI_SERVER_PARAMETERS_ADDR, (u16*)Wifi_Server_message,  64);
}
/******************************Modify file timestamp*******************************************/
FRESULT set_timestamp (
    char *obj,     /* Pointer to the file name */
    int year,
    int month,
    int mday,
    int hour,
    int min,
    int sec
)
{
    FILINFO fno;

    fno.fdate = (WORD)(((year - 1980) * 512U) | month * 32U | mday);
    fno.ftime = (WORD)(hour * 2048U | min * 32U | sec / 2U);

    return f_utime(obj, &fno);
}

/*
*Detecting the existence of a WiFi module
*/

u8 Exist_Wifi_Det(void)
{
#ifndef BOARD_M301_Pro_S 
	u16 i=0,j=0;
	u8 num=0,num1=0;
	for(j=0;j<5;j++)
	{
		Init_Usart2_arg();
		GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN);
		delay_ms(10);
		GPIO_SetBits(WIFI_GPIO,WIFI_GPIO_PIN);
		delay_ms(500);
		if(wifi_buff_start  != wifi_buff_end)
		{
			char *sp_start ;
			if((wifi_buff_end-wifi_buff_start)>80)
	   			num1++;
			for(i=0;i<1280;i+=40)
			{
				sp_start = strstr(&uart2_buff[i],"VERSION");
				if(sp_start!=NULL)
				{
					strncpy(&WF_version[1],sp_start+8,5);
					Add_Message(PRINTER_MESSAGE);
					return 0;
				}
			}
	   	}
	   	else
	   	{
	   		num++;
	   		if(num>2)
	   		{
	   			return 1;
	   		}
	   	}
	}
	if(num1>3)
	{
		WF_version[1]='0';
		WF_version[2]='.';
		WF_version[3]='0';
		WF_version[4]='.';
		WF_version[5]='0';
		Add_Message(PRINTER_MESSAGE);
		my_printf("HV:%s\r\n",WF_version);
		return 0;
	}
#endif
	return 1;
	
}

#endif

void Motor_Unlock_Write_Timer(u16 tim)
{
    u16 flag=tim;
    STMFLASH_Write(MOTOR_UNLOCK_TIME_FLAG_ADDR, &flag,1);
}
u16 Motor_Unlock_Read_Timer(void)
{
    u16 flag;
    STMFLASH_Read(MOTOR_UNLOCK_TIME_FLAG_ADDR, &flag,1);
    return flag;
}

#endif
