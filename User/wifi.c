#include "wifi.h"
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include "sd_print.h"
#include "variable.h"
#include "data_handle.h"

u8 WIFI_Bebug_Flag=0;
u8 WIFI_MODE=0;
u8 Wifi_Server_message[128]="connect to TP-LINK_3928:JIETAI007;17S626D2000000;www.geeetech.net;\r\n";
u8 Wifi_Login_Message[128]="17S626D2000000;login\r\n";
char uart2_buff[1280]={0};
u16 usart2_num=0;
u16 Wifi_REV_num=0;
u8 serial2_complete_flag=0;
u16 serial2_complete_Times=0;
u8 UARST_REDIRECT=UARST1_REDIRECT;

u16 wifi_buff_start = 0;
u16 wifi_buff_end = 0;
static int wifi_count = 0;

static int wifi_bufwrite_count = 0;
static int wifi_bufread_count = 0;
static char wifi_char;
static bool wifi_comment_mode = false;

char wifi_cmdbuffer[WIFI_BUFSIZE][WIFIMAX_CMD_SIZE];
int wifi_bufindr = 0;    //读buffer 用
int wifi_bufindw = 0;   //写buffer 用
extern char Command_Buffer[CMDBUF_SIZE];

u8 Wifi_Using_Flag=0;                 //允许向WiFi、发送数据

extern char sd_file_namebuf[FILE_NUM][FILE_NAME_SIZE];//sd_file_namebuf[255][50];
extern char  SD_Path[512];   
extern u8 dir_index;
extern char  sd_file_name[FILE_NUM][FILE_NAME_SIZE];//sd_file_name[56][50];
char Printf_Buf[512]; 
char Webserver_name[22];//={"www.geeetech.net\r\n"};
WIFI_ASK_FACTOR Wifi_ASK_Factors ={0,0,0,0,0,5,NULL}; 
WIFI_MESSAGE Wifi_Work_Message;
FILE_DOWNLOAD File_DownST;
FILE_DOWNLOAD File_DownST2;
APP_REFRESH APP_Refresh_flag ={
	.App_RefreshSatus_Times=0 ,//定时检查状态
	.Mask_SDPrintButton=DISABLE,//屏蔽标志位，屏蔽打印按钮20秒，防止打印指令冲突
	.Mask_SDPrintButton_Times=0,//屏蔽打印按钮时间
	.App_Sendfiles_FLAG=DISABLE,//发送文件名字成功标志
	.App_sendfiles_dirmun=0,//发送成功的文件名数量
	.App_Refresh_Times=0,//定时更新状态，指令M2109
	.App_Refresh_FLAG=DISABLE,//定时更新状态开关，指令M2109
	.wifi_Recovery_process_FLAG=0//断电续打标志，断电续打是否继续
};


u8 wifi_status=0;/*0:idle;1:reset */
  u16 Delay_Times=0; 
extern u8  Rev_File_Flag;
extern u8 SD_detec_flag;
 char WIFIDATA_Buffer[1280];
u8 Printer_3d_Statu=0;
char WF_version[6]="-null";



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

void my_printf(char *Data)
{
  USART_TypeDef* USARTx;
  if(UARST_REDIRECT==UARST2_REDIRECT&&system_infor.serial_printf_flag != 1)
  {
	USARTx=USART2;
       //Delay_Times=0;
  }
  else if(UARST_REDIRECT==UARST3_REDIRECT)
	 USARTx=USART3;
   else 
   	 USARTx=USART1;

  while ( *Data != 0) 
  {
        USART_SendData(USARTx, *Data);
	while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	if(USARTx==USART2&&Wifi_ASK_Factors.WIFI_COMMEND==0&&*Data=='\n')
		Wifi_Printf_Wait();
	Data++;
  }
}
void WDGOpen_Init(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); 

	TIM_TimeBaseStructure.TIM_Period = 49999;      // 4s中断40000*7200/72000000
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;	   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	//设置优先级  
  	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;    
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//先占优先级1级  
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;       //从优先级0级  
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
  	NVIC_Init(&NVIC_InitStructure);   
	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清中断标志位  
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
	WDGOpen_Init();//wifi传输文件时超时处理
}

/********************wifi server connect*************************************/
void Wifi_Conf_Set(u8 *config_message)
{
	USART2_TxString((u8*)config_message);
	//printf("KK:%s\r\n",config_message);
}
/********************wifi 复位************************************************************/
void Wifi_Reset(void)
{
	GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN);
	delay_ms(1000);delay_ms(1000);
	GPIO_SetBits(WIFI_GPIO,WIFI_GPIO_PIN);
	delay_ms(500);
}

/*********************************************************************************/
void my_delay(u32 i)
{
	u32 j;
	while(i--)
	{
	    for(j=0;j<7200;j++);
	}
}

/*********************************************************************************/

void Get_Locality_ip(void)
{
  u8 i=0;
  u8 num_sp=0;
        char *sp=strstr(uart2_buff,"get ip:");
        if(sp!=NULL)
        {
              num_sp=7;
        }
//        else
//        {
//             sp=strstr(uart2_buff,";IP:");
//             if(sp!=NULL)
//            {
//                  num_sp=4;
//             }
//             else
//             {
//                  sp=strstr(uart2_buff,"ip:");
//                  if(sp!=NULL)
//                  {
//                       num_sp=3;
//                  }
//             }
//        }
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
    
}
/*
sprintf(Printf_Buf, 
my_printf(Printf_Buf); 我
*/
void wifi_CommandExecute(void)//执行获取的指令
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


u8 WIFI_CON_START=0;
u16 REV_FILE_TIME=0;


void Wifi_Mode_Select(void)
{
    static u8 login_auto=DISABLE;//login by auto or by "connect to... and ...login in"
	//char i;
	switch(WIFI_MODE)
		{
                    
			case WIFI_CONNECT_SERVER:
                          switch(WIFI_CON_START)
                          {
                              case 0:
                                 UARST_REDIRECT=UARST2_REDIRECT;
                                 GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN);       
                                  Delay_Times=0;
                                  WIFI_CON_START=1;
                               break;
                              case 1:
                                if(Delay_Times>2000)
                                {
                                  Delay_Times=0;
                                  GPIO_SetBits(WIFI_GPIO,WIFI_GPIO_PIN);
                                  WIFI_CON_START=2;
                                }
                               break;
                               case 2:
                                 if(Delay_Times>2000)
                                 {
                                       memset(uart2_buff,0,sizeof(uart2_buff)); 
				      wifi_buff_start=0;wifi_buff_end=0;

                                      WIFI_MODE=WIFI_SERVER_WAITING;                                   
                                      sprintf(Wifi_Login_Message,"%s;login\r\n",Setting.SN);
									  
						if(login_auto==ENABLE)
							Wifi_Conf_Set("set auto\r\n");
						else
						{
							Wifi_Conf_Set(Wifi_Server_message);
						      printf("CC:%s\r\n",Wifi_Server_message);
						 }
                                     Wifi_Work_Message.Router_STATUS=1;
                                     Wifi_Work_Message.WEBSERVER_STATUS=1;
                                      Delay_Times=0;
                                      WIFI_CON_START=0;
                                 }
                                 break;
                                 default :  break;
                          }
			break;
			case WIFI_SERVER_WAITING:
                              if(Delay_Times>1000)
                              {
					if(strstr(uart2_buff,"connect success")!=NULL)
					{
						printf("erro\r\n");
						Wifi_Work_Message.WIFI_WORK_STATUS=2;
						Add_Message(WIFI_STATUS_MESSAGE);
					if(login_auto==ENABLE)
						{WIFI_MODE=WIFI_LOGIN_WAITING;Delay_Times=0;}
					else
						{
						Wifi_ASK_Factors.WIFI_CONNECT_TIMES=0;
						WIFI_MODE=WIFI_LOGIN_SERVER;
						wifi_buff_start=0;wifi_buff_end=0;Delay_Times=0;
						}
                             Get_Locality_ip();
                             Wifi_Work_Message.Router_STATUS=5;
                             Delay_Times=0;
							Wifi_ASK_Factors.WIFI_CONNECT_TIMES=0;
						break;
					}
					else if(strstr(uart2_buff,"connect to server fail")!=NULL)
					{
						printf("erro1\r\n");
						WIFI_MODE=WIFI_CONNECT_SERVER;
						break;
					}
					else if(strstr(uart2_buff,"connect to router fail")!=NULL)
					{
						printf("erro2\r\n");
						WIFI_MODE=WIFI_CONNECT_SERVER;
						break;
					}
                                  else
                                        {
                                          if(Delay_Times>6000)
                                          	{
                                          		printf("erro3\r\n");
                        
								WIFI_MODE=WIFI_CONNECT_SERVER;
						    		Wifi_ASK_Factors.WIFI_RECONNECT_TIMES--;//换一下位置，防止减到零以后出去直接进不来
                                        	}
                                        }
				       
						
                              }
				break;
			case WIFI_LOGIN_SERVER:
                           if(Delay_Times>100)
                           {
				memset(uart2_buff,0,sizeof(uart2_buff)); 
				Delay_Times=0;wifi_buff_start=0;wifi_buff_end=0;
                           printf("ddff:%s\r\n",Wifi_Login_Message);
                                Wifi_Conf_Set("\r\n");
				Wifi_Conf_Set(Wifi_Login_Message);delay_ms(100);
				WIFI_MODE=WIFI_LOGIN_WAITING;
				serial2_complete_Times=0;
                           }
				break;
			case WIFI_LOGIN_WAITING:
                                     if(Delay_Times>2000)
                                     {
					usart2_num=0;
					Wifi_REV_num=0;
					if(strstr(uart2_buff,"login success")!=NULL)
					{
					   Wifi_Work_Message.WIFI_WORK_STATUS=3;
					   Add_Message(WIFI_STATUS_MESSAGE);
					   Add_Message(WIFI_SERVER_MESSAGE);
					      
						wifi_buff_end=0;
                                        wifi_buff_start=0;
                                        // sprintf(Wifi_Work_Message.WebServer_IP,"%s",Webserver_name);
						WIFI_MODE=WIFI_HANDLE_DATA;
						Add_Message(WIFI_DETAIL_MESSAGE);
	                                 Wifi_Work_Message.WEBSERVER_STATUS=5;
	                                 WIFI_Bebug_Flag=3;
						memset(uart2_buff,0,sizeof(uart2_buff)); 
						//自动重连更新状态
                                        //M2101,M105,M115
						APP_Refresh_flag.App_Refresh_FLAG=ENABLE;
						APP_Refresh_flag.App_Refresh_Times=0;
					}
					else  //connect fail
					{
						if(Delay_Times>6000)
                                           {
                                             if(login_auto==ENABLE)
								WIFI_MODE=WIFI_LOGIN_SERVER;
						    else {
                                                	Wifi_ASK_Factors.WIFI_CONNECT_TIMES++;
                                                	if( Wifi_ASK_Factors.WIFI_CONNECT_TIMES>3)
                                                	{
						     			WIFI_MODE=WIFI_CONNECT_SERVER;
                                                    		GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN);  
								}
                                                	else
                                                     		WIFI_MODE=WIFI_LOGIN_SERVER;
												
                                         	}
                                             //memset(Wifi_Work_Message.WebServer_IP,0,22);*/
                                             printf("erro4\r\n");
						}
							//
                                         	//printf("KKKK:%s:KKKK\r\n",uart2_buff);
						}
                              }
                                     UARST_REDIRECT=UARST1_REDIRECT;
				break;
			case WIFI_HANDLE_DATA:
				
				if(system_infor.serial_printf_flag== 1)//串口打印中，不处理任何app指令
				{ char *strchr_pointer;
				   char str[18];//18
				   strchr_pointer=strstr(uart2_buff,"N-");
				   
					if(strchr_pointer==NULL)//WiFi模块指令处理
					return;
					
					if(strstr(uart2_buff,"N-1")!=NULL)//防止wifi指令里面掺杂了app指令
					    {memset(uart2_buff,0,sizeof(uart2_buff)); 
					     sprintf(uart2_buff, "N-1 M2104 ESP8266 server disconnected*80\r\n"); 
						 wifi_buff_start=0;wifi_buff_end=45;
					    }
					else
					   {
							for(u8 i=0;i<20;i++)
								{str[i]=*strchr_pointer;
								strchr_pointer++;
								}
							memset(uart2_buff,0,sizeof(uart2_buff)); 
					             sprintf(uart2_buff, "%s\r\n",str); 
								 wifi_buff_start=0;wifi_buff_end=20;
					 }		
				}
				
				UARST_REDIRECT=UARST2_REDIRECT;//wifi状态的串口发送指令				 
				if(serial2_complete_flag==1)
				{
                                   REV_FILE_TIME=0;
                                   fetch_wifi_next_command();    
				} 
                          if(Rev_File_Flag==1)
                           {
                                  if(REV_FILE_TIME>6000)
                                  {
                                    REV_FILE_TIME=0;
                                    File_DownST2.Send_timers++;
                                    if(File_DownST2.Send_timers>7)
                                        {  Wifi_Conf_Set("M2110 send stop\r\n");//
                                            File_DownST2.Send_timers=0;
                                            Rev_File_Flag=0;
							delay_ms(500);
							sprintf(Printf_Buf, "M2110 %d fail SD:Err\r\n",File_DownST2.File_Offset_re); 
                                    		Wifi_Conf_Set(Printf_Buf);
							delay_ms(500);
							return;
                                    	}
                                    sprintf(Printf_Buf, "M2110 %d fail 4\r\n",File_DownST2.File_Offset_re); 
                                    Wifi_Conf_Set((u8 *)Printf_Buf);
					   delay_ms(100);
                                  }
                                    
                             }
				else
				{
					wifi_CommandExecute();//执行获取的指令
				
					if(Wifi_ASK_Factors.WIFI_COMMEND!=0)//把刷新文件分开来
					{
					Wifi_Processing_command();
					}
					else if(APP_Refresh_flag.App_Refresh_FLAG==ENABLE)//M2019
					{static char i=0;
						if(APP_Refresh_flag.App_Refresh_Times>2000)
						{	i++;
					 	 APP_Refresh_flag.App_Refresh_Times=0;
					  	switch(i)
							{
                    				case 1: 
								Send_Printer_State();
								break;
							case 2: 
								Send_m105_ask();
								break;
							case 3: 
								Send_machine_info();
								APP_Refresh_flag.App_Refresh_FLAG=DISABLE;
								break;
							default :  
								i=0;
								break;
							}
						}
					}
                             else if(APP_Refresh_flag.Mask_SDPrintButton==DISABLE)
                               {
                                   get_M2101_status();
                               }
			}
				UARST_REDIRECT=UARST1_REDIRECT;//跳出处理WiFi数据时转化为串口指令
				break;
			case WIFI_SET_AP:
                                UARST_REDIRECT=UARST2_REDIRECT;
                                if(Delay_Times>3000)
                                {
                                    GPIO_SetBits(WIFI_GPIO,WIFI_GPIO_PIN);
                                    if(Delay_Times>4000)
                                    {
					    memset(uart2_buff,0,sizeof(uart2_buff)); //清空一下buff数据，不然buff里预留的数据会引起判断出错
					    wifi_buff_start=0;wifi_buff_end=0;
						
				           Wifi_Conf_Set("set ap\r\n");
				           printf("set ap\r\n");
				           WIFI_MODE=WIFI_WAIT_AP;
                                           Delay_Times=0;
                                        }
                                 }
				break;

			case WIFI_WAIT_AP:     
                                if(serial2_complete_flag==1)
                                {
                                       WIFI_Extract_Info();
                                }
				break;
			default :  break;
		
		}
	//UARST_REDIRECT=UARST1_REDIRECT;	
}
/********************************************************************
****功能 ； 网络检测函数每隔1分钟检测一起网络是否正常
****
****
*********************************************************************/
void Network_Detection(void)
{
    sprintf(Printf_Buf,"M2999\r\n");
    my_printf(Printf_Buf);
}

/******************
****打开wifi
****
****
*******************/
void Open_wifi(u8 p)
{
  if(p)
  {
    
    Read_WIFI_Memory();
    
    WIFI_MODE=WIFI_CONNECT_SERVER;
    Wifi_ASK_Factors.WIFI_CONNECT_FLAG=1;
    Wifi_Work_Message.WIFI_WORK_STATUS=1;
    Wifi_Work_Message.Router_STATUS=1;
    Wifi_Work_Message.WEBSERVER_STATUS=1;
    Wifi_ASK_Factors.WIFI_RECONNECT_TIMES=4;
    WIFI_Bebug_Flag=1;
    WIFI_Connect();
    Add_Message(WIFI_STATUS_MESSAGE);
  }
  else
  {
    Wifi_Conf_Set("esp8266_disconnect\r\n");	//复位前发送状态
    delay_ms(200);
    memset(Wifi_Work_Message.SSID,0,sizeof(Wifi_Work_Message.SSID));
    memset(Wifi_Work_Message.Router_IP,0,sizeof(Wifi_Work_Message.Router_IP));
    memset(Wifi_Work_Message.WebServer_IP,0,sizeof(Wifi_Work_Message.WebServer_IP));
    WIFI_MODE=0;
    //memset(&Wifi_Work_Message,0,sizeof(Wifi_Work_Message));
    GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN); 
    Wifi_ASK_Factors.WIFI_CONNECT_FLAG=0;
    Wifi_Work_Message.WEBSERVER_STATUS=0;
    Wifi_Work_Message.Router_STATUS=0;
    Wifi_Work_Message.WIFI_WORK_STATUS=0;
    Add_Message(WIFI_STATUS_MESSAGE);
    printf("Clase wifi\r\n");
     
  }
}
/******************
****打开wifi热点
****
****
*******************/
void Open_APP_Set(u8 s)
{
  if(s)
  {
      WIFI_MODE=WIFI_SET_AP;
     Wifi_ASK_Factors.WIFI_CONNECT_FLAG=1;
     
     Wifi_Work_Message.WIFI_WORK_STATUS=1;
      Wifi_Work_Message.Router_STATUS=1;
       Wifi_Work_Message.WEBSERVER_STATUS=1;
    GPIO_ResetBits(WIFI_GPIO,WIFI_GPIO_PIN);
    Delay_Times=0;
  }
  else
  {
     memset(Wifi_Work_Message.SSID,0,sizeof(Wifi_Work_Message.SSID));
    memset(Wifi_Work_Message.Router_IP,0,sizeof(Wifi_Work_Message.Router_IP));
    memset(Wifi_Work_Message.WebServer_IP,0,sizeof(Wifi_Work_Message.WebServer_IP));
    if(WIFI_MODE==WIFI_WAIT_AP)
    {
     // WIFI_MODE=0;
     Open_wifi(0);
     printf("wifi error\r\n");
      Open_wifi(1);
    //  Wifi_ASK_Factors.WIFI_CONNECT_FLAG=0;
    }
	else
	{
		Open_wifi(0);
		printf("wifi error\r\n");
	}
  }
}

void WIFI_Connect(void)
{
  u8 j=0,num=0;
   Read_WIFI_Memory();
   char *Start_Addr=  strstr(Wifi_Server_message,"connect to");
   //char *temp_Addr;
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
/******************
****打开开机wifi自动连接
****参数 ：0   正常
****       1   连接
*******************/
u8 Get_Wifi_Info(WIFI_MESSAGE *wifi_mes)
{
  if(wifi_mes!=NULL)
  {
   sprintf(wifi_mes->SSID,"%s",Wifi_Work_Message.SSID);
   sprintf(wifi_mes->Router_IP,"%s",Wifi_Work_Message.Router_IP);
   sprintf(wifi_mes->WebServer_IP,"%s",Wifi_Work_Message.WebServer_IP);
   wifi_mes->AUTO_CONNECT=Wifi_Work_Message.AUTO_CONNECT;
   
   wifi_mes->Router_STATUS=Wifi_Work_Message.Router_STATUS;
   
   wifi_mes->WEBSERVER_STATUS=Wifi_Work_Message.WEBSERVER_STATUS;
   wifi_mes->WIFI_WORK_STATUS=Wifi_Work_Message.WIFI_WORK_STATUS;
   return 1;
  }
  else
    return 0;
   
}



/********************************wifi ap set*********************************************************/
void WIFI_Extract_Info(void)
{
	//u8 wifi_server_nm[40];
	//u8 wifi_server_password[40];
	//u8 wifi_webserver_ip[40];
	u8 i=11,j=0,k=0,num=0;
       //char uart2_buff1[128]="pURL_Frame:GET /?ssid:TP-LINK_3928;password:JIETAI007;server:www.geeetech.net ";
	char *Start_Addr=  strstr(uart2_buff,"ssid:"); 
	   
       memset(Wifi_Server_message,0,128);
        sprintf(Wifi_Server_message,"connect to ");
        char ssid[26];
        serial2_complete_flag=0;
	if(Start_Addr!=NULL)
	{
    		 memset(Wifi_Work_Message.SSID,0,sizeof(Wifi_Work_Message.SSID));//清空其他信息，不能清除自动连接标志位
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
                       // Wifi_Server_message[i++]='\0';
                         WIFI_MODE=0;//WIFI_CONNECT_SERVER
                         Store_WIFI_Memory();
			    Add_Message(WIFI_SET_SUCCEE);
                        Wifi_Conf_Set("config ok\r\n");
                        Add_Message(WIFI_SET_SUCCEE);
                        Wifi_Conf_Set("config ok\r\n");
                        printf("config ok\r\n");
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
}

void clear_Wifi_SSID(void)
{
    
   
}

void get_wifi_command(void)
{
  char GetWifi_cmdbuffer[WIFIMAX_CMD_SIZE];
  static long wifiGcode_N = 0;//, wifiGcode_LastN = 0;
  
  char *strchr_pointer;
  
 // wifi_char = '\n';
  wifi_count = 0; //clear buffer
  memset(GetWifi_cmdbuffer,0,WIFIMAX_CMD_SIZE);
  
  while(wifi_buff_start != wifi_buff_end && wifi_bufwrite_count<(WIFI_BUFSIZE-1))
  {            		
        wifi_char = uart2_buff[wifi_buff_start];

	 wifi_buff_start = (wifi_buff_start + 1) % 1280;
		
          if(wifi_char == '\n' || 
		wifi_char == '\r' || 
		(wifi_char == ':' && wifi_comment_mode == false) ||
		wifi_count >= (WIFIMAX_CMD_SIZE - 1) )
          {
              	if(!wifi_count)    //if empty line
              	{
                		wifi_comment_mode = false;   //for new command
                		continue;//结束本次循环
              	}
              	GetWifi_cmdbuffer[wifi_count] = 0;  //terminate string
			//sprintf(GetWifi_cmdbuffer,"N-5 M2102 S4*125\r\n");//写校验用
     			 if(strchr(GetWifi_cmdbuffer, 'N') != NULL)
      			{
        			strchr_pointer = strchr(GetWifi_cmdbuffer, 'N');
        			wifiGcode_N = (strtol(strchr_pointer + 1, NULL, 10));//取N的序号

        			if(strchr(GetWifi_cmdbuffer, '*') != NULL)//判断是否有*
				{
   					u16 checksum = 0;
   					u16 count = 0;
					memset(wifi_cmdbuffer[wifi_bufindw],0,WIFIMAX_CMD_SIZE);//清空指令buff
          				while(GetWifi_cmdbuffer[strchr_pointer-GetWifi_cmdbuffer+count] != '*')//取从N之后到*号之前的指令校验，并且存入指令栈
						{
							wifi_cmdbuffer[wifi_bufindw][count]=GetWifi_cmdbuffer[strchr_pointer-GetWifi_cmdbuffer+count];
							checksum = checksum^GetWifi_cmdbuffer[strchr_pointer-GetWifi_cmdbuffer+count];
							count++;
          					}
						wifi_cmdbuffer[wifi_bufindw][count]=GetWifi_cmdbuffer[strchr_pointer-GetWifi_cmdbuffer+count];
	   				strchr_pointer = strchr(GetWifi_cmdbuffer, '*');
		  
         				if( (u16)(strtod(strchr_pointer + 1, NULL)) != checksum) //如果验证错误
	   				{
	   					if(wifiGcode_N>=0)//只请求app发送过来的指令
	   						{
     								sprintf(Printf_Buf,"Err_MisMatch:%d;\r\n",wifiGcode_N);//请求再发送
             							Wifi_Conf_Set((u8 *)Printf_Buf);
	   						}
						memset(wifi_cmdbuffer[wifi_bufindw],0,WIFIMAX_CMD_SIZE);
          				}//验证成功，已经保存到指令栈里了，不处理,更新指令栈个数
          				else
          					{
                					wifi_bufindw = (wifi_bufindw + 1)%WIFI_BUFSIZE;//清除已存入的指令
                					wifi_bufwrite_count++;
						}
			
				}
	   
      			}
              	wifi_count = 0; //clear buffer，清空指令缓存，为再次接受做准备
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
    
  Init_Usart2_arg();//取完指令清空
}



extern u16 Rx_File_Len;
u8 getWifi_File_Data(void)
{
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
    return 1;
}
void Init_Usart2_arg(void)
{
   serial2_complete_flag=0;
   wifi_buff_start  = 0;
   wifi_buff_end=0;
   memset(uart2_buff,0,1280);
}

u8 Analysis2_RxData(void)
{
	u16 i=0;
       u8 str[10];
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

	
void Get_File_Times(char * Times)//获取文件接收完成附带的时间作为文件创建时间
{
  u16 year,month,mday,hour,min;//,sec;
  u8 str[6]={0},i=0,time_out=0;
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
//u8 Mask_SDPrintButton=DISABLE;//屏蔽打印按钮20秒开关

void fetch_wifi_next_command(void)
{
        char *Times_pointer;
        //char str[10];
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
			 APP_Refresh_flag.Mask_SDPrintButton_Times=0;//屏蔽时间
			  APP_Refresh_flag.Mask_SDPrintButton=ENABLE;//屏蔽打印按钮20秒，防止打印指令冲突
			  Rev_File_Flag=0;
			Queue_wait();
			memset(Command_Buffer,0,CMDBUF_SIZE);//因为指令buffer里面有残留，所以归位以后电机会乱跑
	          	sprintf(Command_Buffer,"M23 %s\r\n",&File_DownST2.Save_FileName[5]);//SD1:/%s
          		Processing_command();
			memset(Command_Buffer,0,CMDBUF_SIZE);//因为指令buffer里面有残留，所以归位以后电机会乱跑
			strcpy(Command_Buffer,"M24\r\n");
          		Processing_command();
            		return;
	    }
         else
         	{  Init_Usart2_arg();
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
                          printf("%s\r\n",Printf_Buf);
                          File_DownST2.File_Offset_re =File_DownST2.File_Offset+1024; 
				SD_Times=0;//清0，正常发送文件清掉发送文件失败标志位，防止一直显示无SD卡
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
                                    printf("%s\r\n",Printf_Buf);
                   }
                }
                else
               { 
                   wifi_buff_start++;
                  if(wifi_buff_start>=2200)
                  {
                        Init_Usart2_arg();
                       sprintf(Printf_Buf, "M2110 %d fail\r\n",File_DownST2.File_Offset_re); 
                        Wifi_Conf_Set((u8 *)Printf_Buf);break;
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
       {  Init_Usart2_arg();
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
		SD_Times=0;//清0，正常发送文件清掉发送文件失败标志位，防止一直显示无SD卡
            Wifi_Conf_Set("M2110 start\r\n");
		delay_ms(200);
		}
            return;
       } 
	 if(strstr(uart2_buff,"M2110 Continue") != NULL)
       {
       	Init_Usart2_arg();
            Rev_File_Flag=1;
            delay_ms(100);
		SD_Times=0;//清0，正常发送文件清掉发送文件失败标志位，防止一直显示无SD卡
            Wifi_Conf_Set("M2110 ContinueStart\r\n");
		delay_ms(200);
            return;
       } 
      get_wifi_command();
     } 
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
   	if(wifirefresh_status==3)//定时更新
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

//static volatile WIFI_TX_STATA Wifi_ASK_State=WIFI_READY;
volatile WIFI_TX_STATA Wifi_ASK_State=WIFI_READY;

void Wifi_Processing_command(void)
{
       static u8  dir_num=0;//,Wifi_Send_Times=0;
	// static u16  Wifi_Out_Time=0;
     		 switch(Wifi_ASK_Factors.WIFI_COMMEND)//WIFI_ASK_COMMAND
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
                                                         my_printf(Printf_Buf);
								 break;
                                                  }		
						Wifi_ASK_State=WIFI_TX_DATA;
						 dir_num=0;
						break;
					case WIFI_TX_DATA:
						if( dir_num<Wifi_ASK_Factors.WIFI_FILE_SUM)
						{
							sprintf(Printf_Buf,"%s---%d\r\n",sd_file_name[dir_num],dir_num);
                                                         my_printf(Printf_Buf);
							Wifi_ASK_State=WIFI_WAIT_ASK;
                                                        Delay_Times=0;
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
                                                      APP_Refresh_flag.App_Sendfiles_FLAG=DISABLE;//成功发送文件标志清零
                                                      dir_num=APP_Refresh_flag.App_sendfiles_dirmun+1;
							 	 Wifi_ASK_State=WIFI_TX_DATA;
                                                      Delay_Times=0;
                                                      Wifi_ASK_Factors.WIFI_TX_TIMES=0; //Wifi_Send_Times=0;
                                                    }
							else if(Delay_Times>2500)
                                                 {
                                                          Delay_Times=0;
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



void Wifi_AP_set(void)
{
	
}
u16 Delay_Times_p=0;
void Delay_MS(u16 time)
{
    Delay_Times_p=0;
      while(1)
      {
           if(Delay_Times_p>time)
            break;
      }
}
void Wifi_Printf_Wait(void)
{
//	if(UARST_REDIRECT==UARST2_REDIRECT)
//		{
//                       Delay_MS(1000);
//		}
}
/****************串口1和WiFi共享数据判断****************************************/
void Uart1_Wifi_ShareData(void)
{
	if(WIFI_MODE==WIFI_HANDLE_DATA)
		{
			UARST_REDIRECT=UARST12_REDIRECT;
		}
}
/***************************wifi/uart select file*********************************************************/
char *strrstr(char *s, char *str)//修正字符串比较函数，从前面开始比较
{
    char *p; 
    int len = strlen(s);
  /*  for (p = s + len - 1; p >= s; p--) {//从后面检测，buff比较大的时候不合算，运算太慢
        if ((*p == *str) && (memcmp(p, str, strlen(str)) == 0)) 
            return p;
    }  */ 
	    for (p = s; p <= s + len - 1; p++) {
        if ((*p == *str) && (memcmp(p, str, strlen(str)) == 0)) 
            return p;
    }
    return NULL;
}

void SD_Delete_File(void)
{
        char *str_begin,*str_end;
        u8 i=0,Size_end=0;
        char received_name[50];
        char delete_file_name[70];
        str_begin=strchr(Command_Buffer, 'M');
        u8 ret=0;
        if(strstr(Command_Buffer, ".gcode"))
        {
             Size_end=5;
             str_end=strstr(Command_Buffer, ".gcode");
         }
        else if(strstr(Command_Buffer,".GCO"))
        {
            Size_end=3;
            str_end=strstr(Command_Buffer,".GCO");
        }
        else if(strstr(Command_Buffer,".gco"))
        {
            Size_end=3;
            str_end=strstr(Command_Buffer,".gco");
        }
        else
        {
            sprintf(Printf_Buf,"erro\r\n");
            my_printf(Printf_Buf);
        }
        if( (str_begin!=NULL) && (str_end!=NULL) )
       {
            for(i=0;i<( (str_end+Size_end)-(str_begin+3) );i++)
           {
             if((*(str_begin+4+i)>64) && (*(str_begin+4+i)<91))
                received_name[i] = *(str_begin+4+i)+32;
	     else
		received_name[i] = *(str_begin+4+i);
           }
           received_name[i]='\0';
                         sprintf(delete_file_name,"SD1:/%s",received_name);
                         ret=f_unlink(delete_file_name);
                         if(ret==0)
                         {
                            SD_detec_flag=1;
                            sprintf(Printf_Buf,"delete file succeed ok\r\n");
                            //sprintf(Printf_Buf,"file select succed! %s\r\n",delete_file_name);
                            my_printf(Printf_Buf);
                         }
                         else if(ret==FR_NO_FILE)
                         {
                             sprintf(Printf_Buf,"delete file succeed ok\r\n");
                             system_infor.files_refresh_flag=1;
                             my_printf(Printf_Buf);
                            }
                         else
                         {
                            sprintf(Printf_Buf,"delete file fail\r\n");
                            my_printf(Printf_Buf);
                         }
            
        }
        else
	{
	    sprintf(Printf_Buf,"erro\r\n");
            my_printf(Printf_Buf);
	} 
        printf("FFF:%s\r\n",received_name);
}

void SD_filePrint_select(void)
{
    char *str_begin,*str_end;
	int Size_end=0;
    char received_name[50];
    u8 i=0,k=0;
    u8 sd_file_name_index=0;
    str_begin=strchr(Command_Buffer, 'M');
	if(strrstr(Command_Buffer, ".gco")||strrstr(Command_Buffer, ".gcode")||strrstr(Command_Buffer, ".GCO") )
		{
			if(strrstr(Command_Buffer, ".gcode"))
                        {
                           Size_end=5;
                           str_end=strrstr(Command_Buffer, ".gcode")+Size_end;
                        }
			else if( strrstr(Command_Buffer, ".gco"))
                        {
                          Size_end=3;
                          str_end=strrstr(Command_Buffer, ".gco")+Size_end;
                        }
                        else if(  strrstr(Command_Buffer, ".GCO"))
                        {
                          Size_end=3;
                          str_end=strrstr(Command_Buffer, ".GCO")+Size_end;
                        }
		if( (str_begin != NULL) && (str_end != NULL) )
		{
			for(i = 0; i < (str_end-(str_begin+3)); i++)
       		{
       			if((*(str_begin+4+i)>64) && (*(str_begin+4+i)<91))
          			received_name[i] = *(str_begin+4+i)+32;
				else
					received_name[i] = *(str_begin+4+i);
       		}
       		received_name[i]='\0';
		}
		else
		{
			sprintf(Printf_Buf,"erro1\r\n");
                        my_printf(Printf_Buf);
			return;//
		}

	}
	else
		{
    			str_end=strchr(Command_Buffer, '*');
			if( (str_begin!=NULL) && (str_end!=NULL) )
    			{
        			for(i=0;i<( (str_end-1)-(str_begin+3) );i++)
        			{
        			if((*(str_begin+4+i)>64) && (*(str_begin+4+i)<91))
          				received_name[i] = *(str_begin+4+i)+32;
				else
            				received_name[i]=*(str_begin+4+i);
        			}
        		received_name[i]='\0';
    			}
			else
			{
			sprintf(Printf_Buf,"erro1\r\n");
                        my_printf(Printf_Buf);
			return;//
			}
	}
	
    //change file name to lowercase letters ,because the received file name is all lowercase
    for(i = 0; i < system_infor.sd_file_num; i++)
    {
        if(sd_file_namebuf[i][0]!='\0')
        {
            while( (sd_file_namebuf[i][k]!='\0')&&(k<FILE_NAME_SIZE))
            {
                if( (sd_file_namebuf[i][k]>64)&&(sd_file_namebuf[i][k]<91) )
                  sd_file_namebuf[i][k]+=32;
                k++;
            }
            k=0;
        }
    }
         /*   k=0;//想要比较，两者都要修改为小写字母
            
            }*/
    
    
    while((sd_file_name_index <= system_infor.sd_file_num))
    {
         
          if(strcmp(&sd_file_namebuf[sd_file_name_index][0],received_name)==0)
          {
          	     system_infor.sd_file_cmp=1;
                  system_infor.selected_file_pos=sd_file_name_index;
			delay_ms(100);
		  sprintf(Printf_Buf,"file select succed!:%s\r\n",received_name);
                 printf("%s\r\n",Printf_Buf);// my_printf(Printf_Buf);
		delay_ms(200);
	          break;
          }
          else
          {
              sd_file_name_index++;
          }
                   
    }
}
/******************
****打开开机wifi自动连接
****参数 ：0   不连接
****       1   连接
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


void Store_WIFI_Memory(void)
{
	STMFLASH_Write(WIFI_SERVER_PARAMETERS_ADDR, (u16*)Wifi_Server_message,  64);
	//STMFLASH_Write(WIFI_USER_PARAMETERS_ADDR, (u16*)Wifi_Login_Message,  64);
}
void Read_WIFI_Memory(void)
{
	STMFLASH_Read(WIFI_SERVER_PARAMETERS_ADDR, (u16*)Wifi_Server_message,  64);
	printf("SS:%s\r\n",Wifi_Server_message);
	//STMFLASH_Read(WIFI_USER_PARAMETERS_ADDR, (u16*)Wifi_Login_Message,  64);
}
void Clear_WIFI_Info(void)
{
	memset(Wifi_Server_message,0,128);
	STMFLASH_Write(WIFI_SERVER_PARAMETERS_ADDR, (u16*)Wifi_Server_message,  64);
}
/******************************修改文件时间戳*******************************************/
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
*测试是否有wifi模块
*/

u8 Exist_Wifi_Det(void)
{
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
			u8 *sp_start ;
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
		printf("HV:%s\r\n",WF_version);
		return 0;
	}
	
	return 1;
	
}

