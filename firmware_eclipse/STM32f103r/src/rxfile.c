#include "stm32f10x.h"
#include "wifi.h"
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
#include "sd_print.h"
#include "rxfile.h"
#include "serial_print.h"
#include "variable.h"

extern char Command_Buffer[1280];
extern char SD_Data_Buffer[SD_DATA_BUF_SIZE];
static char  file_name[50];;
u16 Rx_File_Len=0;
u8 Firmware_Updata_Flag=0;
void Disable_IRQHandler_Usart3(void)
{

    USART_Cmd(USART2, DISABLE);
    USART_ITConfig(USART2, USART_IT_TXE|USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART1, USART_IT_TXE|USART_IT_RXNE, DISABLE);
    TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);  
    TIM_ITConfig(TIM5,TIM_IT_Update,DISABLE);  
    TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE);  
 
    TIM_ITConfig(TIM8,TIM_IT_Update,DISABLE);  
}

u8 Get_boot_Status(FIL *ffs,char *file_paths);



/*****************************************************************************************************************/

u16 Get_RxData_Size(void)
{
	u16 i=0;
	File_DownST.RxData_Len=0;
	for(i=0;i<512;i++)
	{
		if(Command_Buffer[i]=='\n')
			return File_DownST.RxData_Len;
		else
		{ 
			File_DownST.RxData_Len++;
		}
	}
	return 0;
}

/*******************************************CRC ***********************************************************************/
 u16  crc16( char *d,  u16  len)
 {
        u8  b  =   0 ;
        u16  crc  =   0xffff ;
        u16  i, j;
        for (i = 0 ; i < len; i ++ )
        {        
                for (j = 0 ; j < 8 ; j ++ )
                {
                b = (u8)(((d[i] << j) & 0x80) ^ ((crc & 0x8000) >> 8));   
                crc <<= 1;
                if (b != 0 ) 
                    crc ^= 0x1021 ;
            }
        }
        crc = (u16)~crc;
        return crc;
 }

/*******************************************  ***********************************************************************/

// #ifndef BOARD_M301_Pro_S 
extern u16 uart_buff_start,uart_buff_start3;
extern u16 uart_buff_end,uart_buff_end3;
extern char uart_buff[UART_BUFF_SIZE];
#ifndef BOARD_M301_Pro_S 
extern char uart_buff3[UART3_BUFF_SIZE];
#endif
extern char SD_Path[512] ;
u8 get_File_Data(void)
{
  char serial_char;
     Rx_File_Len = 0; //clear buffer
    while(uart_buff_start != uart_buff_end)
   {            		
        serial_char = uart_buff[uart_buff_start];
	uart_buff_start = (uart_buff_start + 1) % UART_BUFF_SIZE;
		
       
          if(Rx_File_Len > 1200 )
          {
                Command_Buffer[Rx_File_Len] = '\0';  //terminate string
               return 0;
          }
          else
          {
                Command_Buffer[Rx_File_Len++] = serial_char;      
            }
        
     }
    return 1;
}


static char file_NM[50];

u8 Analysis_RxData(void)
{
	u16 i=0;
       char str[10];
	char *Start_Addr=  strstr(Command_Buffer,"SD1");
	if(Start_Addr!=NULL)
	{
		sprintf(File_DownST.Save_Location,"SD1");
	}
	Start_Addr+=4;
         memset(File_DownST.Save_FileName,0,sizeof(File_DownST.Save_FileName));
        for(i=0;i<50;i++)
        {
            if(*Start_Addr!=';')
                File_DownST.Save_FileName[i]=*Start_Addr;
            else
            {
                File_DownST.Save_FileName[i]='\0';
                Start_Addr++;
                    break;
            }
            Start_Addr++;
         }
	sprintf(file_NM,"%s",File_DownST.Save_FileName);
        sprintf(SD_Path, "SD1:/%s",File_DownST.Save_FileName);
        strcpy(File_DownST.Save_FileName,SD_Path);
          for(i=0;i<50;i++)
          {
             if(*Start_Addr!=';')
                    str[i]=*Start_Addr;
             else
                {
                    str[i]='\0';
                     File_DownST.File_Offset=  strtol(str, NULL, 10);
                     Start_Addr++;
                     break;
                }
              Start_Addr++;
          }
         for(i=0;i<50;i++)
          {
             if(*Start_Addr!=';')
                    str[i]=*Start_Addr;
             else
                {
                    str[i]='\0';
                     File_DownST.Rx_File_Size=  strtol(str, NULL, 10);
                     Start_Addr++;
                     break;
                }
              Start_Addr++;
          }

	//if(File_DownST.Rx_File_Size<1024&&(strstr(File_DownST.Save_FileName,".bin")!=NULL))
	//{

	//}
		 
          Start_Addr++; // Start_Addr++;  Start_Addr++; 
          for(i=0;i<File_DownST.Rx_File_Size;i++)
          {
            File_DownST.File_Data[i] = *(Start_Addr+i);
          }     
          File_DownST.File_Data[i] = '\0' ;
        //File_DownST.File_Data= Start_Addr;
        Start_Addr+=File_DownST.Rx_File_Size;
        File_DownST.Crc_Data=(*Start_Addr)<<8|(*(Start_Addr+1));
       return 0;
}


void WDGOpen_Rxfile(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		

	TIM_TimeBaseStructure.TIM_Period = 39999;      // 4s  40000*7200/72000000
	TIM_TimeBaseStructure.TIM_Prescaler = 7199;	   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	
	TIM_Cmd(TIM5, DISABLE);
	TIM_Cmd(TIM6, DISABLE);
	TIM_Cmd(TIM7, DISABLE);
	TIM_Cmd(TIM8, DISABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
	USART_ITConfig(USART3, USART_IT_TXE, DISABLE);

	
	TIM_ClearFlag(TIM1, TIM_FLAG_Update);
	TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	
}
void WDGClose_Rxfile(void)
{

TIM_Cmd(TIM1, DISABLE);
TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);  
TIM_Cmd(TIM5, ENABLE);
TIM_Cmd(TIM6, ENABLE);
TIM_Cmd(TIM7, ENABLE);
TIM_Cmd(TIM8, ENABLE);
USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
}

FIL fnew3;		
extern FATFS fats;
extern char SD_Driver_Number[5];
extern u8 SD_detec_flag;
char File_SD_Name[50];
extern u8 TransferError_flag;

u8 AddData_To_File(char *data,u16 len,char *filename,long long offset)
{
    UINT bw;
    u8 res =0,res1=0;
    TransferError_flag=0;
    WDGOpen_Rxfile();
    if(offset>0)
        res = f_open(&fnew3,filename,FA_OPEN_EXISTING|FA_WRITE );// FA_OPEN_ALWAYS | |FA_READ
    else
    {
        res = f_open(&fnew3,filename, FA_CREATE_ALWAYS|FA_WRITE); 
        system_infor.files_refresh_flag=1;
        sprintf(File_SD_Name,"%s",filename);
    }
    if(res == FR_OK)
    {
        res1=f_lseek(&fnew3,offset);
        if(res1||fnew3.fptr != offset)
        {     
            f_close(&fnew3); 	
#ifdef WIFI_MODULE
            Wifi_Conf_Set("M2110 fail SD:Err\r\n");
#endif
            WDGClose_Rxfile();
            return -1;
        }
        res = f_write(&fnew3,data, len, &bw); 
        if(res==FR_OK)
        {
            f_close(&fnew3);
            WDGClose_Rxfile();
            return 1;
        }
    }
    else
    {  
        f_close(&fnew3); 		
        SD_detec_flag = 1;
        SD_File_Detect();
        WDGClose_Rxfile();
        return -1;
    }

    f_close(&fnew3);
    WDGClose_Rxfile();
    return -1;   
}
/*****************Generate the firmware update configuration files*********************/
void AddData_To_ConfigFile(void)
{
    UINT bw;//br;
    u8 res =0;
    char str[256]  ;
    res = f_open(&fnew3,"SD1:/config.txt", FA_CREATE_ALWAYS|FA_WRITE); 
    sprintf(str,"boot:1;file_path:%s;",file_NM);
    if(res == FR_OK)
    {
        f_lseek(&fnew3,0);
        res = f_write(&fnew3,str, strlen(str), &bw); 
        if(res==FR_OK)
        {
            f_close(&fnew3);
        }
    }
    f_close(&fnew3);
}



/**********************transmission bin file  boot upgrade*****************************/
u8  Sent_Boot_Binfile(void)
{
#ifndef BOARD_M301_Pro_S 
    FIL fs;
    char Send_str[1024] ;
    u16 Data_size=0;
    u8 Send_flag=0,Send_data_state=0;
    static u16 Send_Times=0,Send_data_crc=0;
    u32 off_add=0,Print_File_Br;
    u16 read_size=0,crc_volues,file_crc_sum=0;
    char f_status1,f_status2,buf[512];
    char file_pathss[60];
    static u16 Save_Delay_Time;
    if(Get_boot_Status(&fs,file_pathss)!=1)
    {
        return 0;
    }
    else
    {
        Disable_IRQHandler_Usart3();
        for(u8 i=0;i<5;i++)
        {
            if( f_open(&fs,file_pathss,FA_OPEN_EXISTING|FA_READ )!=FR_OK)
            {
                f_close(&fs);
                f_mount(&fats,SD_Driver_Number,1);
                delay_ms(1);
            }
            else
            {
                file_crc_sum=0;
                for(u16 j=0;j<=fs.fsize/512;j++)
                {	
                    if(fs.fsize>=(off_add+512))
                        read_size=512;
                    else
                    {
                        read_size=fs.fsize-off_add;
                    }
                    if(f_lseek(&fs,off_add)!=FR_OK)
                    {
                        f_close(&fs);
                        break;
                    }
                    if(f_read(&fs,SD_Data_Buffer,read_size, &Print_File_Br)==FR_OK)
                    {
                        for(u16 h=0;h<read_size/2;h++)
                        {
                            file_crc_sum += SD_Data_Buffer[h*2]<<8|SD_Data_Buffer[h*2+1];
                        }
                        if(read_size<512)
                        {
                            f_close(&fs);
                            off_add=0;
                            read_size=0;
                            f_lseek(&fs,off_add);
                            printf("file_crc_sum=0x%x\r\n",file_crc_sum);
                            break;
                        }
                    }
					
                    off_add=off_add+512;
                }
                f_close(&fs);
                break;
            }
        }
    }
    Firmware_Updata_Flag=1;
    clear_Uart();
    while(1)
    {
        switch(Send_data_state)
        {
            case S_HEAD:
                memset(Send_str,0,1024);
                sprintf(Send_str,"<M2121 start:A30.bin;>\r\n");
                Data_size=strlen(Send_str);//25
                Send_data_state=W_HEAD;
                Save_Delay_Time=Get_CDelay();
                Send_Times=0;
                Send_flag=1;
                break;
            case W_HEAD:
                if(Get_Delay_ms(Save_Delay_Time)>40)
                {
                    Save_Delay_Time=Get_CDelay();
                    Send_Times++;
                    if(strstr(uart_buff3,"M2121 ok")!=NULL)
                    {
                        clear_Uart();
                        Send_data_state=S_DATA;
                        break;
                    }
                    if(strstr(uart_buff3,"M2121 fail")!=NULL)
                    {
                        clear_Uart();
                        Send_data_state=S_HEAD;
                        break;
                    }
                    if(Send_Times>50)
                    {
                        Send_data_state=S_HEAD;
                    }
                }
            break;
            case S_DATA:
                if( f_open(&fs,file_pathss,FA_OPEN_EXISTING|FA_READ )!=FR_OK)
                {
                    f_close(&fs);
                    f_mount(&fats,SD_Driver_Number,1);
                    delay_ms(1);
                    break;
                }
                printf("data :%d\r\n",off_add);
                if(fs.fsize>=(off_add+512))
                    read_size=512;
                else
                {
                    read_size=fs.fsize-off_add;
                }
                f_status1 = f_lseek(&fs,off_add);
                if(f_status1!=FR_OK)
                {
                    f_close(&fs);
                    break;
                }
                f_status1 = f_read(&fs,SD_Data_Buffer,read_size, &Print_File_Br);
                if(f_status1!=FR_OK)
                {
                    f_close(&fs);
                    break;
                }
                delay_ms(1);
                f_status2 = f_lseek(&fs,off_add);
                if(f_status2!=FR_OK)
                {
                    f_close(&fs);
                    break;
                }
                f_status2 = f_read(&fs, buf, read_size, &Print_File_Br);
                if(f_status2!=FR_OK)
                {
                    f_close(&fs);
                    break;
                }
                if(f_status2!=FR_OK||f_status1!=FR_OK)
                {
                    printf("f_status,f_status1,f_status2:!=FR_OK\r\n");
                    f_close(&fs);
                    break;
                }
                if(strncmp(SD_Data_Buffer,buf,read_size)==0)
                {
                    for(u16 k=0;k<Print_File_Br;k++)
                    {
                        Send_data_crc+=SD_Data_Buffer[k];
                    }

                    f_close(&fs);
                    memset(Send_str,0,1024);
                    memset(buf,0,512);
                    sprintf(Send_str,"<M2121 file:%s;off:%d;size:%d;data:",file_name,off_add,read_size);
                    Data_size=strlen(Send_str);
                    memcpy(&Send_str[Data_size],SD_Data_Buffer,read_size);
                    Data_size=Data_size+read_size;
                    crc_volues=crc16(&Send_str[1],Data_size-1);
                    sprintf(buf,";crc:%d;@#*>\r\n",crc_volues);

                    sprintf(&Send_str[Data_size],";crc:%d;@#*>\r\n",crc_volues);
                    Data_size=Data_size+strlen(buf);

                    Send_flag=1;
                    Send_data_state=W_DATA;
                    Send_Times=0;
                    Save_Delay_Time=Get_CDelay();

                }
                else
                {
                    f_close(&fs);
                }
                delay_ms(1);
            break;
            case W_DATA:
                if(Get_Delay_ms(Save_Delay_Time)>20)//17
                {
                    Save_Delay_Time=Get_CDelay();
                    Send_Times++;
                    if(strstr(uart_buff3,"M2121 ok")!=NULL)
                    {
                        clear_Uart();
                        Send_data_state=S_DATA;
                        if(fs.fsize>(off_add+512))
                            off_add=off_add+512;
                        else
                        {
                            delay_ms(100);
                            Send_data_state  = S_END;
                        }
                        break;
                    }
                    if(strstr(uart_buff3,"M2121 fail")!=NULL)
                    {
                        printf("data :fail\r\n");
                        delay_ms(5);
                        clear_Uart();
                        Send_data_state=S_DATA;
                        break;
                    }
                    if(Send_Times>100)
                    {
                        Send_Times=0;
                        Send_data_state=S_DATA;
                    }
                }
            break;
            case S_END:
                memset(Send_str,0,1024);
                sprintf(Send_str,"M2121 end crc:%d;\r\n",file_crc_sum);//,Send_data_crc
                printf("%s\r\n",Send_str);
                Data_size=25;
                Send_data_state=W_END;
                Save_Delay_Time=Get_CDelay();
                Send_flag=1;
                Send_Times=0;
                delay_ms(50);
            break;
            case W_END:
                if(Get_Delay_ms(Save_Delay_Time)>20)
                {
                    Save_Delay_Time=Get_CDelay();
                    Send_Times++;
                    if(strstr(uart_buff3,"M2121 ok")!=NULL)
                    {
                        clear_Uart();
                        finish_boot_send();
                        delay_ms(500);
                        NVIC_SystemReset();
                        return 0;
                    }
                    if(strstr(uart_buff3,"M2121 fail")!=NULL)
                    {
                        clear_Uart();
                        Send_data_state=S_END;
                        break;
                    }
                    if(Send_Times>80)
                    {
                        Send_data_state=S_END;
                    }
                }
				break;
            case O_TIME:
				
            break;
				
            default :break;
        }

        if(Send_flag==1)
        {
            clear_Uart();
            Send_Data_BIN(Send_str,Data_size);
            Send_flag=0;
        }
    }
#endif
}


void clear_Uart(void)
{
#ifndef BOARD_M301_Pro_S 
	 uart_buff_start3=0;
	 uart_buff_end3=0;
	 memset(uart_buff3,0,sizeof(uart_buff));
#endif
}
/*****************Read the configuration file to read whether to upgrade******************/
u8 Get_boot_Status(FIL *ffs,char *file_paths)
{
	FIL fs;
	u8 res =0,ret=0;
	u16 data_size=0;
	u32 Print_File_Br;
	
	char *spp;
	res = f_open(&fs,"SD1:/config.txt",FA_OPEN_EXISTING|FA_READ );
	if(res==FR_OK)
	{
		if(fs.fsize<512)
			data_size=fs.fsize;
		else
			data_size=512;
		res = f_read(&fs,SD_Data_Buffer,data_size, &Print_File_Br);
		 f_close(&fs);	
		 if(strstr(SD_Data_Buffer,"boot:1")!=NULL)
		 {
		 	       memset(file_name,0,50);
		 		spp=strstr(SD_Data_Buffer,"file_path:");
		 		spp=spp+10;
                         for(u8 i=0;i<50;i++)
		 	{
				if(*spp==';')
					break;
				file_name[i]=*spp;
				spp++;
		 	}
			if(strstr(file_name,"bin")!=NULL)
			{
				 sprintf(file_paths,"SD1:/%s",file_name);
				 my_printf("%s\r\n",file_paths);
				 ret=1;
			}
		 }
	}
    f_close(&fs);
	return ret;
}



u8 finish_boot_send(void)
{
	FIL fnew4;
	u8 res =0;
	u32 pp;
	//while(1)
	//{
		//char stsstr[]="boot:0";
		//res = f_open(&fnew4,"SD1:/config.txt",FA_OPEN_EXISTING|FA_WRITE );
		//f_write(&fnew4,stsstr,30,&pp);
		//f_close(&fnew4);
		res =  f_unlink("SD1:/config.txt");
		if(res == FR_OK )
			return 0;
		else
			f_mount(&fats,SD_Driver_Number,1);
		char stsstr[]="boot:0";
		res = f_open(&fnew4,"SD1:/config.txt",FA_OPEN_EXISTING|FA_WRITE );
		if(res ==FR_OK )
		{
			f_write(&fnew4,stsstr,30,&pp);
		}
		f_close(&fnew4);
	//}
	return 0;
}


void Send_Data_BIN(char *data,u16 size)
{
	for(u16 i=0;i<size;i++)
	{
	  	USART_SendData(USART3, (unsigned char) *(data+i));
     	 	while (!(USART3->SR & USART_FLAG_TXE));
	}
}






