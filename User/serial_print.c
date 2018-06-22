#include "serial_print.h"
#include "wifi.h"
#include "rxfile.h"
#include "variable.h"
#include "fat.h"


static int bufwrite_count = 0,bufwrite_count3 = 0;
static int bufread_count = 0,bufread_count3 = 0;
static char serial_char,serial_char3;
static bool comment_mode = false,comment_mode3 = false;
static int serial_count = 0,serial_count3 = 0;
static bool fromsd3[BUFSIZE3];
static int buflen = 0,buflen3 = 0;
static char *strchr_pointer; // just a pointer to find chars in the command string like X, Y, Z, E, etc
static u8 null_count = 0,null_count3 = 0;
extern char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
char cmdbuffer3[BUFSIZE3][MAX_CMD_SIZE];
int bufindr,bufindr3;    //Abuffer OA
int bufindw,bufindw3;    //D'buffer OA
long gcode_N3, gcode_LastN3, Stopped_gcode_LastN3;
static long gcode_LastN = 0;
extern long gcode_N, Stopped_gcode_LastN;
extern  char Command_Buffer[CMDBUF_SIZE];
extern u8 CntCmdProc;
extern u8 serial_ack_flag;
int command_time = 1200;
int serial_print_time;

char uart_buff[UART_BUFF_SIZE],uart_buff3[UART3_BUFF_SIZE];
u16 uart_buff_start = 0,uart_buff_start3 = 0;
u16 uart_buff_end = 0,uart_buff_end3 = 0;
extern u8 serial_connect_flag;
u8 serial_connect_flag3=DISABLE;

u8 SerialPrinting_error=0;//?????????????
u16 uart_file_crc=0;
extern FIL fnew3;	
extern u8 Firmware_Updata_Flag;



void Disable_IRQHandler(void)
{
    USART_Cmd(USART2, DISABLE);
    USART_ITConfig(USART2, USART_IT_TXE|USART_IT_RXNE, DISABLE);
    USART_ITConfig(USART3, USART_IT_TXE|USART_IT_RXNE, DISABLE);
    TIM_ITConfig(TIM1,TIM_IT_Update,DISABLE);  
    TIM_ITConfig(TIM5,TIM_IT_Update,DISABLE);  
    TIM_ITConfig(TIM7,TIM_IT_Update,DISABLE);  
    TIM_ITConfig(TIM8,TIM_IT_Update,DISABLE);  
}


void serial_command_timeout(void)
{
	
	  printf("erro5:%ld,%ld\r\n",gcode_LastN,gcode_N);
         printf("Resend:%ld\r\n",gcode_LastN + 1);
	  serial_count = 0;
	  command_time = 100;
}

void get_serial_command(void)
{
  
       char gcode_line[16] = {0};  
       serial_char = '\n';
       static u8 Resend_Times=0;
       static u8 Resend_Flag=0;
       while(uart_buff_start != uart_buff_end)
       {            		
             serial_char = uart_buff[uart_buff_start];
	      uart_buff_start = (uart_buff_start + 1) % UART_BUFF_SIZE;
	      
             if(serial_char == 0x0) 
             {
                  null_count++;
                  if(null_count >= 100) 
                  {
                      //NVIC_SystemReset();
                      uart_buff_start=0;
                      uart_buff_end=0;
                      memset(uart_buff,0,sizeof(uart_buff));
                  }
                       continue;
             }
             else
             {  
             		if(serial_char == ':' && system_infor.serial_printf_flag==1 )
             		{
                               null_count = 0; 
                                continue;
             		}
                    if(serial_char == '\n' || serial_char == '\r' || serial_count >= (MAX_CMD_SIZE - 1) )//serial_char == ':' && comment_mode == false) ||
                    {
                           if(!serial_count)    //
                           {
                                  comment_mode = false;   
                                  continue;
                           }
                           cmdbuffer[bufindw][serial_count] = 0;  //terminate string
                           if(!comment_mode)
                          {
                                  comment_mode = false;  //for new command
                                  if(strchr(cmdbuffer[bufindw], 'N') != NULL)
                                  {
                                         strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
                                         gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
                                         if((gcode_N != gcode_LastN + 1))   //Check the Line Number
                                         {
                                                 strchr_pointer = strchr(cmdbuffer[bufindw], 'M');    //M110: Set Current Line Number
                                                  u16 value = (u16)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL));
                                                  if((strchr_pointer != NULL) && (value == 110))
                                                  {
								 gcode_LastN = gcode_N;
                                                  }
                                                  else
                                                  {   
                                                      
                                                      printf("erro1:%ld,%ld\r\n",gcode_LastN,gcode_N);
                                                 
                                                  }
                                       }
                                      if(strchr(cmdbuffer[bufindw], '*') != NULL)  //check the checksum
                                      {
                                             u16 checksum = 0,count=0;
				                  u16 getchecksum = 0;
                                           
                                             while(cmdbuffer[bufindw][count] != '*') 
                                             {
                                                     checksum = checksum^cmdbuffer[bufindw][count++];
                                              }
                                             strchr_pointer = strchr(cmdbuffer[bufindw], '*');
				                  getchecksum = (u16)strtol(&Command_Buffer[strchr_pointer - Command_Buffer + 1], NULL, 10);
                                             if(checksum != (u16)strtol(&Command_Buffer[strchr_pointer - Command_Buffer + 1], NULL, 10)) 
                                             {      
                                                      printf("erro2\r\n");
                                              }
                                              else
                                              {
								 bufindw = (bufindw + 1)%BUFSIZE;
                            				 buflen += 1;
                             				 bufwrite_count++;
                             				 gcode_LastN = gcode_N;  
                             				 command_time = 1200;
                                                      serial_count = 0; //clear buffer,
                                                      return;
                                              }
                                     }
                                    else if(system_infor.serial_printf_flag==1)
                                    {            
                                             printf("erro3\r\n");
                                    }                                      
                             }
                             else
                             {
					 if(strchr(cmdbuffer[bufindw], '*') == NULL)  //check the checksum
					 {
						 bufindw = (bufindw + 1)%BUFSIZE;
                            		buflen += 1;
                             		bufwrite_count++;
                             		gcode_LastN = gcode_N;  
                             		command_time = 1200;
                                         serial_count = 0; //clear buffer,
                              		 return;
					 }
                             }
                             delay_ms(5);     
                              bufwrite_count=0;
                              bufread_count=0;
                              bufindr = 0;   
                              bufindw = 0;  
                              uart_buff_start=0;
                              uart_buff_end=0;
                              printf("Resend:%ld\r\n",gcode_LastN + 1);
                               uart_buff_start = uart_buff_end;
                               serial_count = 0;
                               command_time = 100;
                               delay_ms(5);  
                               return;
                             }	  
                           }
                         else
                         {
                             if(serial_char == ';') comment_mode = true;
                             if(!comment_mode)
                             { 
                                        cmdbuffer[bufindw][serial_count++] = serial_char; 
               				command_time = 1200;
                                        null_count = 0;               
                             }
                         
             		      }
             }
    } 
    return;
}

void get_serial3_command(void)
{
  
       //char gcode_line[16] = {0};  
       serial_char3 = '\n';
       while(uart_buff_start3 != uart_buff_end3)
       {            		
             serial_char3 = uart_buff3[uart_buff_start3];
	     uart_buff_start3 = (uart_buff_start3 + 1) % UART3_BUFF_SIZE;
		
             if(serial_char3 == 0x0) 
             {
                  null_count3++;
                  if(null_count3 >= 100) 
                  {
                    //NVIC_SystemReset();
                    //USART_ITConfig(USART3,USART_IT_RXNE,DISABLE);
                    printf("Uasrt3 error\r\n");
                    //Port_to_Slave_init();
                    uart_buff_start3=0;
                    uart_buff_end3=0;
                    memset(uart_buff3,0,sizeof(uart_buff3));
                  }
                       continue;
             }
             else
             {  
                    if(serial_char3 == '\n' || serial_char3 == '\r' || (serial_char3 == ':' && comment_mode3 == false) ||serial_count3 >= (MAX_CMD_SIZE - 1) )
                    {
                           if(!serial_count3)    //if empty line
                           {
                                  comment_mode3 = false;   //for new command
                                  continue;
                           }
                           cmdbuffer3[bufindw3][serial_count3] = 0;  //terminate string
                           if(!comment_mode3)
                          {
                                  comment_mode3 = false;  //for new command
                             	  
                             bufindw3 = (bufindw3 + 1)%BUFSIZE3;
                             buflen3 += 1;
                             bufwrite_count3++;
                           }
                           serial_count3 = 0; //clear buffer
                           return;
                         }
                         else
                         {
                           if(serial_char3 == ';') comment_mode3 = true;
                             if(!comment_mode3)
                             { 
                               cmdbuffer3[bufindw3][serial_count3++] = serial_char3; 
                               null_count3 = 0;               
                             }
                         }
             }
    
    } 
    return;
}

void Init_Usart1_arg(void)
{
   serial_connect_flag=0;
   uart_buff_start  = 0;
   uart_buff_end=0;
   memset(uart_buff,0,1280);
}
extern void Set_nozzle_enable(void);
void fetch_next_command(void)
{
        char str[10];
	static u8 printf_flag = 1;
        u8 ret=0;
        u16 crc1,crc2;
        u16 num=0;
        u32 br;
        UARST_REDIRECT=UARST1_REDIRECT;
        if(uart_buff_start==uart_buff_end)
        {
           serial_connect_flag=0;
           uart_buff_start=0;
           uart_buff_end=0;
           return;
        }
     if(File_DownST.File_Flag==1)    
     {
         if(strstr(uart_buff,"M2100")!=NULL)
         {
             while(1)
            {
               if(uart_buff[uart_buff_start]=='M'&&uart_buff[uart_buff_start+1]=='2'&&uart_buff[uart_buff_start+2]=='1'&&uart_buff[uart_buff_start+3]=='0'&&uart_buff[uart_buff_start+4]=='0')
               {
                    get_File_Data();
                    if(crc16(Command_Buffer,Rx_File_Len-3)==(Command_Buffer[Rx_File_Len-3]<<8|Command_Buffer[Rx_File_Len-2]))
                    {
                         Analysis_RxData();
                         ret=AddData_To_File(File_DownST.File_Data,File_DownST.Rx_File_Size,File_DownST.Save_FileName,File_DownST.File_Offset);
                    }
                       
                   if(ret==1)
                  {
                  	if(File_DownST.Rx_File_Size<1024&&(strstr(File_DownST.Save_FileName,".bin")!=NULL))
			{
				AddData_To_ConfigFile();
				
			}
			/**************************************************************************/
			crc1=0;
			//for(num=0;num<File_DownST.Rx_File_Size;num++)
			//{
				crc1=crc16(File_DownST.File_Data,File_DownST.Rx_File_Size);
			//}
			if(f_open(&fnew3,File_DownST.Save_FileName, FA_OPEN_EXISTING|FA_READ)==FR_OK)
			{
				   if(f_lseek(&fnew3,File_DownST.File_Offset)==FR_OK)
				   {
				   		memset(File_DownST.File_Data,0,sizeof(File_DownST.File_Data));
					      if(f_read(&fnew3,File_DownST.File_Data, 512, &br)==FR_OK)
					      {
					      		if(File_DownST.Rx_File_Size>512)
					      		{
								 if(f_read(&fnew3,&File_DownST.File_Data[512], File_DownST.Rx_File_Size-512, &br)!=FR_OK)
								 {
									 sprintf(str,"fail");
								        break;
								 }
					      		}
							crc2=0;
							f_close(&fnew3);

							//for(num=0;num<File_DownST.Rx_File_Size;num++)
							//{
								crc2=crc16(File_DownST.File_Data,File_DownST.Rx_File_Size);
							//}
							if(crc1==crc2)
							{
								 sprintf(str,"success");
								 break;
							}
			
					      }
					      else
						   f_close(&fnew3);

				   }
				   else
					f_close(&fnew3);

			}
			else
				f_close(&fnew3);


			/**************************************************************************/

                      sprintf(str,"fail");break;
                   }
                   else
                   {
                      sprintf(str,"fail");break;
                   }
                }
                else
               { 
                   uart_buff_start++;
                  if(uart_buff_start==uart_buff_end)
                  {
                      sprintf(str,"fail");break;
                  }
                  if(uart_buff_start>=1280)
                      uart_buff_start=0; 
                 }
            }
            Init_Usart1_arg();
            printf("%s\r\n",str);
        }
        else
        {
             File_DownST.File_Flag=0;
             Init_Usart1_arg();
             printf("fail\r\n");
             return;
         }
     }
    else
    {
     if(strstr(uart_buff,"M2100")!=NULL)
      {
       delay_ms(300);
        File_DownST.File_Flag=1;
        Firmware_Updata_Flag =1;
        Disable_IRQHandler();
        Init_Usart1_arg();
        printf("fail\r\n");
        return;
      }
     get_serial_command();
    if(bufwrite_count>bufread_count)
    {         
        bufwrite_count = bufwrite_count - bufread_count;
        bufread_count = 0;
        for (int i=0; i<CMDBUF_SIZE; i++)
        {
            Command_Buffer[i] = cmdbuffer[bufindr][i];
        }

        bufindr = (bufindr + 1)%BUFSIZE;
        bufread_count++;
        Processing_command();
        if(system_infor.serial_ack_flag) 
	{
             printf("\r\nok\r\n");
	      delay_ms(1);
        }
    }
    }
   
}


void Display_command(void)
{
	char LCD_Command[96];
        UARST_REDIRECT=UARST3_REDIRECT;
        if(uart_buff_start3==uart_buff_end3)
        {
           serial_connect_flag3=0;
           uart_buff_start3=0;
           uart_buff_end3=0;
           return;
        }
      
     get_serial3_command();
    if(bufwrite_count3>bufread_count3)
    {         
        bufwrite_count3 = bufwrite_count3 - bufread_count3;
        bufread_count3 = 0;
        for (int i=0; i<MAX_CMD_SIZE; i++)
        {
            LCD_Command[i] = cmdbuffer3[bufindr3][i];
        }

        bufindr3 = (bufindr3 + 1)%BUFSIZE3;
        bufread_count3++;
        command_process(LCD_Command);
        if(system_infor.serial_ack_flag) 
	{
             my_printf("ok  \r\n");

	     delay_ms(1);
        }
    }
    else
    {
	bufwrite_count3=0;
	bufread_count3=0;
    }
}

