#include "sd_print.h"
#include "wifi.h"
#include "step_motor.h"
#include "Dgus_screen.h"
#include "rxfile.h"
#include "command.h"
#include "recovery_print.h"
#include "variable.h"
#include "data_handle.h"


FATFS fats;
FIL Print_File;
u32 Print_File_Br;
DIR SD_Dirs;
FILINFO Fileinfo;

char *gfile;
char SD_Driver_Number[5]={"SD1:"};
char SD_Path[512] = {"SD1:"};


char *sd_file_names[128]={""};

DWORD fptr_buf;
DWORD sd_line_num = 0;

DWORD sd_byte = 0; 

static bool Comment_Mode;
extern char Command_Buffer[CMDBUF_SIZE];
char sd_command_buffer[CMDBUF_SIZE];
char sd_command_buffer_befor[CMDBUF_SIZE];
char sd_command_buffer_befor_befor[CMDBUF_SIZE];

vu16 SD_Data_Buffer_Count = 0;
u8 SD_Data_Buffer[SD_DATA_BUF_SIZE];
int file_num;

u8 SD1_flag = 0xFF;
u8 SD_detec_flag = 0xFF;


u16  Sum_layers;
float layer_high=0.2;










extern __IO u8 Print_Time_M;
extern __IO u16 Print_Time_H;


extern __IO u8 Receive_Buffer[64];
extern __IO  u32 Receive_length ;
extern __IO  u32 length ;


extern SD_CardInfo SDCardInfo;
  


extern u8 serial_connect_flag;
extern u16 fan_current_speed;	//风扇当前速度
u8 dir_index;


DIR dir;
FILINFO fno;
char *file_name;
char *file_type;
char lfn[FILE_NAME_SIZE];
char sd_file_name[FILE_NUM][FILE_NAME_SIZE]={""};//SD卡文件名缓存数组
char sd_file_namebuf[FILE_NUM][FILE_NAME_SIZE]={""};//SD卡文件名缓存数组，由于上位机传送文件名全部为小写，所以此数组用于转换文件名为小写后与上位机选择SD卡文件的文件对比，然后再与sd_file_name数组对应进行SD卡打印
float file_size[FILE_NUM]={0};//文件大小，用于显示SD卡打印进度的计算
u32 sd_file_time[FILE_NUM];

  u8 f_status;

u16 SDRead_TimeOut;//,SDRead_TimeOut1;//SD卡打印中暂停时间过长退出
extern void Set_Up_Data_Flag(void);
  
u8 Get_SD_Data_Buffer(void)
{
//  u8 f_status;
  DWORD br;
  u8 buf[SD_DATA_BUF_SIZE];
  FIL file_buf;
  u16 time_out;

  time_out = 1500;
  SDRead_TimeOut=0;
  while(1)
  {  
        if(SDRead_TimeOut>4000)//如果暂停时间超过4s,停止读取数据发送错误信息
        {  
			fptr_buf = Print_File.fptr;     //////////////////////////////////////
			//fptr_buf = br;
			f_close(&Print_File); 
			return 0xfa;
	}
    
      f_status = f_lseek(&Print_File,fptr_buf);
		if(f_status||Print_File.fptr != fptr_buf)//检查文件大小是否已被正确扩展 
			goto ReOpenFile;//注意这是无条件转移，必须保持在函数体内才能看清程序流程
      f_status = f_read(&Print_File, SD_Data_Buffer, SD_DATA_BUF_SIZE, &Print_File_Br);
		if(f_status)
			goto ReOpenFile;
      br = Print_File.fptr;

		

		f_status = f_lseek(&Print_File,fptr_buf);
		if(f_status||Print_File.fptr != fptr_buf)//检查文件大小是否已被正确扩展 
			goto ReOpenFile;
      f_status = f_read(&Print_File, buf, SD_DATA_BUF_SIZE, &Print_File_Br);
	 	if(f_status)
			goto ReOpenFile;

	 
      if(strncmp(SD_Data_Buffer,buf,SD_DATA_BUF_SIZE)==0)
     {
              if(f_status!=FR_OK)
              {
                    char test_Str[]="f_status!=FR_OK";
                     u8  text_length = strlen(test_Str);
	            

              }
			else
				time_out = 1500;
			if(Print_File.fptr != fptr_buf+512||br != fptr_buf+512)
				goto ReOpenFile;
            break;   
      }
      else
      {
      
            
        if(system_infor.print_file_size<=sd_byte+512) 
			break;      
   	ReOpenFile: f_close(&Print_File);
        f_status = f_open(&Print_File,Systembuf_Infos.printer_file_path,FA_OPEN_EXISTING|FA_READ);
        if(f_status != FR_OK && time_out != 0)
        {
            time_out--;
				if(time_out <= 1)
				{
					f_close(&Print_File);
                return f_status;
        }
		else if(time_out ==980)
					SD_Init();	//重新初始化SD卡
      }
	  }
   }
   fptr_buf = Print_File.fptr;     //////////////////////////////////////
   //fptr_buf = br;
  // f_close(&Print_File);

    return f_status;
}

void Get_command(void)//得到一条完整的命令
{
  u8 sd_char;
  u8 count = 0;
  //u8 i = 0;               // dxc  2015-03-10
  //u8 check_sum = 0;       // dxc  2015-03-10
  //u8 check_sum_std = 0;  // dxc  2015-03-10
  u8 sd_state;
 
  u8 cmdbuf[CMDBUF_SIZE] = {0};
  
  do{
    if((SD_Data_Buffer_Count >= SD_DATA_BUF_SIZE) || (SD_Data_Buffer_Count == 0))
    {
      SD_Data_Buffer_Count = 0;    
      sd_line_num = 0;
      sd_byte = fptr_buf;
      sd_state = Get_SD_Data_Buffer();   
      //sd_state = f_read(&Print_File, SD_Data_Buffer, SD_DATA_BUF_SIZE, &Print_File_Br);
      
      if(sd_state != SD_OK)
      {
       sprintf(Printf_Buf, "文件读取错误!!\r\n"); my_printf(Printf_Buf);  //dxc
       sprintf(Printf_Buf, "文件读取错误代码:%d\r\n",sd_state);  my_printf(Printf_Buf); //dxc
        system_infor.sd_print_status = SD_PRINT_PAUSE_F;  
        Add_Message(SD_EXCEPTION);
        printf("%s\r\n",Printf_Buf);
        break;
      }
      else
      {
          system_infor.print_percent = (fptr_buf*100)/system_infor.print_file_size;
      }
     }
      
    sd_char = SD_Data_Buffer[SD_Data_Buffer_Count];
    sd_byte++;
    SD_Data_Buffer_Count++;
    //printf("%c",serial_char);
    switch(sd_char)
    {
      case '\0':break;
      case ';':
              Comment_Mode = true;
              break;
      
      case '\r'://换行
      case '\n'://新行
              //Command_Buffer[count] = '\0';//保证最后一个字节为0x00
              cmdbuf[count] = '\0';//保证最后一个字节为0x00
              Comment_Mode = false;
              if(count == 0)//提高空一行时的运行效率
                sd_char = 0;
              sd_line_num++;
              break;
			  
      default:	
              if(!Comment_Mode)
              {
                //Command_Buffer[count] = serial_char;
                cmdbuf[count] = sd_char;
                count = (count+1)%CMDBUF_SIZE;
              }
              break;
    }
    if(sd_byte >= system_infor.print_file_size) 
    {
        system_infor.system_status = IDLE;  //文件读取完毕，跳出打印循环
        printf("file finish\r\n");
        break;
    }
          
  }while((sd_char != '\r')&&(sd_char != '\n')); 

 static u8 fff=0;
  memset(Command_Buffer,0,128);  //清空缓存，复制指令到指令缓存
   strncpy(Command_Buffer,cmdbuf,128);//cmdbuf

}

void sd_write_error_log(void)
{
    u8 buff[256];
    memset(buff,0,256);
    sprintf(buff, "\r\n\r\n%s\r\n%s\r\n%s\r\n%s\r\n\r\n", Command_Buffer,sd_command_buffer,sd_command_buffer_befor,sd_command_buffer_befor_befor);
    AddData_To_File(SD_Data_Buffer,512,"SD1:/log.txt",(long long)0);
    AddData_To_File(buff,256,"SD1:/log.txt",(long long)512);
}

/********************************LCD/UART/WIFI control SD print*****************************************************/

void Print_3D_SD(void)
{
  u16 i=0,time_h=0,time_m=0;
  u8 setting_locate_mode;
  u16 time_out;
  u8 file_status;
  static u8 fff=0;
  switch(system_infor.sd_print_status)
  {
     case SD_PRINT_START:
          {        
            
          	if(Print_File.fs!=NULL)
			f_close(&Print_File);
            time_out = 50;
            file_status = f_open(&Print_File,Systembuf_Infos.printer_file_path,FA_OPEN_EXISTING|FA_READ);
            while(file_status != FR_OK && time_out > 0)
            {
				f_close(&Print_File);
				file_status = f_open(&Print_File,Systembuf_Infos.printer_file_path,FA_OPEN_EXISTING|FA_READ);
				time_out--;
            }
            if(file_status == FR_OK)
            {
            	   get_print_layer();
            	   SD_Printdata_init();                

                SD_Data_Buffer_Count = 0;      
                file_num = 0; 
                
                Block_clean();
                Current_Block_Clean();
                Set_Up_Data_Flag();
                delay_ms(20);
                Add_Message(PRINTING_STATUS);
                return;
            }
            else
            {
              f_close(&Print_File);
              system_infor.system_status = IDLE;
		 system_infor.sd_print_status = SD_PRINT_PAUSE_F;
              strcpy(SD_Path,SD_Driver_Number); 
            }
          }
          break;
    case SD_PRINTING:  
          if(system_infor.system_status == PRINT)
          {          	
              Get_command();
              Processing_command();  
          } 
         else 
          {
            system_infor.sd_print_status = SD_PRINT_FINISH;
            Alarm_Handle(3,2);
          }
          break;
    case SD_PRINT_FINISH:    
          fptr_buf = 0;
          system_infor.print_percent=100;
          memset(SD_Data_Buffer,0,SD_DATA_BUF_SIZE);
	   SET_FAN1_SPEED(HALF_POWER);
    	   SET_FAN2_SPEED(HALF_POWER);
	   f_closedir(&dir);
          Block_clean();
          Current_Block_Clean();
          Recovery_remove();
      
          delay_ms(2000);
          Send_Printer_State();
          delay_ms(2000);
		system_infor.stop_flag = 1;
		  strcpy(Command_Buffer,"M104 S0\r\n");
		  Processing_command();
		  strcpy(Command_Buffer,"M140 S0\r\n");
		  Processing_command();
		  strcpy(Command_Buffer,"M106 S0\r\n");
		  Processing_command();
		  if(Current_Position[Z_AXIS]<10)
		{
			strcpy(Command_Buffer,"G1 F500 Z15\r\n");
                    Processing_command();
		}
          strcpy(Command_Buffer,"G28 XY\r\n");
          Processing_command();
          
          strcpy(SD_Path,SD_Driver_Number);
          //recovery_enable = false;
          Recovery_remove(); 
          plan_initXY();    
          system_infor.sd_print_flag = DISABLE; 
          Disable_all_Axis();
		  system_infor.print_percent = 0;
		  
          Set_Beep(1000,127);
          delay_ms(1000);
	   Set_Beep(1000,127);

         
          USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
         
          system_infor.sd_print_status = SD_PRINT_IDLE;
		  f_close(&Print_File);
		  Print_File.fs=NULL;
		  Clear_Save_Parameter();
          Add_Message(TEMPERATURE_INFO);
          Add_Message(PRINTER_SD_STATUS);
          break;
		  
	case SD_PRINT_PAUSE_F:
		 Block_clean();
              Current_Block_Clean();
		Recovery_create();
		strcpy(Command_Buffer,"M104\r\n");
             Processing_command();
             strcpy(Command_Buffer,"M140\r\n");
             Processing_command();
		f_close(&Print_File);
		system_infor.sd_print_status = SD_PRINT_PAUSE;	
		Add_Message(TEMPERATURE_INFO);
		printf("Auto_Levele_Flag:%d\r\n",system_infor.Auto_Levele_Flag);
		break;
		
	case SD_PRINT_PAUSE:
		system_infor.sd_print_flag = DISABLE;
		if(Current_Position[Z_AXIS]<10 && system_infor.Auto_Levele_Flag!=1)
		{
			strcpy(Command_Buffer,"G1 F500 Z15\r\n");
                    Processing_command();
		}
		
		strcpy(Command_Buffer,"G28 XY\r\n");
             Processing_command();
             Add_Message(TEMPERATURE_INFO);
             Add_Message(PRINTER_SD_STATUS);

             plan_initXY();  

		break;
		
        case SD_PRINT_RECOVERY:
            system_infor.stop_flag=0;
            Recovery_sdprint();
           
		break;
    default:break;
    }  
}

u16 get_print_layer(void)
{
	
	u16 n=0;
	char *Start_Point;
	s32 layers=0;
	//float layer_high=0.0;
	while(n<6)
	{
      		f_status = f_lseek(&Print_File,500*n );
      		f_status = f_read(&Print_File, SD_Data_Buffer, SD_DATA_BUF_SIZE, &Print_File_Br);
      		if((Start_Point=strstr(SD_Data_Buffer,";Layer count:"))!=NULL)
      		{
			Sum_layers=strtol((Start_Point+13), NULL, 10);
			printf("layers:%d\r\n",layers);
      		}
      		if((Start_Point=strstr(SD_Data_Buffer,";LAYER_COUNT:"))!=NULL)
      		{
			Sum_layers=strtol((Start_Point+13), NULL, 10);
			printf("layers:%d\r\n",layers);
      		}
      		if((Start_Point=strstr(SD_Data_Buffer,";Layer height:"))!=NULL)
      		{
			layer_high=strtod((Start_Point+14), NULL);
			printf("layers:%f\r\n",layer_high);
      		}
      		n++;
      }
      
      
}



//send SD file name to upper computer

char Upload_Dir_Data[580];
void Send_SD_DirtoLCD(void)
{
	u8 i; 
	u8 sp=0;
	char str_updata[60];
	while(sp<system_infor.sd_file_num)
	{
		memset(Upload_Dir_Data,0,580);
		sprintf(str_updata,"<GCODE:%d;",system_infor.sd_file_num);
		strncat(Upload_Dir_Data, str_updata, 30);
		
		while(sp<system_infor.sd_file_num)
		{

			if(strlen(Upload_Dir_Data)>512)
				break;
			memset(str_updata,0,60);
			sprintf(str_updata, "%d:%s;",sp,sd_file_name[sp++]);
			strncat(Upload_Dir_Data, str_updata, 50);
		}
		memset(str_updata,0,30);
		sprintf(str_updata,"*>\r\n");
		strncat(Upload_Dir_Data, str_updata, 30);  
		USART3_printf(Upload_Dir_Data);
		delay_ms(500);
		
	}
}

void Send_SD_Dir(char *path)
{
	u8 i; 
	  
	sprintf(Printf_Buf, "Begin file list\n");my_printf(Printf_Buf);
	for(i = 0; i < system_infor.sd_file_num; i++)
	{
	   my_printf(sd_file_name[i]);
          sprintf(Printf_Buf, "\n");my_printf(Printf_Buf);
	}
  	sprintf(Printf_Buf, "End file list\n");my_printf(Printf_Buf);
  	sprintf(Printf_Buf, "ok\r\n");my_printf(Printf_Buf);                     
  
  	Wifi_ASK_Factors.WIFI_FILE_SUM=system_infor.sd_file_num;
 
}

static void file_rank(FILINFO fno_data, u8 file_num)
{
	u8 i;
	u32 time;
	time = (fno_data.fdate<< 16) + fno_data.ftime; 
	for(i = file_num; i > 0; i--)
	{
		if(sd_file_time[i-1] > time)
			break;
		if(i == FILE_NUM)
			continue;
		
		file_size[i] = file_size[i-1];
		strcpy(&sd_file_name[i][0],&sd_file_name[i-1][0]);
		strcpy(&sd_file_namebuf[i][0],&sd_file_namebuf[i-1][0]);
		sd_file_time[i] = sd_file_time[i-1];
	}
	if(i != FILE_NUM)
	{
		file_size[i] = (float)fno_data.fsize;
		file_name = (*fno.lfname && strlen(fno.lfname) <= (FILE_NAME_SIZE-7)) ? fno.lfname : fno.fname; 
		strcpy(&sd_file_name[i][0],file_name);
		strcpy(&sd_file_namebuf[i][0],file_name);
		sd_file_time[i] = time;
	}
}

void Get_SD_File(char *path)
{
	u8 file_num;
	fno.lfname = lfn;
 	fno.lfsize = sizeof lfn;
  	file_num = 0;
  	memset(sd_file_time, 0, sizeof(sd_file_time));
  	memset(sd_file_name, 0, sizeof(sd_file_name));
  	memset(sd_file_namebuf, 0, sizeof(sd_file_namebuf));
  	if(f_opendir(&dir,path) == FR_OK) 
  	{

    	while(f_readdir(&dir,&fno) == FR_OK)
    	{
      		if(fno.fname[0] == '.') continue; 

      		if( fno.fname[0] == 0 )	break;
	  		if((strstr(fno.fname, "gcode") || strstr(fno.fname, "gco") || strstr(fno.fname, "GCO")) == NULL)continue;

		  	if(file_num < FILE_NUM)
          	{
            	file_num++;
          	}
	  		file_rank(fno, file_num);     
    	}
	}
	f_closedir(&dir); 
	system_infor.sd_file_num = file_num;
       Wifi_ASK_Factors.WIFI_FILE_SUM=system_infor.sd_file_num;
}

void Clear_SD_File(void)
{
	system_infor.sd_file_num = 0;
	memset(sd_file_name, 0, sizeof(sd_file_name));
	memset(sd_file_namebuf, 0, sizeof(sd_file_namebuf));
	memset(file_size, 0, sizeof(file_size));
	memset(sd_file_time, 0, sizeof(sd_file_time));
}

extern u8 Rev_File_Flag;
void SD_File_Detect(void)
{
	if(SD_detec_flag == 0&&system_infor.files_refresh_flag==0)
		return;

	SD_detec_flag = 0;

	if(SD1_flag == 0||system_infor.files_refresh_flag!=0)
	{
		SD_Select_Init();
		if(system_infor.sd_status == SD_OK)
		{
			system_infor.sd_status == f_mount(&fats,SD_Driver_Number,1);
            if(system_infor.sd_status != SD_OK)
            {
            	return; 
            }
			Get_SD_File(SD_Path);
                    system_infor.files_refresh_flag=0;
		}
	}
	else if(SD1_flag == 1)
	{
		system_infor.sd_status = !SD_OK;
		Clear_SD_File();
		printf("SD Card Removed\r\n");
	}
}


void Send_Info_To_Serial(void)
{
    delay_ms(10);

   // Send_machine_info();
    delay_ms(10);
    if(SD_Initialize() != SD_OK)
    {
    	sprintf(Printf_Buf, "echo:SD init fail\n");my_printf(Printf_Buf);
    }
    else
    {
    	sprintf(Printf_Buf, "echo:SD card ok\n");my_printf(Printf_Buf);
    }
    delay_ms(10);
  	sprintf(Printf_Buf, "echo:Unknown command: ""\nok\n");my_printf(Printf_Buf);
    delay_ms(10);
  	sprintf(Printf_Buf, "ok\n");my_printf(Printf_Buf);
    delay_ms(10);
  	sprintf(Printf_Buf, "ok\n");my_printf(Printf_Buf);
    delay_ms(10);
  	sprintf(Printf_Buf, "ok\n");my_printf(Printf_Buf);
    delay_ms(10);
  	sprintf(Printf_Buf, "ok\n");my_printf(Printf_Buf);
    delay_ms(10);
  	sprintf(Printf_Buf, "ok\n");my_printf(Printf_Buf);
    delay_ms(10);
    if(SD_Initialize() != SD_OK)
    {
    	sprintf(Printf_Buf, "echo:SD init fail\n");my_printf(Printf_Buf);
    }
    else
    {
    	sprintf(Printf_Buf, "echo:SD card ok\n");my_printf(Printf_Buf);
    }
    delay_ms(10);
  	sprintf(Printf_Buf, "echo:Unknown command: ""\nok\n");my_printf(Printf_Buf);
    delay_ms(10);
  	sprintf(Printf_Buf, "ok\n");my_printf(Printf_Buf);
    delay_ms(10);
 	
	//Send_machine_info();
    delay_ms(10);
    
  	sprintf(Printf_Buf, "ok B:");my_printf(Printf_Buf);
    delay_us(170);
	sprintf(Printf_Buf, "%.1f",Current_Temperature[BED]);my_printf(Printf_Buf);
	sprintf(Printf_Buf, " /");my_printf(Printf_Buf);my_printf(Printf_Buf);
	sprintf(Printf_Buf, "%.1f",Setting.targe_temperature[BED]);my_printf(Printf_Buf);
	sprintf(Printf_Buf, " T0:");my_printf(Printf_Buf);
	delay_us(170);
	sprintf(Printf_Buf, "%.1f",Current_Temperature[NOZZLE0]);my_printf(Printf_Buf);
	sprintf(Printf_Buf, " /");my_printf(Printf_Buf);
	sprintf(Printf_Buf, "%.1f",Setting.targe_temperature[NOZZLE0]);my_printf(Printf_Buf);
	sprintf(Printf_Buf, " T1:");my_printf(Printf_Buf);
	delay_us(170);
	sprintf(Printf_Buf, "%.1f",Current_Temperature[NOZZLE1]);my_printf(Printf_Buf);
	sprintf(Printf_Buf, " /");my_printf(Printf_Buf);
	sprintf(Printf_Buf, "%.1f",Setting.targe_temperature[NOZZLE1]);my_printf(Printf_Buf);
	sprintf(Printf_Buf, " T2:");my_printf(Printf_Buf);
	delay_us(170);
	sprintf(Printf_Buf, "%.1f",Current_Temperature[NOZZLE2]);my_printf(Printf_Buf);
	sprintf(Printf_Buf, " /");my_printf(Printf_Buf);
	sprintf(Printf_Buf, "%.1f",Setting.targe_temperature[NOZZLE2]);my_printf(Printf_Buf);
    
        sprintf(Printf_Buf, " F:");my_printf(Printf_Buf);
	sprintf(Printf_Buf, "%d", (u16)(fan_current_speed * 100 / 255));my_printf(Printf_Buf);
        sprintf(Printf_Buf, " R:");my_printf(Printf_Buf);
	sprintf(Printf_Buf, "%d", system_infor.feed_tare);my_printf(Printf_Buf);
    
	sprintf(Printf_Buf, " @:0");my_printf(Printf_Buf);
	sprintf(Printf_Buf, " B@:0\n");my_printf(Printf_Buf);
    
    system_infor.sd_status = f_mount(&fats,SD_Driver_Number,1);//挂载SD卡到系统
    if(system_infor.sd_status == FR_OK)
    {
    	Get_SD_File(SD_Path);
        Send_SD_Dir(SD_Path);       
    }
    
    serial_connect_flag=ENABLE;
}
 


