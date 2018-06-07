
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



#include "data_handle.h"
#include "XZK_Configuration.h"
#include "Setting.h"
#include "wifi.h"
#include "variable.h"



extern char Upload_DataS[512];
#define QUEUE_LEN   30
u8 message_queue[QUEUE_LEN]={0};
u8 Start_Queue,End_Queue;

u8 message_queueM[QUEUE_LEN]={0};
u8 Start_QueueM,End_QueueM;
u8 Get_FilamentStatus(void);

#ifdef WIFI_MODULE
extern WIFI_MESSAGE Wifi_Work_Message;

#endif

extern u8 WIFI_Bebug_Flag,Temp_exception_flag,Firmware_Updata_Flag;
extern setting_t Setting;
extern Systembuf_Info  Systembuf_Infos;
extern char Firmware_version[9];
extern char WF_version[6];
extern float Current_Feedrate;
extern float layer_high;
extern u16  Sum_layers;
extern char Command_Buffer[1280];

void Init_Message_Queue(void)
{
	memset(message_queue,0,sizeof(message_queue));
	Start_Queue=0;
	End_Queue=0;
}

void Add_Message(u8 item)
{
	message_queue[End_Queue] = item;
	if(Start_Queue != (End_Queue+1)%QUEUE_LEN)
		End_Queue=(End_Queue+1)%QUEUE_LEN;

}

u8 Get_Message(void)
{
	u8 ret;
	if(Start_Queue==End_Queue)
		return 0;
		
	ret = message_queue[Start_Queue];
	Start_Queue=(Start_Queue+1)%QUEUE_LEN;
	return ret;	
}


void  Updata_To_LCD(u8 item)
{
	memset(Upload_DataS,0,512);
	switch(item)
	{
	       case TEMPERATURE_INFO://temperture
			sprintf(Upload_DataS,"<AUTO_UD:NS:%.1f;NC:%.1f;BS:%.1f;BC:%.1f;*>\r\n",Setting.targe_temperature[NOZZLE0],Current_Temperature[NOZZLE0],Setting.targe_temperature[BED],Current_Temperature[BED]);
		break;
		case PRINTER_SD_STATUS: //printer and SD status
			if(system_infor.serial_printf_flag == 1)
				sprintf(Upload_DataS,"<AUTO_UD:ST:9;SD:%d;*>\r\n",system_infor.sd_status);
			else
				sprintf(Upload_DataS,"<AUTO_UD:ST:%d;SD:%d;*>\r\n",system_infor.sd_print_status,system_infor.sd_status);
		break;
		case COORDINATE_XYZ://coordinate
			//sprintf(Upload_DataS,"<AUTO_UD:XP:%.1f;YP:%.1f;ZP:%.2f;*>\r\n",Current_Position[X_AXIS],Current_Position[Y_AXIS],Current_Position[Z_AXIS]);
			sprintf(Upload_DataS,"<AUTO_UD:ZP:%.1f;*>\r\n",Current_Position[Z_AXIS]);
		break;
		case PRINTER_MESSAGE://printer informention
			sprintf(Upload_DataS,"<AUTO_UD:DN:%s;DM:%s;SN:%s;FV:%s;HV:%s%s;PV:%.0f*%.0f*%.0f;*>\r\n",DEVICE_NAME,MACHINE_TYPE,Setting.SN,Firmware_version,Setting.HV,WF_version, Setting.max_position[X_AXIS], Setting.max_position[Y_AXIS], Setting.max_position[Z_AXIS]);
		break;
		/////////
		case RATE_FAN_LAYER://Layers, progress, speed
			sprintf(Upload_DataS,"<AUTO_UD:FR:%d;CL:%d;SL:%d;PP:%.2f;*>\r\n",system_infor.feed_tare,(u16)((Current_Position[Z_AXIS]/layer_high)),Sum_layers,system_infor.print_percent);
		break;
		case PRINTING_STATUS://GCODE
			sprintf(Upload_DataS,"<AUTO_UD:ST:2;GC:%s;*>\r\n",&Systembuf_Infos.printer_file_path[5]);
		break;
		
		
		case 77://step motor status
			sprintf(Upload_DataS,"MT:%d;",Get_Motor_Status());
		break;
		
		//case 172://
		///	sprintf(Printf_Buf,"FC:%d;FH:%d;",(u16)(system_infor.fan_controler_speed*100.0/255.0+0.5),(u16)(system_infor.fan_hotend_speed*100.0/255.0+0.5));
		//break;
		/////////////
		case FILAMENT_DETECTOR_OFF_ON: //filament detection open state
			sprintf(Upload_DataS,"<AUTO_UD:FF:%d;*>\r\n",system_infor.Filament_Dev_Flag);
		break;
		case FILAMENT_DETECTOR://no filament
			sprintf(Upload_DataS,"<AUTO_UD:FT:1;*>\r\n");
		break;
		case TEMP_EXCEPTION://exception temperature
			sprintf(Upload_DataS,"<AUTO_UD:EX:%d;*>\r\n",Temp_exception_flag); //Add_Message(FILAMENT_DETECTOR);
		break;
		
		case PRINTER_RESET://printer reset
			sprintf(Upload_DataS,"<AUTO_UD:RESET;*>\r\n");
		break;
		case SD_EXCEPTION: //	SD exception
			sprintf(Upload_DataS,"<AUTO_UD:SE:1;*>\r\n");
		break;
		case RECOVER_STANDBY: // Resume sleep
			sprintf(Upload_DataS,"<AUTO_UD:SM:0;*>\r\n");
			break;

		case INPUT_STANDBY: // sleep
			sprintf(Upload_DataS,"<AUTO_UD:SM:1;*>\r\n");
			break;

		case LCD_VERSION_INFO:
			sprintf(Upload_DataS,"<AUTO_UD:VS:1;*>\r\n");
			break;
#ifdef WIFI_MODULE
            	case WIFI_IP_MESSAGE: //ip
			sprintf(Upload_DataS,"<AUTO_UD:WFIP:%s;*>\r\n",Wifi_Work_Message.Router_IP);
		break;
		case WIFI_STATUS_MESSAGE: //WiFi status
			sprintf(Upload_DataS,"<AUTO_UD:WFSU:%d;*>\r\n",Wifi_Work_Message.WIFI_WORK_STATUS);
		break;
		case WIFI_SERVER_MESSAGE://SSID  server ip
  			 sprintf(Upload_DataS,"<AUTO_UD:SSID:%s;WFSE:%s;*>\r\n",Wifi_Work_Message.SSID,Wifi_Work_Message.WebServer_IP);
		break;
	
		case WIFI_DETAIL_MESSAGE:
			sprintf(Upload_DataS,"<AUTO_UD:WFSU:%d;WFAU:%d;SSID:%s;WFSE:%s;WFIP:%s;*>\r\n",Wifi_Work_Message.WIFI_WORK_STATUS,Wifi_Work_Message.AUTO_CONNECT,Wifi_Work_Message.SSID,
			Wifi_Work_Message.WebServer_IP,Wifi_Work_Message.Router_IP);
		break;

		case PRINTER_START_MESSAGE: //auto leveling ,filament Testing ,turn on wifi status
			sprintf(Upload_DataS,"<AUTO_UD:AL:%d;FF:%d;WFSU:%d;WFAU:%d;*>\r\n",system_infor.Auto_Levele_Flag,system_infor.Filament_Dev_Flag,Wifi_Work_Message.WIFI_WORK_STATUS,Wifi_Work_Message.AUTO_CONNECT);
		break;

		case WIFI_SET_SUCCEE:  // wifi set ok
			sprintf(Upload_DataS,"<AUTO_UD:WFCN:1;*>\r\n");
		break;
		case WIFI_EXIST_TEST:  // wifi module exist?
			sprintf(Upload_DataS,"<AUTO_UD:WFET:%d;*>\r\n",Setting.wifi_exist_flag);
		break;
		case WIFI_DEBUG_MESSAGE:  //wifi message 
                    sprintf(Upload_DataS,"<AUTO_UD:WS:%d;*>\r\n",WIFI_Bebug_Flag);
		break;
#endif
		default :  return;
		//break;

	}
    if(Firmware_Updata_Flag!=1)
	USART3_printf(Upload_DataS);
}




void Frash_Update_ToLCD(void)
{
	Updata_To_LCD(Get_Message());
}


void ADD_Item_to_LCD(void)
{
    static u16 Timess=0;
    Timess++;
    if(system_infor.sd_print_status != 2)
    {
        if(Timess%60==0)
        {
        }
        else if(Timess%20==0)
        {
            Add_Message(TEMPERATURE_INFO);
        }
        else if(Timess%3==0)
        {
            
        }
    }
    else
    {
        if(Timess%60==0)
        {
            Add_Message(PRINTING_STATUS);
        }
        else if(Timess%20==0)
        {
            Add_Message(RATE_FAN_LAYER);
        }
        else if(Timess%3==0)
        {
            
        }
    }
}







void Add_MessageM(u8 item)
{
	message_queueM[End_QueueM] = item;
	if(Start_QueueM != (End_QueueM+1)%QUEUE_LEN)
		End_QueueM=(End_QueueM+1)%QUEUE_LEN;
}

u8 Get_MessageM(void)
{
	u8 ret;
	if(Start_QueueM==End_QueueM)
		return 0;
		
	ret = message_queueM[Start_QueueM];
	Start_QueueM=(Start_QueueM+1)%QUEUE_LEN;
	return ret;	
}
///Add_MessageM(CMD_WIFI_DISCONNECTED);
void Main_Commands(u8 item)
{
#ifdef BOARD_A30_MINI_S
     Get_FilamentStatus();
#endif
    switch(item)
    {
        case CMD_FILAMAND_NO:   //no filament
            if(system_infor.sd_print_status==2)
            {
                  strcpy(Command_Buffer,"M25\r\n");
    	            Processing_command();
    	      }
    	      Alarm_Handle(3,2);
        break;
        //M2104
        case CMD_WIFI_DISCONNECTED:
             strcpy(Command_Buffer,"M2104\r\n");
    	       Processing_command();
        break;

        default :  return;
    }
}



void Main_Command_Handle(void)
{
	Main_Commands(Get_MessageM());
}















