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
#include "sd_print.h"
#include "Dgus_screen.h"
#include "vector_3.h"
#include "recovery_print.h"
#include "Gta.h"
#include "variable.h"
#include "data_handle.h"
/**
 * Look here for descriptions of G-codes:
 *  - http://linuxcnc.org/handbook/gcode/g-code.html
 *  - http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 *
 * Help us document these G-codes online:
 *  - http://www.marlinfirmware.org/index.php/G-Code
 *  - http://reprap.org/wiki/G-code
 *
 * -----------------
 * Implemented Codes
 * -----------------
 *
 * "G" Codes
 *
 * G0  -> G1
 * G1  - Coordinated Movement X Y Z E
 * G2  - CW ARC
 * G3  - CCW ARC
 * G4  - Dwell S<seconds> or P<milliseconds>
 * G10 - retract filament according to settings of M207
 * G11 - retract recover filament according to settings of M208
 * G28 - Home one or more axes
 * G29 - Detailed Z-Probe, probes the bed at 3 or more points.  Will fail if you haven't homed yet.
 * G30 - Single Z Probe, probes bed at current XY location.
 * G31 - Dock sled (Z_PROBE_SLED only)
 * G32 - Undock sled (Z_PROBE_SLED only)
 * G90 - Use Absolute Coordinates
 * G91 - Use Relative Coordinates
 * G92 - Set current position to coordinates given
 *
 * "M" Codes
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 * M1   - Same as M0
 * M17  - Enable/Power all stepper motors
 * M18  - Disable all stepper motors; same as M84
 * M20  - List SD card
 * M21  - Init SD card
 * M22  - Release SD card
 * M23  - Select SD file (M23 filename.g)
 * M24  - Start/resume SD print
 * M25  - Pause SD print
 * M26  - Set SD position in bytes (M26 S12345)
 * M27  - Report SD print status
 * M28  - Start SD write (M28 filename.g)
 * M29  - Stop SD write
 * M30  - Delete file from SD (M30 filename.g)
 * M31  - Output time since last M109 or SD card start to serial
 * M32  - Select file and start SD print (Can be used _while_ printing from SD card files):
 *        syntax "M32 /path/filename#", or "M32 S<startpos bytes> !filename#"
 *        Call gcode file : "M32 P !filename#" and return to caller file after finishing (similar to #include).
 *        The '#' is necessary when calling from within sd files, as it stops buffer prereading
 * M33  - Get the longname version of a path
 * M42  - Change pin status via gcode Use M42 Px Sy to set pin x to value y, when omitting Px the onboard led will be used.
 * M48  - Measure Z_Probe repeatability. M48 [P # of points] [X position] [Y position] [V_erboseness #] [E_ngage Probe] [L # of legs of travel]
 * M80  - Turn on Power Supply
 * M81  - Turn off Power Supply
 * M82  - Set E codes absolute (default)
 * M83  - Set E codes relative while in Absolute Coordinates (G90) mode
 * M84  - Disable steppers until next move,
 *        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 * M92  - Set axis_steps_per_unit - same syntax as G92
 * M104 - Set extruder target temp
 * M105 - Read current temp
 * M106 - Fan on
 * M107 - Fan off
 * M109 - Sxxx Wait for extruder current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for extruder current temp to reach target temp. Waits when heating and cooling
 *        IF AUTOTEMP is enabled, S<mintemp> B<maxtemp> F<factor>. Exit autotemp by any M109 without F
 * M111 - Set debug flags with S<mask>. See flag bits defined in Marlin.h.
 * M112 - Emergency stop
 * M114 - Output current position to serial port
 * M115 - Capabilities string
 * M117 - Display a message on the controller screen
 * M119 - Output Endstop status to serial port
 * M120 - Enable endstop detection
 * M121 - Disable endstop detection
 * M126 - Solenoid Air Valve Open (BariCUDA support by jmil)
 * M127 - Solenoid Air Valve Closed (BariCUDA vent to atmospheric pressure by jmil)
 * M128 - EtoP Open (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 * M129 - EtoP Closed (BariCUDA EtoP = electricity to air pressure transducer by jmil)
 * M140 - Set bed target temp
 * M145 - Set the heatup state H<hotend> B<bed> F<fan speed> for S<material> (0=PLA, 1=ABS)
 * M150 - Set BlinkM Color Output R: Red<0-255> U(!): Green<0-255> B: Blue<0-255> over i2c, G for green does not work.
 * M190 - Sxxx Wait for bed current temp to reach target temp. Waits only when heating
 *        Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
 * M200 - set filament diameter and set E axis units to cubic millimeters (use S0 to set back to millimeters).:D<millimeters>- 
 * M201 - Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
 * M202 - Set max acceleration in units/s^2 for travel moves (M202 X1000 Y1000) Unused in Marlin!!
 * M203 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
 * M204 - Set default acceleration: P for Printing moves, R for Retract only (no X, Y, Z) moves and T for Travel (non printing) moves (ex. M204 P800 T3000 R9000) in mm/sec^2
 * M205 -  advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E=maximum E jerk
 * M206 - Set additional homing offset
 * M207 - Set retract length S[positive mm] F[feedrate mm/min] Z[additional zlift/hop], stays in mm regardless of M200 setting
 * M208 - Set recover=unretract length S[positive mm surplus to the M207 S*] F[feedrate mm/min]
 * M209 - S<1=true/0=false> enable automatic retract detect if the slicer did not support G10/11: every normal extrude-only move will be classified as retract depending on the direction.
 * M218 - Set hotend offset (in mm): T<extruder_number> X<offset_on_X> Y<offset_on_Y>
 * M220 - Set speed factor override percentage: S<factor in percent>
 * M221 - Set extrude factor override percentage: S<factor in percent>
 * M226 - Wait until the specified pin reaches the state required: P<pin number> S<pin state>
 * M240 - Trigger a camera to take a photograph
 * M250 - Set LCD contrast C<contrast value> (value 0..63)
 * M280 - Set servo position absolute. P: servo index, S: angle or microseconds
 * M300 - Play beep sound S<frequency Hz> P<duration ms>
 * M301 - Set PID parameters P I and D
 * M302 - Allow cold extrudes, or set the minimum extrude S<temperature>.
 * M303 - PID relay autotune S<temperature> sets the target temperature. (default target temperature = 150C)
 * M304 - Set bed PID parameters P I and D
 * M380 - Activate solenoid on active extruder
 * M381 - Disable all solenoids
 * M400 - Finish all moves
 * M401 - Lower z-probe if present
 * M402 - Raise z-probe if present
 * M404 - N<dia in mm> Enter the nominal filament width (3mm, 1.75mm ) or will display nominal filament width without parameters
 * M405 - Turn on Filament Sensor extrusion control.  Optional D<delay in cm> to set delay in centimeters between sensor and extruder
 * M406 - Turn off Filament Sensor extrusion control
 * M407 - Display measured filament diameter
 * M410 - Quickstop. Abort all the planned moves
 * M420 - Enable/Disable Mesh Leveling (with current values) S1=enable S0=disable
 * M421 - Set a single Z coordinate in the Mesh Leveling grid. X<mm> Y<mm> Z<mm>
 * M428 - Set the home_offset logically based on the current_position
 * M500 - Store parameters in EEPROM
 * M501 - Read parameters from EEPROM (if you need reset them after you changed them temporarily).
 * M502 - Revert to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
 * M503 - Print the current settings (from memory not from EEPROM). Use S0 to leave off headings.
 * M540 - Use S[0|1] to enable or disable the stop SD card print on endstop hit (requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
 * M600 - Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
 * M665 - Set delta configurations: L<diagonal rod> R<delta radius> S<segments/s>
 * M666 - Set delta endstop adjustment
 * M605 - Set dual x-carriage movement mode: S<mode> [ X<duplication x-offset> R<duplication temp offset> ]
 * M907 - Set digital trimpot motor current using axis codes.
 * M908 - Control digital trimpot directly.
 * M350 - Set microstepping mode.
 * M351 - Toggle MS1 MS2 pins directly.
 *
 * ************ SCARA Specific - This can change to suit future G-code regulations
 * M360 - SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 * M361 - SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 * M362 - SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 * M363 - SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 * M364 - SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
 * M365 - SCARA calibration: Scaling factor, X, Y, Z axis
 * ************* SCARA End ***************
 *
 * ************ Custom codes - This can change to suit future G-code regulations
 * M851 - Set probe's Z offset (mm above extruder -- The value will always be negative)


 * M928 - Start SD logging (M928 filename.g) - ended by M29
 * M999 - Restart after being stopped by error
 *
 *	M2000  - set SN and save
 *	M2001  - earse whole flash
 *	M2002  - read print seting (M2002)
 *	M2003  - maxpoint set  (M2003 X280.0 Y160.0)
 *	M2004  - per_mm set (M2004 X80.0)
 *	M2005  - motor dir set (M2005 X 0 Y 1 Z 1 E0 0 E1 1 E2 0)
 *	M2006  - motor max feedrate (M2006 X400 Y400 Z30 E50)	

 * "T" Codes
 *
 * T0-T3 - Select a tool by index (usually an extruder) [ F<mm/min> ]
 *M2557 <connect to RD24G|JISHUBU666,17S125D2000008,www.geeetech.net,>
 */





static float Current_Feedrate,saved_feedrate;
static float Previous_Feedrate;
static u16 saved_feedmultiply;
static u8 Current_Active_Extruder;
static bool Gloabl_axis_go_origin_flag;
static bool Gloabl_axis_reset_coordinate_flag;
static float offset[3]; 
static float Arc_r; 
static u8 targeted_hotend;
static u8 Up_Data_Flag =0;
static u8 motor_enable_status=0;
static u8 AUTO_LEVELE_G29_FLAG=0;
static u8 Up_CL_Flag=0;

char Command_Buffer[1280];//[CMDBUF_SIZE];//CMDBUF_SIZE
char *Strchr_Pointer;
u8   Coordinate_Axias[] = {'X','Y','Z','E'};
char DIR_Axias[6][5] = {"X ", "Y ", "Z ", "E0 ", "E1 ", "E2 "};
const char Gcode[3] = {'G','M','T'};
char Firmware_version[9]="V1.00.58";


long gcode_N = 0;
int servo_endstops[] = SERVO_ENDSTOPS;

extern autohome_st Autohome;
extern char sd_file_name[32][50];
extern char SD_Path[512]; 
extern FATFS fats;
extern u8  serial_connect_flag;
extern FIL Print_File;
extern vu16 SD_Data_Buffer_Count;
extern DWORD fptr_buf,pre_sd_byte;
extern s32 position[4];  
extern float file_size[FILE_NUM];//文件大小，用于显示SD卡打印进度的计算
extern u8 Leveling_SetMotor_flag;//调平电机状态
extern float new_Z_Max_Position;//Z轴打印范围
extern volatile WIFI_TX_STATA Wifi_ASK_State;
extern WIFI_MESSAGE Wifi_Work_Message;

extern u8 Wifi_Server_message[128];
extern char WF_version[6];



extern void command_process(char* str);
extern void USART2_TxString(u8 *string);
extern void Send_SD_Dir(char *path);
u8 Auto_Levele_Offset_Flag=0;

u8 Get_Motor_Status(void)
{
    return motor_enable_status;
}


char Upload_DataS[512];
typedef struct UPLOAD_DATA {
       u8 current_SDPrintstatus;
	float current_x;
	float current_y;
	float current_z;
	u8 current_SDstatus;
	u8 current_MTstatus;
	float current_targeNozzle_temperature;
	float current_CurrentNozzle_temperature;
	float current_targebed_temperature;
	float current_Currentbed_temperature;
	u16 current_rate;
	u16 current_CFan;
	u16 current_HFan;
	float current_percent;
	u16 current_SPlies;
	float  current_Plies;
	char file_name[60];
	u8 wifi_status;
	u8 wifi_auto;
	float Cur_Feedrate;  //
	u8 exception_flag;
	u8 auto_levele_flags;
	u8 filament_decetion_statue;
	u8 filament_OFF_ON;
}AUTO_UPLOAD_DATA;

static AUTO_UPLOAD_DATA auto_upload_datas;
u8 Temp_exception_flag=0;
u8 Uoload_LCDData_Flag=0;
extern u8 Firmware_Updata_Flag;


u8 Add_Upload_data(void)
{
	char str_updata[50];
	u8 ret=0;
	memset(Upload_DataS,0,512);
	static u8 Updata_Times=0,EX_Times=0;
	memset(str_updata,0,30);
	sprintf(str_updata,"<AUTO_UD:");
	strncat(Upload_DataS, str_updata, 30);  
	
	
	if(system_infor.serial_printf_flag != 1)
	{
		
		if(auto_upload_datas.current_SDPrintstatus!=system_infor.sd_print_status && Up_Data_Flag ==0)
		{
			memset(str_updata,0,30);
			
				
			auto_upload_datas.current_SDPrintstatus=system_infor.sd_print_status;
			if(system_infor.serial_printf_flag == 1)
				sprintf(str_updata,"ST:7;");
			else
				sprintf(str_updata,"ST:%d;",system_infor.sd_print_status);
			strncat(Upload_DataS, str_updata, 30);  
			ret=1;
		}
		
		if(system_infor.sd_print_status!=SD_PRINTING)
		{
			if(abs((int)(auto_upload_datas.current_x-Current_Position[X_AXIS]))>2)
			{
				memset(str_updata,0,30);
				auto_upload_datas.current_x=Current_Position[X_AXIS];
				sprintf(str_updata,"XP:%.1f;",Current_Position[X_AXIS]);
				strncat(Upload_DataS, str_updata, 30);  
				ret=1;
			}
			if(abs((int)(auto_upload_datas.current_y-Current_Position[Y_AXIS]))>2)
			{
				memset(str_updata,0,30);
				auto_upload_datas.current_y=Current_Position[Y_AXIS];
				sprintf(str_updata,"YP:%.1f;",Current_Position[Y_AXIS]);
				strncat(Upload_DataS, str_updata, 30);  
				ret=1;
			}
			if(abs((int)(auto_upload_datas.current_z-Current_Position[Z_AXIS]))>2)
			{
				memset(str_updata,0,30);
				auto_upload_datas.current_z=Current_Position[Z_AXIS];
				sprintf(str_updata,"ZP:%.2f;",Current_Position[Z_AXIS]);
				strncat(Upload_DataS, str_updata, 30);  
				ret=1;
			}
		}
	}
	else
	{
		if(auto_upload_datas.current_SDPrintstatus!=system_infor.sd_print_status)
		{
			memset(str_updata,0,30);
			auto_upload_datas.current_SDPrintstatus=system_infor.sd_print_status;
			sprintf(str_updata,"ST:9;");
			strncat(Upload_DataS, str_updata, 30);  
			ret=1;
		}
	}
	if(auto_upload_datas.current_SDstatus!=system_infor.sd_status)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_SDstatus=system_infor.sd_status;
		sprintf(str_updata,"SD:%d;",system_infor.sd_status);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}
	
	if(auto_upload_datas.current_MTstatus!=Get_Motor_Status())
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_MTstatus=Get_Motor_Status();
		sprintf(str_updata,"MT:%d;",Get_Motor_Status());
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}

	if(abs((int)(auto_upload_datas.current_targeNozzle_temperature-Setting.targe_temperature[NOZZLE0]))>=1)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_targeNozzle_temperature=Setting.targe_temperature[NOZZLE0];
		sprintf(str_updata,"NS:%.1f;",Setting.targe_temperature[NOZZLE0]);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}

	if(abs((int)((auto_upload_datas.current_CurrentNozzle_temperature-Current_Temperature[NOZZLE0])*10))>=5)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_CurrentNozzle_temperature=Current_Temperature[NOZZLE0];
		sprintf(str_updata,"NC:%.1f;",Current_Temperature[NOZZLE0]);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}

	if(abs((int)(auto_upload_datas.current_targebed_temperature-Setting.targe_temperature[BED]))>=1)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_targebed_temperature=Setting.targe_temperature[BED];
		sprintf(str_updata,"BS:%.1f;",Setting.targe_temperature[BED]);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}

	if(abs((int)((auto_upload_datas.current_Currentbed_temperature-Current_Temperature[BED])*10))>=5)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_Currentbed_temperature=Current_Temperature[BED];
		sprintf(str_updata,"BC:%.1f;",Current_Temperature[BED]);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}
	
	if(auto_upload_datas.current_rate!=system_infor.feed_tare)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_rate=system_infor.feed_tare;
		sprintf(str_updata,"FR:%d;",system_infor.feed_tare);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}

	if(auto_upload_datas.current_CFan!=system_infor.fan_controler_speed)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_CFan=system_infor.fan_controler_speed;
		sprintf(str_updata,"FC:%d;",(u16)(system_infor.fan_controler_speed*100.0/255.0+0.5));//fan_controler_speed
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}

	if(auto_upload_datas.current_HFan!=system_infor.fan_hotend_speed)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_HFan=system_infor.fan_hotend_speed;
		sprintf(str_updata,"FH:%d;",(u16)(system_infor.fan_hotend_speed*100.0/255.0+0.5));
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}
	if(((system_infor.print_percent-auto_upload_datas.current_percent)*10)>=1)
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_percent=system_infor.print_percent;
		sprintf(str_updata,"PP:%.2f;",system_infor.print_percent);
		strncat(Upload_DataS, str_updata, 30);  
		if(system_infor.print_percent>99.9)
		{
			system_infor.print_percent = 0;
			auto_upload_datas.current_percent =0;
		}
		ret=1;
	}
	if(auto_upload_datas.current_SPlies!=Sum_layers )
	{
		memset(str_updata,0,30);
		auto_upload_datas.current_SPlies=Sum_layers;
		sprintf(str_updata,"SL:%d;",Sum_layers);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}
	if(abs((int)((auto_upload_datas.current_Plies*10)-(Current_Position[Z_AXIS]*10)))>1)//&&abs((int)((auto_upload_datas.current_Plies*10)-(Current_Position[Z_AXIS]*10)))<15
	{
		if(((Current_Position[Z_AXIS]/layer_high)<2100&&(Up_CL_Flag==0))&&(system_infor.sd_print_status==SD_PRINTING||system_infor.serial_printf_flag==1))
		{
			memset(str_updata,0,30);
			auto_upload_datas.current_Plies=Current_Position[Z_AXIS];
			sprintf(str_updata,"CL:%d;",(u16)((Current_Position[Z_AXIS]/layer_high)));
			strncat(Upload_DataS, str_updata, 30);  
			ret=1;
		}
	}
	if(system_infor.sd_print_status==2&&(strstr(auto_upload_datas.file_name,"gcode")==NULL&&strstr(auto_upload_datas.file_name,"gco")==NULL&&strstr(auto_upload_datas.file_name,"GCO")==NULL))
	{
		memset(str_updata,0,50);
		memcpy(auto_upload_datas.file_name,&Systembuf_Infos.printer_file_path[5],strlen(Systembuf_Infos.printer_file_path)-5);
		
		sprintf(str_updata,"GC:%s;",&Systembuf_Infos.printer_file_path[5]);
		strncat(Upload_DataS, str_updata, 50);  
		//ret=1;
	}
	if(auto_upload_datas.wifi_status!=Wifi_Work_Message.WIFI_WORK_STATUS)
	{
		memset(str_updata,0,30);
		auto_upload_datas.wifi_status=Wifi_Work_Message.WIFI_WORK_STATUS;
		sprintf(str_updata,"WFSU:%d;",Wifi_Work_Message.WIFI_WORK_STATUS);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}
	if(auto_upload_datas.wifi_auto!=Wifi_Work_Message.AUTO_CONNECT)
	{
		memset(str_updata,0,30);
		auto_upload_datas.wifi_auto=Wifi_Work_Message.AUTO_CONNECT;
		sprintf(str_updata,"WFAU:%d;",Wifi_Work_Message.AUTO_CONNECT);
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}
	if(abs((int)(auto_upload_datas.Cur_Feedrate-Current_Feedrate))>=0.5)
	{
		memset(str_updata,0,30);
		auto_upload_datas.Cur_Feedrate=Current_Feedrate;
		sprintf(str_updata,"CF:%.1f;",Current_Feedrate*(system_infor.feed_tare/100.0));
		strncat(Upload_DataS, str_updata, 30);  
		ret=1;
	}
	if(auto_upload_datas.exception_flag!=Temp_exception_flag && EX_Times<5)
	{
		
		if(Temp_exception_flag!=0)
		{
			Updata_Times++;
			memset(str_updata,0,30);
			if(Updata_Times>3)
			{
				Temp_exception_flag=0;
				//auto_upload_datas.exception_flag=Temp_exception_flag;
				Updata_Times=0;
			}
			sprintf(str_updata,"EX:%d;",Temp_exception_flag);
			strncat(Upload_DataS, str_updata, 30);  
			EX_Times++;
			ret=1;
			
		}
	}
	if(auto_upload_datas.auto_levele_flags!=system_infor.Auto_Levele_Flag)
	{
		memset(str_updata,0,30);
		auto_upload_datas.auto_levele_flags=system_infor.Auto_Levele_Flag;
		sprintf(str_updata,"AL:%d;",system_infor.Auto_Levele_Flag);
		strncat(Upload_DataS, str_updata, 30);  
             
		ret=1;
	}
	//if(auto_upload_datas.filament_decetion_statue!=Get_FilamentStatus(E_AXIS))//耗材检测Filament_Dev_Flag
	//{
	//	memset(str_updata,0,30);
	//	auto_upload_datas.filament_decetion_statue=Get_FilamentStatus(E_AXIS);
	///	sprintf(str_updata,"FT:%d;",Get_FilamentStatus(E_AXIS));
	//	strncat(Upload_DataS, str_updata, 30);  
       //         
	//	ret=1;
	//}
	if(auto_upload_datas.filament_OFF_ON!=system_infor.Filament_Dev_Flag)//耗材检测Filament_Dev_Flag
	{
		memset(str_updata,0,30);
		auto_upload_datas.filament_OFF_ON=system_infor.Filament_Dev_Flag;
		sprintf(str_updata,"FF:%d;",system_infor.Filament_Dev_Flag);
		strncat(Upload_DataS, str_updata, 30);  
                
		ret=1;
	}
	
	memset(str_updata,0,30);
	sprintf(str_updata,"*>\r\n");
	strncat(Upload_DataS, str_updata, 30);  
	if(ret==1)
	{
		USART3_printf(Upload_DataS);
	}
	return ret;
}


void WIF_EXIST_TEST(void)
{
	if(Setting.wifi_exist_flag ==1)
	{
		sprintf(Printf_Buf,"<AUTO_UD:WFET:1;*>\r\n");
              USART3_printf(Printf_Buf);
             printf("%s",Printf_Buf);
	}
	else if(Setting.wifi_exist_flag ==0)
	{
		sprintf(Printf_Buf,"<AUTO_UD:WFET:0;*>\r\n");
             USART3_printf(Printf_Buf);
             printf("%s",Printf_Buf);
	}

}


/******************************************************************************/

#pragma inline=forced
float Command_value_float(char *ch_point)
{
  return (strtod(&Command_Buffer[ch_point - Command_Buffer + 1], NULL));
}

/******************************************************************************/

#pragma inline=forced
s32 Command_value_long(char *ch_point)
{
  return (strtol(&Command_Buffer[ch_point - Command_Buffer + 1], NULL, 10));
}

/******************************************************************************/

#pragma inline=forced
u8 Command_is(char code, char** ch_point)
{
  Strchr_Pointer = strchr(Command_Buffer, code);
  *ch_point = Strchr_Pointer;
  if(Strchr_Pointer != NULL)
    return 1;
  else
    return 0;   
}

/******************************************************************************/

#pragma inline=forced
u32 Command_Time_value(char* ch_point)
{      //char* ch_point;
	u32 time = 0;
	if(Command_is('h',&ch_point) == 1)
	{
		ch_point -= 3;
		time = 3600 * Command_value_long(ch_point);
	}
	if(Command_is('m',&ch_point) == 1)
	{
		ch_point -= 3;
		time += 60 * Command_value_long(ch_point);
	}
	if(Command_is('s',&ch_point) == 1)
	{
		ch_point -= 3;
		time += Command_value_long(ch_point);
	}
  	return time;
}

/******************************************************************************/

#pragma inline=forced
u8 CommandStr_is(char *str,char** ch_point)
{
  Strchr_Pointer = strstr(Command_Buffer,str);
   *ch_point=Strchr_Pointer;
  if(Strchr_Pointer != NULL)
  {
    Strchr_Pointer = Strchr_Pointer + strlen(str) - 1;
    *ch_point=Strchr_Pointer;
    return 1;
  }
  else
    return 0;   
}


/******************************************************************************/


#pragma inline=forced
char Gcode_is(char** ch_point)
{
  u8 i,j;
  for(i=0;i<CMDBUF_SIZE;i++)
  {
    for(j=0;j<3;j++)
    {
      if(Command_Buffer[i] == Gcode[j])
      {
        Strchr_Pointer = Command_Buffer + i;
        *ch_point=Strchr_Pointer;
        return Gcode[j];
      }
    }
  }
  return NULL; 
}

/******************compare received file name with selected SD file*************************/

static void Serial_AXIS_Move_Cmd_Processing(void)
{
    char *axis_pointer,*f_pointer,*end_pointer,temp_cmd[50],str_float[8],i;
    float temp_value=0;
    
    axis_pointer=strchr(Command_Buffer, 'X');
    if(axis_pointer == NULL)
    {
        axis_pointer=strchr(Command_Buffer, 'Y');
        if(axis_pointer == NULL)
        {
            axis_pointer=strchr(Command_Buffer, 'Z');
            if(axis_pointer == NULL)
            {
                axis_pointer=strchr(Command_Buffer, 'E');
                if(axis_pointer != NULL)
                {
                    temp_value=strtod(&Command_Buffer[axis_pointer - Command_Buffer + 1], NULL);
                    temp_value+=Current_Position[E_AXIS];
                    sprintf(str_float,"%0.3f",temp_value);
                    
                    f_pointer=strchr(Command_Buffer, 'F');   
                    for(i=0;i<35;i++)
                    {
                        temp_cmd[i]=Command_Buffer[f_pointer-Command_Buffer-1+i];
                    }
                    
                    strcpy(&Command_Buffer[axis_pointer-Command_Buffer+1],str_float);
                    end_pointer=strchr(Command_Buffer, '\0');
                    strcpy(&Command_Buffer[end_pointer-Command_Buffer],temp_cmd);
                }
            }
            else
            {
                temp_value=strtod(&Command_Buffer[axis_pointer - Command_Buffer + 1], NULL);
                temp_value+=Current_Position[Z_AXIS];
                sprintf(str_float,"%0.3f",temp_value);
                
                f_pointer=strchr(Command_Buffer, 'F');   
                for(i=0;i<35;i++)
                {
                    temp_cmd[i]=Command_Buffer[f_pointer-Command_Buffer-1+i];
                }
                
                strcpy(&Command_Buffer[axis_pointer-Command_Buffer+1],str_float);
                end_pointer=strchr(Command_Buffer, '\0');
                strcpy(&Command_Buffer[end_pointer-Command_Buffer],temp_cmd);
            }
        }
        else
        {
            temp_value=strtod(&Command_Buffer[axis_pointer - Command_Buffer + 1], NULL);
            
            //if(YEndstop == MINENSTOP) 
                temp_value+=Current_Position[Y_AXIS];
            //else
                //temp_value=Setting.max_position[Y_AXIS]-(temp_value+Current_Position[Y_AXIS]);
                
            sprintf(str_float,"%0.3f",temp_value);
            
            f_pointer=strchr(Command_Buffer, 'F');   
            for(i=0;i<35;i++)
            {
                temp_cmd[i]=Command_Buffer[f_pointer-Command_Buffer-1+i];
            }
            
            strcpy(&Command_Buffer[axis_pointer-Command_Buffer+1],str_float);
            end_pointer=strchr(Command_Buffer, '\0');
            strcpy(&Command_Buffer[end_pointer-Command_Buffer],temp_cmd);
        }
    }
    else
    {
        temp_value=strtod(&Command_Buffer[axis_pointer - Command_Buffer + 1], NULL);
        temp_value+=Current_Position[X_AXIS];
        sprintf(str_float,"%0.3f",temp_value);
    
        f_pointer=strchr(Command_Buffer, 'F');   
        for(i=0;i<35;i++)
        {
            temp_cmd[i]=Command_Buffer[f_pointer-Command_Buffer-1+i];
        }
          
        strcpy(&Command_Buffer[axis_pointer-Command_Buffer+1],str_float);
        end_pointer=strchr(Command_Buffer, '\0');
        strcpy(&Command_Buffer[end_pointer-Command_Buffer],temp_cmd);
    }
}


/******************************************************************************/
static void Avoid_exceed_endstops(float *destination_coordinate)
{
  if(destination_coordinate[X_AXIS] > Setting.max_position[X_AXIS])
    destination_coordinate[X_AXIS] = Setting.max_position[X_AXIS];
  if(destination_coordinate[Y_AXIS] > Setting.max_position[Y_AXIS])
    destination_coordinate[Y_AXIS] = Setting.max_position[Y_AXIS];
  if(destination_coordinate[Z_AXIS] > Setting.max_position[Z_AXIS])
    destination_coordinate[Z_AXIS] = Setting.max_position[Z_AXIS];
  
  if(destination_coordinate[X_AXIS]< Setting.min_position[X_AXIS])
    destination_coordinate[X_AXIS]= Setting.min_position[X_AXIS];
  if(destination_coordinate[Y_AXIS]< Setting.min_position[Y_AXIS])
    destination_coordinate[Y_AXIS]= Setting.min_position[Y_AXIS];
  if(destination_coordinate[Z_AXIS]< Setting.min_position[Z_AXIS])
    destination_coordinate[Z_AXIS]= Setting.min_position[Z_AXIS];
}
/******************************************************************************/


static void Get_linear_coordinates(void)
{
  char* ch_point;
  for(u8 i=0;i<NUM_AXIS;i++)
  {
    if(Command_is(Coordinate_Axias[i],&ch_point))
    {
       if(system_infor.serial_axis_move_cmd == ENABLE)
	       Destination[i] = Current_Position[i] + Command_value_float(ch_point); 
	   else
           Destination[i] = Command_value_float(ch_point); 
	axis_known_position[i]=false;//有移动，归位标志清空
    }
    else
      Destination[i] = Current_Position[i];
  }
  if(Command_is('F',&ch_point))
  {
  	if(Destination[2] != Current_Position[2])
  	{
  		if(Command_value_float(ch_point)>1800)
  		{
			Current_Feedrate = 30.0;//换算成mm/s的单位
  		}
  		else
    			Current_Feedrate = Command_value_float(ch_point)/60.0;//换算成mm/s的单位
    		//printf("FF:%s\r\n",Command_Buffer);
    	}
    	else
    	{
		Current_Feedrate = Command_value_float(ch_point)/60.0;//换算成mm/s的单位
    	}
  }
  else
    Current_Feedrate = Previous_Feedrate;
  
  Previous_Feedrate = Current_Feedrate;
}
/******************************************************************************/

static void Get_arc_coordinates(void)
{
     char* ch_point;
  Get_linear_coordinates();
  if(Command_is('R',&ch_point))
    Arc_r = Command_value_float(ch_point);
  else
    Arc_r = 0.0;
  if(Command_is('I',&ch_point))
    offset[0] = Command_value_float(ch_point);
  else
    offset[0] = 0.0;
  if(Command_is('J',&ch_point))
    offset[1] = Command_value_float(ch_point);
  else 
    offset[1] = 0.0;
}
/******************************************************************************/
extern DWORD sd_line_num;

static void Prepare_linear_move(void)
{

   Avoid_exceed_endstops(Destination);
  if(  (Destination[X_AXIS] == Current_Position[X_AXIS])
     &&(Destination[Y_AXIS] == Current_Position[Y_AXIS]))
  {
      
    Plan_buffer_line(Destination[X_AXIS],
                     Destination[Y_AXIS],
                     Destination[Z_AXIS],
                     Destination[E_AXIS],
                     Current_Feedrate,
                     Current_Active_Extruder);
    
  }
  else
  {
    
    Plan_buffer_line(Destination[X_AXIS],
                     Destination[Y_AXIS],
                     Destination[Z_AXIS],
                     Destination[E_AXIS],
                     Current_Feedrate*(Setting.extrude_multiply/100),
                     Current_Active_Extruder);
    
  }
  for(unsigned char i=0;i<NUM_AXIS;i++)
    Current_Position[i] = Destination[i]; 
  
  if(system_infor.Unexpected_events==ENABLE)//发生超出正常动作时，回退回到当前的指令，继续读卡解析
  	{
  	SD_Data_Buffer_Count = 0;
	while(sd_line_num!=0&&(block_buffer_head != block_buffer_tail))/////////////再思考完善
		{
			sd_line_num--;
			if(block_buffer_head==0)
			block_buffer_head=BLOCK_BUFFER_SIZE;
			
			block_buffer_head --;
		}
	if(system_infor.Auto_Levele_Flag==1)
	{
	Current_Position[X_AXIS]=block_buffer[block_buffer_head].bed_ForCalulate_point_x;
	Current_Position[Y_AXIS]=block_buffer[block_buffer_head].bed_ForCalulate_point_y;
	Current_Position[Z_AXIS]=block_buffer[block_buffer_head].bed_ForCalulate_point_z;
	Current_Position[E_AXIS]=block_buffer[block_buffer_head].point_e;
	fptr_buf = block_buffer[block_buffer_head].sd_byte;
	Current_Feedrate=block_buffer[block_buffer_head].feed_rate;

	position[X_AXIS] = lround(block_buffer[block_buffer_head].point_x*Setting.steps_per_mm[X_AXIS]);
	position[Y_AXIS] = lround(block_buffer[block_buffer_head].point_y*Setting.steps_per_mm[Y_AXIS]);
	position[Z_AXIS] = lround(block_buffer[block_buffer_head].point_z*Setting.steps_per_mm[Z_AXIS]);     
	position[E_AXIS] = lround(block_buffer[block_buffer_head].point_e*Setting.steps_per_mm[E_AXIS]);
	}
	else
	{
	Current_Position[X_AXIS]=block_buffer[block_buffer_head].point_x;
	Current_Position[Y_AXIS]=block_buffer[block_buffer_head].point_y;
	Current_Position[Z_AXIS]=block_buffer[block_buffer_head].point_z;
	Current_Position[E_AXIS]=block_buffer[block_buffer_head].point_e;
	fptr_buf = block_buffer[block_buffer_head].sd_byte;
	Current_Feedrate=block_buffer[block_buffer_head].feed_rate;

	position[X_AXIS] = lround(Current_Position[X_AXIS]*Setting.steps_per_mm[X_AXIS]);
	position[Y_AXIS] = lround(Current_Position[Y_AXIS]*Setting.steps_per_mm[Y_AXIS]);
	position[Z_AXIS] = lround(Current_Position[Z_AXIS]*Setting.steps_per_mm[Z_AXIS]);     
	position[E_AXIS] = lround(Current_Position[E_AXIS]*Setting.steps_per_mm[E_AXIS]);
	}
	block_buffer_head += 1;//这个buff作为存储的最后一个数据，下一个buff才是要写入的指令
	system_infor.Unexpected_events=DISABLE;
	}
  	
}
/******************************************************************************/

static void Prepare_arc_move(unsigned char arc_dir)
{
  /*Plan_buffer_arc(Current_Position, Destination, offset, X_AXIS, Y_AXIS, Z_AXIS, 
  	          Current_Feedrate*Setting.extrude_multiply/100, Arc_r, arc_dir, Current_Active_Extruder);
  for(unsigned char i=0; i < NUM_AXIS; i++)
    Current_Position[i] = Destination[i];*/
}
/******************************************************************************/



//Allows programming of steps per unit (usually mm) for motor drives. These values are reset to firmware defaults on power on, unless saved to EEPROM if available (M500 in Marlin) or in the configuration file (config.g in RepRapFirmware). Very useful for calibration.
static void Set_axis_steps_per_unit(void)  
{
    char* ch_point;
  for(u8 i=0;i<NUM_AXIS;i++)
  {
    if(Command_is(Coordinate_Axias[i],&ch_point))
    {
       Setting.steps_per_mm[i] = Command_value_float(ch_point); 
    } 
  }
}

static void Set_hotend_pid(void)  //Set PID parameters
{
  u8 num=0;
  char* ch_point;
  if(Command_is('H',&ch_point)) num = (u8)Command_value_float(ch_point);
  if(Command_is('P',&ch_point)) Setting.Kp[num] = Command_value_float(ch_point);
  if(Command_is('I',&ch_point)) Setting.Ki[num] = Command_value_float(ch_point);
  if(Command_is('D',&ch_point)) Setting.Kd[num] = Command_value_float(ch_point);
}

static void Set_bed_pid(void)  //Set PID parameters
{
      char* ch_point;
  if(Command_is('P',&ch_point)) Setting.Kp[BED] = Command_value_float(ch_point);
  if(Command_is('I',&ch_point)) Setting.Ki[BED] = Command_value_float(ch_point);
  if(Command_is('D',&ch_point)) Setting.Kd[BED] = Command_value_float(ch_point);
}
extern volatile long endstops_trigsteps[3];
static void checkHitEndstops(void)
{
  if(Read_endstop_Z_hit()) {
  	sprintf(Printf_Buf, "we last said to move to Z: %f\r\n",Current_Position[Z_AXIS]);//输出在(x,y)位置测量到的Z值
      my_printf(Printf_Buf);
      sprintf(Printf_Buf, "Hit endstop Z: %f\r\n",(float)endstops_trigsteps[Z_AXIS]/Setting.steps_per_mm[Z_AXIS]);//输出在(x,y)位置测量到的Z值
      my_printf(Printf_Buf);
      Write_endstop_Z_hit(false);
  }
}
static void setup_for_endstop_move() {
    saved_feedrate = Current_Feedrate;
    saved_feedmultiply = Setting.extrude_multiply;
    Setting.extrude_multiply = 100;

    enable_endstops(true);//记得添加stepper的程序
}
static void clean_up_after_endstop_move() {
    enable_endstops(false);
    Current_Feedrate = saved_feedrate;
    Setting.extrude_multiply = saved_feedmultiply;
}

static void engage_z_probe() {
    // Engage Z Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
    if (servo_endstops[Z_AXIS] > -1) {

	//暂时只做了自动调平的功能，如若需要，可添加选择伺服电机号servo_index

	BLTouch_StateSet(0, SERVO_Push_pin_Down);//3dtouch状态设置


    }
    #endif
}
static void retract_z_probe() {
    // Retract Z Servo endstop if enabled
    #ifdef SERVO_ENDSTOPS
    if (servo_endstops[Z_AXIS] > -1) {

	//暂时只做了自动调平的功能，如若需要，可添加选择伺服电机号servo_index
	BLTouch_StateSet(0, SERVO_Push_pin_Up);//3dtouch状态设置

    }
    #endif
}
#ifdef AUTO_BED_LEVELING_GRID
static void set_bed_level_equation_lsq(double *plane_equation_coefficients)
{
    vector_3 planeNormal;
	vector_3_init(&planeNormal, -(float)plane_equation_coefficients[0],  -(float)plane_equation_coefficients[1],  (float)1.0);
	sprintf(Printf_Buf, "planeNormal x: %f y: %f z: %f\r\n",planeNormal.x,planeNormal.y,planeNormal.z);//输出平面法线的值
	printf("TOO:%s\r\n",Printf_Buf);//my_printf(Printf_Buf);

	matrix_3x3_create_look_at(&Setting.plan_bed_level_matrix, planeNormal);

    vector_3 corrected_position;
	 plan_get_position(&corrected_position);//取当前XYZ轴的值mm

    Current_Position[X_AXIS] = corrected_position.x;
    Current_Position[Y_AXIS] = corrected_position.y;
    Current_Position[Z_AXIS] = corrected_position.z;

    // 让热床的高度为0，这样我们就不用下降热床
    Current_Position[Z_AXIS] = Setting.zprobe_zoffset; 

    plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS]);
}

#else // not AUTO_BED_LEVELING_GRID
static void set_bed_level_equation_3pts(float z_at_pt_1, float z_at_pt_2, float z_at_pt_3) {
	vector_3 from_2_to_1,from_2_to_3,planeNormal;

	matrix_3x3_set_to_identity(&Setting.plan_bed_level_matrix);

    vector_3 pt1,pt2,pt3;
	vector_3_init(&pt1,ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, z_at_pt_1);//写法可能不对
    	vector_3_init(&pt2,ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, z_at_pt_2);
   	vector_3_init(&pt3,ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, z_at_pt_3);

	vector_3_get_normal(&from_2_to_1, (pt1 - pt2));
	vector_3_get_normal(&from_2_to_3, (pt3 - pt2));


	vector_3_cross(&planeNormal, from_2_to_1, from_2_to_3);
	vector_3_get_normal(&planeNormal, planeNormal);
    	vector_3_init(&planeNormal,planeNormal.x, planeNormal.y, abs(planeNormal.z));

	matrix_3x3_create_look_at(&Setting.plan_bed_level_matrix, planeNormal);

    vector_3 corrected_position;
	 plan_get_position(&corrected_position);//取当前XYZ轴的值mm
    Current_Position[X_AXIS] = corrected_position.x;
    Current_Position[Y_AXIS] = corrected_position.y;
    Current_Position[Z_AXIS] = corrected_position.z;


    Current_Position[Z_AXIS] = Setting.zprobe_zoffset;

    plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS]);

}

#endif // AUTO_BED_LEVELING_GRID
static void run_z_probe() {
	u8 i=0;
	float Z_Current_Position=0.0;
	matrix_3x3_set_to_identity(&Setting.plan_bed_level_matrix);

    Current_Feedrate = Setting.home_speed[Z_AXIS];

    // 往下移动直到找到热床
    float zPosition = -420;
    Plan_buffer_line(Current_Position[X_AXIS], Current_Position[Y_AXIS], zPosition, Current_Position[E_AXIS], Current_Feedrate/100, Current_Active_Extruder);//Current_Feedrate/60
   Queue_wait();
   
   checkHitEndstops();//测试用，检查目前的Z值
   
        // 必须让planner知道现在所在的位置，因为现在的位置并不是预计要走到的位置
    zPosition = st_get_position_mm(Z_AXIS);
    //Current_Position[Z_AXIS] = 0.0;
    plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], zPosition, Current_Position[E_AXIS]);

    // 上升回缩的距离??
    //zPosition += home_retract_mm(Z_AXIS);
    for(i=0;i<10;i++)
    {
 #ifdef CC_3D_Touch
    	 if(GPIO_ReadInputDataBit(Endstop,Max_Z)==1)
	 {
		//strcpy(com_cpy,"M2103\r\n");   
		if(system_infor.sd_print_status != SD_PRINT_IDLE )
            		command_process("M2103\r\n");
            	return;
	 }
 #endif
    	 zPosition += 5;
   	 Plan_buffer_line(Current_Position[X_AXIS], Current_Position[Y_AXIS], zPosition, Current_Position[E_AXIS], Current_Feedrate/200, Current_Active_Extruder);//Current_Feedrate/120
    	 Queue_wait();
	
    	// 降低速度再次寻找热床
    	Current_Feedrate = Setting.home_speed[Z_AXIS]/6;
    	zPosition -= 5 * 2;
    	Plan_buffer_line(Current_Position[X_AXIS], Current_Position[Y_AXIS], zPosition, Current_Position[E_AXIS], Current_Feedrate/300, Current_Active_Extruder);//Current_Feedrate/180
   	Queue_wait();
   
   	checkHitEndstops();//测试用，检查目前的Z值
   
    	Current_Position[Z_AXIS] = st_get_position_mm(Z_AXIS);
    	if((Current_Position[Z_AXIS]-Z_Current_Position)<0.03&&(Current_Position[Z_AXIS]-Z_Current_Position)>-0.03)
    	{
    		Z_Current_Position = Current_Position[Z_AXIS];
    		plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS]);
    		break;
    	}
    	else
    	{
    		Z_Current_Position = Current_Position[Z_AXIS];
    	}
    }
    
}

static void do_blocking_move_to(float x, float y, float z) {
    float oldFeedRate = Current_Feedrate;

    Current_Feedrate = Setting.home_speed[Z_AXIS];

    Current_Position[Z_AXIS] = z;
    Plan_buffer_line(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS], Current_Feedrate/80, Current_Active_Extruder);//Current_Feedrate/40
    Queue_wait();

    Current_Feedrate = XY_TRAVEL_SPEED;

    Current_Position[X_AXIS] = x;
    Current_Position[Y_AXIS] = y;
    Plan_buffer_line(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS], Current_Feedrate/50, Current_Active_Extruder);//Current_Feedrate/50
    Queue_wait();

    Current_Feedrate = oldFeedRate;
}


static float probe_pt(float x, float y, float z_before) {
  // move to right place
  do_blocking_move_to(Current_Position[X_AXIS], Current_Position[Y_AXIS], z_before);
  do_blocking_move_to(x - X_PROBE_OFFSET_FROM_EXTRUDER, y - Y_PROBE_OFFSET_FROM_EXTRUDER, Current_Position[Z_AXIS]);

  engage_z_probe();   // Engage Z Servo endstop if available
  run_z_probe();
  float measured_z = Current_Position[Z_AXIS];
  retract_z_probe();

  sprintf(Printf_Buf, "Llit x: %f y: %f z: %f\r\n",x,y,measured_z);//输出在(x,y)位置测量到的Z值
  printf("%s",Printf_Buf);//my_printf(Printf_Buf);

  return measured_z;
}

static void Zprobe_home_all_axis(void)
{
		TIM_Cmd(TIM8, DISABLE);
              for(u8 k=0;k<2;k++)
              {
                Autohome.period[k] = (int)(100000/(Setting.steps_per_mm[k]*Setting.home_speed[k]));           
                Autohome.step[k] = 0;             
                Autohome.flag[k] = 1; 
              }
              Autohome.count = 0;
              Autohome.timer = 899;
              TIM8->ARR = Autohome.timer;
              TIM_Cmd(TIM8, ENABLE);

            if(Setting.home_position[X_AXIS] == MINENDSTOP)
            {
                Current_Position[X_AXIS] = 0;
            }
            else
            {
                Current_Position[X_AXIS] = Setting.max_position[X_AXIS];  
            }
            
            if(Setting.home_position[Y_AXIS] == MINENDSTOP)
            {
                Current_Position[Y_AXIS] = 0;
            }
            else
            {
                Current_Position[Y_AXIS] = Setting.max_position[Y_AXIS];
            }
            
            if(Setting.home_position[Z_AXIS] == MINENDSTOP)
            {
                Current_Position[Z_AXIS] = 0;
            }
            else
            {
            	   Setting.max_position[Z_AXIS]=Setting.max_position[Z_AXIS];
            }
}
static u8 Auto_origin_flags=0;

void Reset_G29Z_Origin(void)
{
	setup_for_endstop_move();
	float measured_z = probe_pt((float)Z_SAFE_HOMING_X_POINT, (float)Z_SAFE_HOMING_Y_POINT,Z_RAISE_BEFORE_PROBING );//开始探测并进行探测流程Z_RAISE_BEFORE_PROBING
	Current_Position[Z_AXIS] = 0;
	Current_Position[E_AXIS] = 0;
	plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS]);
	clean_up_after_endstop_move();

}

void Auto_Leveling_run_G28(void)
{
	u8 res=0xFF;
		res = Recovery_detect();  //一、判断是否为续打
		if(res != 0)
		{	
			system_infor.Motor_Disable_Times=0;
			Leveling_SetMotor_flag =ENABLE;
             		while((Autohome.flag[0]>0) || (Autohome.flag[1]>0) || (Autohome.flag[2]>0));//等待归位完成
   #ifdef  CC_3D_Touch
             		//二、BLTouch传感器自测试
             		i=BLTouch_SelfCheck(0);
			if(true==i)
                    {
                          plan_init();
                          return;
                    }	
    #endif
			plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS]);
              	setup_for_endstop_move();
              
             		float measured_z = probe_pt((float)Z_SAFE_HOMING_X_POINT, (float)Z_SAFE_HOMING_Y_POINT,Z_RAISE_BEFORE_PROBING );//开始探测并进行探测流程Z_RAISE_BEFORE_PROBING
		
		 	Queue_wait();
			delay_beep(200);
			clean_up_after_endstop_move();
			if(Auto_Levele_Offset_Flag==0)
			{
				Zprobe_home_all_axis();//测量完毕归位并重设当前坐标
				//Auto_Levele_Offset_Flag=0;
			}  
			Current_Position[Z_AXIS] = 0;
			Current_Position[E_AXIS] = 0;
			plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS]);
			system_infor.Motor_Disable_Times=0;
			Leveling_SetMotor_flag = DISABLE;
			if(Auto_Levele_Offset_Flag==1)
			{
				 Auto_Levele_Offset_Flag=0;
                            command_process("G0 Y160.0 F1800");
			}

		}
		else
			plan_init();
}



void Send_machine_info(void)
{
    	if(Firmware_Updata_Flag==1)
	   return;
    sprintf(Printf_Buf,"MACHINE_TYPE:%s UUID:%s FIRMWARE_NAME:%s\r\n",MACHINE_TYPE,Setting.SN,Firmware_version);
    my_printf(Printf_Buf);
    printf("%s",Printf_Buf);
    delay_ms(100);
    sprintf(Printf_Buf,"PROTOCOL_VERSION:V1.0 EXTRUDER_COUNT:1\r\n");
    my_printf(Printf_Buf);
    printf("%s",Printf_Buf);
    delay_ms(100);
    
  
}



extern float Current_Temperature[5]; 

extern u8 Rev_File_Flag;
void Send_Printer_State(void)
{
    if(Rev_File_Flag==1)
    return;
    if(system_infor.system_status == STANDBY)
    {
             sprintf(Printf_Buf,"printer_standby;SD:%d;fm:1;\r\n",system_infor.sd_status);
             USART2_TxString((u8*)Printf_Buf);

           
    }
    else
    {
              if((system_infor.system_status == PRINT&&(system_infor.sd_print_status!=SD_PRINT_PAUSE))||system_infor.serial_printf_flag==ENABLE)
              {
                     sprintf(Printf_Buf,"printer_printing;file:%s;precent:%d%;mot:%d;SD:%d;fm:1;\r\n",&Systembuf_Infos.printer_file_path[5],(u16)system_infor.print_percent,Get_Motor_Status(),system_infor.sd_status);
                     USART2_TxString((u8*)Printf_Buf);
              }
              else if(system_infor.sd_print_status == SD_PRINT_PAUSE)
              {
                     sprintf(Printf_Buf,"printer_paused;file:%s;precent:%d%;mot:%d;SD:%d;fm:1;\r\n",&Systembuf_Infos.printer_file_path[5],(u16)system_infor.print_percent,Get_Motor_Status(),system_infor.sd_status);
                     USART2_TxString((u8*)Printf_Buf);
              }
              else if(system_infor.sd_print_status==SD_PRINT_FINISH)
              {
                      sprintf(Printf_Buf,"printer_finish;file:%s;mot:%d;SD:%d;fm:1;\r\n",&Systembuf_Infos.printer_file_path[5],Get_Motor_Status(),system_infor.sd_status);
                      USART2_TxString((u8*)Printf_Buf);

              }
              else
              { 
                     sprintf(Printf_Buf,"printer_idle;mot:%d;SD:%d;fm:1;\r\n",Get_Motor_Status(),system_infor.sd_status);
                     USART2_TxString((u8*)Printf_Buf);
              }
     }
}

void send_xyz_coordinate(void)
{
            sprintf(Printf_Buf, "X:");my_printf(Printf_Buf);
            sprintf(Printf_Buf, "%.3f;",Current_Position[X_AXIS]);my_printf(Printf_Buf);
            sprintf(Printf_Buf, "Y:");my_printf(Printf_Buf);
            sprintf(Printf_Buf, "%.3f;",Current_Position[Y_AXIS]);my_printf(Printf_Buf);
            sprintf(Printf_Buf, "Z:");my_printf(Printf_Buf);
            sprintf(Printf_Buf, "%.3f;",Current_Position[Z_AXIS]);my_printf(Printf_Buf);
            sprintf(Printf_Buf, "\n");my_printf(Printf_Buf);

}
void Set_home_position(void)
{
	if(Setting.home_position[X_AXIS] == MINENDSTOP)
            {
                Current_Position[X_AXIS] = 0;
            }
            else
            {
                Current_Position[X_AXIS] = Setting.max_position[X_AXIS];  
            }
            
            if(Setting.home_position[Y_AXIS] == MINENDSTOP)
            {
                Current_Position[Y_AXIS] = 0;
            }
            else
            {
                Current_Position[Y_AXIS] = Setting.max_position[Y_AXIS];
            }
            
            if(Setting.home_position[Z_AXIS] == MINENDSTOP)
            {
                Current_Position[Z_AXIS] = 0;
            }
            else
            {
                Current_Position[Z_AXIS] = Setting.max_position[Z_AXIS]; 
            }
            Current_Position[E_AXIS] = 0;
            send_xyz_coordinate();
}
void Send_m105_ask(void)
{
	if(Rev_File_Flag!=1)
	 sprintf(Printf_Buf," ok B:%.1f /%.1f T0:%.1f /%.1f T1:%.1f /%.1f T2:%.1f /%.1f F:%d R:%d @:0 B@:0\r\n",
			   Current_Temperature[BED],Setting.targe_temperature[BED],
			   Current_Temperature[NOZZLE0],Setting.targe_temperature[NOZZLE0],
			   Current_Temperature[NOZZLE1],Setting.targe_temperature[NOZZLE1],
			   Current_Temperature[NOZZLE2],Setting.targe_temperature[NOZZLE2],
			    
			    system_infor.fan_hotend_speed*100/255 ,
			     system_infor.feed_tare);
	    USART2_TxString((u8 *)Printf_Buf);
}
void  Return_M105_Command(void)
{
	//if(Rev_File_Flag!=1)
	//{
		printf("\r\nok  B:%.1f /%.1f T0:%.1f /%.1f T1:%.1f /%.1f T2:%.1f /%.1f F:%d R:%d @:0 B@:0\r\n",
			   Current_Temperature[BED],Setting.targe_temperature[BED],
			   Current_Temperature[NOZZLE0],Setting.targe_temperature[NOZZLE0],
			   Current_Temperature[NOZZLE1],Setting.targe_temperature[NOZZLE1],
			   Current_Temperature[NOZZLE2],Setting.targe_temperature[NOZZLE2],
			    (u16)(system_infor.fan_hotend_speed * 100 / 255),
			     system_infor.feed_tare);

		system_infor.serial_ack_flag = 0;//不响应ok的字符反馈信号,上面的字符串传输上已经有了
		//my_printf(Printf_Buf);
	//}
}
void Send_Printer_Status(void)
{
if(Rev_File_Flag!=1)
  printf("printer_status:%d;precent:%d;SD:%d;file:%s;\n",system_infor.sd_print_status,(u32)system_infor.print_percent,system_infor.sd_status,&Systembuf_Infos.printer_file_path[5]);           
}

void Send_printer_data(void)
{
     Add_Upload_data();
}




void Processing_command(void)
{
  int k;
  float temp_temperature;
  u16 temp_fan_speed;
  char g_code;
  float targe_temperatu[2];
  char* ch_point;
  float x_tmp, y_tmp, z_tmp, real_z;
  system_infor.serial_ack_flag = 1;
   static float tempZ;
  g_code = Gcode_is(&ch_point);
  u8 wifi_temp=0;
  char com_cpy[30];
  u8 Z_Move_Flag=0;
  if(g_code == 'G')
  {
    system_infor.Motor_Disable_Times =0;
    switch(Command_value_long(ch_point))
    {
      case 0:
      case 1://直线运动 
		motor_enable_status=1;
             Get_linear_coordinates();
             Prepare_linear_move();
        return;
///////////////////////////////////////////////////////////////////////////////	        
      case 2://顺时针做弧形运动
              Get_arc_coordinates();
              Prepare_arc_move(CLOCKWISE);
        return;
///////////////////////////////////////////////////////////////////////////////	        
      case 3://逆时针做弧形运动
              Get_arc_coordinates();
              Prepare_arc_move(ANTICLOCKWISE);
        return;     

      case 4://暂停
        system_infor.pause_time = 0;
        if(Command_is('P',&ch_point))
          system_infor.pause_time = Command_value_long(ch_point);//ms
        if(Command_is('S',&ch_point))
          system_infor.pause_time = Command_value_long(ch_point)*1000;//s
        system_infor.pause_flag = true;
        Queue_wait();
        while(system_infor.pause_time);
        system_infor.pause_flag = false;//结束暂停*/
        break;
      
      case 20://设定使用的单位为英寸
        Setting.unit = INCH;
        return;
				
      case 21://设定使用的单位为毫米
        Setting.unit = MM;
        if((system_infor.serial_printf_flag == DISABLE) && (serial_connect_flag == ENABLE))
        {
                system_infor.serial_printf_flag=ENABLE;
                system_infor.system_status=SERIAL_CONNECT;
        }
        return;
  
      case 28://所有轴归零
         printf("Setting.zprobe_zoffset = %f\r\n",Setting.zprobe_zoffset);
        motor_enable_status=1;
        Gloabl_axis_go_origin_flag = true;
        Queue_wait();
        for(unsigned char i=0;i<3;i++)     //dxc 2015-03-31   原来是i<2
        {
          if(Command_is(Coordinate_Axias[i],&ch_point))
          {
          	if(system_infor.Auto_Levele_Flag ==1&&i==2)
          	{
			break;
          	}
           	Signal_axis_go_origin(Command_value_float(ch_point),i);
		signal_axis_plan_init(i);
             Gloabl_axis_go_origin_flag = false;
            
            switch(i)
            {
                case 0:
                       if(Setting.home_position[X_AXIS] == MINENDSTOP)
                           Current_Position[i] = 0;
                       else
                           Current_Position[i] = Setting.max_position[X_AXIS];
                       printf("X:%.3f\n",Current_Position[X_AXIS]);
                       break;
                       
                case 1:
                       if(Setting.home_position[Y_AXIS] == MINENDSTOP)
                           Current_Position[i] = 0;
                       else
                           Current_Position[i] = Setting.max_position[Y_AXIS];
			   printf("Y:%.3f\n",Current_Position[Y_AXIS]);
                       break;
                       
                case 2:
                
                       if(Setting.home_position[Z_AXIS] == MINENDSTOP)
                           Current_Position[i] = 0;
                       else
                           Current_Position[i] = Setting.max_position[Z_AXIS];
                       Z_Move_Flag=1;
                       printf("Z:%.3f\n",Current_Position[Z_AXIS]);
                       break;
            }
          }
        }
        if(Gloabl_axis_go_origin_flag)
        {
              TIM_Cmd(TIM8, DISABLE);
              for(k=0;k<3;k++)
              {
              	if(system_infor.Auto_Levele_Flag ==1&&k==2)
          		{
				break;
          		}
          		Z_Move_Flag=1;
                    Autohome.period[k] = (int)(100000/(Setting.steps_per_mm[k]*Setting.home_speed[k]));           
                    Autohome.step[k] = 0;             
                    Autohome.flag[k] = 1; 
                   
              }
              Autohome.count = 0;
              Autohome.timer = 899;
              TIM8->ARR = Autohome.timer;
              TIM_Cmd(TIM8, ENABLE);
		 
		Set_home_position();	           
		 if(system_infor.Auto_Levele_Flag ==1 )
		 {
			Auto_Leveling_run_G28();
		}	
	      else//自动调平和非自动调平初始化不一样
	      {
			plan_init();
			if(Z_Move_Flag==1)
			{
				sprintf(com_cpy,"G92 Z-%.2f\r\n",Setting.zz_offset);         
            			command_process(com_cpy);
	     			strcpy(com_cpy,"G1 Z0 F800\r\n");         
            			command_process(com_cpy);
            		}
		}            		
            Add_Message(COORDINATE_XYZ);
        }
		if(AUTO_LEVELE_G29_FLAG==2)
		{
			Up_CL_Flag=1;
			AUTO_LEVELE_G29_FLAG=0;
			strcpy(Command_Buffer,"G29\r\n");
    			Processing_command();
    			Up_CL_Flag=0;
		}
        	
        	break;
       
    case 29: // G29 Z轴多点探测
        {
	    	 if(system_infor.Auto_Levele_Flag !=1 ||AUTO_LEVELE_G29_FLAG!=0)    
	     		break;
        	
              Queue_wait();//运行以前指令buff必须运行完毕
              while((Autohome.flag[0]>0) || (Autohome.flag[1]>0));//等待归位完成，现在这里做测试用

              if (! (axis_known_position[X_AXIS] && axis_known_position[Y_AXIS]) )//使用G29以前，XYZ轴必须归位，并且运行过原点测试，否则报错并返回
              {
		     sprintf(Printf_Buf, "Home X/Y/Z before running Z-Probe\r\n");
		     my_printf(Printf_Buf);
		     if(system_infor.sd_print_status==SD_PRINTING)//检查到错误，打印终止
		     {
			    system_infor.sd_print_status=SD_PRINT_FINISH;
		     }
                   break; 
              }
		 axis_known_position[X_AXIS]=false;//归位标志清空
            	 axis_known_position[Y_AXIS]=false;//归位标志清空

            	
		 printf("position before G29 x: %f y: %f z: %f\r\n",Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS]);//输出在(x,y)位置测量到的Z值
		 
		//初始化矩阵
		matrix_3x3_set_to_identity(&Setting.plan_bed_level_matrix);//设置3x3单位矩阵(主对角线为1)
		vector_3 uncorrected_position;
	      plan_get_position(&uncorrected_position);//取当前XYZ轴的值

            Current_Position[X_AXIS] = uncorrected_position.x;
            Current_Position[Y_AXIS] = uncorrected_position.y;
            Current_Position[Z_AXIS] = uncorrected_position.z;
            plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS]);
            setup_for_endstop_move();//保存运行速度，速度百分比等，使能stepper中关于此功能的相关运行代码
            
            Current_Feedrate =  Setting.home_speed[Z_AXIS];
#ifdef AUTO_BED_LEVELING_GRID
            // probe at the points of a lattice grid

            int xGridSpacing = (RIGHT_PROBE_BED_POSITION - LEFT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);
            int yGridSpacing = (BACK_PROBE_BED_POSITION - FRONT_PROBE_BED_POSITION) / (AUTO_BED_LEVELING_GRID_POINTS-1);


            double eqnAMatrix[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS*3];
            double eqnBVector[AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS];
      


            int probePointCounter = 0;//探测点
            bool zig = true;//这个是换行的方向

            for (int yProbe=FRONT_PROBE_BED_POSITION; yProbe <= BACK_PROBE_BED_POSITION; yProbe += yGridSpacing)//y方向上在范围内的探测点个数
            {
              int xProbe, xInc;
              if (zig)//这个是切换x方向上的起始点位置
              {
                xProbe = LEFT_PROBE_BED_POSITION;
                xInc = xGridSpacing;
                zig = false;
              } else // zag=faluse
              {
                xProbe = RIGHT_PROBE_BED_POSITION;
                xInc = -xGridSpacing;
                zig = true;
              }

              for (int xCount=0; xCount < AUTO_BED_LEVELING_GRID_POINTS; xCount++)
              {
                float z_before;
                if (probePointCounter == 0)//探测开始和探测中Z轴抬高的距离
                {
                  // raise before probing
                  z_before = Z_RAISE_BEFORE_PROBING;
                } else
                {
                  // raise extruder
                  z_before = Current_Position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS;
                }
		   sprintf(Printf_Buf, "raise extruder Z%d: %f\r\n",probePointCounter,z_before);//输出Z轴探测点抬高距离
		   printf("%s\r\n",Printf_Buf);//my_printf(Printf_Buf);

                float measured_z = probe_pt(xProbe, yProbe, z_before);//开始探测并进行探测流程

                eqnBVector[probePointCounter] = measured_z;

                eqnAMatrix[probePointCounter + 0*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = xProbe;
                eqnAMatrix[probePointCounter + 1*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = yProbe;
                eqnAMatrix[probePointCounter + 2*AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS] = 1;
                probePointCounter++;
                xProbe += xInc;
              }
            }//循环检测到此结束

			
            clean_up_after_endstop_move();//恢复运行速度，速度百分比等，失能stepper中关于此功能的相关运行代码
            Reset_G29Z_Origin();
		double plane_equation_coefficients[3];
            // solve lsq problem等会看着修正函数，好像传递参数不对
            qr_solve(plane_equation_coefficients,AUTO_BED_LEVELING_GRID_POINTS*AUTO_BED_LEVELING_GRID_POINTS, 3, eqnAMatrix, eqnBVector);

		sprintf(Printf_Buf, "Eqn coefficients: a: %f b: %f d: %f\r\n",plane_equation_coefficients[0],plane_equation_coefficients[1],plane_equation_coefficients[2]);
		printf("AUU:%s\r\n",Printf_Buf);//my_printf(Printf_Buf);


            set_bed_level_equation_lsq(plane_equation_coefficients);
#else // AUTO_BED_LEVELING_GRID not defined

            // Probe at 3 arbitrary points
            // probe 1
            float z_at_pt_1 = probe_pt(ABL_PROBE_PT_1_X, ABL_PROBE_PT_1_Y, Z_RAISE_BEFORE_PROBING);

            // probe 2
            float z_at_pt_2 = probe_pt(ABL_PROBE_PT_2_X, ABL_PROBE_PT_2_Y, 	Current_Position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

            // probe 3
            float z_at_pt_3 = probe_pt(ABL_PROBE_PT_3_X, ABL_PROBE_PT_3_Y, Current_Position[Z_AXIS] + Z_RAISE_BETWEEN_PROBINGS);

            clean_up_after_endstop_move();

            set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);
#endif // AUTO_BED_LEVELING_GRID
            //st_synchronize();
            Queue_wait();
            
            //real_z = (float)(st_get_position(Z_AXIS))/Setting.steps_per_mm[Z_AXIS];  //get the real Z (since the auto bed leveling is already correcting the plane)
            x_tmp = Current_Position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER;
            y_tmp = Current_Position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER;
            z_tmp = Current_Position[Z_AXIS];

            apply_rotation_xyz(Setting.plan_bed_level_matrix, &x_tmp, &y_tmp, &z_tmp);         //Apply the correction sending the probe offset坐标倾斜转换
            Current_Position[Z_AXIS] = z_tmp;// - real_z + Current_Position[Z_AXIS];   //The difference is added to current position and sent to planner.
            plan_set_position(Current_Position[X_AXIS], Current_Position[Y_AXIS], Current_Position[Z_AXIS], Current_Position[E_AXIS]);
        }
        break;
      case 90://设定使用绝对坐标
        Setting.locate_mode = ABSOLUTE_COORDINATE;
		system_infor.serial_axis_move_cmd=DISABLE;
		system_infor.Motor_Disable_Times =0;
        break;
      case 91://设定使用相对坐标
        Setting.locate_mode = RELATIVE_COORDINATE;
        system_infor.serial_axis_move_cmd=ENABLE;     
        system_infor.Motor_Disable_Times =0;
        break;	        
      case 92://重置当前的坐标
        system_infor.Motor_Disable_Times =0;
        Gloabl_axis_reset_coordinate_flag = true;
        Queue_wait();
        for(unsigned char i=0;i<NUM_AXIS;i++)
        {
          if(Command_is(Coordinate_Axias[i],&ch_point))
          {
               Single_axis_reset_coordinate(Command_value_float(ch_point),i);
               Gloabl_axis_reset_coordinate_flag = false;
          }
        }
        if(Gloabl_axis_reset_coordinate_flag)
            All_axis_reset_coordinate();

        return;	
    default: break;
    }
  }	
  else if(g_code == 'M')
  {
    switch(Command_value_long(ch_point))
    {
        //Queue_wait();
      case 17://enable all step motor
       motor_enable_status=1;
        Enable_X_Axis();
        Enable_Y_Axis();
        Enable_Z_Axis();
        Enable_E0_Axis();
        Enable_E1_Axis();
        Enable_E2_Axis();
        break;
      case 18://disable all step motor
                motor_enable_status=0;
      		  Disable_all_Axis();
        break;
    case 20:
	   if(strstr(Command_Buffer, "LCD")!=NULL)
	   {
	   	 if(system_infor.sd_status == SD_OK)
	   	 {
	   	 	Send_SD_DirtoLCD();	
	   	 }
	   }
	    else
	    {
	        	if(system_infor.sd_status != SD_OK)
		            {
		                sprintf(Printf_Buf, "SD: fail\r\n");my_printf(Printf_Buf);
				   delay_ms(200);	
				   break;
		           }
			    else if(CommandStr_is("M20 sent succeed!---",&ch_point))
		           {
		             		APP_Refresh_flag.App_sendfiles_dirmun=(u8)Command_value_long(ch_point);
		                	APP_Refresh_flag.App_Sendfiles_FLAG=ENABLE;
					break;
			    }
			     else if(UARST_REDIRECT==UARST2_REDIRECT)
				{
					Wifi_ASK_Factors.WIFI_COMMEND=20;
					Wifi_ASK_State=WIFI_READY;
					APP_Refresh_flag.App_Sendfiles_FLAG=DISABLE;
					break;
				}
				
		            if(system_infor.sd_status != SD_OK)//电脑用
		            {
		                sprintf(Printf_Buf, "SD: fail\r\n");my_printf(Printf_Buf);
		                get_M2101_status();
		           }
		            else
			        Send_SD_Dir(SD_Path);
	    	}
        break;
    case 21:
       if(SD_Initialize() != SD_OK)
       {
             printf("echo:SD init fail\n");
       }
       else
       {
             printf("echo:SD card ok\n");
       }
       break;
    case 23:
       SD_filePrint_select();
       break;
    case 24:
    		
		Up_Data_Flag =1;
		system_infor.fan_controler_speed = 256;
		system_infor.fan_hotend_speed = 256;
		SET_FAN0_SPEED(FULL_POWER);
		SET_FAN1_SPEED(FULL_POWER);
    		SET_FAN2_SPEED(FULL_POWER);
    		printf("SD_PRINT_RECOVERY...\r\n");
		if(system_infor.sd_print_status==SD_PRINT_PAUSE)
		{
			system_infor.sd_print_status = SD_PRINT_RECOVERY;
			system_infor.sd_print_flag = ENABLE;
			system_infor.stop_flag =0;
			USART3_printf("<AUTO_UD:ST:2;*>\r\n");
			printf("SD_PRINT_RECOVERY...\r\n");
			return;
		}
      if(system_infor.sd_file_cmp==1&&system_infor.sd_print_flag!=1)
      {
          system_infor.print_file_size=file_size[system_infor.selected_file_pos];
          if((strstr( &sd_file_name[system_infor.selected_file_pos][0], ".gcode") != NULL)||(strstr( &sd_file_name[system_infor.selected_file_pos][0], ".gco") != NULL)||(strstr( &sd_file_name[system_infor.selected_file_pos][0], ".GCO") != NULL))
          {
              plan_init();
              sprintf(&SD_Path[strlen(SD_Path)], "/%s", sd_file_name[system_infor.selected_file_pos]);
              strcpy(Systembuf_Infos.printer_file_path,SD_Path);

			 system_infor.sd_print_flag = ENABLE;		  
			 system_infor.system_status = PRINT;
			system_infor.stop_flag =0;
			 system_infor.sd_print_status = SD_PRINT_START;
			 system_infor.sd_file_cmp=0;
                      
                         sprintf(Printf_Buf,"Start Print:%s\r\n",Systembuf_Infos.printer_file_path); 
                         my_printf(Printf_Buf);
                         printf("%s\r\n",Printf_Buf);
                         AUTO_LEVELE_G29_FLAG=2;
          }
	   else
	   {
		printf("Print file fail!\r\n");
		sprintf(Upload_DataS,"<AUTO_UD:ST:%d;*>\r\n",system_infor.sd_print_status);
		USART3_printf(Upload_DataS);
		//USART3_printf("<AUTO_UD:ST:0;*>\r\n");
	   }

          break;
          
      }
      else
      {
		sprintf(Upload_DataS,"<AUTO_UD:ST:%d;*>\r\n",system_infor.sd_print_status);
		USART3_printf(Upload_DataS);
      }
      break;
    case 25:
    		if(system_infor.sd_print_status==SD_PRINTING)
		{
    			SET_FAN1_SPEED(HALF_POWER);
    			SET_FAN2_SPEED(HALF_POWER);
    	     		system_infor.stop_flag =1;
            		 system_infor.sd_print_status=SD_PRINT_PAUSE_F;
             		USART3_printf("<AUTO_UD:ST:4;*>\r\n");
             		sprintf(Printf_Buf,"Stop Print\r\n"); 
             		my_printf(Printf_Buf);
             }
             else
             {
			sprintf(Upload_DataS,"<AUTO_UD:ST:%d;*>\r\n",system_infor.sd_print_status);
		       USART3_printf(Upload_DataS);
             }
          break;
    case 30:
       SD_Delete_File();
           break;
    case 32: //select SD file print (uart and wifi)(2017.1.4 add)
    
		  break;
          ///////////////////////////////////////////////////////////////////////////////	
      case 84://disable all step motor
           SetMotorEnableFlag(0);
          Disable_all_Axis();

	   	if(system_infor.serial_printf_flag == 1)
	   	{		
			system_infor.serial_printf_flag = 0;
			system_infor.print_percent = 100;
			
			if(WIFI_MODE == WIFI_HANDLE_DATA && UARST_REDIRECT == UARST1_REDIRECT)
				{
				sprintf(Printf_Buf,"printer_SerialPrinting;precent:100%;\r\n");
				USART2_TxString((u8 *)(&Printf_Buf));
				}
			
			command_process("M17\r\n");//归位
	             command_process("G28 XY\r\n");
			while((Autohome.flag[0]>0) || (Autohome.flag[1]>0) || (Autohome.flag[2]>0));
			command_process("M18\r\n");

			Set_Beep(1000,127);
          		delay_ms(1000);
		  	Set_Beep(1000,127);
	   	}
                                               
        break;
		
	  case 80://休眠恢复
	  	if(system_infor.system_status !=STANDBY)
	  	{
	  		delay_ms(100);
                    Send_Printer_State();
                    delay_ms(200);
		       return;
	  	}
            else if(system_infor.system_status == STANDBY)
                {
        	  	system_infor.system_status = IDLE;
			delay_ms(100);
			Send_Printer_State();
			delay_ms(200);
        		Add_Message(RECOVER_STANDBY);
        	  	 Send_Printer_State();
        		SetMotorEnableFlag(1);
			Set_Fan_Power(1,FULL_POWER);//重新开启挤出机风扇
        		for(unsigned char i=0;i<3;i++)
        		{
        			Signal_axis_go_origin(Command_value_float(ch_point),i);
        			signal_axis_plan_init(i);
        		}
                        while(Autohome.flag[0] || Autohome.flag[1] || Autohome.flag[2]);
                }
		break;
		
	  case 81://休眠
	  	if(system_infor.system_status ==STANDBY)
	  	{
	  		delay_ms(100);

		if(system_infor.sd_print_status==SD_PRINT_PAUSE)//休眠状态在打印的话发送打印信息
          		sprintf(Printf_Buf,"printer_standby;file:%s;precent:%d%;mot:%d;fm:%d;\r\n",&Systembuf_Infos.printer_file_path[5],(u32)system_infor.print_percent,Get_Motor_Status(),get_filament_status());
		else
			sprintf(Printf_Buf,"printer_standby;mot:%d;fm:%d;\r\n",Get_Motor_Status(),get_filament_status());

		my_printf(Printf_Buf);
		delay_ms(200);
				return;
	  	}
            if(system_infor.sd_print_status!=SD_PRINT_RECOVERY&&system_infor.serial_printf_flag != 1)
            {
            system_infor.system_status = STANDBY; 
                delay_ms(100);
                sprintf(Printf_Buf,"printer_standby;\r\n");my_printf(Printf_Buf);
                delay_ms(200);
                if(system_infor.sd_print_status== SD_PRINTING)
                {
                      
                      my_printf(Printf_Buf);
                      Block_clean();
                      Current_Block_Clean();
		      plan_init();
		      Recovery_create();
		      strcpy(Command_Buffer,"M104\r\n");
                   Processing_command();
                   strcpy(Command_Buffer,"M140\r\n");
                   Processing_command();
		      f_close(&Print_File);
                      system_infor.sd_print_flag = DISABLE; 
                      system_infor.sd_print_status=SD_PRINT_PAUSE; 
                      sprintf(Printf_Buf,"Stop Print\r\n"); 
                      my_printf(Printf_Buf);
                }
	  	Add_Message(INPUT_STANDBY);
	  	Setting.hotend_enable[NOZZLE0] = DISABLE;
		Setting.hotend_enable[NOZZLE1] = DISABLE;
		Setting.hotend_enable[NOZZLE2] = DISABLE;
		Setting.hotend_enable[BED] = DISABLE;
		Set_All_Fan_Power(SHUT_OFF_POWER);//关闭所有风扇电源
		for(unsigned char i=0;i<3;i++)
		{
			Signal_axis_go_origin(Command_value_float(ch_point),i);
			signal_axis_plan_init(i);
		}
                while(Autohome.flag[0] || Autohome.flag[1] || Autohome.flag[2]);//等待所有轴归零
                
           if(Setting.home_position[X_AXIS] == MINENDSTOP)//防止休眠恢复以后移动指令引起附加的位移，必须在归位以后把坐标也归位
                Current_Position[X_AXIS] = 0;
           else
                Current_Position[X_AXIS] = Setting.max_position[X_AXIS];  
            if(Setting.home_position[Y_AXIS] == MINENDSTOP)
                Current_Position[Y_AXIS] = 0;
            else
                Current_Position[Y_AXIS] = Setting.max_position[Y_AXIS];
            if(Setting.home_position[Z_AXIS] == MINENDSTOP)
                Current_Position[Z_AXIS] = 0;
            else
                Current_Position[Z_AXIS] = Setting.max_position[Z_AXIS]+Setting.zprobe_zoffset; 
		   Current_Position[E_AXIS] = 0;
		Disable_all_Axis();
		SetMotorEnableFlag(0);
		
            }
            
               
	  	break;
        
      case 92://   M92: Set axis_steps_per_unit    
        Queue_wait();
        Set_axis_steps_per_unit();      
        break;
        
      case 104://set print temperature

          if(Command_is('T',&ch_point))
          {
            targeted_hotend = (u8)Command_value_float(ch_point);
          }
          else 
            targeted_hotend = NOZZLE0;
          
          if(targeted_hotend >= Setting.hotend_num)
          {
            printf("no find this hotend\r\n");   //dxc
            targeted_hotend = NOZZLE0;
          }
          if(Command_is('S',&ch_point))
          {
            temp_temperature = Command_value_float(ch_point);
            Setting.targe_temperature[targeted_hotend] = temp_temperature;
            Setting.hotend_enable[targeted_hotend] = ENABLE;
          }
          else
          {
            Setting.targe_temperature[targeted_hotend] = 0.0;
            Setting.hotend_enable[targeted_hotend] = DISABLE;
          }

          Temperature_Sampling_Timer_Config();
          Temperature_Sampling_NVIC_Configuration();
          Temperature_Measure_Enable();
          Temperature_Control_Enable();
          //sprintf(Printf_Buf, "temperature set is ok\r\n");
          return;
///////////////////////////////////////////////////////////////////////////////	
    case  105:
         Return_M105_Command();
        
        return;
/**********************************************************************************/
    case 106://set fan speed  fan_controler_speed,fan_hotend_speed
      {
      u8 fan_num = 0; 
      if(Command_is('P',&ch_point))
        {
          fan_num = (u8)Command_value_long(ch_point);
        }
        if(Command_is('S',&ch_point))
        {   if(fan_num==0)
                  system_infor.fan_hotend_speed = Command_value_long(ch_point);
		 if(fan_num==2)
                  system_infor.fan_controler_speed = Command_value_long(ch_point);
        	temp_fan_speed = Command_value_long(ch_point) * FULL_POWER / 255;
          if((Command_value_long(ch_point))>=255)//gcode风速S值超出或等于255时全速运行
          {
            Set_Fan_Power(fan_num,FULL_POWER);
          }
          else if(Command_value_long(ch_point)<=0)//gcode风速值设定小于或等于0时关闭风扇
          {
              Set_Fan_Power(fan_num,SHUT_OFF_POWER);
          }
          else if((Command_value_long(ch_point) > 0) && (Command_value_long(ch_point) < 50))
          {
          	  Set_Fan_Power(fan_num,FULL_POWER);
              Set_Fan_Power(fan_num,800);                           
          }
          else
          {
              temp_fan_speed=(u16)((Command_value_long(ch_point))<<4);//把gcode风速S值0-255按比例算出PWM值（PWM全速运行的TIM比较值为4095），即4096*（S的值/256）=16*S的值                           
              Set_Fan_Power(fan_num,temp_fan_speed);              
          }
        }
      }
        break;
    case 107://turn off the fan
      {
        u8 fan_num_off = 0;
        if(Command_is('P',&ch_point))
        {
          fan_num_off = (u8)Command_value_long(ch_point);
        }

        Set_Fan_Power(fan_num_off,SHUT_OFF_POWER); 
        if(fan_num_off==0)
        break;
      }	
       break;
///////////////////////////////////////////////////////////////////////////////	      	 
    case 109:
      if(Command_is('T',&ch_point))
      {
        targeted_hotend = (u8)Command_value_float(ch_point);
      }
      else 
        targeted_hotend = NOZZLE0;
      
      if(targeted_hotend >= Setting.hotend_num)
      {
        targeted_hotend = NOZZLE0;
      }
      if(Command_is('S',&ch_point))
      {
            temp_temperature = Command_value_float(ch_point);
        Setting.targe_temperature[targeted_hotend] = temp_temperature;
        Setting.hotend_enable[targeted_hotend] = ENABLE;
		Temperature_Sampling_Timer_Config();
        Temperature_Sampling_NVIC_Configuration();
        Temperature_Measure_Enable();
        Temperature_Control_Enable();
		temp_temperature = Current_Temperature[targeted_hotend];	  
        u8 num=0,UARST123,k=0;//        
        Add_Message(TEMPERATURE_INFO);

        while((Current_Temperature[targeted_hotend] < Setting.targe_temperature[targeted_hotend])&&(Setting.hotend_enable[targeted_hotend]))
        {
               delay_ms(100);  
               num++;
        		 if(WIFI_MODE==WIFI_HANDLE_DATA&&num%5==0)
                     {
                     	if(k==1)
				{Send_Printer_State();k=2;}
				else
				{
					UARST123=UARST_REDIRECT;
					UARST_REDIRECT =UARST2_REDIRECT;
					delay_ms(1000); 
					
					sprintf(Printf_Buf, "B:%.1f /%.1f T0:%.1f /%.1f T1:%.1f /%.1f T2:%.1f /%.1f F:%d R:%d @:0 B@:0\r\n",
			  			 Current_Temperature[BED],Setting.targe_temperature[BED],
			   			Current_Temperature[NOZZLE0],Setting.targe_temperature[NOZZLE0],
			   			Current_Temperature[NOZZLE1],Setting.targe_temperature[NOZZLE1],
			   			Current_Temperature[NOZZLE2],Setting.targe_temperature[NOZZLE2],
			    			(u16)(system_infor.fan_hotend_speed * 100 / 255),
			     			system_infor.feed_tare);

		
		 			 my_printf(Printf_Buf);
		 			
	         		     
	         		     }
         		     UARST_REDIRECT=UARST123;
                     }
         		  if(fabs(Current_Temperature[targeted_hotend] - temp_temperature) >= 1)
         		  {
         		  	  temp_temperature = Current_Temperature[targeted_hotend];

			      printf("B:%.1f /%.1f T0:%.1f /%.1f T1:%.1f /%.1f T2:%.1f /%.1f F%d R:%d @:0 B@:0\r\n",
			     Current_Temperature[BED],Setting.targe_temperature[BED],
			     Current_Temperature[NOZZLE0],Setting.targe_temperature[NOZZLE0],
			     Current_Temperature[NOZZLE1],Setting.targe_temperature[NOZZLE1],
			     Current_Temperature[NOZZLE2],Setting.targe_temperature[NOZZLE2],
			     system_infor.fan_hotend_speed*100/255 ,
			     system_infor.feed_tare);

			      Add_Message(TEMPERATURE_INFO);
         		  }
         		
			 delay_ms(100);  
			
                   //UARST_REDIRECT=UARST1_REDIRECT;
          if(system_infor.stop_flag)
          {
              	return;
          	}
          if((Setting.targe_temperature[targeted_hotend] >= 180) && (Current_Temperature[targeted_hotend] >= Setting.targe_temperature[targeted_hotend]-1))
	   {
                break;
          }
        }  
       Add_Message(TEMPERATURE_INFO);
      }
      return;
///////////////////////////////////////////////////////////////////////////////	
    case  110:  //Set Current Line Number
      if(Command_is('N',&ch_point))
      {
        gcode_N = (long)Command_value_float(ch_point);
      }
      
      return;
    case  114:
        sprintf(Printf_Buf, "X:");my_printf(Printf_Buf);
        sprintf(Printf_Buf, "%.3f",Current_Position[X_AXIS]);my_printf(Printf_Buf);
        sprintf(Printf_Buf, "Y:");my_printf(Printf_Buf);
        sprintf(Printf_Buf, "%.3f",Current_Position[Y_AXIS]);my_printf(Printf_Buf);
        sprintf(Printf_Buf, "Z:");my_printf(Printf_Buf);
        sprintf(Printf_Buf, "%.3f",Current_Position[Z_AXIS]);my_printf(Printf_Buf);
        sprintf(Printf_Buf, "E:");my_printf(Printf_Buf);
        sprintf(Printf_Buf, "%.3f",Current_Position[E_AXIS]);my_printf(Printf_Buf);
        sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
        return;
    case  115:
    if(Firmware_version[3]=='0' &&Firmware_version[4]== '0')
    {
	  Add_Message(LCD_VERSION_INFO);
    }
    else
    {
	  Send_machine_info();
    }
        return;

			


 /* M117:Display Message */
    case 117:
	

		if(CommandStr_is("ETE",&ch_point))
		{
			u32 time = Command_Time_value(ch_point);
				system_infor.serial_printf_flag = 1;
				if(system_infor.serial_print_time < time)
                		{
					system_infor.serial_print_time = time;
                		}
				system_infor.print_percent = (system_infor.serial_print_time - time) * 100 / system_infor.serial_print_time;
			
		}
		
        break;
                  
/*M119:return the endstop state*/        
    case 119:
/*X MIN*/
	if(get_endstop_state( X_AXIS,MINENDSTOP))
		{
			sprintf(Printf_Buf, "X MIN Endstop:ON");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
	else
		{
			sprintf(Printf_Buf, "X MIN Endstop:OFF");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}

/*X MAX*/
	if(get_endstop_state( X_AXIS,MAXENDSTOP))
		{
			sprintf(Printf_Buf, "X MAX Endstop:ON");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
	else
		{
			sprintf(Printf_Buf, "X MAX Endstop:OFF");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}

/*Y MIN*/
	if(get_endstop_state( Y_AXIS,MINENDSTOP))
		{
			sprintf(Printf_Buf, "Y MIN Endstop:ON");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
	else
		{
			sprintf(Printf_Buf, "Y MIN Endstop:OFF");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
/*Y MAX*/
	if(get_endstop_state( Y_AXIS,MAXENDSTOP))
		{
			sprintf(Printf_Buf, "Y MAX Endstop:ON");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
	else
		{
			sprintf(Printf_Buf, "Y MAX Endstop:OFF");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
	
/*Z MIN*/
	if(get_endstop_state( Z_AXIS,MINENDSTOP))
		{
			sprintf(Printf_Buf, "Z MIN Endstop:ON");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
	else
		{
			sprintf(Printf_Buf, "Z MIN Endstop:OFF");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}


/*Z MAX*/	
	if(get_endstop_state( Z_AXIS,MAXENDSTOP))
		{
			sprintf(Printf_Buf, "Z MAX Endstop:ON");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
	else
		{
			sprintf(Printf_Buf, "Z MAX Endstop:OFF");my_printf(Printf_Buf);
			sprintf(Printf_Buf, "\r\n");my_printf(Printf_Buf);
		}
		   
        return;
        
     case 140:
  
      if(Command_is('S',&ch_point))//set bed temperature
      {
       
         temp_temperature = Command_value_float(ch_point);
       
        Setting.targe_temperature[BED] = temp_temperature;
        Setting.hotend_enable[BED] = ENABLE;
      }
      else
      {
        Setting.targe_temperature[BED] = 0.0;
        Setting.hotend_enable[BED] = DISABLE;
      }
      return;             
      
   case 190:
        
      if(Command_is('S',&ch_point))
      {
             temp_temperature = Command_value_float(ch_point);           
        }
        Setting.targe_temperature[BED] = temp_temperature;
        Setting.hotend_enable[BED] = ENABLE;
        
        Temperature_Sampling_Timer_Config();
        Temperature_Sampling_NVIC_Configuration();
        Temperature_Measure_Enable();
        Temperature_Control_Enable();

		temp_temperature = Current_Temperature[BED];
        
        Add_Message(TEMPERATURE_INFO);
        while((Current_Temperature[BED] < Setting.targe_temperature[BED])&&(Setting.hotend_enable[BED]))
        {
          			delay_ms(200);   
                          if(fabs(Current_Temperature[BED] - temp_temperature) >= 1)
                		  {          
                		  	  temp_temperature = Current_Temperature[BED];
                	          
                	          sprintf(Printf_Buf, " B:%.1f T0:%.1f T1:%.1f T2:%.1f FC:%d FH:%d R:%d\n",Current_Temperature[BED],Current_Temperature[NOZZLE0],Current_Temperature[NOZZLE1],\
                	          Current_Temperature[NOZZLE2],(u16)(system_infor.fan_controler_speed * 100 / 255),(u16)(system_infor.fan_hotend_speed * 100 / 255),system_infor.feed_tare);
                	          my_printf(Printf_Buf);
                	          Add_Message(TEMPERATURE_INFO);
                          }
			
                                      //UARST_REDIRECT=UARST1_REDIRECT;
          if(system_infor.stop_flag)
          {
          	return;

          }
          if(Current_Temperature[BED] >= Setting.targe_temperature[BED]-3)
          {
                 break;
          }
       
      }
      Add_Message(TEMPERATURE_INFO);
      return;
      
    case 220:

      if(Command_is('S',&ch_point))
      {
        system_infor.feed_tare=(int)Command_value_long(ch_point);
        if(system_infor.feed_tare > 200)
        {
          system_infor.feed_tare = 200;             
        }
        else if(system_infor.feed_tare < 10)
        {
          system_infor.feed_tare = 10;
        }
	
      }
      break;
    case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
	 {
        int servo_index = -1;
        u8 servo_position = 0;
		
        if (Command_is('P',&ch_point))
          servo_index = Command_value_long(ch_point);
        if (Command_is('S',&ch_point)) 
	{
		servo_position = Command_value_long(ch_point);
		  
		if ((servo_index >= 0) && (servo_index < NUM_SERVOS)) 
			BLTouch_StateSet(servo_index,servo_position);
      		else 
	  	{
			sprintf(Printf_Buf, "Servo %d out of range\r\n",servo_index);
			my_printf(Printf_Buf);
          	}
	}
	else if (servo_index >= 0) {
	   	sprintf(Printf_Buf, "Servo %d: Err\r\n",servo_index);
		my_printf(Printf_Buf);
        }
      }
      break;
    case 300:
      if(Command_is('S',&ch_point)) system_infor.Beep_period = 10000/(u16)Command_value_long(ch_point); 
      if(Command_is('P',&ch_point)) system_infor.Beep_duration = (int)(Command_value_long(ch_point)*10);
      break;
      
    case 301:  //Set PID parameters,Hot hotend only;
      Set_hotend_pid();
      break;
      
    case 304:  //Set PID parameters,Bed only;
      Set_bed_pid();
      break;
      
    case 500: //M500: Store parameters in EEPROM or Flash
      Store_Memory(USER_SETTINGS);
      break;
      
    case 502: //Revert to the default "factory settings."
	Set_Beep(1000,127);
	
	//引起所有的数据初始化
	//Clear_WIFI_Info();
       Restore_Defaults(USER_SETTINGS);
	NVIC_SystemReset();
      break;      

    case 851:
    {
      float value;
      if (Command_is('Z',&ch_point))
      {
        value = Command_value_float(ch_point);
        if ((Z_PROBE_OFFSET_RANGE_MIN <= value) && (value <= Z_PROBE_OFFSET_RANGE_MAX))
        {
          Setting.zprobe_zoffset = -value; 
	   sprintf(Printf_Buf, "Z Offset: -%f\r\n",Setting.zprobe_zoffset);
	   my_printf(Printf_Buf);
        }
        else
        {
          sprintf(Printf_Buf, "Z Offset  z_min: %f z_max: %f\r\n",Z_PROBE_OFFSET_RANGE_MIN,Z_PROBE_OFFSET_RANGE_MAX);
	   my_printf(Printf_Buf);
        }
      }
      else
      {
	   sprintf(Printf_Buf, "Z Offset: -%f\r\n",Setting.zprobe_zoffset);
	   my_printf(Printf_Buf);
      }
      break;
    }
	case 2000://set sn
		
		//char *ret = strchr(Command_Buffer, '\n');
		//if(ret>0)
		{
			memcpy(Setting.SN,Command_Buffer+6,16);
			Store_SerialNumber();
			sprintf(Printf_Buf, "SN_set_ok\n");my_printf(Printf_Buf);
		}
		break;
      	 //LCD_2004_Show_Wait_Temperature();
	
	case 2002:
		char bp[512];
		memset(bp, 0, 512);
		sprintf(bp,"min_position[X_AXIS]:%.2f;min_position[Y_AXIS]:%.2f;min_position[Z_AXIS]:%.2f;max_position[X_AXIS]:%.2f;max_position[Y_AXIS]:%.2f;max_position[Z_AXIS]:%.2f;",\
					  Setting.min_position[X_AXIS],\
					  Setting.min_position[Y_AXIS],\
					  Setting.min_position[Z_AXIS],\
					  Setting.max_position[X_AXIS],\
					  Setting.max_position[Y_AXIS],\
					  Setting.max_position[Z_AXIS]);
		sprintf(Printf_Buf, "%s",bp);my_printf(Printf_Buf);
		memset(bp, 0, 512);
		sprintf(bp,"steps_per_mm[X_AXIS]:%.6f;steps_per_mm[Y_AXIS]:%.6f;steps_per_mm[Z_AXIS]:%.6f;steps_per_mm[E_AXIS]:%.6f;",\
					  Setting.steps_per_mm[X_AXIS],\
					  Setting.steps_per_mm[Y_AXIS],\
					  Setting.steps_per_mm[Z_AXIS],\
					  Setting.steps_per_mm[E_AXIS]);		
		sprintf(Printf_Buf, "%s",bp);my_printf(Printf_Buf);  
		memset(bp, 0, 512);
		sprintf(bp,"max_feedrate[X_AXIS]:%d;max_feedrate[Y_AXIS]:%d;max_feedrate[Z_AXIS]:%d;max_feedrate[E_AXIS]:%d;",\
					  Setting.max_feedrate[X_AXIS],\
					  Setting.max_feedrate[Y_AXIS],\
					  Setting.max_feedrate[Z_AXIS],\
					  Setting.max_feedrate[E_AXIS]);
		sprintf(Printf_Buf, "%s",bp);my_printf(Printf_Buf);
		memset(bp, 0, 512);
		sprintf(bp,"home_speed[X_AXIS]:%d;home_speed[Y_AXIS]:%d;home_speed[Z_AXIS]:%d;home_position[X_AXIS]:%d;home_position[Y_AXIS]:%d;home_position[Z_AXIS]:%d;",\
					  Setting.home_speed[X_AXIS],\
					  Setting.home_speed[Y_AXIS],\
					  Setting.home_speed[Z_AXIS],\
					  Setting.home_position[X_AXIS],\
					  Setting.home_position[Y_AXIS],\
					  Setting.home_position[Z_AXIS]);
		sprintf(Printf_Buf, "%s",bp);my_printf(Printf_Buf);
		memset(bp, 0, 512);
		sprintf(bp,"motor_direction[X_AXIS]:%d;motor_direction[Y_AXIS]:%d;motor_direction[Z_AXIS]:%d;motor_direction[E_AXIS]:%d;motor_direction[E1_AXIS]:%d;motor_direction[E2_AXIS]:%d;",\
					  Setting.motor_direction[X_AXIS],\
					  Setting.motor_direction[Y_AXIS],\
					  Setting.motor_direction[Z_AXIS],\
					  Setting.motor_direction[E_AXIS],\
					  Setting.motor_direction[E1_AXIS],\
					  Setting.motor_direction[E2_AXIS]);
		sprintf(Printf_Buf, "%s",bp);my_printf(Printf_Buf);
		memset(bp, 0, 512);
		sprintf(bp,"SN:%s;",\
					  Setting.SN);my_printf(Printf_Buf);
		sprintf(bp,"HV:%s%s;",\
					  Setting.HV,WF_version);my_printf(Printf_Buf);
		sprintf(Printf_Buf, "%s\r\n",bp);my_printf(Printf_Buf);
		break;	 

	case 2003:
		for(u8 i=0;i<NUM_AXIS-1;i++)
  		{
    		if(Command_is(Coordinate_Axias[i],&ch_point))
    		{       	
       			Setting.max_position[i] = Command_value_float(ch_point); 
			}
		}
		Store_Memory(FACTORY_SETTINGS);
		sprintf(Printf_Buf, "max_position_set_ok\n");my_printf(Printf_Buf);
		break;

	case 2004:
		for(u8 i=0;i<NUM_AXIS;i++)
  		{
    		if(Command_is(Coordinate_Axias[i],&ch_point))
    		{       	
       			Setting.steps_per_mm[i] = Command_value_float(ch_point); 
			}
		}
		Store_Memory(FACTORY_SETTINGS);
		sprintf(Printf_Buf, "steps_per_mm_set_ok\n");my_printf(Printf_Buf);
		break;
		
	case 2005:
		for(u8 i=0;i<6;i++)
  		{
                  if(CommandStr_is(DIR_Axias[i],&ch_point))
                  {       	
                    Setting.motor_direction[i] = Command_value_long(ch_point); 
                  }
		}
		Store_Memory(FACTORY_SETTINGS);
		sprintf(Printf_Buf, "motor_direction_set_ok\n");my_printf(Printf_Buf);
		break;
		
	case 2006:
		for(u8 i=0;i<NUM_AXIS;i++)
  		{
    		if(Command_is(Coordinate_Axias[i],&ch_point))
    		{       	
       			Setting.max_feedrate[i] = Command_value_long(ch_point); 
			}
		}
		Store_Memory(FACTORY_SETTINGS);
		sprintf(Printf_Buf, "max_feedrate_set_ok\n");my_printf(Printf_Buf);
		Store_Memory(USER_SETTINGS);
		NVIC_SystemReset();
		break;
		
	case 2007:
		for(u8 i=0;i<NUM_AXIS-1;i++)
  		{
    		if(Command_is(Coordinate_Axias[i],&ch_point))
    		{       	
       			Setting.home_speed[i] = Command_value_long(ch_point); 
			}
		}
		Store_Memory(FACTORY_SETTINGS);
		sprintf(Printf_Buf, "home_speed_set_ok\n");my_printf(Printf_Buf);
		break;	

	case 2009://set hardware_version
		{
			memcpy(Setting.HV,Command_Buffer+6,6);
			Store_Hardware_Version();
			sprintf(Printf_Buf, "HV_set_ok\n");my_printf(Printf_Buf);
		}
	break;

	
	case 2100:
//		M2100_CMD();
		break;
        case 2101:    //发送打印机状态
        	  if(strstr(Command_Buffer, "LCD")!=NULL)
	   	 {
	   	 	memset(&auto_upload_datas,3,sizeof(auto_upload_datas));
	   	 }
		 else
                Send_Printer_Status();
		break;
        case 2102://获取wifi信号强度
                  if(Command_is('S',&ch_point))
                  {
                      Wifi_Work_Message.WEBSERVER_STATUS=Command_value_long(ch_point);
                  }
		break;
        case 2103:  //终止打印
                        system_infor.stop_flag =1;
			system_infor.sd_print_status = SD_PRINT_FINISH;
			printf("SD_PRINT_FINISH 5\r\n");
                     system_infor.sd_print_flag = ENABLE;
			system_infor.system_status = IDLE;
			memset(&auto_upload_datas,0,sizeof(auto_upload_datas));
			Add_Message(TEMPERATURE_INFO);
			printf("Final Print!\r\n");
                break;
        case 2104://网络断开
                    Open_wifi(0);
                    Open_wifi(1);
                    printf("wifi duankai\r\n");
		break;
         case 2105: //挤出头出料和退料
           if(system_infor.system_status == STANDBY)
            return;
               if(Command_is('S',&ch_point))
                {
                	 	 Display_Update_Timer_Config();
                      Filament_Change(Command_value_long(ch_point));
                }
               break;
        case 2106: // 耗材检测
               if(Command_is('S',&ch_point))
                {
                       Filament_Detector(Command_value_long(ch_point));
                }
               break;
        case 2107: //调平
          if(system_infor.system_status == STANDBY)
            return;
               if(Command_is('S',&ch_point))
                {
                    switch(Command_value_long(ch_point)) 
                    {
                        case 0:
                          
                             tempZ = Input_Leveling_Page();
                            //sprintf(Printf_Buf, "M2107 Z:%3.2f\n",tempZ);
                            Set_Motor_Flag(1);

                            Current_Position[Z_AXIS]=tempZ;
				  auto_upload_datas.current_z=99.9;
                             Add_Message(COORDINATE_XYZ);
                             
				   sprintf(Printf_Buf, "M2107 Z:%3.2f\n",tempZ);
                              my_printf(Printf_Buf);
                              system_infor.Motor_Lock =1;
                              
                            break;
                        case 1:
                        case 2:
                        case 3:
                        case 4:
                        case 5:
                          {
                           u8 rett= Leveling_Page_ChoosePi(Command_value_long(ch_point));
                            Add_Message(COORDINATE_XYZ);
				 if(rett==0)
                              sprintf(Printf_Buf, "M2107 ok\n");
                           else
                            sprintf(Printf_Buf, "M2107 fail\n");
                            my_printf(Printf_Buf);
                          }
                            break;
                        case 7:
                          
                            tempZ=Leveling_Page_Zadjust(3,5);
                           
                            
			        Current_Position[Z_AXIS]=tempZ;	
			       
                            Add_Message(COORDINATE_XYZ);
				  sprintf(Printf_Buf, "M2107 Z:%.2f\n",tempZ);
                            my_printf(Printf_Buf);
                            printf("%s\r\n",Printf_Buf);
                            break;
                          case 9:
                      
                            tempZ=Leveling_Page_Zadjust(3,4);
			        Current_Position[Z_AXIS]=tempZ;	
			         
                           Add_Message(COORDINATE_XYZ);
                            break;
                        case 6:
                            // Leveling_Page_Zadjust(0);
                         // sprintf(Printf_Buf, "M2107 Z:%3.2f\n",Leveling_Page_Zadjust(0));
                         tempZ=Leveling_Page_Zadjust(2,5);
                           Current_Position[Z_AXIS]=tempZ;
                            
                            Add_Message(COORDINATE_XYZ);
							
				sprintf(Printf_Buf, "M2107 Z:%.2f\n",tempZ);
                            my_printf(Printf_Buf);
                            printf("%s\r\n",Printf_Buf);
                            break;
                        case 10:
                            // Leveling_Page_Zadjust(0);
                         // sprintf(Printf_Buf, "M2107 Z:%3.2f\n",Leveling_Page_Zadjust(0));
                           tempZ=Leveling_Page_Zadjust(2,4);
                           Current_Position[Z_AXIS]=tempZ;
                           
                            Add_Message(COORDINATE_XYZ);
                            printf("%s\r\n",Printf_Buf);
                            break;
                        case 8:
                            Leveling_Page_SaveZ();
                            system_infor.Motor_Lock =0;
                            sprintf(Printf_Buf, "M2107 save success\n" );
                            USART3_printf(Printf_Buf);
				sprintf(Printf_Buf, "M2107 save success\n" );
                            my_printf(Printf_Buf);
                            break;
                      default: 
		            break;

                    }
                }    
		break;
        case 2108: // 设置解锁时间澹澹?)(范围大于100s，小于一百秒不进入解锁)
           if(Command_is('S',&ch_point))
           {
             u16 unlock_time=(u16)Command_value_long(ch_point);
             if(unlock_time<60)
                unlock_time=60;
             Motor_Unlock_Write_Timer(unlock_time);
             unlock_time=0;
             unlock_time=Motor_Unlock_Read_Timer();
              sprintf(Printf_Buf, "M2108 S%d\n",unlock_time);
                            my_printf(Printf_Buf);
              
           }

            
              break;
        case 2111: //电机控制
		memset(Printf_Buf,0,sizeof(Printf_Buf));
               if(Command_is('S',&ch_point))
                {
                    switch(Command_value_long(ch_point)) 
                    {
                        case 0:	//电机使能
					command_process("M17\r\n");
	                           command_process("G28\r\n");	
                            break;
                        case 1://电机失能
                        		while((Autohome.flag[0]>0) || (Autohome.flag[1]>0) || (Autohome.flag[2]>0));
					command_process("M18\r\n");
					break;
                        case 2://X-,X+移动X轴
                                //	ch_point = strchr(Command_Buffer, 'X');
                        		Command_is('X',&ch_point);
					sprintf(Printf_Buf, "G1 X%3.2f F4800\r\n",Command_value_float(ch_point));
					command_process("G91\r\n");
	                           command_process(Printf_Buf);	
					command_process("G90\r\n");
                            break;	
                        case 3://Y-,Y+移动Y轴
                        		Command_is('Y',&ch_point);
					sprintf(Printf_Buf, "G1 Y%3.2f F4800\r\n",Command_value_float(ch_point));
					command_process("G91\r\n");
	                           command_process(Printf_Buf);	
					command_process("G90\r\n");
                            break;	
                        case 4://Z-,Z+移动Z轴
                          {
                        		Command_is('Z',&ch_point);

                        		float tempp = Command_value_float(ch_point);
                        		//if(tempp<0)
					//	sprintf(Printf_Buf, "G1 Z%3.2f F4800\r\n",abs(tempp));
					//else
						sprintf(Printf_Buf, "G1 Z%3.2f F4800\r\n",(0.0-tempp));
					command_process("G91\r\n");
	                           command_process(Printf_Buf);	
					command_process("G90\r\n");
                          }
                            break;	
							
			default: 
				break;
			}
			memset(Printf_Buf,0,sizeof(Printf_Buf));
                }    
            
              break;
	 case 2223: // wifi set
               if(Command_is('P',&ch_point))
                {
                	switch(Command_value_long(ch_point))
                	{
				case 0:    //open wifi
					 if(Command_is('S',&ch_point))
					 {
					 	wifi_temp=Command_value_long(ch_point);
						if(wifi_temp==1)
						{
							Open_wifi(0);
							printf("Clase wifi 2223\r\n");
							
						}
						else if(wifi_temp==0)
						{
                                                      
							Open_wifi(1);
						}
						else
						{

						}
					 }
					break;
				case 1:   //config wifi
					 if(Command_is('S',&ch_point))
					 {
					 	wifi_temp=Command_value_long(ch_point);
						if(wifi_temp==1)
						{
							Open_APP_Set(1);
						}
						else if(wifi_temp==0)
						{
							Open_APP_Set(0);
						}
						else
						{

						}

					 }
					break;
				case 2:   // wifi auto
					 if(Command_is('S',&ch_point))
					 {
					 	wifi_temp=Command_value_long(ch_point);
						if(wifi_temp==1)
						{
							WIFI_Write_AutoConnect_Flag(1);
						}
						else if(wifi_temp==0)
						{
							WIFI_Write_AutoConnect_Flag(0);
						}
						else
						{

						}

					 }
					 break;
				case 3:   //connect
					Open_wifi(1);
					break;
					break;


			}
                }
               break;
	   case 2112:
	   	//Get_WIFI_Information();
	   	if(Wifi_ASK_Factors.WIFI_CONNECT_FLAG!=0)
	   	         Add_Message(WIFI_DETAIL_MESSAGE);
		break;

	case 2201:
		Add_Message(PRINTER_MESSAGE);
		break;
	case 2222:
		 if(Command_is('S',&ch_point))
		{
			u8 motor_f=Command_value_long(ch_point);
		       Set_E0_Motor_Flag(motor_f);
		 }
		break;

	case 2225:
		 if(Command_is('P',&ch_point))
                {
                	switch(Command_value_long(ch_point))
                	{
				case 0:    //set auto_levele
					 if(Command_is('S',&ch_point))
					 {
					 	wifi_temp=Command_value_long(ch_point);
						if(wifi_temp==1)
						{
							    system_infor.Auto_Levele_Flag =1;    //打开自动调平
							    Store_AutoLevele_Flag(1);
						}
						else 
						{
                                                     system_infor.Auto_Levele_Flag=2; //关闭自动调平
                                                     Store_AutoLevele_Flag(2);
						}
						//command_process("G28\r\n");
					 }
					break;
					case 1:   //控制探针
					 if(Command_is('S',&ch_point))
					 {
					 	wifi_temp=Command_value_long(ch_point);
						if(wifi_temp==0)/*探针上升*/
						{
							 BLTouch_StateSet(0, SERVO_Push_pin_Up);//3dtouch状态设置
						}
						else if(wifi_temp==1)/*探针下降*/
						{
							BLTouch_StateSet(0, SERVO_Push_pin_Down);//3dtouch状态设置
						}
						else /*解除探针错误警报*/
						{
							BLTouch_StateSet(0, SERVO_Alarm_Release);//3dtouch状态设置
						}
					 }
					break;
				case 2:   //保存偏差
					 if(Command_is('S',&ch_point))
					 {
					 	float z_offset=Command_value_float(ch_point);
						if(z_offset<10)
						{
							Setting.zprobe_zoffset=z_offset;
							Current_Position[Z_AXIS] = Setting.max_position[Z_AXIS]+z_offset;
							printf("z_offset:%f;Current_Position[Z_AXIS]:%f\r\n",z_offset,Current_Position[Z_AXIS]);
							command_process("M500\r\n");
						}
					 }
					 break;
				case 3:   //z轴向上移动
					 if(Command_is('S',&ch_point))
					 {
					 	wifi_temp=Command_value_long(ch_point);
						if(wifi_temp>=0&&wifi_temp<4)
						{
							tempZ=Leveling_Page_Zadjust(0,wifi_temp);
			       			Current_Position[Z_AXIS]=tempZ;		   
                            			Add_Message(COORDINATE_XYZ);
						}
					 }
					 break;
				case 4:   //z轴向下移动
					 if(Command_is('S',&ch_point))
					 {
					 	wifi_temp=Command_value_long(ch_point);
						if(wifi_temp>=0&&wifi_temp<4)
						{
							tempZ=Leveling_Page_Zadjust(1,wifi_temp);
			       			Current_Position[Z_AXIS]=tempZ;		   
                            			Add_Message(COORDINATE_XYZ);
						}
					 }
					 break;
				case 5:
				   Set_Motor_Flag(1);
				   //command_process("G28\r\n");
				break;
				case 6:
				   new_Z_Max_Position = 0.0;
				   if(Get_Motor_Status()==0&&Auto_Levele_Offset_Flag!=1)
				   {
					command_process("G28\r\n");
				   }
				break;
				case 7:
				     if(Command_is('S',&ch_point))
					 {
					 	Auto_Levele_Offset_Flag=1;
					 	command_process("G28\r\n");
					 }
				break;
				}
			}
		break;
		case 2226:
			if(strstr(Command_Buffer,"S1")!=NULL)
			{
				Store_Filament_Dev_Flag(1);
				system_infor.Filament_Dev_Flag =1;
			}
			else
			{
				Store_Filament_Dev_Flag(0);
				system_infor.Filament_Dev_Flag=0;
			}
		break;

	
	   case 2555:
	   		
			NVIC_SystemReset();
		break;
		case 2556: //判断wifi是否存在
			WIF_EXIST_TEST();
	  break;
	  case 2557: 
	  {
	      u8 num =0,j=0;
	 	memset(Wifi_Server_message,0,sizeof(Wifi_Server_message));
	 	memcpy(Wifi_Server_message,"connect to ",11);
	 	u8 *sp1=strchr(Command_Buffer,'<');
	 	u8 *sp2=strchr(Command_Buffer,'>');
	 	num =sp2-sp1;
	 	j=11;
	 	for(u8 i=7;i<num+6 ;i++)
		{
			sp1++;
			if(*sp1 =='|')
				Wifi_Server_message[j++] =':';
			else
				Wifi_Server_message[j++] =*sp1;
			
		}
		Wifi_Server_message[j++] = ';';
		for(u8 i=0;i<strlen(Setting.SN) ;i++)
		{
			Wifi_Server_message[j++] =Setting.SN[i];
			
		}
		sprintf(&Wifi_Server_message[j], ";www.geeetech.net;\r\n");
	 	Store_WIFI_Memory();
	 	printf("<%d><%s>\r\n",(sp2-sp1),Wifi_Server_message);
		
		}
	 break;
     case 2558:
     {
     	  u8 *sp = strchr(Command_Buffer,'S');
	  if(sp!=NULL)
          {	 
        	if(*(sp+1)>='0' && *(sp+1)<='9')
        	{
			Firmware_version[3] = *(sp+1);
        	}
        	if(*(sp+2)>='0' && *(sp+2)<='9')
        	{
			Firmware_version[4] = *(sp+2);
        	}
        	Send_machine_info();
          }
     }
     break;
    default: 
		break;
    }
  }
  else if(g_code == 'T')
  //else if(Command_is('T',&))
  {
      switch(Command_value_long(ch_point))
      {
          


          case 0: 
            
                  break;
          
          case 1:            
                  
                  break;

	  case 2:            
               
                  break;
          default:  break;      
      }
  }
  else
  {
    //printf("GGG:%s\r\n",Command_Buffer);
  }
  
}

void Set_Up_Data_Flag(void)
{
	Up_Data_Flag =0;
}

