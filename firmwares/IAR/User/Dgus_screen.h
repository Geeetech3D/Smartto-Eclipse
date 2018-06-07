#ifndef _Dgus_screen_H_
#define _Dgus_screen_H_


#define DGUS_TEXT_LEN 64


#define	LOGO_PAGE 			0x0
#define STANDBY_PAGE		0x01
#define	MAIN_PAGE 			0x10
#define	PRINT_PAUSE_PAGE 	0x14
#define	PRINT_KILL_PAGE 	0x15
#define	PAUSED_HINT_PAGE 	0x16
#define	RECOVERING_PAGE 	0x17
#define PAUSE_SAVE_PAGE		0x18
#define	CONTROL_PAGE 		0x20
#define	SET_PAGE			0x30
#define	SET_LED_PAGE		0x31
#define	SET_LEVELING_PAGE	0x32
#define	SET_FILAMENT_PAGE	0x33
#define	SET_BACKLIGHT_PAGE	0x34
#define	SET_SOUND_PAGE		0x35
#define	SET_FAN_PAGE		0x36
#define	SET_ABOUT_PAGE		0x37
#define	SET_RESET_PAGE		0x38
#define	SET_FACTORY_PAGE	0x39
#define	SET_WIFI_PAGE		0x3A
#define	SET_WIFI_SET_PAGE	0x3B
#define	SET_WIFI_CON_PAGE	0x3C
#define	SET_CHANG_PAGE  	0x3D
#define	SET_DETECTOR_PAGE  	0x3E
#define	SET_SETING_FL_PAGE  	0x3F
#define	SD_FILE_PAGE		0x40
#define	INFORMATION_PAGE	0x50
#define	PRINT_RESUME_PAGE	0x60
#define	NO_FIND_FILE_PAGE	0x61
#define	NO_FILAMENT_PAGE	0x70
#define PRINT_HINT_PAGE		0x81

typedef enum
{
	DGUS_STANDBY_MODE,
	DGUS_POWERON_MODE,	
}DgusModeType;

typedef struct 
{
u16 grid_points;
float probe_arbritratry_XY[2];//X=0, Y=1
float HotendToProbe_Z_offset;
}AUTO_LEVELING;

void DGUS_Refresh_Dispaly(void);
void DGUS_Display_Timer_Config(void);



void DGUS_Set_Mode(DgusModeType mode);

void Dgus_erro_out(u8 index, char* text, u8 text_size);

void  DGUS_Set_Page(u8 page);
void Filament_Change(u8 i);
void Leveling_Page_SaveZ(void);
u8 Leveling_Page_ChoosePi(u8 num);
float Leveling_Page_Zadjust(u8 tt);
u8 Get_Motor_Status(void);
void Motor_Disable_IT(void);
u8 get_filament_status(void);
void Set_Motor_Flag(u8 ff);
float  Input_Leveling_Page(void);
float AutoLeveling_Page_Zadjust(u8 tt,u8 spaces );

#ifdef BOARD_M301_Pro_S  
void  USART3_Configuration(void) ;
void USART3_NVIC_Configuration(void);
#endif

#endif
