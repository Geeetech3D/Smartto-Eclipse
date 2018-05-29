#ifndef _LCD2004_H_
#define _LCD2004_H_

#include "stm32f10x.h"
#include "XZK_Configuration.h"




  #define RS_H  GPIO_SetBits(GPIOE,GPIO_Pin_6)       //data register
  #define RS_L  GPIO_ResetBits(GPIOE,GPIO_Pin_6)     //command register

  #define EN_H  GPIO_SetBits(GPIOE,GPIO_Pin_14)        //LCD receive command
  #define EN_L  GPIO_ResetBits(GPIOE,GPIO_Pin_14)      //LCD process command

  #define LCM_D4_L GPIO_ResetBits(GPIOD,GPIO_Pin_8)
  #define LCM_D4_H GPIO_SetBits(GPIOD,GPIO_Pin_8)

  #define LCM_D5_L GPIO_ResetBits(GPIOD,GPIO_Pin_9)
  #define LCM_D5_H GPIO_SetBits(GPIOD,GPIO_Pin_9)

  #define LCM_D6_L GPIO_ResetBits(GPIOD,GPIO_Pin_10)
  #define LCM_D6_H GPIO_SetBits(GPIOD,GPIO_Pin_10)

  #define LCM_D7_L GPIO_ResetBits(GPIOE,GPIO_Pin_15)
  #define LCM_D7_H GPIO_SetBits(GPIOE,GPIO_Pin_15)

  #define EXTI_EC1      ((uint32_t)0x00200)  /*!< External interrupt line 9 */
  #define EXTI_EC2      ((uint32_t)0x00100)  /*!< External interrupt line 8 */
  #define EXTI_PRESS    ((uint32_t)0x02000)  /*!< External interrupt line 13 */

  #define READ_EC1      (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_8))
  #define READ_EC2      (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_9))
  #define READ_PRESS    (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_13))

#define LCD_HIGHT 4
#define LCD_WIDTH 20

#define LCD_W_DELAY 8

#define HEATER_UI       0x00
#define HOTBED_UI       0x01
#define PERCENT_UI      0x02
#define TIME_UI         0x03
#define DEGREE_UI       0x04
#define BACK_UI         0x05
#define ENTER_UI        0x06
#define DOCUMENT_UI     0x07

#define AUTO_HOME 0x01

#define IDLE 0
#define MAINMENU 1
#define PREPARE 2
#define CONTROL 3
#define SDCARD 4  
#define MOVE_AXIS 5
#define TEMPERATURE 6
#define MOTION 7
#define PRINT 8
#define MOVE_10MM 9
#define MOVE_1MM 10
#define MOVE_0_1MM 11
#define N_PID 12
#define B_PID 13
#define PREHEAT_PLA_CONF 14
#define PREHEAT_ABS_CONF 15
#define PRINT_MAINMENU 16
#define TUNE 17
#define PAUSE_PRINT 18
#define STOP_PRINT 19
#define SERIAL_CONNECT 20
#define HOMING 21
#define STANDBY 22
typedef struct     //test
{
  u8 *name;
  u8 first_line;
  u8 min_menu_value;
  u8 max_menu_value;
  u8 refresh_flag;
  u8 show_index;
  u8 select_index;
  char *page[20];
}Page_struct;

typedef struct
{
  signed int pointer_index;
  u8 decoder_dir;    
  u8 decoder_turn_flag;     
  float decoder_turn_rate;   
  u8 key_press; 
}decoder_struct;

typedef struct
{
  u8 layer;
  char *path;

}Sd_dir;

#ifdef BOARD_M301_Pro_S
//void Encoder_GPIO_Config(void);
//void Encoder_EXTI_Config(void);
//void Enable_Encoder_EXTI(void);

void Disable_Encoder_EXTI(void);
void Encoder_IRQ(void);


void LCD_2004_Init(void);
void LCD_Clear(void);
void Show_Char(u8 x,u8 y,char data);
void Show_String_right(u8 x,u8 y,char *str);
void Show_String(u8 x,u8 y,char *str);
void Show_String_len(u8 x,u8 y,char *str,u8 len);
void Show_UI(u8 x,u8 y,char ui_num);
void Show_Int(u8 x,u8 y,char *format,int value);
void Show_Int_right(u8 x,u8 y,char *format,int value);
void Show_Float(u8 x,u8 y,char *format,float value);
void Show_Float_right(u8 x,u8 y,char *format,float value);
//void Display_Update_Timer_Config(void);
void get_sd_path(void);

static s16 get_line_num(char *str[],char *string);
static int sizeof_strbuf(char *str[]);
static void display_ui(void);
static void display_icon_value(u8 y,char *line);
static void value_adj(void);
static float value_turn(float min, float max,float rate,float value);
void page_init(void);
void page_nomal_refresh(void);
void message_refresh(void);
static void dispay_page_info(void);
void page_info_refresh(void);
void show_info_screen(void);
static signed int select_item(void);
static void show_select_item(void);
void decoder_inc(void);
void decoder_dec(void);
void decoder_press(void);
static void feedrate_turn(void);
void display_refresh(void);
static void str_array_cpy(Page_struct *page_stru,char *src[]);
static void page_buf_copy(char *src[]);
static int sizeof_sbuf(void);
static void str_change(char *str,char *pD, char *pS);
void sd_plug_detect(void);
void show_sd_dir(char *path);

static void page_enter(void);
static void page_exit(void);


void page_go_info(void);  //fdwong
u8 Recovery_select(void);
//u8 Recovery_error(void);

#endif

void Display_Update_Timer_Config(void);
u8 Recovery_error(void);
void Enable_Encoder_EXTI(void);
void Encoder_GPIO_Config(void);
void Encoder_EXTI_Config(void);

#endif
