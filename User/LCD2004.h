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



typedef struct
{
  signed int pointer_index;
  u8 decoder_dir;    //旋转方向，0逆时针，1顺时针
  u8 decoder_turn_flag;     //0:未转动,  1:在转动
  float decoder_turn_rate;   //旋转系数
  u8 key_press; 
}decoder_struct;

typedef struct
{
  u8 layer;
  char *path;

}Sd_dir;


void Encoder_GPIO_Config(void);
void Encoder_EXTI_Config(void);
void Enable_Encoder_EXTI(void);


void LCD_2004_Init(void);
void Display_Update_Timer_Config(void);

void page_init(void);
static signed int select_item(void);
static void show_select_item(void);



u8 Recovery_select(void);
u8 Recovery_error(void);


#endif