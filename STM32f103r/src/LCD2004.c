
#include "LCD2004.h"
#include "command.h"
#include "delay.h"
#include "XZK_Rule.h"
#include "fat.h"
#include "setting.h"
#include "string.h"
#include "sd.h"
#include "adc.h"
#include "step_motor.h"
#include "stm32f10x_gpio.h"
#include "stdio.h"
#include "planner.h"
#include "sd_print.h"
#include "variable.h"
#include "mmc_sd.h"
static decoder_struct decoder = {0,0xff,0,1,0};



#ifdef BOARD_M301_Pro_S
u16 fan_speed=0;
extern char SD_Driver_Number[5];
extern u8 SD1_flag;
extern u8 SD2_flag;

extern float Current_Position[4] , Current_Temperature[5];
extern __IO u8 Print_Time_S,Print_Time_M;
extern __IO u16 Print_Time_H; 
extern u8   dir_index ,selected_file_pos  ;
extern u8  serial_connect_flag  ,serial_print_flag  ,serial_stop_flag;//serial_time_show_flag; 
extern char sd_file_name[255][50];
extern char SD_Driver_Number[5];
extern FATFS fats;
float print_file_size;

u8 dir_layer=0,next_layer_flag=0, dir_number = 0;//dir_layer:directory layer number      next_layer_flag=1:enter next directory layer
extern char dir_name[10][50] , *file_type  , SD_Path[512];
extern char Command_Buffer[CMDBUF_SIZE];

extern float print_file_size;
extern u8 System_message;
u8 custom_conf_select = 0;
u8 refresh_time_flag=1;
extern u32 turn_back_count;

extern SystemInfor system_infor;//system_infor.sd_print_flag


const char LCD_UI_Table[]={
                          0x04,0x0a,0x0a,0x0a,0x0a,0x11,0x11,0x0e,//hotheat               
                          0x00,0x1f,0x15,0x11,0x15,0x1f,0x00,0x00,//hotbed
                          0x1c,0x10,0x18,0x17,0x05,0x06,0x05,0x00,//speed percent
                          0x00,0x0e,0x13,0x15,0x11,0x0e,0x00,0x00,//time
                          0x0c,0x12,0x12,0x0c,0x00,0x00,0x00,0x00,//degree
                          0x04,0x0e,0x1f,0x04,0x1c,0x00,0x00,0x00,//back
                          0x00,0x04,0x02,0x1f,0x02,0x04,0x00,0x00,//enter
                          0x00,0x1c,0x1f,0x11,0x11,0x1f,0x00,0x00//document
                         };

void LCD_GPIO_Config(void)
{
      GPIO_InitTypeDef GPIO_InitStructure;
    
      RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOE, ENABLE);
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;				 
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          //DATA
      GPIO_Init(GPIOD, &GPIO_InitStructure);      
      GPIO_ResetBits(GPIOD, GPIO_Pin_8 | GPIO_Pin_9| GPIO_Pin_10);
      
      GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_14 | GPIO_Pin_15;				 
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //
      GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;          //
      GPIO_Init(GPIOE, &GPIO_InitStructure);
      GPIO_SetBits(GPIOE,GPIO_Pin_6 | GPIO_Pin_14 | GPIO_Pin_15);      
}







void Disable_Encoder_EXTI(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;   
    
    EXTI_InitStructure.EXTI_Line = EXTI_EC1;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);      
       
    EXTI_InitStructure.EXTI_Line = EXTI_PRESS;
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);   
    
}




//data out
static void ValueOutput(char data)
{
    switch((data&0x08)>>3)
    {
        case 0:LCM_D7_L;break;
        case 1:LCM_D7_H;break;
        default:break;
    }
    switch((data&0x04)>>2)
    {
        case 0:LCM_D6_L;break;
        case 1:LCM_D6_H;break;
        default:break;
    }
    switch((data&0x02)>>1)
    {
        case 0:LCM_D5_L;break;
        case 1:LCM_D5_H;break;
        default:break;
    }
    switch(data&0x01)
    {
        case 0:LCM_D4_L;break;
        case 1:LCM_D4_H;break;
        default:break;
    }
}

//write command
void WriteCommand(char command)
{
       u8 command_buf; 
       
       command_buf=command;
       RS_L;
       delay_us(LCD_W_DELAY);
       EN_L;
       delay_us(LCD_W_DELAY);
       
       ValueOutput(command_buf>>4);//write high 4 bit
       EN_H;
       delay_us(LCD_W_DELAY);    
       EN_L;
       delay_us(LCD_W_DELAY);
       
       ValueOutput(command_buf);//write low 4 bit
       EN_H;
       delay_us(LCD_W_DELAY);       
       EN_L;
       delay_us(LCD_W_DELAY);
}


//write data funtion
void WriteData(char data)
{
       u8 data_buf;
       
       data_buf=data;
       RS_H;
       delay_us(LCD_W_DELAY);
       EN_L;
       delay_us(LCD_W_DELAY);
       
       ValueOutput(data_buf>>4);//write high 4 bit
       EN_H;
       delay_us(LCD_W_DELAY);
       EN_L;
       delay_us(LCD_W_DELAY);
       
       ValueOutput(data_buf);//write low 4 bit
       EN_H;
       delay_us(LCD_W_DELAY);
       EN_L;
       delay_us(LCD_W_DELAY); 
}   


//set display position
void Set_XY(u8 x,u8 y)
{
      char x_index;
      x=x%4;       //Row limit  0~3
      y=y%20;      //Column limit  0-19
      switch(x)
      {
          case 0:x_index=0x80;break;//first row
          case 1:x_index=0xc0;break;//second row
          case 2:x_index=0x94;break;//3
          case 3:x_index=0xd4;break;//4
      }
        
      WriteCommand(x_index+y);//set column 

}


/******************************************display function***************************************************/
//display one char
void Show_Char(u8 x,u8 y,char data)
{
        Set_XY(x,y);
        WriteData(data); 
}

//display string
void Show_String(u8 x,u8 y,char *str)
{
        Set_XY(x,y);
        u8 i = y;
        while(*str)
        {

                if(i==LCD_WIDTH) 
                {
                  x++;
                  Set_XY(x,y);
                  i = 0;
                }
                else 
                {
                  WriteData(*str);;
                  str++;
                  i++;
                }
        }
}

void Show_String_right(u8 x,u8 y,char *str)
{
        Set_XY(x,y-strlen(str));
        while(*str)
        {
                WriteData(*str);;
                str++;
        }
}

void Show_String_len(u8 x,u8 y,char *str,u8 len)
{
        u8 i = 0;
        Set_XY(x,y);
        while(*str)
        {
                WriteData(*str);;
                str++;
                i++;
                if(i>=len)break;
        }
}

//display Integer
void Show_Int(u8 x,u8 y,char *format,int value)
{
      char int_data[5];
      sprintf(int_data,format,value);
      Show_String(x,y,int_data);
}

void Show_Int_right(u8 x,u8 y, char *format,int value)
{
      char int_data[5];
      sprintf(int_data,format,value);
      Show_String(x,y-strlen(int_data),int_data);
}

//Display single-precision floating-point numbers
void Show_Float(u8 x,u8 y,char *format,float value)
{
      char float_data[6];
      sprintf(float_data,format,value);
      Show_String(x,y,float_data);
}

void Show_Float_right(u8 x,u8 y,char *format,float value)
{
      char float_data[6];
      sprintf(float_data,format,value);    
      Show_String(x,y-strlen(float_data),float_data);
}

//Display icon
void Show_UI(u8 x,u8 y,char ui_num)
{
      Show_Char(x,y,ui_num);
}


                
                
/*****************************************************************************************************/

void UI_Define(void)
{
      u8 i=0;
      WriteCommand(0x40);
      for(i=0;i<64;i++)
      {
        WriteData(LCD_UI_Table[i]);
      }
}

//clear lcd
void LCD_Clear(void)               
{ 
    WriteCommand(0x01);
    delay_ms(2);
}
 

//init lcd
void LCD_2004_Init(void)
{
    
  LCD_GPIO_Config();
  WriteCommand(0x28);
  delay_ms(10);
  WriteCommand(0x06);
  delay_ms(10);
  WriteCommand(0x0C);
  delay_ms(10);
  WriteCommand(0x01); 
  delay_ms(10);
  UI_Define();   
}

extern u8 dir_index;
extern DIR dir;
extern FILINFO fno;
extern char *file_name;
extern char *file_type;
  
extern char lfn[256];
extern float file_size[128];
extern char *sd_file_names[128];

extern char *machine_name;
extern mixer_t mixer;

extern u8 color_change_flag;
extern autohome_st Autohome;

char page_buf[129][50] = {0};



char message[CMDBUF_SIZE] = "GTM32 3D Controller";
char page_stack[20][50] = {"Info screen"};  //Cache page name
u8 page_stack_index = 0;
u8 Page_refresh_flag = 0;
Page_struct Page_sbuf = {0,0,0,0};   //20160627


const char *page_info[] = {
  "",
  "    WELCOME!!!",
  "www.geeetech.com",
  "",
};

char *page_main[] = {
  "Info screen",
  "Prepare",
  "Control",
#if (defined EXTRUDER_2IN1) || (defined EXTRUDER_3IN1)
  "Mixer",
#endif 
  "SD Card",
  "",   
};

const char *page_prepare[] = {
  "Main",
  "Disable Steppers",
  "Auto Home",
  "Preheat PLA",
  "Preheat ABS",
  "Cooldown",
  "Move Axis",
  "",  
};

const char *page_turn[] = {
  "Main",
  "Nozzle:",
#if(HOTHEAD_1)
  "Nozzle 1:",
#endif
  "Bed:",
  "Fan speed:",
  "",  
};

const char *page_control[] = {
  "Main",
  "Temperature", 
  "Printer Settings",
#ifdef DELTA    //Settings for Delta
  "Delta",
#endif
  "Motion",
  "Store Memory",
  "Restore Defaults",
  "Version",
  "",    
};

const char *page_temp[] = {
  "Control",
  "Nozzle:",
#if(HOTHEAD_1)
  "Nozzle 1:",
#endif
  "Bed:",
  "Fan speed:",
  "Autotemp:",
  "   Min:",
  "   Max:",
  "   Fact:",
  "N-PID",
  "B-PID",
  "Preheat PLA Conf",
  "Preheat ABS Conf",
  "",  
};

const char *page_tnozzle[] = {
  " ",
  "Nozzle:",
  "",  
};

#if(HOTHEAD_1)
const char *page_tnozzle1[] = {
  " ",
  "Nozzle 1:",
  "",  
};
#endif

const char *page_tbed[] = {
  " ",
  "Bed:",
  "",  
};

const char *page_fanspeed[] = {
  " ",
  "Fan speed:",
  "",
};

const char *page_mixer[] = {
  "Prepare",
#if (defined EXTRUDER_2IN1) || (defined EXTRUDER_3IN1)
  "Filament 0:",
  "Filament 1:",
#endif
#ifdef EXTRUDER_3IN1
  "Filament 2:",
#endif
  "OFP",
  "Templates",
  "Custom",
  "",
};
  
const char *page_template[] = { 
  "Mixer",
  "Template 1",
  "Template 2",
  "Template 3",
  "Template 4",
  "Template 5",
  "Template 6",
  "Template 7",
  "Template 8", 
  "Custom 1",
  "Custom 2",
  "Custom 3",
  "Custom 4",
  "Custom 5",
  "Custom 6",
  "",
};

const char *page_custom[] = { 
  "Mixer",
  "Custom 1 Conf",
  "Custom 2 Conf",
  "Custom 3 Conf",
  "Custom 4 Conf",
  "Custom 5 Conf",
  "Custom 6 Conf",  
  "",
};

const char *page_mix_filament0[] = {
  " ",
  "Filament 0:",
  "",
};

const char *page_mix_filament1[] = {
  " ",
  "Filament 1:",
  "",
};

const char *page_mix_filament2[] = {
  " ",
  "Filament 2:",
  "",
};

const char *page_ofp_conf[] = {
  "Mixer",
  "OFP Max:",
  "OFP Min:",
  "Store Memory",
  "",
};

const char *page_ofp_min[] = {
  " ",
  "OFP Min:",
  "",
};

const char *page_custom_conf[] = {
  "Custom",
  "Start Percent:",
  "End Percent:",
  "Start Height:",
  "End Height:",
  "Store Memory",
  "",
};

const char *page_start_percent[] = {
  " ",
  "Start Percent:",
  "",
};
const char *page_end_percent[] = {
  " ",
  "End Percent:",
  "",
};
const char *page_start_height[] = {
  " ",
  "Start Height:",
  "",
};
const char *page_end_height[] = {
  " ",
  "End Height:",
  "",
};

const char *page_ofp_max[] = {
  " ",
  "OFP Max:",
  "",
};


const char *page_move[] = {
  "Prepare",
  "Move 10mm",
  "Move 1mm",
  "Move 0.1mm",
  "",
};

const char *page_move_10mm[] = {
  "Move Axis",
  "Move X",
  "Move Y" ,
  "Move Z",
  "",  
};

const char *page_move_1mm[] = {
  "Move Axis",
  "Move X",
  "Move Y", 
  "Move Z",
  #ifdef EXTRUDER_0
  "Extruder 0",
  #endif
  #ifdef EXTRUDER_1
  "Extruder 1",
  #endif
  #ifdef EXTRUDER_2
  "Extruder 2",
  #endif
  "",
};

const char *page_move_01mm[] = {
  "Move Axis",
  "Move X",
  "Move Y", 
  "Move Z",
  #ifdef EXTRUDER_0
  "Extruder 0",
  #endif
  #ifdef EXTRUDER_1
  "Extruder 1",
  #endif
  #ifdef EXTRUDER_2
  "Extruder 2",
  #endif
  "",
};

const char *page_move_x[] = {
  " ",
  "X:",
  "",
};

const char *page_move_y[] = {
  " ",
  "Y:",
  "",
};

const char *page_move_z[] = {
  " ",
  "Z:",
  "",
};

const char *page_move_extruder0[] = {
  " ",
  "Extruder 0:",
  "",
};

const char *page_move_extruder1[] = {
  " ",
  "Extruder 1:",
  "",
};

const char *page_move_extruder2[] = {
  " ",
  "Extruder 2:",
  "",
};

const char *page_npid[] = {
  "Temperature",
  "N-PID-P:",
  "N-PID-I:",
  "N-PID-D:",
  "",
};

const char *page_npid_p[] = {
  " ",
  "N-PID-P:",
  "",
};

const char *page_npid_i[] = {
  " ",
  "N-PID-I:",
  "",
};

const char *page_npid_d[] = {
  "  ",
  "N-PID-D:",
  "",
};

const char *page_bpid[] = {
  "Temperature",
  "B-PID-P:",
  "B-PID-I:",
  "B-PID-D:",
  "",
};

const char *page_bpid_p[] = {
  "  ",
  "B-PID-P:",
  "",
};

const char *page_bpid_i[] = {
  "B- ",
  "B-PID-I:",
  "",
};

const char *page_bpid_d[] = {
  " ",
  "B-PID-D:",
  "",
};

const char *page_pla_conf[] = {
  "Temperature",
  "Fan speed:",
  "Nozzle:",
  "Bed:",
  "",
};

const char *page_abs_conf[] = {
  "Temperature",
   "Fan speed:",
  "Nozzle:",
  "Bed:",
  "",
};

#ifdef DELTA
const char *page_delta[] = {
  "Control",
  "Segments/sec:",
  "Diagonal Rod:",   // Center-to-center distance of the holes in the diagonal push rods.
  "Smooth offs:",  // Horizontal offset from middle of printer to smooth rod center.
  "Effector offs:",  // Horizontal offset of the universal joints on the end effector.
  "Carriage offs:",  // Horizontal offset of the universal joints on the carriages.
  "Delta Radius:",  // Horizontal distance bridged by diagonal push rods when effector is centered.
  "Print Radius:",  // Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
  "",
};

const char *page_delta_segments_per_sec[] = {
  " ",
  "Segments/sec:",
  "",
};

const char *page_delta_diagonal_rod[] = {
  " ",
  "Diagonal Rod:",
  "",
};

const char *page_delta_smooth_offset[] = {
  " ",
  "Smooth offs:",
  "",
};

const char *page_delta_effector_offset[] = {
  " ",
  "Effector offs:",
  "",
};

const char *page_delta_carriage_offset[] = {
  " ",
  "Carriage offs:",
  "",
};

const char *page_delta_radius[] = {
  " ",
  "Delta Radius:",
  "",
};

const char *page_delta_printable_radius[] = {
  " ",
  "Print Radius:",
  "",
};
#endif

const char *page_motion[] = {
  "Control",
  "Z offset:",
  "Accel:",
  "Vx-jerk:",
  "Vy-jerk:",
  "Vz-jerk:",
  "Ve-jerk:",
  "Vmax X:",
  "Vmax Y:",
  "Vmax Z:",
  "Vmax E:",
  "Vmin:",
  "VTrav min:",
  "Amax X:",
  "Amax Y:",
  "Amax Z:",
  "Amax E:",
  "A-retract:",
  "Xsteps/mm:",
  "Ysteps/mm:",
  "Zsteps/mm:",
  "Esteps/mm:",
  "Home speed X:",
  "Home speed Y:",
  "Home speed Z:",
  "",
};

const char *page_zoffset[] = {
  " ",
  "Z offset:",
  "",
};

const char *page_accel[] = {
  " ",
  "Accel:",
  "",
};

const char *page_vxjerk[] = {
  " ",
  "Vx-jerk:",
  "",
};

const char *page_vyjerk[] = {
  " ",
  "Vy-jerk:",
  "",
};

const char *page_vzjerk[] = {
  " ",
  "Vz-jerk:",
  "",
};

const char *page_vejerk[] = {
  " ",
  "Ve-jerk:",
  "",
};

const char *page_vmaxx[] = {
  " ",
  "Vmax X:",
  "",
};

const char *page_vmaxy[] = {
  " ",
  "Vmax Y:",
  "",
};

const char *page_vmaxz[] = {
  " ",
  "Vmax Z:",
  "",
};

const char *page_vmaxe[] = {
  " ",
  "Vmax E:",
  "",
};

const char *page_vmin[] = {
  "",
  "Vmin",
  "",
};

const char *page_vtraval_min[] = {
  " ",
  "VTrav min:",
  "",
};

const char *page_amaxx[] = {
  " ",
  "Amax X:",
  "",
};

const char *page_amaxy[] = {
  " ",
  "Amax Y:",
  "",
};

const char *page_amaxz[] = {
  " ",
  "Amax Z:",
  "",
};

const char *page_amaxe[] = {
  " ",
  "Amax E:",
  "",
};

const char *page_aretract[] = {
  " ",
  "A-retract:",
  "",
};

const char *page_xsteps_mm[] = {
  " ",
  "Xsteps/mm:",
  "",
};

const char *page_ysteps_mm[] = {
  " ",
  "Ysteps/mm:",
  "",
};

const char *page_zsteps_mm[] = {
  " ",
  "Zsteps/mm:",
  "",
};

const char *page_esteps_mm[] = {
  " ",
  "Esteps/mm:",
  "",
};

const char *page_homespeed_x[] = {
  " ",
  "Home speed X:",
  "",
};

const char *page_homespeed_y[] = {
  " ",
  "Home speed Y:",
  "",
};

const char *page_homespeed_z[] = {
  " ",
  "Home speed Z:",
  "",
};

const char *page_version[] = {
  " ",
  "",
};

//Printer settings
const char *page_printer[] = {
  "Control",
  "X Max:",
  "X Min:",
  "Y Max:",
  "Y Min:",
  "Z Max:",
  "Z Min:",
  "Home X:",
  "Home Y:",
  "Home Z:",
  "X Motor Dir:", 
  "Y Motor Dir:",
  "Z Motor Dir:",
  "E Motor Dir:",
#if (defined EXTRUDER_2IN1) || (defined EXTRUDER_3IN1) || (defined EXTRUDER_DUAL)
  "E1 Motor Dir:",
#endif
#if defined(EXTRUDER_3IN1)  
  "E2 Motor Dir:",
#endif
  "XmaxEndstop",
  "XminEndstop",
  "YmaxEndstop",
  "YminEndstop",
  "ZmaxEndstop",
  "ZminEndstop", 
  "",
};

const char *page_xmax[] = {
  " ",
  "X Max:",
  "", 
};

const char *page_xmin[] = {
  " ",
  "X Min:",
  "", 
};

const char *page_ymax[] = {
  " ",
  "Y Max:",
  "", 
};

const char *page_ymin[] = {
  " ",
  "Y Min:",
  " ", 
};

const char *page_zmax[] = {
  " ",
  "Z Max:",
  "", 
};

const char *page_zmin[] = {
  " ",
  "Z Min:",
  " ", 
};

const char *page_homex[] = {
  " ",
  "Home X:",
  "", 
};

const char *page_homey[] = {
  " ",
  "Home Y:",
  "", 
};

const char *page_homez[] = {
  " ",
  "Home Z:",
  "", 
};

const char *page_dir_x[] = {
  " ",
  "X Motor Dir:",
  "", 
};

const char *page_dir_y[] = {
  " ",
  "Y Motor Dir:",
  "", 
};

const char *page_dir_z[] = {
  " ",
  "Z Motor Dir:",
  "", 
};

const char *page_dir_e[] = {
  " ",
  "E Motor Dir:",
  "", 
};
#if defined(EXTRUDER_2IN1) || defined(EXTRUDER_3IN1)
const char *page_dir_e1[] = {
  " ",
  "E1 Motor Dir:",
  "", 
};
#endif

#if defined(EXTRUDER_3IN1)
const char *page_dir_e2[] = {
  " ",
  "E2 Motor Dir:",
  "", 
};
#endif

const char *page_endstop_status[] = {
  "Printer settings",
  "Logic level:",
  "Status:",
  "", 
};

void page_init(void)
{
  page_buf_copy(page_info);
  strcpy(message,machine_name);
}

void page_nomal_refresh(void)
{
  if(strcmp(page_stack[page_stack_index],"Mixer")==0)
    Page_refresh_flag = 1;
}

void page_go_info(void)
{
  memset(page_stack,0,sizeof(page_stack));
  strcpy(page_stack[0],"Info screen");
  page_stack_index = 0;
  page_buf_copy(page_info);
  LCD_Clear();
}
  
 
void dispay_page_info(void)
{
  page_buf_copy(page_info);
  show_info_screen();
  Show_String_len(LCD_HIGHT-1,0,message,LCD_WIDTH);
}

void decoder_inc(void)
{
  decoder.decoder_dir = 1;
  decoder.pointer_index++;
  decoder.decoder_turn_flag = 1;
  Page_refresh_flag = 1; 
}

void decoder_dec(void)
{
  decoder.decoder_dir = 0;
  decoder.pointer_index--;
  decoder.decoder_turn_flag = 1;
  Page_refresh_flag = 1;
}

void decoder_press(void)
{
  if(decoder.key_press) return;
  else
  {
    decoder.key_press = 1;
    Page_sbuf.select_index = decoder.pointer_index;
   // decoder.pointer_index = 0;
    Page_refresh_flag = 1;
   // Menu_chage = 1; 
  }
}

static void feedrate_turn(void)
{
  if(decoder.decoder_turn_flag == 1)
  {
    if(decoder.decoder_dir==0) 
    {
      system_infor.feed_tare--;
      if(system_infor.feed_tare<15) system_infor.feed_tare = 15;
    }
    else if(decoder.decoder_dir == 1) 
    {
      system_infor.feed_tare++;
      if(system_infor.feed_tare>300) system_infor.feed_tare = 300;
    }
    decoder.decoder_dir = 0xff;
    decoder.decoder_turn_flag = 0;
  }
}

static void page_buf_copy(char *src[])
{
  int i;  
  memset(page_buf,0,sizeof(page_buf));
  for(i=0;i<sizeof_strbuf(src);i++)
  {
    if(src[i][0]==NULL)break;
    strcpy(page_buf[i],src[i]);
  }
  decoder.pointer_index = 0;
  Page_sbuf.select_index = 0;
  decoder.decoder_turn_flag = 0;
  Page_sbuf.first_line = 0; 
}

static int sizeof_sbuf(void)
{
   int i = 0;
   while(page_buf[i][0]!=NULL)
   {
      i++;
   }
   return i;
}

static int sizeof_strbuf(char *str[])
{
   int i = 0;
   while(str[i][0]!=NULL)
   {
      i++;
   }
   return i;
}

static void str_change(char *str,char *pD, char *pS)
{
  if(strcmp(str,pD)==0)
  strcpy(str,pS);
  return;
}

void page_info_refresh(void)
{
  if(strcmp(page_stack[page_stack_index],"Info screen")==0)
  {
    show_info_screen();
    return;
  }
}
 
static s16 get_line_num(char *str[],char *string)
{
   u8 i=0,a,num;
   a = (u8)sizeof_strbuf(str);
   while(i<a)
   {
      if(str[i][0] == NULL ) break;
     if(strcmp(str[i],string)==0) return i;
      i++;     
   }
   return -1; 
}

void show_info_screen(void)
{ 
  static u8 SD_check=0 , SD_previous_status=0;

  u8 i;
  if(page_stack_index != 0) {LCD_Clear();refresh_time_flag=0;}
  feedrate_turn();
  Show_UI(0,0,HEATER_UI);
  if(Current_Temperature[NOZZLE0]<0)
      Show_Int(0,1,"%3d",0);
  else                     
      Show_Int(0,1,"%3d",(int)Current_Temperature[NOZZLE0]); 
  
  Show_String(0,4,"/");
  Show_Int(0,5,"%d",(int)Setting.targe_temperature[NOZZLE0]);
  Show_Int(0,15,"%d",(int)Setting.targe_temperature[BED]);
       
  if(Setting.targe_temperature[NOZZLE0] < 10)                      
  {       
      Show_UI(0,6,DEGREE_UI); 
      Show_String(0,7,"  ");
  }
  else if(Setting.targe_temperature[NOZZLE0] < 100)
  {
      Show_UI(0,7,DEGREE_UI);  
      Show_String(0,8," ");
  }
  else 
  {
      Show_UI(0,8,DEGREE_UI);
  }
  
  
  if(Setting.targe_temperature[BED] < 10)                      
  {
      Show_UI(0,16,DEGREE_UI);
      Show_String(0,17,"  ");
  }
  else if(Setting.targe_temperature[BED] < 100)
  {
      Show_UI(0,17,DEGREE_UI); 
      Show_String(0,18," ");
  }
  else 
  {
      Show_UI(0,18,DEGREE_UI);
  }
  
  
  Show_Char(0,9,' ');
  Show_UI(0,10,HOTBED_UI);
  
  if(Current_Temperature[BED]<0)
      Show_Int(0,11,"%3d",0);
  else                     
      Show_Int(0,11,"%3d",(int)Current_Temperature[BED]); 
 
  Show_String(0,14,"/"); 
#if (defined EXTRUDER_2IN1) || (defined EXTRUDER_3IN1)
  u8 k,n;
  if(mixer.rate[NOZZLE0]>=100) k = 3;
  else if((mixer.rate[NOZZLE0]>=10)&&(mixer.rate[NOZZLE0]<100)) k = 2;
  else if(mixer.rate[NOZZLE0]<100) k = 1;
  Show_String(1,0,"Mx        ");
  Show_Char(1,2+k,'/');
  Show_Int_right(1,2+k,"%d",mixer.rate[NOZZLE0]); 
  Show_Int(1,3+k,"%d",mixer.rate[NOZZLE1]);
#if defined EXTRUDER_3IN1
  if(mixer.rate[NOZZLE1]>=100) n = 3;
  else if((mixer.rate[NOZZLE1]>=10)&&(mixer.rate[NOZZLE1]<100)) n = 2;
  else if(mixer.rate[NOZZLE1]<100) n = 1;
  Show_Char(1,3+n+k,'/');
  Show_Int(1,4+n+k,"%d",mixer.rate[NOZZLE2]); 
#endif
#else
#if (HOTHEAD_1)
  Show_UI(1,0,HEATER_UI);
  if(Current_Temperature[NOZZLE0]<0)
      Show_Int(1,1,"%3d",0);
  else                     
      Show_Int(1,1,"%3d",(int)Current_Temperature[NOZZLE1]); 
  
  Show_String(1,4,"/");
  Show_Int(1,5,"%d",(int)Setting.targe_temperature[NOZZLE1]);
       
  if(Setting.targe_temperature[NOZZLE1] < 10)                      
  {       
      Show_UI(1,6,DEGREE_UI); 
      Show_String(1,7,"  ");
  }
  else if(Setting.targe_temperature[NOZZLE1] < 100)
  {
      Show_UI(1,7,DEGREE_UI);  
      Show_String(1,8," ");
  }
  else 
  {
      Show_UI(1,8,DEGREE_UI);
  } 
#else
  Show_Char(1,0,'X');
  Show_Int(1,1,"%3d",(int)Current_Position[0]);
  Show_String(1,4,"  ");
  Show_Char(1,6,'Y');
  Show_Int(1,7,"%3d",(int)Current_Position[1]);
#endif
#endif 
  Show_String(1,10,"  ");
 /* Show_String(1,12,"Z00");  
  if(Current_Position[2] < 9.995)
      Show_Float(1,15,"%0.2f",Current_Position[2]);
  else if(Current_Position[2] < 99.995)
      Show_Float(1,14,"%0.2f",Current_Position[2]);
  else
      Show_Float(1,13,"%0.2f",Current_Position[2]);
  */
  Show_String(1,12,"Z      "); 
  //Show_Float_right(1,19,"%0.2f",Current_Position[2]);   
  Show_Float(1,13,"%0.2f",Current_Position[2]);  
  
  Show_UI(2,0,PERCENT_UI); 
  Show_Int(2,1,"%3d",system_infor.feed_tare);
  

  if(system_infor.sd_print_status == SD_PRINTING)
  {
      Show_String(2,4,"% SD   "); 
      Show_Int(2,8,"%3d",(int)system_infor.print_percent);
      Show_String(2,11,"% "); 
  }
  else
  {
      Show_String(2,4,"% SD---% ");
  }
  
  if( Print_Time_S % 2) Show_Char(2,17,':');
      else Show_Char(2,17,' ');
  if(refresh_time_flag==0 ||refresh_time_flag !=Print_Time_M)
  {
       Show_UI(2,13,TIME_UI);
       refresh_time_flag = Print_Time_M;
      if(Print_Time_H < 9.995)
      {
          Show_String(2,14,"00");
          Show_Float(2,16,"%0.0f",Print_Time_H);
      }
      else if(Print_Time_H < 99.995)
      {
          Show_String(2,14,"0");
          Show_Float(2,15,"%0.0f",Print_Time_H);
      }
      else
          Show_Float(2,14,"%0.0f",Print_Time_H);
      
      //if( Print_Time_S % 2) Show_Char(2,17,':');
      //else Show_Char(2,17,' ');
      
      if(Print_Time_M < 9.995)
      {
          Show_String(2,18,"0");
          Show_Float(2,19,"%0.0f",Print_Time_M);
      }
      else
          Show_Float(2,18,"%0.0f",Print_Time_M);
  }
  
  Show_String_len(LCD_HIGHT-1,0,message,LCD_WIDTH);

}

void sd_plug_detect(void)
{     
  s16 num;
  num = MAX(get_line_num(page_main,"SD Card"),get_line_num(page_main,"No SD Card"));
  SD_Select_Init();
  
  if(system_infor.sd_status == SD_OK)
  {
    memset(message,0,CMDBUF_SIZE);
    strcpy(message,"SD Card OK!");
    page_main[num] = (char*) malloc(strlen("SD Card"));
    strcpy(page_main[num],"SD Card"); 
    if(strcmp(page_stack[page_stack_index],"Main")==0)
    {
      strcpy(page_buf[num],"SD Card");   
    }      
    Page_refresh_flag = 1;   
  }
  else
  {
    memset(message,0,CMDBUF_SIZE);
    strcpy(message,"SD Card Removed!");
    page_main[num] = (char*)malloc(strlen("No SD Card"));
    strcpy(page_main[num],"No SD Card");  
    if(strcmp(page_stack[page_stack_index],"Main")==0)
    {
      strcpy(page_buf[num],"No SD Card");   
    } 
    Page_refresh_flag = 1;      
  }
  message_refresh();
}

void message_refresh(void)
{
  if(page_stack_index == 0)
  {
    Show_String(3,0,"                    ");
    //Show_String(3,0,message);
    Show_String_len(LCD_HIGHT-1,0,message,LCD_WIDTH);
  }
}


void get_sd_path(void)
{
    //TIM_Cmd(TIM5, DISABLE);
    file_type = (u8*)(strstr(page_buf[Page_sbuf.select_index], "gcode") || strstr(page_buf[Page_sbuf.select_index], "gco") || strstr(page_buf[Page_sbuf.select_index], "GCO"));
    print_file_size=file_size[Page_sbuf.select_index];
    if(file_type != NULL)
    { 
    
              
              sprintf(Command_Buffer,"M23 %s\r\n",page_buf[Page_sbuf.select_index]);
              Processing_command();
              sprintf(Command_Buffer,"M24\r\n");
              Processing_command();
              my_printf("SD_Path:%s\r\n",page_buf[Page_sbuf.select_index]);
              page_go_info();
			  dir_layer = 0;
        /*plan_init();
        sprintf(&SD_Path[strlen(SD_Path)], "/%s",page_buf[Page_sbuf.select_index]);
        //Systembuf_Infos.printer_file_path = SD_Path;
        printf("SD_Path:%s\r\n",SD_Path);
        strcpy(Systembuf_Infos.printer_file_path,SD_Path);
        memset(message,0,CMDBUF_SIZE);
        strcpy(message,page_buf[Page_sbuf.select_index]);
        page_go_info();
        system_infor.sd_print_flag = ENABLE;         
        system_infor.system_status  = PRINT;
        system_infor.feed_tare = 100;
        system_infor.sd_print_status  = SD_PRINT_START;
        dir_layer = 0;

#ifdef RECOVERY_MODE_USE
        recovery_enable = true;
#endif
*/
        return;                                              
    }
    else
    {
      file_type = strstr(page_buf[Page_sbuf.select_index], ".");
      if(file_type != NULL)
      {
          LCD_Clear();
          Show_String(1,0,"FILE TYPE NOT MATCH!");
          delay_s(3);
          next_layer_flag=1;
      }
      else
      {
          next_layer_flag=1;          
          sprintf(&SD_Path[strlen(SD_Path)], "/%s",page_buf[Page_sbuf.select_index]);
          strcpy(dir_name[dir_layer],page_buf[Page_sbuf.select_index]);
          dir_layer++;
          show_sd_dir(SD_Path); 
          decoder.pointer_index = 0;    
          Page_sbuf.select_index = 0;
      }
    }
    //TIM_Cmd(TIM5, ENABLE);
}
 

void show_sd_dir(char *path)
{     
  u8 dir_x=1,i;
  fno.lfname = lfn;
  fno.lfsize = sizeof lfn;
  Systembuf_Infos.printer_file_path[0] = NULL;
  dir_index = 1;
  dir_x=1;
  
  //TIM_Cmd(TIM5, DISABLE);
  
  LCD_Clear();
  Show_String(1,3,"SD Loading...");
  if(f_opendir(&dir,path) == FR_OK) 
  {  
    //memset(sd_file_name,0,sizeof(sd_file_name));  
    /*while((f_readdir(&dir,&fno) == FR_OK)&&(dir_index < 128))
      {
          if(fno.fname[0] == '.') continue; 
          
          if( fno.fname[0] == 0 ) break;
                      
          file_name = *fno.lfname ? fno.lfname : fno.fname;
          file_size[dir_index]=(float)fno.fsize;
          //strcpy(&sd_file_name[dir_index-1][0],file_name);     
          if((dir_layer==0)&&(dir_index-1==0))
          {
            strcpy(sd_file_name[0],"Main"); 
          }
          else if((dir_layer>0)&&(dir_index-1==0))
          {
            strcpy(sd_file_name[0],".."); 
          }                
          strcpy(&sd_file_name[dir_index][0],file_name); 
          
          dir_index++;
      }      */
      memset(page_buf,0,sizeof(page_buf));
      for(i=0;i<129;i++)
      {
        if(sd_file_name[i][0] == NULL)  break;
        strcpy(page_buf[i],sd_file_name[i]);
      }
      Page_sbuf.first_line = 0;
      decoder.pointer_index = 0;
      Page_sbuf.select_index = 0;
      decoder.decoder_turn_flag = 0;
  }
  else
  {
      LCD_Clear();
      Show_String(1,3,"OPEN DIR ERR!");         
      delay_s(3);
      dir_layer=0;
      strcpy(SD_Path,SD_Driver_Number); 
      Page_sbuf.first_line = 0;
      decoder.pointer_index = 0;
      Page_sbuf.select_index = 0;
      decoder.decoder_turn_flag = 0;
      return;
  }
  f_closedir(&dir);
  dir_number = dir_index-1;
  dir_index = 3;
  next_layer_flag=0;   
  if(f_opendir(&dir,path)== FR_OK)
  {    
      while((next_layer_flag == 0) && (system_infor.system_status  != SERIAL_CONNECT) && (system_infor.system_status  != IDLE)&&(system_infor.system_status  != HOMING))
      {                         
          if(serial_connect_flag == ENABLE)
          {
              fetch_next_command();
          }                    
          if((system_infor.system_status  == MAINMENU) || (system_infor.system_status  == PRINT))
          {
              dir_layer=0;
              return;
          }                   
      }         
  }
  else
  {
      LCD_Clear();
      Show_String(1,3,"OPEN DIR ERR!");
      delay_s(3);
      system_infor.system_status =MAINMENU;
      dir_layer=0;
      strcpy(SD_Path,SD_Driver_Number);
      Page_sbuf.first_line = 0;
      decoder.pointer_index = 0;
      Page_sbuf.select_index = 0;
      decoder.decoder_turn_flag = 0;
  }
} 

void page_enter(void)
{
  page_stack_index++;
  strcpy(page_stack[page_stack_index],page_buf[Page_sbuf.select_index]);
}

void page_exit(void)
{
  memset(page_stack[page_stack_index],0,sizeof(50));
  page_stack_index--;  
}

void display_ui(void)
{
  u8 i;
  
  if(page_stack_index==0)
  {
    show_info_screen();
  }
  else
  {
    
    if(decoder.pointer_index<=0)
      decoder.pointer_index = Page_sbuf.min_menu_value;
    
     else if(decoder.pointer_index>=sizeof_sbuf()-1)
      decoder.pointer_index=sizeof_sbuf()-1;
    
    if(decoder.decoder_dir==1)
    {
      if(decoder.pointer_index-Page_sbuf.first_line >= LCD_HIGHT)
      {     
        Page_sbuf.first_line++;
      }
    }
    
    else if(decoder.decoder_dir==0)
    {  
      if(Page_sbuf.first_line > decoder.pointer_index)
      {
       Page_sbuf.first_line = decoder.pointer_index;
      }
    }
   
    Page_sbuf.show_index = decoder.pointer_index - Page_sbuf.first_line; 
    
    LCD_Clear();
    
    if(Page_sbuf.show_index>=LCD_HIGHT)
    {
      Page_sbuf.show_index = LCD_HIGHT-1;
    }
    
    for(i=0;i<LCD_HIGHT;i++)
    {
      
      if(strcmp(page_stack[page_stack_index - dir_layer],"SD Card") == 0)
      {
        file_type = strstr(*(page_buf+Page_sbuf.first_line+i), ".");
        if((file_type == NULL)&& \
            (strcmp(*(page_buf+Page_sbuf.first_line+i),"Main")!=0)|| \
               (strcmp(*(page_buf+Page_sbuf.first_line+i),"..")==0))          
        {
            Show_UI(i,1,DOCUMENT_UI);
            Show_String_len(i,2,*(page_buf+Page_sbuf.first_line+i),LCD_WIDTH-3);
        }
        else
        {
            Show_String_len(i,1,*(page_buf+Page_sbuf.first_line+i),LCD_WIDTH-2);
        }
      }
      else
      {
        Show_String_len(i,1,*(page_buf+Page_sbuf.first_line+i),LCD_WIDTH-2);    
      }
     display_icon_value(i,*(page_buf+Page_sbuf.first_line+i));
     
    }
    
    if(strcmp(page_stack[page_stack_index],"Info screen")!=0)
    {
      Show_Char(Page_sbuf.show_index,0,'>');
      if(decoder.pointer_index == 0)
      {
          Show_UI(0,0,BACK_UI);
      }
      if(Page_sbuf.first_line == 0)
        Show_UI(i,LCD_WIDTH-1,BACK_UI);    
    } 
  }
  value_adj();
  decoder.decoder_dir = 0xff;
  
}

void display_icon_value(u8 x,char *line)
{   
     if((strcmp(line,"Prepare")==0) || \
        (strcmp(line,"Turn")==0) || \
        (strcmp(line,"Control")==0) || \
        (strcmp(line,"SD Card")==0) || \
        (strcmp(line,"Mixer")==0) || \
        (strcmp(line,"Templates")==0) || \
        (strcmp(line,"Custom")==0) || \
        (strcmp(line,"Custom 1 Conf")==0) || \
        (strcmp(line,"Custom 2 Conf")==0) || \
        (strcmp(line,"Custom 3 Conf")==0) || \
        (strcmp(line,"Custom 4 Conf")==0) || \
        (strcmp(line,"Custom 5 Conf")==0) || \
        (strcmp(line,"Custom 6 Conf")==0) || \
        (strcmp(line,"OFP")==0) || \
        (strcmp(line,"Move Axis")==0) || \
        (strcmp(line,"Move 10mm")==0) || \
        (strcmp(line,"Move 1mm")==0) || \
        (strcmp(line,"Move 0.1mm")==0) || \
        (strcmp(line,"Move X")==0) || \
        (strcmp(line,"Move Y")==0) || \
        (strcmp(line,"Move Z")==0) || \
        (strcmp(line,"Extruder 0")==0) || \
        (strcmp(line,"Extruder 1")==0) || \
        (strcmp(line,"Extruder 2")==0) || \
        (strcmp(line,"Temperature")==0) || \
        (strcmp(line,"Motion")==0) || \
        (strcmp(line,"Printer Settings")==0) || \
        (strcmp(line,"Delta")==0) || \
        (strcmp(line,"N-PID")==0) || \
        (strcmp(line,"B-PID")==0) || \
        (strcmp(line,"Preheat PLA Conf")==0) || \
        (strcmp(line,"Preheat ABS Conf")==0) || \
        (strcmp(line,"XmaxEndstop")==0)  || \
        (strcmp(line,"XminEndstop")==0)  || \
        (strcmp(line,"YmaxEndstop")==0)  || \
        (strcmp(line,"YminEndstop")==0)  || \
        (strcmp(line,"ZmaxEndstop")==0)  || \
        (strcmp(line,"ZminEndstop")==0) )
        {
          Show_Char(x,LCD_WIDTH-1,0x7e);
        }
     if((strcmp(page_stack[page_stack_index],"Templates") == 0) && (color_change_flag == Page_sbuf.first_line+x))
     {
        Show_Char(x,LCD_WIDTH-1,'*');
     }

    if(strcmp(line,"   Min:")==0)
    {
      Show_UI(x,2,HEATER_UI);
      Show_Int_right(x,19,"%3d",210.); 
    }
    
    if(strcmp(line,"   Max:")==0) 
    {
      Show_UI(x,2,HEATER_UI);
      Show_Int_right(x,19,"%3d",250.);     
    }
    if(strcmp(line,"   Fact:")==0)
    {
      Show_UI(x,2,HEATER_UI);
    }  
    if(strcmp(line,"Nozzle:")==0)
    {          
      if(strcmp(page_stack[page_stack_index],"Temperature")==0) Show_Int_right(x,19,"%5d",(int)Setting.targe_temperature[NOZZLE0]); 
      else if(strcmp(page_stack[page_stack_index],"Turn")==0) Show_Int_right(x,19,"%5d",(int)Setting.targe_temperature[NOZZLE0]);
      else if(strcmp(page_stack[page_stack_index],"Preheat PLA Conf")==0) Show_Int_right(x,19,"%5d",(int)Setting.Preheat_conf[PLA][1]); 
      else if(strcmp(page_stack[page_stack_index],"Preheat ABS Conf")==0) Show_Int_right(x,19,"%5d",(int)Setting.Preheat_conf[ABS][1]); 
    } 
#if(HOTHEAD_1)
    if(strcmp(line,"Nozzle 1:")==0)
    {          
      if(strcmp(page_stack[page_stack_index],"Temperature")==0) Show_Int_right(x,19,"%5d",(int)Setting.targe_temperature[NOZZLE1]); 
      else if(strcmp(page_stack[page_stack_index],"Turn")==0) Show_Int_right(x,19,"%5d",(int)Setting.targe_temperature[NOZZLE1]);
      else if(strcmp(page_stack[page_stack_index],"Preheat PLA Conf")==0) Show_Int_right(x,19,"%5d",(int)Setting.Preheat_conf[PLA][1]); 
      else if(strcmp(page_stack[page_stack_index],"Preheat ABS Conf")==0) Show_Int_right(x,19,"%5d",(int)Setting.Preheat_conf[ABS][1]); 
    } 
#endif
    if(strcmp(line,"Bed:")==0)
    {     
      if(strcmp(page_stack[page_stack_index],"Temperature")==0) Show_Int_right(x,19,"%5d",Setting.targe_temperature[BED]); 
      else if(strcmp(page_stack[page_stack_index],"Turn")==0) Show_Int_right(x,19,"%5d",(int)Setting.targe_temperature[BED]);
      else if(strcmp(page_stack[page_stack_index],"Preheat PLA Conf")==0) Show_Int_right(x,19,"%5d",(int)Setting.Preheat_conf[PLA][2]); 
      else if(strcmp(page_stack[page_stack_index],"Preheat ABS Conf")==0) Show_Int_right(x,19,"%5d",(int)Setting.Preheat_conf[ABS][2]); 
    }  
    
    if(strcmp(line,"Fan speed:")==0)
    {          
      if(strcmp(page_stack[page_stack_index],"Temperature")==0) Show_Int_right(x,19,"%3d",(int)Setting.fanspeed); 
      else if(strcmp(page_stack[page_stack_index],"Turn")==0) Show_Int_right(x,19,"%3d",(int)Setting.fanspeed); 
      else if(strcmp(page_stack[page_stack_index],"Preheat PLA Conf")==0) Show_Int_right(x,19,"%3d",(int)Setting.Preheat_conf[PLA][0]); 
      else if(strcmp(page_stack[page_stack_index],"Preheat ABS Conf")==0) Show_Int_right(x,19,"%3d",(int)Setting.Preheat_conf[ABS][0]); 
    } 
    
    //Motion
    if(strcmp(line,"Z offset:")==0)      Show_Float_right(x,19,"%0.2f",Setting.z_offset);   
    if(strcmp(line,"Accel:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.acceleration);
    if(strcmp(line,"Vx-jerk:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.max_x_jerk);
    if(strcmp(line,"Vy-jerk:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.max_y_jerk);
    if(strcmp(line,"Vz-jerk:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.max_z_jerk); 
    if(strcmp(line,"Ve-jerk:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.max_e_jerk);
    if(strcmp(line,"Vmax X:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.max_feedrate[X_AXIS]);  
    if(strcmp(line,"Vmax Y:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.max_feedrate[Y_AXIS]);
    if(strcmp(line,"Vmax Z:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.max_feedrate[Z_AXIS]);
    if(strcmp(line,"Vmax E:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.max_feedrate[E_AXIS]);
    if(strcmp(line,"Vmin:")==0)      Show_Int_right(x,19,"%5d",(int)Setting.min_feedrate);
    if(strcmp(line,"VTrav min:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.min_travel_feedrate);
    if(strcmp(line,"Amax X:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.max_acceleration[X_AXIS]);
    if(strcmp(line,"Amax Y:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.max_acceleration[Y_AXIS]);
    if(strcmp(line,"Amax Z:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.max_acceleration[Z_AXIS]);
    if(strcmp(line,"Amax E:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.max_acceleration[E_AXIS]);
    if(strcmp(line,"A-retract:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.retract_acceleration);
    if(strcmp(line,"Xsteps/mm:")==0)     Show_Float_right(x,19,"%0.2f",Setting.steps_per_mm[X_AXIS]); 
    if(strcmp(line,"Ysteps/mm:")==0)     Show_Float_right(x,19,"%0.2f",Setting.steps_per_mm[Y_AXIS]); 
    if(strcmp(line,"Zsteps/mm:")==0)     Show_Float_right(x,19,"%0.2f",Setting.steps_per_mm[Z_AXIS]); 
    if(strcmp(line,"Esteps/mm:")==0)     Show_Float_right(x,19,"%0.2f",Setting.steps_per_mm[E_AXIS]); 
    if(strcmp(line,"Home speed X:")==0)     Show_Int_right(x,19,"%5d",Setting.home_speed[X_AXIS]);
    if(strcmp(line,"Home speed Y:")==0)     Show_Int_right(x,19,"%5d",Setting.home_speed[Y_AXIS]);
    if(strcmp(line,"Home speed Z:")==0)     Show_Int_right(x,19,"%5d",Setting.home_speed[Z_AXIS]);
    
    //Printer Settings   
    if(strcmp(line,"X Max:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.max_position[X_AXIS]);
    if(strcmp(line,"X Min:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.min_position[X_AXIS]);
    if(strcmp(line,"Y Max:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.max_position[Y_AXIS]);
    if(strcmp(line,"Y Min:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.min_position[Y_AXIS]);
    if(strcmp(line,"Z Max:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.max_position[Z_AXIS]);
    if(strcmp(line,"Z Min:")==0)     Show_Int_right(x,19,"%5d",(int)Setting.min_position[Z_AXIS]);
    if(strcmp(line,"Home X:")==0)
    {
      if(Setting.home_position[X_AXIS])  Show_String_right(x,19,"Max");
      else Show_String_right(x,19,"Min");
    }
    if(strcmp(line,"Home Y:")==0)
    {
      if(Setting.home_position[Y_AXIS])  Show_String_right(x,19,"Max");
      else Show_String_right(x,19,"Min");
    }    
    if(strcmp(line,"Home Z:")==0)
    {
      if(Setting.home_position[Z_AXIS])  Show_String_right(x,19,"Max");
      else Show_String_right(x,19,"Min");
    }  
    if(strcmp(line,"X Motor Dir:")==0)
    {
      if(Setting.motor_direction[X_AXIS])  Show_String_right(x,19,"True");
      else Show_String_right(x,19,"False");
    }    
    if(strcmp(line,"Y Motor Dir:")==0)
    {
      if(Setting.motor_direction[Y_AXIS])  Show_String_right(x,19,"True");
      else Show_String_right(x,19,"False");
    }   
    if(strcmp(line,"Z Motor Dir:")==0)
    {
      if(Setting.motor_direction[Z_AXIS])  Show_String_right(x,19,"True");
      else Show_String_right(x,19,"False");
    }   
    if(strcmp(line,"E Motor Dir:")==0)
    {
      if(Setting.motor_direction[E_AXIS])  Show_String_right(x,19,"True");
      else Show_String_right(x,19,"False");
    }   
#if defined(EXTRUDER_2IN1) || defined(EXTRUDER_3IN1) || (defined EXTRUDER_DUAL)   
    if(strcmp(line,"E1 Motor Dir:")==0)
    {
      if(Setting.motor_direction[E1_AXIS])  Show_String_right(x,19,"True");
      else Show_String_right(x,19,"False");
    }
#endif
    
#if defined(EXTRUDER_3IN1)    
    if(strcmp(line,"E2 Motor Dir:")==0)
    {
      if(Setting.motor_direction[E2_AXIS])  Show_String_right(x,19,"True");
      else Show_String_right(x,19,"False");
    }
#endif
    
    if(strcmp(line,"Logic level:")==0)
    {
      if(strcmp(page_stack[page_stack_index],"XminEndstop")==0)  
      {
        if(Setting.endstop_level[X_MIN]) Show_String_right(x,19,"High");
        else Show_String_right(x,19,"Low");
      }
      else if(strcmp(page_stack[page_stack_index],"XmaxEndstop")==0)  
      {
        if(Setting.endstop_level[X_MAX]) Show_String_right(x,19,"High");
        else Show_String_right(x,19,"Low");
      } 
      else if(strcmp(page_stack[page_stack_index],"YminEndstop")==0)  
      {
        if(Setting.endstop_level[Y_MIN]) Show_String_right(x,19,"High");
        else Show_String_right(x,19,"Low");
      }
      else if(strcmp(page_stack[page_stack_index],"YmaxEndstop")==0)  
      {
        if(Setting.endstop_level[Y_MAX]) Show_String_right(x,19,"High");
        else Show_String_right(x,19,"Low");
      } 
      else if(strcmp(page_stack[page_stack_index],"ZminEndstop")==0)  
      {
        if(Setting.endstop_level[Z_MIN]) Show_String_right(x,19,"High");
        else Show_String_right(x,19,"Low");
      }
      else if(strcmp(page_stack[page_stack_index],"ZmaxEndstop")==0)  
      {
        if(Setting.endstop_level[Z_MAX]) Show_String_right(x,19,"High");
        else Show_String_right(x,19,"Low");
      }       
    }
    if(strcmp(line,"Status:")==0)
    {
      if(strcmp(page_stack[page_stack_index],"XminEndstop")==0)  
      {
        if(Setting.endstop_status[X_MIN]) Show_String_right(x,19,"NO");
        else Show_String_right(x,19,"NC");
      }
      else if(strcmp(page_stack[page_stack_index],"XmaxEndstop")==0)  
      {
        if(Setting.endstop_status[X_MAX]) Show_String_right(x,19,"NO");
        else Show_String_right(x,19,"NC");
      } 
      else if(strcmp(page_stack[page_stack_index],"YminEndstop")==0)  
      {
        if(Setting.endstop_status[Y_MIN]) Show_String_right(x,19,"NO");
        else Show_String_right(x,19,"NC");
      }
      else if(strcmp(page_stack[page_stack_index],"YmaxEndstop")==0)  
      {
        if(Setting.endstop_status[Y_MAX]) Show_String_right(x,19,"NO");
        else Show_String_right(x,19,"NC");
      } 
      else if(strcmp(page_stack[page_stack_index],"ZminEndstop")==0)  
      {
        if(Setting.endstop_status[Z_MIN]) Show_String_right(x,19,"NO");
        else Show_String_right(x,19,"NC");
      }
      else if(strcmp(page_stack[page_stack_index],"ZmaxEndstop")==0)  
      {
        if(Setting.endstop_status[Z_MAX]) Show_String_right(x,19,"NO");
        else Show_String_right(x,19,"NC");
      }       
    }    
    //End Printer Settings 
 
    //Delta Settings
#ifdef DELTA 
    if(strcmp(line,"Segments/sec:")==0)      Show_Int_right(x,19,"%4d",(int)Setting.delta_segments_per_sec);   
    if(strcmp(line,"Diagonal Rod:")==0)      Show_Int_right(x,19,"%4d",(int)Setting.delta_diagonal_rod);
    if(strcmp(line,"Smooth offs:")==0)      Show_Int_right(x,19,"%4d",(int)Setting.delta_smooth_rod_offset);
    if(strcmp(line,"Effector offs:")==0)      Show_Int_right(x,19,"%4d",(int)Setting.delta_effector_offset);
    if(strcmp(line,"Carriage offs:")==0)      Show_Int_right(x,19,"%4d",(int)Setting.delta_carriage_offset); 
    if(strcmp(line,"Delta Radius:")==0)      Show_Float_right(x,19,"%0.1f",Setting.delta_radius);
    if(strcmp(line,"Print Radius:")==0)      Show_Int_right(x,19,"%4d",(int)Setting.delta_printable_radius);  
#endif
    //End Delta Settings
    
    if(strcmp(line,"Autotemp:")==0)     
    {     
      Show_String_right(x,19,"Off"); 
    } 
    
    //Mixer
   /* if(strcmp(line,"OFP:")==0)
    {     
        if(Setting.mixer_ofp) Show_String_right(x,19,"ON");
        else Show_String_right(x,19,"OFF");
    }*/
    if(strcmp(line,"Filament 0:")==0)
    {     
      Show_Char(x,18,'%');
      if(mixer.rate[NOZZLE0] <= Setting.mixer_ofp_min) mixer.rate[NOZZLE0] = Setting.mixer_ofp_min;
      if(mixer.rate[NOZZLE0] >= Setting.mixer_ofp_max) mixer.rate[NOZZLE0] = Setting.mixer_ofp_max;
      Show_Int_right(x,18,"%5d",mixer.rate[NOZZLE0]); 
    }  
    if(strcmp(line,"Filament 1:")==0)
    {     
      Show_Char(x,18,'%');
      if(mixer.rate[NOZZLE1] <= Setting.mixer_ofp_min) mixer.rate[NOZZLE1] = Setting.mixer_ofp_min;
      if(mixer.rate[NOZZLE1] >= Setting.mixer_ofp_max) mixer.rate[NOZZLE1] = Setting.mixer_ofp_max;
      Show_Int_right(x,18,"%5d",mixer.rate[NOZZLE1]); 
    }
 
    if(strcmp(line,"Filament 2:")==0)
    {     
      Show_Char(x,18,'%');
      if(mixer.rate[NOZZLE2] <= Setting.mixer_ofp_min) mixer.rate[NOZZLE2] = Setting.mixer_ofp_min;
      if(mixer.rate[NOZZLE2] >= Setting.mixer_ofp_max) mixer.rate[NOZZLE2] = Setting.mixer_ofp_max;
      Show_Int_right(x,18,"%5d",mixer.rate[NOZZLE2]); 
    }   
     
    if(strcmp(line,"OFP Min:")==0)
    {      
      Show_Char(x,18,'%');
      Show_Int_right(x,18,"%5d",(int)Setting.mixer_ofp_min);
      Mixer_Init();
    }
    
    if(strcmp(line,"OFP Max:")==0)
    {      
      Show_Char(x,18,'%');
      Show_Int_right(x,18,"%5d",(int)Setting.mixer_ofp_max);
      Mixer_Init();
    }
    
    if(strcmp(line,"Start Percent:")==0)
    {
      Show_Char(x,18,'%');
      Show_Int_right(x,18,"%3d",(int)Setting.custom_conf_start_percent[custom_conf_select-1]);
    }
    if(strcmp(line,"End Percent:")==0)
    {
      Show_Char(x,18,'%');
      Show_Int_right(x,18,"%3d",(int)Setting.custom_conf_end_percent[custom_conf_select-1]);
    }    
    if(strcmp(line,"Start Height:")==0)
    {
      Show_Float_right(x,19,"%0.1f",Setting.custom_conf_start_height[custom_conf_select-1]);
    } 
    if(strcmp(line,"End Height:")==0)
    {
      Show_Float_right(x,19,"%0.1f",Setting.custom_conf_end_height[custom_conf_select-1]);
    }     

    if(strcmp(line,"N-PID-P:")==0)
    {     
      Show_Char(x,11,'+');
      Show_Float_right(x,19,"%07.2f",Setting.Kp[NOZZLE0]); 
;
    }   
    if(strcmp(line,"N-PID-I:")==0)
    {     
      Show_Char(x,11,'+');
      Show_Float_right(x,19,"%07.2f",Setting.Ki[NOZZLE0]); 
    }
    if(strcmp(line,"N-PID-D:")==0)
    {     
      Show_Char(x,11,'+');
      Show_Float_right(x,19,"%07.2f",Setting.Kd[NOZZLE0]); 
    }
    if(strcmp(line,"B-PID-P:")==0)
    {     
      Show_Char(x,11,'+');
      Show_Float_right(x,19,"%07.2f",Setting.Kp[BED]); 
    }   
    if(strcmp(line,"B-PID-I:")==0)
    {     
      Show_Char(x,11,'+');
      Show_Float_right(x,19,"%07.2f",Setting.Ki[BED]); 
    }
    if(strcmp(line,"B-PID-D:")==0)
    {     
      Show_Char(x,11,'+');
      Show_Float_right(x,19,"%07.2f",Setting.Kd[BED]); 
    }    
}

void display_refresh(void)
{ 
    int i;
    u8 refresh = 0;
    s16 num;
    char name[50] = {0};
    
    if(Page_refresh_flag == 1)
    {
      Page_refresh_flag = 0;
      refresh = 1;   
    }
    
    if(decoder.key_press)
    {
      //decoder.key_press = 0;   
      refresh = 1;
      if(page_stack_index == 0) 
      {
        strcpy(name,"Main"); 
        page_stack_index++;;
        strcpy(page_stack[page_stack_index],name);
      }
      else if((Page_sbuf.select_index == 0) || \
               (strcmp(page_stack[page_stack_index],"Move X")==0)|| \
               (strcmp(page_stack[page_stack_index],"Move Y")==0)|| \
               (strcmp(page_stack[page_stack_index],"Move Z")==0)|| \
               (strcmp(page_stack[page_stack_index],"Extruder 0")==0) || \
               (strcmp(page_stack[page_stack_index],"Extruder 1")==0) || \
               (strcmp(page_stack[page_stack_index],"Extruder 2")==0) || \
               (strcmp(page_stack[page_stack_index],"N-PID-P:")==0) || \
               (strcmp(page_stack[page_stack_index],"N-PID-I:")==0) || \
               (strcmp(page_stack[page_stack_index],"N-PID-D:")==0) || \
               (strcmp(page_stack[page_stack_index],"B-PID-P:")==0) || \
               (strcmp(page_stack[page_stack_index],"B-PID-I:")==0) || \
               (strcmp(page_stack[page_stack_index],"B-PID-D:")==0) || \
               (strcmp(page_stack[page_stack_index],"Filament 0:")==0) || \
               (strcmp(page_stack[page_stack_index],"Filament 1:")==0) || \
               (strcmp(page_stack[page_stack_index],"Filament 2:")==0) )
        
      {
        strcpy(name,page_stack[page_stack_index-1]);
        page_exit();
      }
      
      else if((strcmp(page_buf[Page_sbuf.select_index],"   Min:")==0) || \
               (strcmp(page_buf[Page_sbuf.select_index],"   Max:")==0) || \
               (strcmp(page_buf[Page_sbuf.select_index],"   Fact:")==0) );     
      else 
      {
        strcpy(name,page_buf[Page_sbuf.select_index]);
        page_enter();
      }

          if(strcmp(name,"Info screen")==0)
          {
            page_go_info();
          }
          
          else if(strcmp(name,"Prepare")==0)
          {
            page_buf_copy(page_prepare);
          }
          
          else if(strcmp(name,"Turn")==0)
          {
            page_buf_copy(page_turn);
            
          }  
          
          else if(strcmp(name,"Control")==0)
          {
            page_buf_copy(page_control);
          }       
          
          else if(strcmp(name,"No SD Card")==0)
          {
            page_exit();
          }   
              
          else if(strcmp(name,"Main")==0)
          {
            do{
              page_buf_copy(page_main);
              if(system_infor.sd_print_flag == ENABLE)
              {
                for(i=0;i<sizeof_sbuf()-1;i++)
                {
                  str_change(page_buf[i],"Prepare","Turn");
                }

                num = MAX(get_line_num(page_main,"SD Card"),get_line_num(page_main,"No SD Card"));
                if(num == -1) break;
                if(system_infor.pause_flag  == false)
                {
                  strcpy(page_buf[num],"Pause Print");
                  strcpy(page_buf[num+1],"Stop Print");         
                }
                if(system_infor.pause_flag  == true)
                {
                  strcpy(page_buf[num],"Resume Print");
                  strcpy(page_buf[num+1],"Stop Print");  
                }
              }
            }while(0);
          }
          
          else if(strcmp(name,"Pause Print")==0)
          {
            num = MAX(get_line_num(page_main,"SD Card"),get_line_num(page_main,"No SD Card"));
            system_infor.pause_flag  = true;
            //system_status = PAUSE_PRINT;
            strcpy(page_buf[num],"Resume Print");
            page_exit();
          }

          else if(strcmp(name,"Resume Print")==0)
          {
            num = MAX(get_line_num(page_main,"SD Card"),get_line_num(page_main,"No SD Card"));
            system_infor.pause_flag  = false;
           // system_status = PRINT;
            strcpy(page_buf[num],"Pause Print");
            page_exit();
          } 
          
          else if(strcmp(name,"Stop Print")==0)
          {
            system_infor.sd_print_flag = DISABLE;
            system_infor.stop_flag = 1;
            system_infor.system_status  = IDLE;
            memset(message,0,CMDBUF_SIZE);
            strcpy(message,"Print stopping...");
            page_go_info();
          }       
          
          else if(strcmp(name,"Disable Steppers")==0)
          {
            Disable_all_Axis();
            page_exit();
          } 
          
          else if(strcmp(name,"Auto Home")==0)
          {            
             System_message = 1;
             page_exit();
          }   
          
          else if(strcmp(name,"Preheat PLA")==0)
          {
            Setting.targe_temperature[NOZZLE0] = Setting.Preheat_conf[PLA][1]; 
            Setting.targe_temperature[BED] = Setting.Preheat_conf[PLA][2];       
            Temperature_Control_Enable();
            page_go_info();
          }
          
          else if(strcmp(name,"Preheat ABS")==0)
          {
            Setting.targe_temperature[NOZZLE0] = Setting.Preheat_conf[ABS][1];
            Setting.targe_temperature[BED] = Setting.Preheat_conf[ABS][2];
            Temperature_Control_Enable();
            page_go_info();
          } 
          
          else if(strcmp(name,"Cooldown")==0)
          {
            Set_All_Fan_Power(FULL_POWER);
            page_go_info();
          }      
     
          else if(strcmp(name,"Mixer")==0)
          {
            page_buf_copy(page_mixer); 
          }
          
          else if(strcmp(name,"Filament 0:")==0)
          {
            page_buf_copy(page_mix_filament0);
          }
          else if(strcmp(name,"Filament 1:")==0)
          {
            page_buf_copy(page_mix_filament1);
          }
          else if(strcmp(name,"Filament 2:")==0)
          {
            page_buf_copy(page_mix_filament1);
          }
          else if(strcmp(name,"Templates")==0)
          {
            page_buf_copy(page_template); 
          }     
          else if(strcmp(name,"Custom")==0)
          {
            page_buf_copy(page_custom); 
          }       
          else if(strcmp(name,"Template 1")==0)   
          {
            if(color_change_flag == TEMPLATE_1) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = TEMPLATE_1;
            }
            page_exit();
          }
          else if(strcmp(name,"Template 2")==0)   
          {
            if(color_change_flag == TEMPLATE_2) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = TEMPLATE_2;
            }
            page_exit();
          }  
          else if(strcmp(name,"Template 3")==0)   
          {
            if(color_change_flag == TEMPLATE_3) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = TEMPLATE_3;
            }
            page_exit();
          }        
          else if(strcmp(name,"Template 4")==0)   
          {
            if(color_change_flag == TEMPLATE_4) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = TEMPLATE_4;
            }
            page_exit();
          }  
          else if(strcmp(name,"Template 5")==0)   
          {
            if(color_change_flag == TEMPLATE_5) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = TEMPLATE_5;
            }
            page_exit();
          }        
          else if(strcmp(name,"Template 6")==0)   
          {
            if(color_change_flag == TEMPLATE_6) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = TEMPLATE_6;
            }
            page_exit();
          }
          else if(strcmp(name,"Template 7")==0)   
          {
            if(color_change_flag == TEMPLATE_7) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = TEMPLATE_7;
            }
            page_exit();
          } 
          else if(strcmp(name,"Template 8")==0)   
          {
            if(color_change_flag == TEMPLATE_8) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = TEMPLATE_8;
            }
            page_exit();
          } 
      
          else if(strcmp(name,"Custom 1")==0)   
          {
            if(color_change_flag == 8 + CUSTOM_1) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = 8 + CUSTOM_1;
            }
            page_exit();
          }      
          else if(strcmp(name,"Custom 2")==0)   
          {
            if(color_change_flag == 8 + CUSTOM_2) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = 8 + CUSTOM_2;
            }
            page_exit();
          }    

          else if(strcmp(name,"Custom 3")==0)   
          {
            if(color_change_flag == 8 + CUSTOM_3) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = 8 + CUSTOM_3;
            }
            page_exit();
          }         
  
          else if(strcmp(name,"Custom 4")==0)   
          {
            if(color_change_flag == 8 + CUSTOM_4) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = 8 + CUSTOM_4;
            }
            page_exit();
          }
      
          else if(strcmp(name,"Custom 5")==0)   
          {
            if(color_change_flag == 8 + CUSTOM_5) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = 8 + CUSTOM_5;
            }
            page_exit();
          }       
          else if(strcmp(name,"Custom 6")==0)   
          {
            if(color_change_flag == 8 + CUSTOM_6) color_change_flag = 0;
            else
            {
              get_current_position(Z_AXIS);
              color_change_flag = 8 + CUSTOM_6;
            }
            page_exit();
          }        
           
          else if(strcmp(name,"Custom 1 Conf")==0)
          {
            page_buf_copy(page_custom_conf); 
            custom_conf_select = CUSTOM_1;
          }
          else if(strcmp(name,"Custom 2 Conf")==0)
          {
            page_buf_copy(page_custom_conf); 
            custom_conf_select = CUSTOM_2;
          }
          else if(strcmp(name,"Custom 3 Conf")==0)
          {
            page_buf_copy(page_custom_conf); 
            custom_conf_select = CUSTOM_3;
          }
          else if(strcmp(name,"Custom 4 Conf")==0)
          {
            page_buf_copy(page_custom_conf); 
            custom_conf_select = CUSTOM_4;
          }
          else if(strcmp(name,"Custom 5 Conf")==0)
          {
            page_buf_copy(page_custom_conf); 
            custom_conf_select = CUSTOM_5;
          }
          else if(strcmp(name,"Custom 6 Conf")==0)
          {
            page_buf_copy(page_custom_conf); 
            custom_conf_select = CUSTOM_6;
          }
          else if(strcmp(name,"Start Percent:")==0)
          {
            page_buf_copy(page_start_percent); 
          }
          else if(strcmp(name,"End Percent:")==0)
          {
            page_buf_copy(page_end_percent); 
          }
          else if(strcmp(name,"Start Height:")==0)
          {
            page_buf_copy(page_start_height); 
          }
          else if(strcmp(name,"End Height:")==0)
          {
            page_buf_copy(page_end_height); 
          }      
          /*else if(strcmp(name,"OFP:")==0)   
          {
            if(Setting.mixer_ofp) 
            {
              Setting.mixer_ofp = DISABLE;
              mixer.max = 100;
              mixer.min = 100 - mixer.max;
            }
            else
            {
              Setting.mixer_ofp = ENABLE;
              mixer.max = Setting.mixer_ofp_max;
              mixer.min = 100 - mixer.max;
              if(mixer.rate[NOZZLE0] >= mixer.max) mixer.rate[NOZZLE0] = mixer.max;
              if(mixer.rate[NOZZLE0] <= mixer.min)  mixer.rate[NOZZLE0] = mixer.min;
              mixer.rate[NOZZLE1] = 100 - mixer.rate[NOZZLE0];              
            }           
            page_exit();
          }*/
          else if(strcmp(name,"OFP")==0)
          {
            page_buf_copy(page_ofp_conf);
          }          
          else if(strcmp(name,"OFP Min:")==0)
          {
            page_buf_copy(page_ofp_min);
          }         
          else if(strcmp(name,"OFP Max:")==0)
          {
            page_buf_copy(page_ofp_max);
          }      

          else if(strcmp(name,"Temperature")==0)
          {
            page_buf_copy(page_temp);
          } 
          
          else if(strcmp(name,"Nozzle:")==0)
          {
            page_buf_copy(page_tnozzle);
          } 
#if(HOTHEAD_1)
          else if(strcmp(name,"Nozzle 1:")==0)
          {
            page_buf_copy(page_tnozzle1);
          } 
#endif
          else if(strcmp(name,"Bed:")==0)
          {
            page_buf_copy(page_tbed);
          }        
          else if(strcmp(name,"Fan speed:")==0)
          {
            page_buf_copy(page_fanspeed);
          }             
          else if(strcmp(name,"N-PID")==0)
          {
            page_buf_copy(page_npid);
          }   
          else if(strcmp(name,"N-PID-P:")==0)
          {
            page_buf_copy(page_npid_p);
          }        
          else if(strcmp(name,"N-PID-I:")==0)
          {
            page_buf_copy(page_npid_i);
          }           
          else if(strcmp(name,"N-PID-D:")==0)
          {
            page_buf_copy(page_npid_d);
          }   
          else if(strcmp(name,"B-PID")==0)
          {
            page_buf_copy(page_bpid);
          } 
          else if(strcmp(name,"B-PID-P:")==0)
          {
            page_buf_copy(page_bpid_p);
          }        
          else if(strcmp(name,"B-PID-I:")==0)
          {
            page_buf_copy(page_bpid_i);
          }           
          else if(strcmp(name,"B-PID-D:")==0)
          {
            page_buf_copy(page_bpid_d);
          }           
          else if(strcmp(name,"Preheat PLA Conf")==0)
          {
            page_buf_copy(page_pla_conf);
          }       
      
          else if(strcmp(name,"Preheat ABS Conf")==0)
          {
            page_buf_copy(page_abs_conf);
          }         
          else if(strcmp(name,"Motion")==0)
          {
            page_buf_copy(page_motion);
            decoder.decoder_turn_flag = 0;
          }       
     
          else if(strcmp(name,"Printer Settings")==0)
          {
            page_buf_copy(page_printer);
          }  
#ifdef DELTA
          else if(strcmp(name,"Delta")==0)
          {
            page_buf_copy(page_delta);
          }  
          else if(strcmp(name,"Segments/sec:")==0)
          {
            page_buf_copy(page_delta_segments_per_sec);
          }  
          else if(strcmp(name,"Diagonal Rod:")==0)
          {
            page_buf_copy(page_delta_diagonal_rod);
          } 
          else if(strcmp(name,"Smooth offs:")==0)
          {
            page_buf_copy(page_delta_smooth_offset);
          } 
          else if(strcmp(name,"Effector offs:")==0)
          {
            page_buf_copy(page_delta_effector_offset);
          } 
          else if(strcmp(name,"Carriage offs:")==0)
          {
            page_buf_copy(page_delta_carriage_offset);
          } 
          else if(strcmp(name,"Delta Radius:")==0)
          {
            page_buf_copy(page_delta_radius);
          } 
          else if(strcmp(name,"Print Radius:")==0)
          {
            page_buf_copy(page_delta_printable_radius);
          }
#endif      
          else if(strcmp(name,"Store Memory")==0)
          {      
            Store_Memory(USER_SETTINGS);
            page_exit();
          }
      
          else if(strcmp(name,"Restore Defaults")==0)
          {      
            Restore_Defaults(USER_SETTINGS);
            page_exit();
          }
      
          else if(strcmp(name,"Version")==0)
          {      
            page_buf_copy(page_version);                  
          }     
          else if(strcmp(name,"Version")==0)
          {      
            page_exit();                  
          }      
      
          else if(strcmp(name,"Move Axis")==0)
          {
            page_buf_copy(page_move);
          } 
          
          else if(strcmp(name,"Move 10mm")==0)
          {
            page_buf_copy(page_move_10mm);
          }   
          
          else if(strcmp(name,"Move 1mm")==0)
          {
            page_buf_copy(page_move_1mm);          
          } 
          
          else if(strcmp(name,"Move 0.1mm")==0)
          {
            page_buf_copy(page_move_01mm);
          }
          else if(strcmp(name,"Move X")==0)
          {
            page_buf_copy(page_move_x);            
          }
          else if(strcmp(name,"Move Y")==0)
          {
            page_buf_copy(page_move_y);  
          }
          else if(strcmp(name,"Move Z")==0)
          {
            page_buf_copy(page_move_z);  
          }
          else if(strcmp(name,"Extruder 0")==0)
          {
            page_buf_copy(page_move_extruder0);  
          }
          else if(strcmp(name,"Extruder 1")==0)
          {
            page_buf_copy(page_move_extruder1);  
          }
          else if(strcmp(name,"Extruder 2")==0)
          {
            page_buf_copy(page_move_extruder2);  
          }      
          else if(strcmp(name,"SD Card")==0)
          {
            SD_Select_Init();
           // for(i=0;i<100;i++)
            //{
              //system_infor.sd_status = f_mount(&fats,SD_Driver_Number,1);
            // if(system_infor.sd_status == FR_OK) break;
           // }
            if(system_infor.sd_status == FR_OK)
            {
              dir_layer = 0;
              show_sd_dir(SD_Path); 
              decoder.pointer_index = 0;
              Page_sbuf.select_index = 0;
            }
            else page_exit();
          }           
                 
          else if(strcmp(page_stack[2],"SD Card") == 0)
          {
            if(strcmp(page_buf[Page_sbuf.select_index],"..")==0)
            { 
                strcpy(SD_Path,SD_Driver_Number);
                for(i=3;i<=page_stack_index;i++)
                {
                  sprintf(&SD_Path[strlen(SD_Path)], "/%s",page_stack[i]);
                }
                  
                dir_layer--;
                show_sd_dir(SD_Path);
                decoder.pointer_index = 0; 
                Page_sbuf.select_index = 0;
            }
            else
            {
              get_sd_path();              
             // Page_refresh_flag = 1; 
              refresh = 1;
            }       
          }
      //Motion
          else if(strcmp(name,"Z offset")==0)
          {
            page_exit();  
          }
          else if(strcmp(name,"Accel:")==0)
          {
            page_buf_copy(page_accel);  
          }
          else if(strcmp(name,"Vx-jerk:")==0)
          {
            page_buf_copy(page_vxjerk);  
          }
          else if(strcmp(name,"Vy-jerk:")==0)
          {
            page_buf_copy(page_vyjerk);  
          }
          else if(strcmp(name,"Vz-jerk:")==0)
          {
            page_buf_copy(page_vzjerk);  
          }
          else if(strcmp(name,"Ve-jerk:")==0)
          {
            page_buf_copy(page_vejerk);  
          }
          else if(strcmp(name,"Vmax X:")==0)
          {
            page_buf_copy(page_vmaxx);  
          }
          else if(strcmp(name,"Vmax Y:")==0)
          {
            page_buf_copy(page_vmaxy);  
          }
          else if(strcmp(name,"Vmax Z:")==0)
          {
            page_buf_copy(page_vmaxz);  
          }
          else if(strcmp(name,"Vmax E:")==0)
          {
            page_buf_copy(page_vmaxe);  
          }
          else if(strcmp(name,"Vmin:")==0)
          {
            page_buf_copy(page_vmin);  
          }
          else if(strcmp(name,"Amax X:")==0)
          {
            page_buf_copy(page_amaxx);  
          }
          else if(strcmp(name,"Amax Y:")==0)
          {
            page_buf_copy(page_amaxy);  
          }
          else if(strcmp(name,"Amax Z:")==0)
          {
            page_buf_copy(page_amaxz);  
          }
          else if(strcmp(name,"Amax E:")==0)
          {
            page_buf_copy(page_amaxe);  
          }
          else if(strcmp(name,"A-retract:")==0)
          {
            page_buf_copy(page_aretract);  
          }
          else if(strcmp(name,"Xsteps/mm:")==0)
          {
            page_buf_copy(page_xsteps_mm);  
          }
          else if(strcmp(name,"Ysteps/mm:")==0)
          {
            page_buf_copy(page_ysteps_mm);  
          }
          else if(strcmp(name,"Zsteps/mm:")==0)
          {
            page_buf_copy(page_zsteps_mm);  
          }
          else if(strcmp(name,"Esteps/mm:")==0)
          {
            page_buf_copy(page_esteps_mm);  
          }
          else if(strcmp(name,"Home speed X:")==0)
          {
            page_buf_copy(page_homespeed_x);  
          }
          else if(strcmp(name,"Home speed Y:")==0)
          {
            page_buf_copy(page_homespeed_y);  
          }
          else if(strcmp(name,"Home speed Z:")==0)
          {
            page_buf_copy(page_homespeed_z);  
          }      
      //Printer Settings
      
          else if(strcmp(name,"X Max:")==0)
          {
            page_buf_copy(page_xmax);  
          }
          else if(strcmp(name,"X Min:")==0)
          {
            page_buf_copy(page_xmin);  
          }        
          else if(strcmp(name,"Y Max")==0)
          {
            page_buf_copy(page_ymax);  
          }
          else if(strcmp(name,"Y Min:")==0)
          {
            page_buf_copy(page_ymin);  
          }
          else if(strcmp(name,"Z Max:")==0)
          {
            page_buf_copy(page_zmax);  
          }
          else if(strcmp(name,"Z Min:")==0)
          {
            page_buf_copy(page_zmin);  
          }
          else if(strcmp(name,"Home X:")==0)
          {
            Setting.home_position[X_AXIS] = ~Setting.home_position[X_AXIS] & 0x0001;
            page_exit();  
          }
          else if(strcmp(name,"Home Y:")==0)
          {
            Setting.home_position[Y_AXIS] = ~Setting.home_position[Y_AXIS] & 0x0001;
            page_exit();  
          } 
          else if(strcmp(name,"Home Z:")==0)
          {
            Setting.home_position[Z_AXIS] = (~Setting.home_position[Z_AXIS]) & 0x0001;
            page_exit();  
          }      
          else if(strcmp(name,"Home X:")==0)
          {
            Setting.home_position[X_AXIS] = ~Setting.home_position[X_AXIS] & 0x0001;
            page_exit();  
          }
          else if(strcmp(name,"X Motor Dir:")==0)
          {
            Setting.motor_direction[X_AXIS] = ~Setting.motor_direction[X_AXIS] & 0x0001;
            page_exit();  
          } 
          else if(strcmp(name,"Y Motor Dir:")==0)
          {
            Setting.motor_direction[Y_AXIS] = ~Setting.motor_direction[Y_AXIS] & 0x0001;
            page_exit();  
          }  
          else if(strcmp(name,"Z Motor Dir:")==0)
          {
            Setting.motor_direction[Z_AXIS] = ~Setting.motor_direction[Z_AXIS] & 0x0001;
            page_exit();  
          } 
          else if(strcmp(name,"E Motor Dir:")==0)
          {
            Setting.motor_direction[E_AXIS] = ~Setting.motor_direction[E_AXIS] & 0x0001;
            page_exit();  
          } 
#if defined(EXTRUDER_2IN1) || defined(EXTRUDER_3IN1) || (defined EXTRUDER_DUAL)
          else if(strcmp(name,"E1 Motor Dir:")==0)
          {
            Setting.motor_direction[E1_AXIS] = ~Setting.motor_direction[E1_AXIS] & 0x0001;
            page_exit();  
          } 
#endif
      
#if defined(EXTRUDER_3IN1)      
          else if(strcmp(name,"E2 Motor Dir:")==0)
          {
            Setting.motor_direction[E2_AXIS] = ~Setting.motor_direction[E2_AXIS] & 0x0001;
            page_exit();  
          }
#endif
          else if((strcmp(name,"XmaxEndstop")==0) || \
                   (strcmp(name,"XminEndstop")==0) || \
                   (strcmp(name,"YmaxEndstop")==0) || \
                   (strcmp(name,"YminEndstop")==0) || \
                   (strcmp(name,"ZmaxEndstop")==0) || \
                   (strcmp(name,"ZminEndstop")==0) )
          {
            page_buf_copy(page_endstop_status);  
          } 
          else if(strcmp(name,"Logic level:")==0)
          {
            if(strcmp(page_stack[page_stack_index-1],"XminEndstop")==0) Setting.endstop_level[X_MIN] = ~Setting.endstop_level[X_MIN] & 0x0001;
            else if(strcmp(page_stack[page_stack_index-1],"XmaxEndstop")==0) Setting.endstop_level[X_MAX] = ~Setting.endstop_level[X_MAX] & 0x0001;
            else if(strcmp(page_stack[page_stack_index-1],"YminEndstop")==0) Setting.endstop_level[Y_MIN] = ~Setting.endstop_level[Y_MIN] & 0x0001;
            else if(strcmp(page_stack[page_stack_index-1],"YmaxEndstop")==0) Setting.endstop_level[Y_MAX] = ~Setting.endstop_level[Y_MAX] & 0x0001;     
            else if(strcmp(page_stack[page_stack_index-1],"ZminEndstop")==0) Setting.endstop_level[Z_MIN] = ~Setting.endstop_level[Z_MIN] & 0x0001;
            else if(strcmp(page_stack[page_stack_index-1],"ZmaxEndstop")==0) Setting.endstop_level[Z_MAX] = ~Setting.endstop_level[Z_MAX] & 0x0001; 
            Endstop_GPIO_config();
            page_exit();
          }
          else if(strcmp(name,"Status:")==0)
          {
            if(strcmp(page_stack[page_stack_index-1],"XminEndstop")==0) Setting.endstop_status[X_MIN] = ~Setting.endstop_status[X_MIN] & 0x0001;
            else if(strcmp(page_stack[page_stack_index-1],"XmaxEndstop")==0) Setting.endstop_status[X_MAX] = ~Setting.endstop_status[X_MAX] & 0x0001;
            else if(strcmp(page_stack[page_stack_index-1],"YminEndstop")==0) Setting.endstop_status[Y_MIN] = ~Setting.endstop_status[Y_MIN] & 0x0001;
            else if(strcmp(page_stack[page_stack_index-1],"YmaxEndstop")==0) Setting.endstop_status[Y_MAX] = ~Setting.endstop_status[Y_MAX] & 0x0001;     
            else if(strcmp(page_stack[page_stack_index-1],"ZminEndstop")==0) Setting.endstop_status[Z_MIN] = ~Setting.endstop_status[Z_MIN] & 0x0001;
            else if(strcmp(page_stack[page_stack_index-1],"ZmaxEndstop")==0) Setting.endstop_status[Z_MAX] = ~Setting.endstop_status[Z_MAX] & 0x0001; 
            Endstop_GPIO_config();
            page_exit();
          }   
      /*
          else 
          {
            page_buf_copy(page_main);  
          }*/
      //page_get_flag = 1;
      //memset(current_page_name,0,20);
      //strcpy(page_stack[page_stack_index],name);
      decoder.key_press = 0;
    } 
    if(refresh == 1)
    {
      display_ui();
    }
} 

void value_adj(void)
{
  float temp1,temp2;
  if(strcmp(page_buf[1],"X:")==0)  
  {
    LCD_Clear();
    Show_String(1,1,"X:");
     
    if(decoder.decoder_turn_flag == 1)
    {  
      strcpy(Command_Buffer,"G91\r\n");
      Processing_command();
      if(decoder.decoder_dir==1)
      {    
        if(Current_Position[X_AXIS] < Setting.min_position[X_AXIS])
        {
          Current_Position[X_AXIS] = Setting.min_position[X_AXIS];
        } 
        
        else
        {
           if(strcmp(page_stack[page_stack_index-1],"Move 10mm") == 0) strcpy(Command_Buffer,"G1 X10 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 X1 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 X0.1 F4800 \r\n");
        }
      }
      else if(decoder.decoder_dir==0)
      {
        if(Current_Position[X_AXIS] > Setting.max_position[X_AXIS])
        {
          Current_Position[X_AXIS] = Setting.max_position[X_AXIS];
        } 
        else
        {
           if(strcmp(page_stack[page_stack_index-1],"Move 10mm") == 0) strcpy(Command_Buffer,"G1 X-10 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 X-1 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 X-0.1 F4800 \r\n");
        }
      }
      Processing_command();
      strcpy(Command_Buffer,"G90\r\n");
      Processing_command();
      decoder.decoder_turn_flag = 0;
      decoder.decoder_dir = 0xff;
    }
    Show_Float_right(1,19,"%0.01f",Current_Position[X_AXIS]);
  }
  
  if(strcmp(page_buf[1],"Y:")==0)  
  {
    LCD_Clear();
    Show_String(1,1,"Y:");
     
    if(decoder.decoder_turn_flag == 1)
    {  
      strcpy(Command_Buffer,"G91\r\n");
      Processing_command();
      if(decoder.decoder_dir==1)
      {    
        if(Current_Position[Y_AXIS] < Setting.min_position[Y_AXIS])
        {
          Current_Position[Y_AXIS] = Setting.min_position[Y_AXIS];
        } 
        
        else
        { 
           if(strcmp(page_stack[page_stack_index-1],"Move 10mm") == 0) strcpy(Command_Buffer,"G1 Y10 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 Y1 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 Y0.1 F4800 \r\n");
        }
      }
      else if(decoder.decoder_dir==0)
      {
        if(Current_Position[Y_AXIS] > Setting.max_position[Y_AXIS])
        {
          Current_Position[Y_AXIS] = Setting.max_position[Y_AXIS];
        } 
        else
        {
           if(strcmp(page_stack[page_stack_index-1],"Move 10mm") == 0) strcpy(Command_Buffer,"G1 Y-10 F4800 \r\n");
           if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 Y-1 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 Y-0.1 F4800 \r\n");
        }
      }
      Processing_command();
      strcpy(Command_Buffer,"G90\r\n");
      Processing_command();
      decoder.decoder_turn_flag = 0;
      decoder.decoder_dir = 0xff;
    }      
    Show_Float_right(1,19,"%0.01f",Current_Position[Y_AXIS]);
  }  

  if(strcmp(page_buf[1],"Z:")==0)  
  {
    LCD_Clear();
    Show_String(1,1,"Z:");
     
    if(decoder.decoder_turn_flag == 1)
    {  
      strcpy(Command_Buffer,"G91\r\n");
      Processing_command();
      if(decoder.decoder_dir==1)
      {    
        if(Current_Position[Z_AXIS] < Setting.min_position[Z_AXIS])
        {
          Current_Position[Z_AXIS] = Setting.min_position[Z_AXIS];
        } 
        
        else
        {
           if(strcmp(page_stack[page_stack_index-1],"Move 10mm") == 0) strcpy(Command_Buffer,"G1 Z10 4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 Z1 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 Z0.1 F4800 \r\n");
        }
      }
      else if(decoder.decoder_dir==0)
      {
        if(Current_Position[Z_AXIS] > Setting.max_position[Z_AXIS])
        {
          Current_Position[Z_AXIS] = Setting.max_position[Z_AXIS];
        } 
        else
        {
           if(strcmp(page_stack[page_stack_index-1],"Move 10mm") == 0) strcpy(Command_Buffer,"G1 Z-10 F4800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 Z-1 F4800 \r\n");
           if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 Z-0.1 F4800 \r\n");
        }
      }
      Processing_command();
      strcpy(Command_Buffer,"G90\r\n");
      Processing_command();
      decoder.decoder_turn_flag = 0;
      decoder.decoder_dir = 0xff;
    }
       
    Show_Float_right(1,19,"%0.01f",Current_Position[Z_AXIS]);
  }    

  if(strcmp(page_buf[1],"Extruder 0:")==0)  
  {
    LCD_Clear();
    Show_String(1,1,"Extruder 0:");
     
    if(decoder.decoder_turn_flag == 1)
    {  
      strcpy(Command_Buffer,"T0\r\n");
      Processing_command();
      strcpy(Command_Buffer,"G91\r\n");
      Processing_command();
      if(decoder.decoder_dir==1)
      {    
           if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0)  strcpy(Command_Buffer,"G1 E1 1800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 E0.1 F1800 \r\n");
      }
      else if(decoder.decoder_dir==0)
      {
           if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 E-1 F1800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 E-0.1 F1800 \r\n");
      }
      Processing_command();
      strcpy(Command_Buffer,"G90\r\n");
      Processing_command();
      decoder.decoder_turn_flag = 0;
      decoder.decoder_dir = 0xff;
    }      
    Show_Float_right(1,19,"%0.01f",Current_Position[E_AXIS]);
  } 
  if(strcmp(page_buf[1],"Extruder 1:")==0)  
  {
    LCD_Clear();
    Show_String(1,1,"Extruder 1:");
     
    if(decoder.decoder_turn_flag == 1)
    {  
      strcpy(Command_Buffer,"T1\r\n");
      Processing_command();
      strcpy(Command_Buffer,"G91\r\n");
      Processing_command();
      if(decoder.decoder_dir==1)
      {     
           if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 E1 F1800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 E0.1 F1800 \r\n");
      }
      else if(decoder.decoder_dir==0)
      {
           if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 E-1 F1800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 E-0.1 F1800 \r\n");
      }
      Processing_command();
      strcpy(Command_Buffer,"G90\r\n");
      Processing_command();
      decoder.decoder_turn_flag = 0;
      decoder.decoder_dir = 0xff;
      color_change_flag = 0;
    }      
    Show_Float_right(1,19,"%0.01f",Current_Position[E_AXIS]);
  } 
  
  if(strcmp(page_buf[1],"Extruder 2:")==0)  
  {
    LCD_Clear();
    Show_String(1,1,"Extruder 2:");
     
    if(decoder.decoder_turn_flag == 1)
    {  
      strcpy(Command_Buffer,"T2\r\n");
      Processing_command();
      strcpy(Command_Buffer,"G91\r\n");
      Processing_command();
      if(decoder.decoder_dir==1)
      {     
           if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 E1 F1800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 E0.1 F1800 \r\n");
      }
      else if(decoder.decoder_dir==0)
      {
           if(strcmp(page_stack[page_stack_index-1],"Move 1mm") == 0) strcpy(Command_Buffer,"G1 E-1 F1800 \r\n");
           else if(strcmp(page_stack[page_stack_index-1],"Move 0.1mm") == 0) strcpy(Command_Buffer,"G1 E-0.1 F1800 \r\n");
      }
      Processing_command();
      strcpy(Command_Buffer,"G90\r\n");
      Processing_command();
      decoder.decoder_turn_flag = 0;
      decoder.decoder_dir = 0xff;
      color_change_flag = 0;
    }      
    Show_Float_right(1,19,"%0.01f",Current_Position[E_AXIS]);
  } 
  
  if(strcmp(page_stack[page_stack_index],"Filament 0:")==0)  
  {   
    mixer.rate[NOZZLE0] = (int)value_turn(Setting.mixer_ofp_min,Setting.mixer_ofp_max,1,mixer.rate[NOZZLE0]); 
#ifdef EXTRUDER_3IN1
    mixer.rate[NOZZLE2] = 100-mixer.rate[NOZZLE0]-mixer.rate[NOZZLE1];
#else
    mixer.rate[NOZZLE1] = 100-mixer.rate[NOZZLE0];
#endif
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"Filament 0:");
    Show_Char(1,18,'%');
    Show_Int_right(1,18,"%5d",mixer.rate[NOZZLE0]); 
    color_change_flag = 0;
  }
  
  if(strcmp(page_stack[page_stack_index],"Filament 1:")==0)
  {  
    mixer.rate[NOZZLE1] = (int)value_turn(Setting.mixer_ofp_min,Setting.mixer_ofp_max,1,mixer.rate[NOZZLE1]); 
#ifdef EXTRUDER_3IN1
    if(mixer.rate[NOZZLE1]>=(100-mixer.rate[NOZZLE0]))   mixer.rate[NOZZLE1] = 100-mixer.rate[NOZZLE0];
    mixer.rate[NOZZLE2] = 100-mixer.rate[NOZZLE0]-mixer.rate[NOZZLE1];
#else
    mixer.rate[NOZZLE0] = 100-mixer.rate[NOZZLE1];   
#endif
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"Filament 1:");
    Show_Char(1,18,'%');
    Show_Int_right(1,18,"%5d",mixer.rate[NOZZLE1]); 
    color_change_flag = 0;
  } 
  
  if(strcmp(page_stack[page_stack_index],"Filament 2:")==0)
  {  
    mixer.rate[NOZZLE2] = (int)value_turn(Setting.mixer_ofp_min,100-mixer.rate[NOZZLE0],1,mixer.rate[NOZZLE2]); 
#ifdef EXTRUDER_3IN1
    if(mixer.rate[NOZZLE2]>=(100-mixer.rate[NOZZLE0]-mixer.rate[NOZZLE1]))   mixer.rate[NOZZLE2] = 100-mixer.rate[NOZZLE0]-mixer.rate[NOZZLE1];
#endif
    mixer.rate[NOZZLE0] = 100-(mixer.rate[NOZZLE1]+mixer.rate[NOZZLE2]);  
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"Filament 2:");
    Show_Char(1,18,'%');
    Show_Int_right(1,18,"%5d",mixer.rate[NOZZLE2]); 
    color_change_flag = 0;
  }  
  if(strcmp(page_stack[page_stack_index],"OFP Min:")==0)  
  {   
    Setting.mixer_ofp_min = (int)value_turn(0,100,1,Setting.mixer_ofp_min);
#ifdef EXTRUDER_3IN1    
    Setting.mixer_ofp_max = 100 - Setting.mixer_ofp_min*2;
#else
    Setting.mixer_ofp_max = 100 - Setting.mixer_ofp_min;    
#endif
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"OFP Min:");
    Show_Char(1,18,'%');
    Show_Int_right(1,18,"%5d",Setting.mixer_ofp_min); 
  }
  
  if(strcmp(page_stack[page_stack_index],"OFP Max:")==0)  
  {      
#ifdef EXTRUDER_3IN1
    Setting.mixer_ofp_max = (int)value_turn(0,100,2,Setting.mixer_ofp_max); 
    Setting.mixer_ofp_min = (100 - Setting.mixer_ofp_max)>>1;
#else
    Setting.mixer_ofp_max = (int)value_turn(0,100,1,Setting.mixer_ofp_max);
    Setting.mixer_ofp_min = 100 - Setting.mixer_ofp_max;
#endif
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"OFP Max:");
    Show_Char(1,18,'%');
    Show_Int_right(1,18,"%5d",Setting.mixer_ofp_max); 
    color_change_flag = 0;
  }
  
  if(strcmp(page_stack[page_stack_index],"Start Percent:")==0)
  {
    Setting.custom_conf_start_percent[custom_conf_select-1] = (int)value_turn(0,100,1,Setting.custom_conf_start_percent[custom_conf_select-1]);
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"Start Percent:");
    Show_Char(1,18,'%');
    Show_Int_right(1,18,"%3d",(int)Setting.custom_conf_start_percent[custom_conf_select-1]); 
  }
  if(strcmp(page_stack[page_stack_index],"End Percent:")==0)
  {
    Setting.custom_conf_end_percent[custom_conf_select-1] = (int)value_turn(0,100,1,Setting.custom_conf_end_percent[custom_conf_select-1]);
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"End Percent:");
    Show_Char(1,18,'%');
    Show_Int_right(1,18,"%3d",(int)Setting.custom_conf_end_percent[custom_conf_select-1]); 
  }  

  if(strcmp(page_stack[page_stack_index],"Start Height:")==0)
  {
    Setting.custom_conf_start_height[custom_conf_select-1] = value_turn(0,Z_MAX_POSITION,1,Setting.custom_conf_start_height[custom_conf_select-1]);
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"Start Height:");
    Show_Float_right(1,19,"%0.1f",Setting.custom_conf_start_height[custom_conf_select-1]); 
  }
  if(strcmp(page_stack[page_stack_index],"End Height:")==0)
  {
    Setting.custom_conf_end_height[custom_conf_select-1] = value_turn(0,Z_MAX_POSITION,1,Setting.custom_conf_end_height[custom_conf_select-1]);
    Page_sbuf.select_index = 0;
    LCD_Clear();
    Show_String(1,1,"End Height:");
    Show_Float_right(1,19,"%0.1f",Setting.custom_conf_end_height[custom_conf_select-1]); 
  }  
  
  if(strcmp(page_stack[page_stack_index],"Nozzle:")==0)
  {
    LCD_Clear();
    Show_String(1,1,"Nozzle:");
    if((strcmp(page_stack[page_stack_index-1],"Temperature") == 0) || (strcmp(page_stack[page_stack_index-1],"Turn") == 0))
    {
      Setting.targe_temperature[NOZZLE0] = (int)value_turn(Setting.min_temperature[NOZZLE0],Setting.max_temperature[NOZZLE0],1,Setting.targe_temperature[NOZZLE0]); 
      Show_Int_right(1,19,"%5d",Setting.targe_temperature[NOZZLE0]);   
    }
    else if(strcmp(page_stack[page_stack_index-1],"Preheat PLA Conf") == 0)
    {
      Setting.Preheat_conf[PLA][1] = (int)value_turn(Setting.min_temperature[NOZZLE0],Setting.max_temperature[NOZZLE0],1,Setting.Preheat_conf[PLA][1]); 
      Show_Int_right(1,19,"%5d",Setting.Preheat_conf[PLA][1]);   
    }   
    else if(strcmp(page_stack[page_stack_index-1],"Preheat ABS Conf") == 0)
    {
      Setting.Preheat_conf[ABS][1] = (int)value_turn(Setting.min_temperature[NOZZLE0],Setting.max_temperature[NOZZLE0],1,Setting.Preheat_conf[ABS][1]); 
      Show_Int_right(1,19,"%5d",Setting.Preheat_conf[ABS][1]);   
    }       
    Page_sbuf.select_index = 0;
  } 
#if(HOTHEAD_1)
  if(strcmp(page_stack[page_stack_index],"Nozzle 1:")==0)
  {
    LCD_Clear();
    Show_String(1,1,"Nozzle 1:");
    if((strcmp(page_stack[page_stack_index-1],"Temperature") == 0) || (strcmp(page_stack[page_stack_index-1],"Turn") == 0))
    {
      Setting.targe_temperature[NOZZLE1] = (int)value_turn(Setting.min_temperature[NOZZLE1],Setting.max_temperature[NOZZLE1],1,Setting.targe_temperature[NOZZLE1]); 
      Show_Int_right(1,19,"%5d",Setting.targe_temperature[NOZZLE1]);   
    }
    else if(strcmp(page_stack[page_stack_index-1],"Preheat PLA Conf") == 0)
    {
      Setting.Preheat_conf[PLA][1] = (int)value_turn(Setting.min_temperature[NOZZLE1],Setting.max_temperature[NOZZLE1],1,Setting.Preheat_conf[PLA][1]); 
      Show_Int_right(1,19,"%5d",Setting.Preheat_conf[PLA][1]);   
    }   
    else if(strcmp(page_stack[page_stack_index-1],"Preheat ABS Conf") == 0)
    {
      Setting.Preheat_conf[ABS][1] = (int)value_turn(Setting.min_temperature[NOZZLE1],Setting.max_temperature[NOZZLE1],1,Setting.Preheat_conf[ABS][1]); 
      Show_Int_right(1,19,"%5d",Setting.Preheat_conf[ABS][1]);   
    }       
    Page_sbuf.select_index = 0;
  } 
#endif  
  if(strcmp(page_stack[page_stack_index],"Bed:")==0)
  {
    LCD_Clear();
    Show_String(1,1,"Bed:");
    if((strcmp(page_stack[page_stack_index-1],"Temperature") == 0) || (strcmp(page_stack[page_stack_index-1],"Turn") == 0))
    {
      Setting.targe_temperature[BED] = (int)value_turn(Setting.min_temperature[BED],Setting.max_temperature[BED],1,Setting.targe_temperature[BED]);
      Show_Int_right(1,19,"%5d",Setting.targe_temperature[BED]); 
    }
    else if(strcmp(page_stack[page_stack_index-1],"Preheat PLA Conf") == 0)
    {
      Setting.Preheat_conf[PLA][2] = (int)value_turn(Setting.min_temperature[BED],Setting.max_temperature[BED],1,Setting.Preheat_conf[PLA][2]); 
      Show_Int_right(1,19,"%5d",Setting.Preheat_conf[PLA][2]);    
    }   
    else if(strcmp(page_stack[page_stack_index-1],"Preheat ABS Conf") == 0)
    {
      Setting.Preheat_conf[ABS][2] = (int)value_turn(Setting.min_temperature[BED],Setting.max_temperature[BED],1,Setting.Preheat_conf[ABS][2]);   
      Show_Int_right(1,19,"%5d",Setting.Preheat_conf[ABS][2]);     
    }    
    Page_sbuf.select_index = 0;
  } 

  if(strcmp(page_stack[page_stack_index],"Fan speed:")==0)   //need to check
  {
    LCD_Clear();
    Show_String(1,1,"Fan speed:");
    if((strcmp(page_stack[page_stack_index-1],"Temperature") == 0) || (strcmp(page_stack[page_stack_index-1],"Turn") == 0))
    {
      Setting.fanspeed = (int)value_turn(0,255,1,Setting.fanspeed);
      Show_Int_right(1,19,"%5d",Setting.fanspeed); 
    }
    else if(strcmp(page_stack[page_stack_index-1],"Preheat PLA Conf") == 0)
    {
      Setting.Preheat_conf[PLA][0] = (int)value_turn(0,255,1,Setting.Preheat_conf[PLA][0]);
      Show_Int_right(1,19,"%5d",Setting.Preheat_conf[PLA][0]);   
    }   
    else if(strcmp(page_stack[page_stack_index-1],"Preheat ABS Conf") == 0)
    {
      Setting.Preheat_conf[ABS][0] = (int)value_turn(0,255,1,Setting.Preheat_conf[ABS][0]);
      Show_Int_right(1,19,"%5d",Setting.Preheat_conf[ABS][0]);    
    }      
    Page_sbuf.select_index = 0;
  } 
  
  if(strcmp(page_stack[page_stack_index],"N-PID-P:")==0)    
  {
    Setting.Kp[NOZZLE0] = value_turn(-65535,0xFFFF,0.01,Setting.Kp[NOZZLE0]);
    LCD_Clear();
    Show_String(1,1,"N-PID-P:");
    Show_Char(1,11,'+');
    Show_Float_right(1,19,"%07.2f",Setting.Kp[NOZZLE0]); 
  }   
  if(strcmp(page_stack[page_stack_index],"N-PID-I:")==0)    
  {
    Setting.Ki[NOZZLE0] = (int)value_turn(-65535,0xFFFF,0.01,Setting.Ki[NOZZLE0]);
    LCD_Clear();
    Show_String(1,1,"N-PID-I:");
    Show_Char(1,11,'+');
    Show_Float_right(1,19,"%07.2f",Setting.Ki[NOZZLE0]); 
  } 
  if(strcmp(page_stack[page_stack_index],"N-PID-D:")==0)    
  {
    Setting.Kd[NOZZLE0] = value_turn(-65535,0xFFFF,0.01,Setting.Kd[NOZZLE0]);
    LCD_Clear();
    Show_String(1,1,"N-PID-D:");
    Show_Char(1,11,'+');
    Show_Float_right(1,19,"%07.2f",Setting.Kd[NOZZLE0]); 
  } 
  if(strcmp(page_stack[page_stack_index],"B-PID-P:")==0)    
  {
    Setting.Kp[BED] = value_turn(-65535,0xFFFF,0.01,Setting.Kp[BED]);
    LCD_Clear();
    Show_String(1,1,"B-PID-P:");
    Show_Char(1,11,'+');
    Show_Float_right(1,19,"%07.2f",Setting.Kp[BED]); 
  } 
  
  if(strcmp(page_stack[page_stack_index],"B-PID-I:")==0)    
  {
    Setting.Ki[BED] = value_turn(-65535,0xFFFF,0.01,Setting.Ki[BED]);
    LCD_Clear();
    Show_String(1,1,"B-PID-I:");
    Show_Char(1,11,'+');
    Show_Float_right(1,19,"%07.2f",Setting.Ki[BED]); 
  } 
  if(strcmp(page_stack[page_stack_index],"B-PID-D:")==0)    
  {
    Setting.Kd[BED] = value_turn(-65535,0xFFFF,0.01,Setting.Kd[BED]);
    LCD_Clear();
    Show_String(1,1,"B-PID-D:");
    Show_Char(1,11,'+');
   // Show_Float(1,13,"%07.2f",Setting.Kd[BED]); 
    Show_Float_right(1,19,"%07.2f",Setting.Kd[BED]);
  } 
    //Motion

  if(strcmp(page_stack[page_stack_index],"Z offset:")==0)   
  {
    Setting.z_offset = (float)value_turn(-65535,0xFFFF,0.1,Setting.z_offset);
    LCD_Clear();
    Show_String(1,1,"Z offset:");
    if(Setting.z_offset < 0) Show_Char(1,15,'-');
    Show_Float_right(1,19,"%0.2f",Setting.z_offset);
    Current_Position[Z_AXIS] = Current_Position[Z_AXIS]-0.1;
  } 
  if(strcmp(page_stack[page_stack_index],"Accel:")==0)    
  {
    Setting.acceleration = (int)value_turn(0,0xFFFF,1,Setting.acceleration);
    LCD_Clear();
    Show_String(1,1,"Accel:");
    Show_Int_right(1,19,"%5d",(int)Setting.acceleration);
  } 
  
  if(strcmp(page_stack[page_stack_index],"Vx-jerk:")==0)    
  {
    Setting.max_x_jerk = (int)value_turn(0,0xFFFF,1,Setting.max_x_jerk);
    LCD_Clear();
    Show_String(1,1,"Vx-jerk:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_x_jerk);
  } 
  if(strcmp(page_stack[page_stack_index],"Vy-jerk:")==0)    
  {
    Setting.max_y_jerk = (int)value_turn(0,0xFFFF,1,Setting.max_y_jerk);
    LCD_Clear();
    Show_String(1,1,"Vy-jerk:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_y_jerk);
  } 
  if(strcmp(page_stack[page_stack_index],"Vz-jerk:")==0)    
  {
    Setting.max_z_jerk = (int)value_turn(0,0xFFFF,1,Setting.max_z_jerk);
    LCD_Clear();
    Show_String(1,1,"Vz-jerk:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_z_jerk);
  }   
  if(strcmp(page_stack[page_stack_index],"Ve-jerk:")==0)    
  {
    Setting.max_e_jerk = (int)value_turn(0,0xFFFF,1,Setting.max_e_jerk);
    LCD_Clear();
    Show_String(1,1,"Ve-jerk:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_e_jerk);
  }     
  if(strcmp(page_stack[page_stack_index],"Vmax X:")==0)    
  {
    Setting.max_feedrate[X_AXIS] = (int)value_turn(0,0xFFFF,1,Setting.max_feedrate[X_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Vmax X:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_feedrate[X_AXIS]);
  }   
   if(strcmp(page_stack[page_stack_index],"Vmax Y:")==0)    
  {
    Setting.max_feedrate[Y_AXIS] = (int)value_turn(0,0xFFFF,1,Setting.max_feedrate[Y_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Vmax Y:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_feedrate[Y_AXIS]);
  } 
  if(strcmp(page_stack[page_stack_index],"Vmax Z:")==0)    
  {
    Setting.max_feedrate[Z_AXIS] = (int)value_turn(0,0xFFFF,1,Setting.max_feedrate[Z_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Vmax Z:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_feedrate[Z_AXIS]);
  } 
  if(strcmp(page_stack[page_stack_index],"Vmax E:")==0)    
  {
    Setting.max_feedrate[E_AXIS] = (int)value_turn(0,0xFFFF,1,Setting.max_feedrate[E_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Vmax E:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_feedrate[E_AXIS]);
  } 
  if(strcmp(page_stack[page_stack_index],"Vmin:")==0)    
  {
    Setting.min_feedrate = (int)value_turn(0,0xFFFF,1,Setting.min_feedrate);
    LCD_Clear();
    Show_String(1,1,"Vmin:");
    Show_Int_right(1,19,"%5d",(int)Setting.min_feedrate);
  }
  if(strcmp(page_stack[page_stack_index],"VTrav min:")==0)    
  {
    Setting.min_travel_feedrate = (int)value_turn(0,0xFFFF,1,Setting.min_travel_feedrate);
    LCD_Clear();
    Show_String(1,1,"VTrav min:");
    Show_Int_right(1,19,"%5d",(int)Setting.min_travel_feedrate);
  }  
  if(strcmp(page_stack[page_stack_index],"Amax X:")==0)    
  {
    Setting.max_acceleration[X_AXIS] = (int)value_turn(0,0xFFFF,1,Setting.max_acceleration[X_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Amax X:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_acceleration[X_AXIS]);
  }   
   if(strcmp(page_stack[page_stack_index],"Amax Y:")==0)    
  {
    Setting.max_acceleration[Y_AXIS] = (int)value_turn(0,0xFFFF,1,Setting.max_acceleration[Y_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Amax Y:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_acceleration[Y_AXIS]);
  } 
  if(strcmp(page_stack[page_stack_index],"Amax Z:")==0)    
  {
    Setting.max_acceleration[Z_AXIS] = (int)value_turn(0,0xFFFF,1,Setting.max_acceleration[Z_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Amax Z:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_acceleration[Z_AXIS]);    
  } 
  if(strcmp(page_stack[page_stack_index],"Amax E:")==0)    
  {
    Setting.max_acceleration[E_AXIS] = (int)value_turn(0,0xFFFF,1,Setting.max_acceleration[E_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Amax E:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_acceleration[E_AXIS]);    
  }  
  if(strcmp(page_stack[page_stack_index],"A-retract:")==0)    
  {
    Setting.retract_acceleration = (int)value_turn(0,0xFFFF,1,Setting.retract_acceleration);
    LCD_Clear();
    Show_String(1,1,"A-retract:");
    Show_Int_right(1,19,"%5d",(int)Setting.retract_acceleration);   
  }
  if(strcmp(page_stack[page_stack_index],"Xsteps/mm:")==0)    
  {
    Setting.steps_per_mm[X_AXIS] = value_turn(0,0xFFFF,0.1,Setting.steps_per_mm[X_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Xsteps/mm:");
    Show_Float_right(1,19,"%0.2f",Setting.steps_per_mm[X_AXIS]);   
  }   
   if(strcmp(page_stack[page_stack_index],"Ysteps/mm:")==0)    
  {
    Setting.steps_per_mm[Y_AXIS] = value_turn(0,0xFFFF,0.1,Setting.steps_per_mm[Y_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Ysteps/mm:");
    Show_Float_right(1,19,"%0.2f",Setting.steps_per_mm[Y_AXIS]);   
  } 
  if(strcmp(page_stack[page_stack_index],"Zsteps/mm:")==0)    
  {
    Setting.steps_per_mm[Z_AXIS] = value_turn(0,0xFFFF,0.1,Setting.steps_per_mm[Z_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Zsteps/mm:");
    Show_Float_right(1,19,"%0.2f",Setting.steps_per_mm[Z_AXIS]);    
  } 
  if(strcmp(page_stack[page_stack_index],"Esteps/mm:")==0)    
  {
    Setting.steps_per_mm[E_AXIS] = value_turn(0,0xFFFF,0.1,Setting.steps_per_mm[E_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Esteps/mm:");
    Show_Float_right(1,19,"%0.2f",Setting.steps_per_mm[E_AXIS]); 
  }
  if(strcmp(page_stack[page_stack_index],"Home speed X:")==0)    
  {
    Setting.home_speed[X_AXIS] = value_turn(0,0xFFFF,1,Setting.home_speed[X_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Home speed X:");
    Show_Int_right(1,19,"%5d",Setting.home_speed[X_AXIS]); 
  }
  if(strcmp(page_stack[page_stack_index],"Home speed Y:")==0)    
  {
    Setting.home_speed[Y_AXIS] = value_turn(0,0xFFFF,1,Setting.home_speed[Y_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Home speed Y:");
    Show_Int_right(1,19,"%5d",Setting.home_speed[Y_AXIS]); 
  }
  if(strcmp(page_stack[page_stack_index],"Home speed Z:")==0)    
  {
    Setting.home_speed[Z_AXIS] = value_turn(0,0xFFFF,1,Setting.home_speed[Z_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Home speed Z:");
    Show_Int_right(1,19,"%5d",Setting.home_speed[Z_AXIS]); 
  }
  //Printer Settings
  if(strcmp(page_stack[page_stack_index],"X Max:")==0)    
  {
    Setting.max_position[X_AXIS] = (int)value_turn(Setting.min_position[X_AXIS],0xFFFF,1,Setting.max_position[X_AXIS]);
    LCD_Clear();
    Show_String(1,1,"X Max:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_position[X_AXIS]);    
  }
  if(strcmp(page_stack[page_stack_index],"X Min:")==0)    
  {
    Setting.min_position[X_AXIS] = (int)value_turn(0,Setting.max_position[X_AXIS],1,Setting.min_position[X_AXIS]);
    LCD_Clear();
    Show_String(1,1,"X Min:");
    Show_Int_right(1,19,"%5d",(int)Setting.min_position[X_AXIS]);  
  }
  if(strcmp(page_stack[page_stack_index],"Y Max:")==0)    
  {
    Setting.max_position[Y_AXIS] = (int)value_turn(Setting.min_position[Y_AXIS],0xFFFF,1,Setting.max_position[Y_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Y Max:");
    Show_Int_right(1,19,"%5d",(int)Setting.max_position[Y_AXIS]);
     
  }
  if(strcmp(page_stack[page_stack_index],"Y Min:")==0)    
  {
    Setting.min_position[Y_AXIS] = (int)value_turn(0,Setting.max_position[Y_AXIS],1,Setting.min_position[Y_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Y Min:");
    Show_Int_right(1,19,"%5d",(int)Setting.min_position[Y_AXIS]);
  }  
  if(strcmp(page_stack[page_stack_index],"Z Max:")==0)    
  {
    Setting.max_position[Z_AXIS] = (int)value_turn(Setting.min_position[Z_AXIS],0xFFFF,1,Setting.max_position[Z_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Z Max:");
    Show_Float_right(1,19,"%0.2f",(int)Setting.max_position[Z_AXIS]);
  }     
  if(strcmp(page_stack[page_stack_index],"Z Min:")==0)    
  {
    Setting.min_position[Z_AXIS] = (int)value_turn(0,Setting.max_position[Z_AXIS],1,Setting.min_position[Z_AXIS]);
    LCD_Clear();
    Show_String(1,1,"Z Min:");
    Show_Int_right(1,19,"%5d",(int)Setting.min_position[Z_AXIS]);
  }    
  
  if(strcmp(page_stack[page_stack_index],"Version")==0)    
  {
    LCD_Clear();

	Show_String(0,0,MACHINE_TYPE);
	Show_String(2,0,VERSION);
  }   
  
  //Delta Settings
#ifdef DELTA
  if(strcmp(page_stack[page_stack_index],"Segments/sec:")==0)   
  {
    Setting.delta_segments_per_sec = (float)value_turn(0,0xFFFF,1,Setting.delta_segments_per_sec);
    LCD_Clear();
    Show_String(1,1,"Segments/sec:");
    if(Setting.delta_segments_per_sec < 0) Show_Char(1,15,'-');
    Show_Int_right(1,19,"%4d",Setting.delta_segments_per_sec);
    Delta_Init();
  } 
  if(strcmp(page_stack[page_stack_index],"Diagonal Rod:")==0)   
  {
    Setting.delta_diagonal_rod = (float)value_turn(0,0xFFFF,1,Setting.delta_diagonal_rod);
    LCD_Clear();
    Show_String(1,1,"Diagonal Rod:");
    if(Setting.delta_diagonal_rod < 0) Show_Char(1,15,'-');
    Show_Int_right(1,19,"%4d",Setting.delta_diagonal_rod);
    Delta_Init();
  } 
  if(strcmp(page_stack[page_stack_index],"Smooth offs:")==0)   
  {
    Setting.delta_smooth_rod_offset = (float)value_turn(0,0xFFFF,1,Setting.delta_smooth_rod_offset);
    LCD_Clear();
    Show_String(1,1,"Smooth offs:");
    if(Setting.delta_smooth_rod_offset < 0) Show_Char(1,15,'-');
    Show_Int_right(1,19,"%4d",Setting.delta_smooth_rod_offset);
    Delta_Init();
  } 
  if(strcmp(page_stack[page_stack_index],"Effector offs:")==0)   
  {
    Setting.delta_effector_offset = (float)value_turn(0,0xFFFF,1,Setting.delta_effector_offset);
    LCD_Clear();
    Show_String(1,1,"Effector offs:");
    if(Setting.delta_effector_offset < 0) Show_Char(1,15,'-');
    Show_Int_right(1,19,"%4d",Setting.delta_effector_offset);
    Delta_Init();
  } 
  if(strcmp(page_stack[page_stack_index],"Carriage offs:")==0)   
  {
    Setting.delta_carriage_offset = (float)value_turn(0,0xFFFF,1,Setting.delta_carriage_offset);
    LCD_Clear();
    Show_String(1,1,"Carriage offs:");
    if(Setting.delta_carriage_offset < 0) Show_Char(1,15,'-');
    Show_Int_right(1,19,"%4d",Setting.delta_carriage_offset);
    Delta_Init();
  } 
  if(strcmp(page_stack[page_stack_index],"Delta Radius:")==0)   
  {
    Setting.delta_radius_error = value_turn(-65535,0xFFFF,0.1,Setting.delta_radius_error);    
    LCD_Clear();
    Delta_Init();
    Show_String(1,1,"Delta Radius:");
    if(Setting.delta_radius < 0) Show_Char(1,15,'-');   
    Show_Float_right(1,19,"%0.1f",Setting.delta_radius);
    
  } 
  
  if(strcmp(page_stack[page_stack_index],"Print Radius:")==0)   
  {
    Setting.delta_printable_radius = (float)value_turn(0,0xFFFF,1,Setting.delta_printable_radius);
    LCD_Clear();
    Show_String(1,1,"Print Radius:");
    if(Setting.delta_printable_radius < 0) Show_Char(1,15,'-');
    Show_Int_right(1,19,"%4d",Setting.delta_printable_radius);
    Delta_Init();
  } 
  
#endif
  //End Delta settings
}

float value_turn(float min, float max,float rate,float value)
{
    if((decoder.decoder_turn_flag == 1)&&(decoder.decoder_dir==1))
    {     
      if(value >= max) value = max;
      else value = value + rate;
    }
    if((decoder.decoder_turn_flag == 1)&&(decoder.decoder_dir==0))
    {
      if(value <= min) value = min;
      else value = value - rate;
    }
    decoder.decoder_turn_flag = 0;
    decoder.pointer_index = 0;
    decoder.decoder_dir = 0xff;
    return value;
}

void Encoder_IRQ(void)
{
  delay_us(100);
  if(EXTI_GetITStatus(EXTI_PRESS)!=RESET)
  {       
      delay_ms(1);     
      if(READ_PRESS == 0)
      {
          delay_ms(20);  
          if(READ_PRESS == 0)
          {
              SET_BEEP(1000,127);
              delay_ms(2);
              SET_BEEP(1000,0);
              while(!(READ_PRESS));
              decoder_press();
              turn_back_count = 0;               
          }  
       }
    EXTI_ClearITPendingBit(EXTI_PRESS);
  }

 
  if(EXTI_GetITStatus(EXTI_EC1) != RESET)
  { 
    delay_us(100);   
    if(READ_EC2 == 1)
    {       
      delay_us(200);
      if((READ_EC2 == 1) && (READ_EC1 == 0))        
      {    
          decoder_inc();//test
          turn_back_count=0;
      }
      else if((READ_EC2 == 1) && (READ_EC1 == 1))  
      {
          decoder_dec();//test
          turn_back_count=0;          
      }
    //delay_ms(10);         
    }
    EXTI_ClearITPendingBit(EXTI_EC1);
  }
 
}



#ifdef  RECOVERY_MODE_USE
u8 Recovery_select(void)
{

  u8 i = 3;
  LCD_Clear();
  Show_String(0,0,"An interrupted printjob is detected, do you want to resume?");
  Show_String(3,0,"    Yes     No      ");
  Show_Char(3,3,'>');
  while(1)
  {
    if(decoder.decoder_turn_flag == 1)
    {
      if(i==3) i=11;
      else i=3;
      Show_String(3,0,"    Yes     No      ");
      Show_Char(3,i,'>');
      decoder.decoder_turn_flag = 0;
    }
    if(decoder.key_press)
    {
      decoder.key_press = 0;
      if(i==3) return ENABLE;
      else return DISABLE;   
    }
  }
}

u8 Recovery_error(void)
{

  u8 i = 2;
  LCD_Clear();
  Show_String(0,0,"Can't find the Gcode,please check the SD card?");
  Show_String(3,0,"   Continue   Stop  ");
  Show_Char(3,2,'>');
  while(1)
  {
    if(decoder.decoder_turn_flag == 1)
    {
      if(i==2) i=13;
      else i=2;
      Show_String(3,0,"   Continue   Stop  ");
      Show_Char(3,i,'>');
      decoder.decoder_turn_flag = 0;
    }
    if(decoder.key_press)
    {
      decoder.key_press = 0;
      LCD_Clear();
      if(i==2) return ENABLE;
      else return DISABLE;   
    }
  }
}
#endif




#endif
void Encoder_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;       
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOE, ENABLE);
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);

}

void Encoder_EXTI_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure; 
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource9); 
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource13);  
    
    EXTI_InitStructure.EXTI_Line= EXTI_EC1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);       
          

    EXTI_InitStructure.EXTI_Line = EXTI_PRESS;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = DISABLE;
    EXTI_Init(&EXTI_InitStructure);  
  
  
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn ;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);
    
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
    
}

void Enable_Encoder_EXTI(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;   
    
    EXTI_InitStructure.EXTI_Line = EXTI_EC1;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);  
    
    EXTI_InitStructure.EXTI_Line = EXTI_PRESS;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;   
    EXTI_Init(&EXTI_InitStructure);  

}




void Display_Update_Timer_Config(void)
{
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    
    TIM_TimeBaseStructure.TIM_Period = 9;//999;   //9999
    TIM_TimeBaseStructure.TIM_Prescaler = 720-1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); 
    
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);  
    TIM_Cmd(TIM5, DISABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_Cmd(TIM5, ENABLE);
}


u8 Recovery_error(void)
{

  u8 i = 2;
  while(1)
  {
    if(decoder.decoder_turn_flag == 1)
    {
      if(i==2) i=13;
      else i=2;
      decoder.decoder_turn_flag = 0;
    }
    if(decoder.key_press)
    {
      decoder.key_press = 0;
      if(i==2) return ENABLE;
      else return DISABLE;   
    }
  }
}

