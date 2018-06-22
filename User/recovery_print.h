#ifndef __RECOVERY_PRINT_H__
#define __RECOVERY_PRINT_H__

#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "XZK_Configuration.h"


#include "sd_print.h"   //20160628

typedef struct
{
	DWORD Current_sd_byte; 
	float Current_feed_rate;
	float Current_point_x;
	float Current_point_y;
	float Current_point_z;
	float Current_point_e;
	float bed_calulate_Current_point_x;
	float bed_calulate_Current_point_y;
	float bed_calulate_Current_point_z;
	s32  Current_position_block[3];
}RecoveryParameterType;



  void Recovery_GPIO_Config(void);
  void Recovery_save(void);
  void Recovery_sdprint(void);
  u8 Recovery_detect(void);
  void Recovery_process(void);
  void Recovery_create(void);
  void Recovery_remove(void);
  u8 Recovery_file_search(u8 *path, float file_size);
  u8 file_search(u8 *path, float file_size);
void get_file_disname(void);
#endif