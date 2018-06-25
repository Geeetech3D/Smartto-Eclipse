

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
 * Geeetech's Smartto dual license offers users a protective and flexible way to maximize their innovation and creativity.  
 * Smartto aims to be applicable to as many control boards and configurations as possible.But to exclusively support Geeetech customers,we makes sure that the
 * releases here are stable and guaranted to work properly on all the printers and hardware sold by Geeetech. 
 * We encourage the community to be active and pursuing the spirits of sharing and mutual help. 
 * The GPL v2 license grants complete use of Smartto to common users. These users are not distributing proprietary modifications or derivatives of Smartto. 
 * If so then there is no need for them to acquire the legal protections of a commercial license.
 * For other users however, who want to use Smartto in their commercial products or have other requirements that are not compatible with the GPLv2, the GPLv2 is not  

 * applicable to them.Even if you want to do so then you must acquire written permission from Geeetech.
 * Only under written condition, Geeetech, the exclusive licensor of Smartto, offers Smartto commercial license to meet their needs. 
 * A Smartto commercial license gives customers legal permission to modify Smartto or incorporate it into their products without the obligation of sharing the final 

 * code under the 
 * GPL v2 license. 
 * Fees vary with the application and the scale of its use. For more detailed information, please contact the Geeetech marketing department directly.
 *  
 * Geeetech commits itself to promoting the open source spirit.
*/


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
  u8 Recovery_file_search(char *path, float file_size);
  u8 file_search(u8 *path, float file_size);
void get_file_disname(void);
#endif
