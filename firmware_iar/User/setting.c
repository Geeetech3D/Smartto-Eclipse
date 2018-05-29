#include "Setting.h"
#include "stmflash.h"
#include "step_motor.h"
#include "String.h"
#include "Dgus_screen.h"
#include "variable.h"

#define STORE_LEN 127	

#define SETTINGS_FLAG 0xa5aa

setting_t Setting;
setting_s Settings;

static void Init_Defaults_Setting(void)
{
    float Kid=0.0;
    Kid=(TEMPERATURE_SAMPLING_PERIOD)/10.0;
	
    Setting.hotend_num = HOTHEAD_NUMBER;       
    Setting.hotend_enable[BED] = (FunctionalState)HOTBED;
    Setting.hotend_enable[NOZZLE0] = (FunctionalState)HOTHEAD_1;
    Setting.hotend_enable[NOZZLE1] = (FunctionalState)HOTHEAD_2;
    Setting.hotend_enable[NOZZLE2] = (FunctionalState)HOTHEAD_3;
    
    Setting.temperature_sampling_period = TEMPERATURE_SAMPLING_PERIOD;

    Setting.min_temperature[BED] = MIN_TEMPERATURE_BED;
    Setting.min_temperature[NOZZLE0] = MIN_TEMPERATURE_NO_1;
    Setting.min_temperature[NOZZLE1] = MIN_TEMPERATURE_NO_2;
    Setting.min_temperature[NOZZLE2] = MIN_TEMPERATURE_NO_3;
    
    Setting.max_temperature[BED] = MAX_TEMPERATURE_BED;
    Setting.max_temperature[NOZZLE0] = MAX_TEMPERATURE_NO_1;
    Setting.max_temperature[NOZZLE1] = MAX_TEMPERATURE_NO_2;
    Setting.max_temperature[NOZZLE2] = MAX_TEMPERATURE_NO_3;

    	Setting.Kp[BED] = KP_BED;
    Setting.Kp[NOZZLE0] = KP_NO_1;
          
    Setting.Ki[BED] = KI_BED*Kid;
    Setting.Ki[NOZZLE0] = KI_NO_1*Kid;
         
    Setting.Kd[BED] = KD_BED/Kid;
    Setting.Kd[NOZZLE0] = KD_NO_1/Kid;
    Setting.Kp[NOZZLE1] = KP_NO_2;
    Setting.Kp[NOZZLE2] = KP_NO_3;
    
    Setting.Ki[NOZZLE1] = KI_NO_2*Kid;
    Setting.Ki[NOZZLE2] = KI_NO_3*Kid;
    
    Setting.Kd[NOZZLE1] = KD_NO_2/Kid;
    Setting.Kd[NOZZLE2] = KD_NO_3/Kid;


    	

           
    Setting.extrude_multiply = EXTRUDE_MULTIPLY;
    Setting.min_segment_steps = MIN_SEGMENT_STEPS;
    Setting.min_segment_time = MIN_SEGMENT_TIME;
    
    Setting.pid_adjust_range = TEMPERATURE_ADJUST_RANGE;
  
	Setting.hotend_enable[NOZZLE0] = ENABLE;
	Setting.hotend_enable[NOZZLE1] = ENABLE;
	Setting.hotend_enable[NOZZLE2] = ENABLE;
	Setting.hotend_enable[BED] = ENABLE;
	Setting.targe_temperature[NOZZLE0]=0;
	Setting.targe_temperature[NOZZLE1]=0;
	Setting.targe_temperature[NOZZLE2]=0;
	Setting.targe_temperature[BED]=0;
	Setting.locate_mode = ABSOLUTE_COORDINATE;

	Setting.max_acceleration[X_AXIS] = AXIS_ACCELERATION_FOR_X;
      Setting.max_acceleration[Y_AXIS] = AXIS_ACCELERATION_FOR_Y;
      Setting.max_acceleration[Z_AXIS] = AXIS_ACCELERATION_FOR_Z;
     Setting.max_acceleration[E_AXIS] = AXIS_ACCELERATION_FOR_E;  

         Setting.max_jerk = MAX(MAX_X_JERK,MAX(MAX_Y_JERK,MAX(MAX_Z_JERK,MAX_E_JERK)));
    Setting.max_x_jerk = MAX_X_JERK;
    Setting.max_y_jerk = MAX_Y_JERK;
    Setting.max_z_jerk = MAX_Z_JERK;
    Setting.max_e_jerk = MAX_E_JERK;  

	  Setting.home_speed[X_AXIS] = HOME_SPEED_X;
    Setting.home_speed[Y_AXIS] = HOME_SPEED_Y;
    Setting.home_speed[Z_AXIS] = HOME_SPEED_Z;

    Setting.home_position[X_AXIS] = HOMEX;
    Setting.home_position[Y_AXIS] = HOMEY;
    Setting.home_position[Z_AXIS] = HOMEZ;

     Setting.min_travel_feedrate = MIN_TRAVEL_FEEDRATE;
    Setting.min_feedrate = MIN_FEEDRATE;
    Setting.retract_acceleration = RETRACT_ACCELERATION;
    Setting.acceleration = ACCELERATION;
    
    Setting.axis_steps_per_sqr_second[X_AXIS] = (unsigned long)ceil(AXIS_ACCELERATION_FOR_X*Setting.steps_per_mm[X_AXIS]);
    Setting.axis_steps_per_sqr_second[Y_AXIS] = (unsigned long)ceil(AXIS_ACCELERATION_FOR_Y*Setting.steps_per_mm[Y_AXIS]);
    Setting.axis_steps_per_sqr_second[Z_AXIS] = (unsigned long)ceil(AXIS_ACCELERATION_FOR_Z*Setting.steps_per_mm[Z_AXIS]);
    Setting.axis_steps_per_sqr_second[E_AXIS] = (unsigned long)ceil(AXIS_ACCELERATION_FOR_E*Setting.steps_per_mm[E_AXIS]);

        Setting.endstop_level[X_MIN] = X_MIN_ENDSTOP_LEVEL;
    Setting.endstop_level[X_MAX] = X_MAX_ENDSTOP_LEVEL;
    Setting.endstop_level[Y_MIN] = Y_MIN_ENDSTOP_LEVEL;
    Setting.endstop_level[Y_MAX] = Y_MAX_ENDSTOP_LEVEL;
    Setting.endstop_level[Z_MIN] = Z_MIN_ENDSTOP_LEVEL;
    Setting.endstop_level[Z_MAX] = Z_MAX_ENDSTOP_LEVEL;
    Setting.endstop_status[X_MIN] = X_MIN_ENDSTOP_STATUS;
    Setting.endstop_status[X_MAX] = X_MAX_ENDSTOP_STATUS;
    Setting.endstop_status[Y_MIN] = Y_MIN_ENDSTOP_STATUS;
    Setting.endstop_status[Y_MAX] = Y_MAX_ENDSTOP_STATUS;
    Setting.endstop_status[Z_MIN] = Z_MIN_ENDSTOP_STATUS;
    Setting.endstop_status[Z_MAX] = Z_MAX_ENDSTOP_STATUS;
    
	    Setting.plan_bed_level_matrix.matrix[0]=1.0;
    Setting.plan_bed_level_matrix.matrix[1]=0.0;
    Setting.plan_bed_level_matrix.matrix[2]=0.0;
    Setting.plan_bed_level_matrix.matrix[3]=0.0;
    Setting.plan_bed_level_matrix.matrix[4]=1.0;
    Setting.plan_bed_level_matrix.matrix[5]=0.0;
    Setting.plan_bed_level_matrix.matrix[6]=0.0;
    Setting.plan_bed_level_matrix.matrix[7]=0.0;
    Setting.plan_bed_level_matrix.matrix[8]=1.0;
	
}

static void Get_Base_Setting(void)
{
	//float Kid=0.0;

    //Kid=(TEMPERATURE_SAMPLING_PERIOD)/10.0;

    
 
    
    Setting.zz_offset = 0.0;
    Setting.min_position[X_AXIS] = X_MIN_POSITION;
    Setting.max_position[X_AXIS] = X_MAX_POSITION;
    Setting.min_position[Y_AXIS] = Y_MIN_POSITION;
    Setting.max_position[Y_AXIS] = Y_MAX_POSITION;
    Setting.min_position[Z_AXIS] = Z_MIN_POSITION;
    Setting.max_position[Z_AXIS] = Z_MAX_POSITION;
    
    Setting.steps_per_mm[X_AXIS] = STEPS_PER_mm_FOR_X;
    Setting.steps_per_mm[Y_AXIS] = STEPS_PER_mm_FOR_Y;
    Setting.steps_per_mm[Z_AXIS] = STEPS_PER_mm_FOR_Z;
    Setting.steps_per_mm[E_AXIS] = STEPS_PER_mm_FOR_E;
    
    Setting.max_feedrate[X_AXIS] = MAX_FEEDRAT_FOR_X;
    Setting.max_feedrate[Y_AXIS] = MAX_FEEDRAT_FOR_Y;
    Setting.max_feedrate[Z_AXIS] = MAX_FEEDRAT_FOR_Z;
    Setting.max_feedrate[E_AXIS] = MAX_TRAVEL_FEEDRATE;

    Setting.motor_direction[X_AXIS] = DIR_X;
    Setting.motor_direction[Y_AXIS] = DIR_Y;
    Setting.motor_direction[Z_AXIS] = DIR_Z;
    Setting.motor_direction[E_AXIS] = DIR_E;
    Setting.motor_direction[E1_AXIS] = DIR_E1;
    Setting.motor_direction[E2_AXIS] = DIR_E2;
    #ifdef DELTA
        Setting.delta_segments_per_sec = DELTA_SEGMENTS_PER_SECOND;
        Setting.delta_diagonal_rod = DELTA_DIAGONAL_ROD;
        Setting.delta_smooth_rod_offset = DELTA_SMOOTH_ROD_OFFSET;
        Setting.delta_effector_offset = DELTA_EFFECTOR_OFFSET;
        Setting.delta_carriage_offset = DELTA_CARRIAGE_OFFSET;
        Setting.delta_radius = DELTA_RADIUS;
        Setting.delta_radius_error = RADIUS_ERROR;
        Setting.delta_printable_radius = DELTA_PRINTABLE_RADIUS;
		Setting.endstop_adj[X_AXIS] = 0.0;
		Setting.endstop_adj[Y_AXIS] = 0.0;
		Setting.endstop_adj[Z_AXIS] = 0.0;
         Setting.mixer_ofp_min = MIN_MIXER_OFP;
        Setting.mixer_ofp_max = MAX_MIXER_OFP;
#endif
#ifdef BOARD_A30_MINI_S
    Setting.zprobe_zoffset = -(Z_PROBE_OFFSET_FROM_EXTRUDER);
#endif

}



void Get_Printer_Factory_Settings(void)
{
	 //u16 store_flag=0;    
         Get_Base_Setting();
	 STMFLASH_Write(USER_SETTINGS_PARAMETERS_ADDR, (u16*)(&Setting), 128);//restore factory setting

}
/**********************************************************
***Function:     Get_Printer_User_Settings
***Description: The initialization 3D printer parameter
***Input:  
***Output: 
***Return:
***********************************************************/
void Get_Printer_User_Settings(void)
{
    u16 store_flag=0;
    STMFLASH_Read(USER_SETTINGS_STORE_FLAG_ADDR, &store_flag, 1);
    if(store_flag != SETTINGS_FLAG)
    {    
        Get_Printer_Factory_Settings();//restore factory setting
        store_flag=SETTINGS_FLAG;
        STMFLASH_Write(USER_SETTINGS_STORE_FLAG_ADDR, &store_flag, 1);
    }
    else
    {
        STMFLASH_Read(USER_SETTINGS_PARAMETERS_ADDR, (u16*)(&Setting),  128);
    }

    Init_Defaults_Setting();
    Get_SerialNumber();
    Get_Hardware_Version();
}





void Store_Memory(SettingMode mode)
{
	u16 store_flag = 0;
        STMFLASH_Write(USER_SETTINGS_PARAMETERS_ADDR, (u16*)(&Setting),  128);
	store_flag = (u16)SETTINGS_FLAG;
	STMFLASH_Write(USER_SETTINGS_STORE_FLAG_ADDR, &store_flag, 1);    
}

void Restore_Defaults(SettingMode mode)
{
	u16 resume = 0;
       resume=0xFFFF;
       STMFLASH_Write(USER_SETTINGS_STORE_FLAG_ADDR, &resume, 1);
       Get_Printer_User_Settings();
}

void Store_SerialNumber(void)
{	
	char SN_data[16];
	memcpy(SN_data, Setting.SN, sizeof(SN_data));
	STMFLASH_Write(SERIAL_NUMBER_ADDR, (u16*)SN_data, sizeof(SN_data)/2);
}

void Get_SerialNumber(void)
{
	char SN_data[16];
	STMFLASH_Read(SERIAL_NUMBER_ADDR, (u16*)SN_data, sizeof(SN_data)/2);
	memcpy(Setting.SN, SN_data, sizeof(SN_data));
}

void Store_Hardware_Version(void)
{	
	char HV_data[6];
	memcpy(HV_data, Setting.HV, sizeof(HV_data));
	STMFLASH_Write(HARDWARE_VERSION_ADDR, (u16*)HV_data, sizeof(HV_data)/2);
}

void Get_Hardware_Version(void)
{
	char HV_data[6];
	STMFLASH_Read(HARDWARE_VERSION_ADDR, (u16*)HV_data, sizeof(HV_data)/2);
	memcpy(Setting.HV, HV_data, sizeof(HV_data));
}
void Store_AutoLevele_Flag(u8 flag)
{	
	STMFLASH_Write(AUTO_LEVELEFLAG_ADDR, (u16*)&flag, 1);
}
u8 Get_AutoLevele_Flag(void)
{
	u16 flag;
	STMFLASH_Read(AUTO_LEVELEFLAG_ADDR, (u16*)&flag, 1);
	return flag;
}
void Store_Filament_Dev_Flag(u8 flag)
{	
	STMFLASH_Write(AUTO_FILAMENT_DEV_ADDR, (u16*)&flag, 1);
}
u8 Get_Filament_Dev_Flag(void)
{
	u16 flag;
	STMFLASH_Read(AUTO_FILAMENT_DEV_ADDR, (u16*)&flag, 1);
	return flag;
}


