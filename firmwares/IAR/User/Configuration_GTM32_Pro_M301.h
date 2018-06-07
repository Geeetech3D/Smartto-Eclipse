#ifndef __Configuration_GinatArm_S200_H__
#define __Configuration_GinatArm_S200_H__
#include "Configuration_Select_Printer.h"




#define EXTRUDER_3IN1   //Three in and one out printer
#define LCD2004_USE  
#define DELTA
#define EXTRUDER_0		//Extruder 0 Motor
#define EXTRUDER_1		//Extruder 1 Motor
#define EXTRUDER_2		//Extruder 2 Motor
#define VERSION "V1.0.05" 
#define DEVICE_NAME "GEEETECH-M301"     //Device name
#define MACHINE_TYPE "Rostock301"
#define MACHINE_INFO  "FIRMWARE_NAME:"##VERSION##" PROTOCOL_VERSION:V1.0 MACHINE_TYPE:"##MACHINE_TYPE##" EXTRUDER_COUNT:3 UUID:00000000-0000-0000-0000-000000000000\nok\n"

#define F_CPU 72                         //MHz
#define HOTHEAD_NUMBER 5      //Extrusion head number 
//Compiling different printers through macro definition


#define NOZZLE_PT3_25E2       // Thermistor type
#define HOTBED       1   //ENABLE  //
#define HOTHEAD_1 1   // ENABLE //
#define HOTHEAD_2 1   // ENABLE //  // dxc 2015-03-10
#define HOTHEAD_3 0   ////
#define HOTHEAD_4 0   //

#define TEMPERATURE_SAMPLING_PERIOD 1


#define TEMPERATURE_SENSOR 1   //B3950
#define TEMPERATURE_ADJUST_RANGE  10   //PID  Adjustment range
#define TRAGE_TEMPERATURE_BED  70.0   //dxc   before is 60.0
#define MIN_TEMPERATURE_BED   0.0//  Extruder minimum temperature
#define MIN_TEMPERATURE_NO_1  0.0//  Extruder minimum temperature
#define MIN_TEMPERATURE_NO_2  0.0//
#define MIN_TEMPERATURE_NO_3  0.0//
#define MIN_TEMPERATURE_NO_4  0.0//


#define MAX_TEMPERATURE_BED   130.0
#define MAX_TEMPERATURE_NO_1  270.0



#define MAX_TEMPERATURE_NO_2  270.0//230.0
#define MAX_TEMPERATURE_NO_3  270.0//230.0
#define MAX_TEMPERATURE_NO_4  270.0//230.0


//PID  parameter
#define KP_BED  369.610
#define KP_NO_1 608.045288
#define KP_NO_2 608.045288//170.012
#define KP_NO_3 608.045288//170.012
#define KP_NO_4 170.012
#define KI_BED 54.132
#define KI_NO_1 36.630539
#define KI_NO_2 36.630539//18.665
#define KI_NO_3 36.630539//18.665
#define KI_NO_4 18.665
#define KD_BED 602.870
#define KD_NO_1 2709.194824
#define KD_NO_2 2709.194824//389.381
#define KD_NO_3 2709.194824//389.381
#define KD_NO_4 389.381


//Print range definition
#define PREHEAT_PLA_CONF_FAN         0
#define PREHEAT_PLA_CONF_NOZZLE      200
#define PREHEAT_PLA_CONF_BED         70
#define PREHEAT_ABS_CONF_FAN         0
#define PREHEAT_ABS_CONF_NOZZLE      240
#define PREHEAT_ABS_CONF_BED         95



    #define Z_OFFSET        0.0     
    #define X_MIN_POSITION	            (-85.0)
    #define X_MAX_POSITION             85.0
  
    #define Y_MIN_POSITION	            (-85.0)
    #define Y_MAX_POSITION             85.0
  
    #define Z_MIN_POSITION	            0.0
    #define Z_MAX_POSITION            229//228//230.0
    
  // Make delta curves from many straight lines (linear interpolation).
  // This is a trade-off between visible corners (not enough segments)
  // and processor overload (too many expensive sqrt calls).
  #define DELTA_SEGMENTS_PER_SECOND 200

  // NOTE NB all values for DELTA_* values MUST be floating point, so always have a decimal point in them

  // Center-to-center distance of the holes in the diagonal push rods.
  #define DELTA_DIAGONAL_ROD 196//208//198.5//200//196 // mm

  // Horizontal offset from middle of printer to smooth rod center.
  #define DELTA_SMOOTH_ROD_OFFSET 160 // mm

  // Horizontal offset of the universal joints on the end effector.
  #define DELTA_EFFECTOR_OFFSET 36 // mm

  // Horizontal offset of the universal joints on the carriages.
  #define DELTA_CARRIAGE_OFFSET 33 // mm

  // Horizontal distance bridged by diagonal push rods when effector is centered.
  #define DELTA_RADIUS (DELTA_SMOOTH_ROD_OFFSET-DELTA_EFFECTOR_OFFSET-DELTA_CARRIAGE_OFFSET+1)

  #define RADIUS_ERROR -5
  // Print surface diameter/2 minus unreachable space (avoid collisions with vertical towers).
  #define DELTA_PRINTABLE_RADIUS 90






#define X_MAX_LENGTH (X_MAX_POSITION - X_MIN_POSITION)
#define Y_MAX_LENGTH (Y_MAX_POSITION - Y_MIN_POSITION)
#define Z_MAX_LENGTH (Z_MAX_POSITION - Z_MIN_POSITION)




#define HOMEX                   MAXENDSTOP      //MAXENDSTOP
#define HOMEY                   MAXENDSTOP      //MAXENDSTOP
#define HOMEZ                   MAXENDSTOP      //MAXENDSTOP
#define DIR_X                   POSITIVE//OPPOSITE
#define DIR_Y                   POSITIVE
#define DIR_Z                   OPPOSITE
#define DIR_E                   OPPOSITE//POSITIVE
#define DIR_E1                 OPPOSITE// POSITIVE
#define DIR_E2                 OPPOSITE// POSITIVE





#define X_MIN_ENDSTOP_LEVEL    HIGH   //LOW
#define X_MAX_ENDSTOP_LEVEL    HIGH
#define Y_MIN_ENDSTOP_LEVEL    HIGH
#define Y_MAX_ENDSTOP_LEVEL    HIGH
#define Z_MIN_ENDSTOP_LEVEL    HIGH
#define Z_MAX_ENDSTOP_LEVEL    HIGH
#define X_MIN_ENDSTOP_STATUS    CLOSE    //OPEN
#define X_MAX_ENDSTOP_STATUS    CLOSE
#define Y_MIN_ENDSTOP_STATUS    CLOSE
#define Y_MAX_ENDSTOP_STATUS    CLOSE
#define Z_MIN_ENDSTOP_STATUS    CLOSE
#define Z_MAX_ENDSTOP_STATUS    CLOSE




#define NUM_AXIS                  4
#define EXT_NUM 3

#define EXTRUDE_MAXLENGTH         10.0




#define STEPS_PER_mm_FOR_X       80.2649//X-axis motor steps per millimeter  mm
#define STEPS_PER_mm_FOR_Y       80.2649//Y-axis motor steps per millimeter  /mm
#define STEPS_PER_mm_FOR_Z       80.2649//Z-axis motor steps per millimeter   mm
#define STEPS_PER_mm_FOR_E       95        //E-axis motor steps per millimeter   /mm

#define MAX_FEEDRAT_FOR_X         400//X-axis Maximum operating speed   mm/s 
#define MAX_FEEDRAT_FOR_Y         400//Y-axis Maximum operating speed   mm/s 

#define MAX_FEEDRAT_FOR_Z         400//35//Z-axis Maximum operating speed   mm/s      

#define HOME_SPEED_ALL 360

#define HOME_SPEED_X    HOME_SPEED_ALL   //X-Axis homing speed  mm/s 
#define HOME_SPEED_Y    HOME_SPEED_ALL   //Y-Axis homing speed  mm/s 
#define HOME_SPEED_Z    HOME_SPEED_ALL	//Z-Axis homing speed  mm/s 




#define MAX_TRAVEL_FEEDRATE       50//mm/s 
#define MIN_TRAVEL_FEEDRATE       5 //mm/s 

#define MIN_FEEDRATE              5
#define EXTRUDE_MULTIPLY          100 //
#define EXTRUDE_MIN_TEMP          170 //
#define MIN_SEGMENT_STEPS         8//
#define MIN_SEGMENT_TIME          80000 //

#define ACCELERATION_CHANGE_DISTANCE 5//mm



#define RETRACT_ACCELERATION      3000//Extruder acceleration   mm/s^2    
#define ACCELERATION              5000 //Spatial Acceleration  mm/s^2
  
#define AXIS_ACCELERATION_FOR_X     5000//800//5000//X-Axis  Extruder acceleration mm/s^2
#define AXIS_ACCELERATION_FOR_Y     5000//800//5000 //-Axis  Extruder acceleration mm/s^2
#define AXIS_ACCELERATION_FOR_Z     5000//800 //50//Z-Axis  Extruder acceleration mm/s^2
#define AXIS_ACCELERATION_FOR_E     800 //5000//E-Axis  Extruder acceleration  mm/s^2     
#define MAX_X_JERK  20//mm/s
#define MAX_Y_JERK  20//mm/s
#define MAX_Z_JERK  20//mm/S
#define MAX_E_JERK  5////mm/S   //dxc   defore is 20
#define MIN_MIXER_OFP   2
#define MAX_MIXER_OFP   (100-2*MIN_MIXER_OFP)



#endif
