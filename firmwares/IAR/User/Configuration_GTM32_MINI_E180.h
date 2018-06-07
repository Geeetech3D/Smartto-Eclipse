#ifndef __Configuration_GinatArm_S200_H__
#define __Configuration_GinatArm_S200_H__
#include "Configuration_Select_Printer.h"





#define DEVICE_NAME "GEEETECH-E180"
#define MACHINE_TYPE "E180"
#define MACHINE_INFO  "FIRMWARE_NAME:"##VERSION##" PROTOCOL_VERSION:V1.0 MACHINE_TYPE:"##MACHINE_TYPE##" EXTRUDER_COUNT:1 "
#define WIFI_MODULE               1    //wifi module
#define F_CPU 72                            //MHz
#define HOTHEAD_NUMBER 5          //Extrusion head number 
//Compiling different printers through macro definition




#define NOZZLE_PT3_25E2       // Thermistor type
#define HOTBED       1   //ENABLE  
#define HOTHEAD_1 1   // ENABLE 
#define HOTHEAD_2 1   // ENABLE 
#define HOTHEAD_3 0   ////
#define HOTHEAD_4 0   //
#define TEMPERATURE_SAMPLING_PERIOD 3    //Temperature sampling period


#define TEMPERATURE_SENSOR 1   //B3950
#define TEMPERATURE_ADJUST_RANGE  10   //PID  Adjustment range
#define TRAGE_TEMPERATURE_BED  70.0   //dxc   before is 60.0


#define MIN_TEMPERATURE_BED    0.0// Hot bed temperature minimum
#define MIN_TEMPERATURE_NO_1  0.0//  Extruder minimum temperature
#define MIN_TEMPERATURE_NO_2  0.0//
#define MIN_TEMPERATURE_NO_3  0.0//
#define MIN_TEMPERATURE_NO_4  0.0//

#define MAX_TEMPERATURE_BED   120.0
#define MAX_TEMPERATURE_NO_1  240.0 //Extrusion head maximum temperature
#define MAX_TEMPERATURE_NO_2  270.0//230.0
#define MAX_TEMPERATURE_NO_3  270.0//230.0
#define MAX_TEMPERATURE_NO_4  270.0//230.0


/*
Extrusion head and hot bed temperature are adjusted by PID algorithm
The PID parameters can be modified by commands ."M301 H0 P1600.045288,M301 H0 P2011.045288"
*/
//PID  parameter
#define KP_BED  369.610
#define KP_NO_1 1600.045288//M301 H0 P1600.045288,M301 H0 P2011.045288
#define KP_NO_2 608.045288//170.012
#define KP_NO_3 608.045288//170.012
#define KP_NO_4 170.012
#define KI_BED 54.132
#define KI_NO_1 105.630539//M301 H0 I21.21.179
#define KI_NO_2 36.630539//18.665
#define KI_NO_3 36.630539//18.665
#define KI_NO_4 18.665
#define KD_BED 602.870
#define KD_NO_1 1109.194824//M301 H0 D3009.194824,M301 H0 D9509.194824
#define KD_NO_2 2709.194824//389.381
#define KD_NO_3 2709.194824//389.381
#define KD_NO_4 389.381


//The maximum and minimum values of the XYZ axis.
#define X_MIN_POSITION	            0.0
#define Y_MIN_POSITION	            0.0
#define Z_MIN_POSITION	            0.0
#define X_MAX_POSITION             130.0
#define Y_MAX_POSITION             130.0
#define Z_MAX_POSITION             132.0 
//Printer's print range
#define X_MAX_LENGTH (X_MAX_POSITION - X_MIN_POSITION)
#define Y_MAX_LENGTH (Y_MAX_POSITION - Y_MIN_POSITION)
#define Z_MAX_LENGTH (Z_MAX_POSITION - Z_MIN_POSITION)


//The direction of home,Limit switch position
#define HOMEX                   MINENDSTOP      
#define HOMEY                   MAXENDSTOP         
#define HOMEZ                   MAXENDSTOP
//Motor movement direction
#define DIR_X                   OPPOSITE//OPPOSITE
#define DIR_Y                   POSITIVE///////////////////////////////////////////////////////
#define DIR_Z                   POSITIVE  
#define DIR_E                   POSITIVE             ///////////////////2016.6.6 mark
#define DIR_E1                  OPPOSITE
#define DIR_E2                  POSITIVE


//Limit switch polarity setting
#define X_MIN_ENDSTOP_LEVEL    LOW  //LOW  //HIGH
#define X_MAX_ENDSTOP_LEVEL    LOW
#define Y_MIN_ENDSTOP_LEVEL    LOW
#define Y_MAX_ENDSTOP_LEVEL    LOW
#define Z_MIN_ENDSTOP_LEVEL    LOW
#define Z_MAX_ENDSTOP_LEVEL    LOW

#define X_MIN_ENDSTOP_STATUS    OPEN    //OPEN  CLOSE
#define X_MAX_ENDSTOP_STATUS    OPEN
#define Y_MIN_ENDSTOP_STATUS    OPEN
#define Y_MAX_ENDSTOP_STATUS    OPEN
#define Z_MIN_ENDSTOP_STATUS    OPEN
#define Z_MAX_ENDSTOP_STATUS    OPEN



#define NUM_AXIS                  4
#define EXT_NUM 3

#define EXTRUDE_MAXLENGTH         10.0


//Stepper motor steps per millimeter
#define STEPS_PER_mm_FOR_X       80
#define STEPS_PER_mm_FOR_Y       80
#define STEPS_PER_mm_FOR_Z       400.000
#define STEPS_PER_mm_FOR_E       88


//Stepper motor maximum speed
#define MAX_FEEDRAT_FOR_X       550 
#define MAX_FEEDRAT_FOR_Y       550
#define MAX_FEEDRAT_FOR_Z        35 

//Stepping motor homing speed
#define HOME_SPEED_X   300 
#define HOME_SPEED_Y   300
#define HOME_SPEED_Z   60



//E axis Stepper motor maximum speed
#define MAX_TRAVEL_FEEDRATE       50//mm/s 
#define MIN_TRAVEL_FEEDRATE       5 //mm/s 

//Minimum speed
#define MIN_FEEDRATE              5
#define EXTRUDE_MULTIPLY          100 //
#define EXTRUDE_MIN_TEMP          170 //
#define MIN_SEGMENT_STEPS         8//
#define MIN_SEGMENT_TIME          80000 //
#define ACCELERATION_CHANGE_DISTANCE 5//mm

//Each axis acceleration
#define RETRACT_ACCELERATION      1000//3000 //
#define ACCELERATION              1000 //mm/s^2
  
#define AXIS_ACCELERATION_FOR_X     5000   //5000//800//5000//Xmm/s^2    //2016.6.3
#define AXIS_ACCELERATION_FOR_Y     5000  //5000//800//5000 //Ymm/s^2    //2016.6.3
#define AXIS_ACCELERATION_FOR_Z     50 //50//Zmm/s^2
#define AXIS_ACCELERATION_FOR_E     5000 //5000//800 //5000//Emm/s^2    //2016.6.3 


#define MAX_X_JERK  10//20//mm/s     //2016.6.3
#define MAX_Y_JERK  10//20//mm/s     //2016.6.3
#define MAX_Z_JERK  0.5//mm/S
#define MAX_E_JERK  10////mm/S   //dxc   defore is 20
#define MIN_MIXER_OFP   0
#define MAX_MIXER_OFP   (100-MIN_MIXER_OFP)


#endif
