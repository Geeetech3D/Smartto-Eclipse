#ifndef __Configuration_GinatArm_S200_H__
#define __Configuration_GinatArm_S200_H__

//////////////
#define DEVICE_NAME "GEEETECH-A30"
#define MACHINE_TYPE "A30"
#define VERSION     "V1.00.57" 
#define HARDWARE_VERSION "V1.00.02" 

#define MACHINE_INFO  "FIRMWARE_NAME:"##VERSION##" PROTOCOL_VERSION:V1.0 MACHINE_TYPE:"##MACHINE_TYPE##" EXTRUDER_COUNT:1 "


#define F_CPU 72//MHz

#define HOTHEAD_NUMBER 5
  
#define BOARD_GTM32_MINI_S    

#define EXTRUDER_DUAL  

#define NOZZLE_PT3_25E2

#define HOTBED       1//ENABLE  //
#define HOTHEAD_1 1// ENABLE //
#define HOTHEAD_2 1// ENABLE //  // dxc 2015-03-10
#define HOTHEAD_3 0////
#define HOTHEAD_4 0//

#define TEMPERATURE_SAMPLING_PERIOD 3    //


#define TEMPERATURE_SENSOR 1


#define TEMPERATURE_ADJUST_RANGE  10 //PID

//
#define TRAGE_TEMPERATURE_BED  70.0   //dxc   before is 60.0

#define TRAGE_TEMPERATURE_NO_1 200.0
#define TRAGE_TEMPERATURE_NO_2 200.0
#define TRAGE_TEMPERATURE_NO_3 200.0
#define TRAGE_TEMPERATURE_NO_4 200.0

#define MIN_TEMPERATURE_BED   0.0//45.0

#define MIN_TEMPERATURE_NO_1  0.0//170.0
#define MIN_TEMPERATURE_NO_2  0.0//170.0
#define MIN_TEMPERATURE_NO_3  0.0//170.0
#define MIN_TEMPERATURE_NO_4  0.0//170.0

#define MAX_TEMPERATURE_BED   120.0

#define MAX_TEMPERATURE_NO_1  250.0 //250.0//2017.05.09ÐÞÕý¼·³öÍ·¿ÉÉèÖÃµÄ×î´óÎÂ¶ÈÎª230
#define MAX_TEMPERATURE_NO_2  230.0//230.0
#define MAX_TEMPERATURE_NO_3  230.0//230.0
#define MAX_TEMPERATURE_NO_4  230.0//230.0

//PIDÏà¹ØÏµÊý
#define KP_BED  369.610

#define KP_NO_1 500.045288//M301 H0 P1600.045288,M301 H0 P2011.045288
#define KP_NO_2 608.045288//170.012
#define KP_NO_3 608.045288//170.012
#define KP_NO_4 170.012

#define KI_BED 54.132

#define KI_NO_1 11.66539 //M301 H0 I21.21.179
#define KI_NO_2 36.630539//18.665
#define KI_NO_3 36.630539//18.665
#define KI_NO_4 18.665

#define KD_BED 602.870

#define KD_NO_1 1800.1924 //M301 H0 D3009.194824,M301 H0 D9509.194824
#define KD_NO_2 2709.194824//389.381
#define KD_NO_3 2709.194824//389.381
#define KD_NO_4 389.381

#define PREHEAT_PLA_CONF_FAN         0
#define PREHEAT_PLA_CONF_NOZZLE      200
#define PREHEAT_PLA_CONF_BED         70

#define PREHEAT_ABS_CONF_FAN         0
#define PREHEAT_ABS_CONF_NOZZLE      240
#define PREHEAT_ABS_CONF_BED         95

#define MAX_MOVE_BUFFER_SIZE    32//×î´óµÄ»º³åÇø³ß´ç


#define MIN_END 1
#define MAX_END 0

#define TOUCH_END_SIGNAL  1
#define NOMINAL_END_SIGNAL  0


#define ORIGIN MIN_END//Ô­µã·½ÏòµÄÉè¶¨

#if(ORIGIN == MAX_END)

#define ORIGIN_DIR MAX_END
#define INVERSELY_ORIGIN_DIR MIN_END
#define X_ORIGIN_END_SIGNAL Max_X
#define Y_ORIGIN_END_SIGNAL Max_Y
#define Z_ORIGIN_END_SIGNAL Max_Z

#else
#define ORIGIN_DIR MIN_END
#define INVERSELY_ORIGIN_DIR MAX_END
#define X_ORIGIN_END_SIGNAL Min_X
#define Y_ORIGIN_END_SIGNAL Min_Y
#define Z_ORIGIN_END_SIGNAL Min_Z

#endif

//´òÓ¡·¶Î§µÄ¶¨Òå

#define Z_OFFSET        0.0            

#define X_MIN_POSITION	            0.0
#define X_MAX_POSITION             320.0

#define Y_MIN_POSITION	            0.0
#define Y_MAX_POSITION             320.0

#define Z_MIN_POSITION	            0.0//0.0
#define Z_MAX_POSITION             420.0 //159.0
#define X_MAX_LENGTH (X_MAX_POSITION - X_MIN_POSITION)
#define Y_MAX_LENGTH (Y_MAX_POSITION - Y_MIN_POSITION)
#define Z_MAX_LENGTH (Z_MAX_POSITION - Z_MIN_POSITION)

#define HOMEX                   MINENDSTOP  //MINENDSTOP      //MAXENDSTOP
#define HOMEY                   MINENDSTOP      //MINENDSTOP      //MAXENDSTOP//////////////////////////////
#define HOMEZ                   MINENDSTOP   // MAXENDSTOP

#define DIR_X                   POSITIVE//OPPOSITE//OPPOSITE
#define DIR_Y                   POSITIVE///////////////////////////////////////////////////////
#define DIR_Z                   POSITIVE  
#define DIR_E                   POSITIVE             ///////////////////2016.6.6 mark
#define DIR_E1                  OPPOSITE
#define DIR_E2                  POSITIVE

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

#define STEPS_PER_mm_FOR_X       80//44.1503//79.379761////
#define STEPS_PER_mm_FOR_Y       80//44.1503// 79.760479// //
#define STEPS_PER_mm_FOR_Z       400.000//2560.000//2560.000//400.000//640.0000 //4000.000000//Zmm
#define STEPS_PER_mm_FOR_E       85//65//65//70//95//105.2356// 109.134893//E/mm

#define MAX_FEEDRAT_FOR_X       550 // 400//Xmm/s 
#define MAX_FEEDRAT_FOR_Y       550 //400//Ymm/s 

#define MAX_FEEDRAT_FOR_Z        35 //10//35//Zmm/s     //dxc

#define HOME_SPEED_X   200 //200   //Xmm/s 
#define HOME_SPEED_Y   200 // 200   //Ymm/s 
#define HOME_SPEED_Z   60   // 80	//Zmm/s 


#define MAX_TRAVEL_FEEDRATE       50//mm/s 
#define MIN_TRAVEL_FEEDRATE       5 //mm/s 

#define MIN_FEEDRATE              5
#define EXTRUDE_MULTIPLY          100 //
#define EXTRUDE_MIN_TEMP          190 //
#define MIN_SEGMENT_STEPS         8//
#define MIN_SEGMENT_TIME          80000 //

#define ACCELERATION_CHANGE_DISTANCE 5//mm

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


//============================= Bed Auto Leveling ===========================

#define ENABLE_AUTO_BED_LEVELING //
  #define AUTO_BED_LEVELING_GRID//

  #ifdef AUTO_BED_LEVELING_GRID

    #define LEFT_PROBE_BED_POSITION 60
    #define RIGHT_PROBE_BED_POSITION 270//270
    #define BACK_PROBE_BED_POSITION  282//270//115
    #define FRONT_PROBE_BED_POSITION 60//50//10

    #define AUTO_BED_LEVELING_GRID_POINTS 3//
    
    #else //

      #define ABL_PROBE_PT_1_X 15//
      #define ABL_PROBE_PT_1_Y 180
      #define ABL_PROBE_PT_2_X 15
      #define ABL_PROBE_PT_2_Y 20
      #define ABL_PROBE_PT_3_X 50
      #define ABL_PROBE_PT_3_Y 50
      #define  BL_PROBE_PT_3_Y 20

  #endif // AUTO_BED_LEV ELING_GRID


  // 
  // X
  #define X_PROBE_OFFSET_FROM_EXTRUDER -2//-2//
  #define Y_PROBE_OFFSET_FROM_EXTRUDER -51//-50//
  #define Z_PROBE_OFFSET_FROM_EXTRUDER -1.0//
  #define Z_PROBE_OFFSET_RANGE_MIN -15.0//M851
  #define Z_PROBE_OFFSET_RANGE_MAX -0.5

  #define Z_RAISE_BEFORE_HOMING 4       // (in mm)
                                       

  #define XY_TRAVEL_SPEED 4800         // 
  #define Z_RAISE_BEFORE_PROBING 15    //
  #define Z_RAISE_BETWEEN_PROBINGS 10//10  //

    #define Z_SAFE_HOMING_X_POINT (X_MAX_LENGTH/2)    //  (G28)
    #define Z_SAFE_HOMING_Y_POINT (Y_MAX_LENGTH/2)    // (G28)
  
//#endif // ENABLE_AUTO_BED_LEVELING

/*********************************************************************\
* R/C SERVO support
**********************************************************************/

#define NUM_SERVOS 1 //

 // X,Y,Z
 #define SERVO_Push_pin_Down 10//
 #define SERVO_Push_pin_Up 90//
 #define SERVO_Self_test 120
 #define SERVO_Alarm_Release 160
 #define SERVO_M119_Test_Mode 60

#define SERVO_ENDSTOPS {-1, -1, 0} // X, Y, Z-1

/**********************************************************************/


#endif
