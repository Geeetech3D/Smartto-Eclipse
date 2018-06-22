#ifndef __Configuration_GinatArm_S200_H__
#define __Configuration_GinatArm_S200_H__




#ifdef BOARD_A30_MINI_S
	#define DEVICE_NAME "GEEETECH-A30"     //Device name
	#define MACHINE_TYPE "A30"                   //Equipment model
	#define MACHINE_INFO  "FIRMWARE_NAME:"##VERSION##" PROTOCOL_VERSION:V1.0 MACHINE_TYPE:"##MACHINE_TYPE##" EXTRUDER_COUNT:1 "
       #define WIFI_MODULE               1    //wifi module
	#define ENABLE_AUTO_BED_LEVELING 1  //utomatic leveling

#elif BOARD_E180_MINI_S
	#define DEVICE_NAME "GEEETECH-E180"
	#define MACHINE_TYPE "E180"
	#define MACHINE_INFO  "FIRMWARE_NAME:"##VERSION##" PROTOCOL_VERSION:V1.0 MACHINE_TYPE:"##MACHINE_TYPE##" EXTRUDER_COUNT:1 "
       #define WIFI_MODULE               1    //wifi module

#elif BOARD_M301_Pro_S
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


#endif
#define F_CPU 72                         //MHz

#define HOTHEAD_NUMBER 5      //Extrusion head number 

//Compiling different printers through macro definition




#define NOZZLE_PT3_25E2       // Thermistor type

#define HOTBED       1   //ENABLE  //
#define HOTHEAD_1 1   // ENABLE //
#define HOTHEAD_2 1   // ENABLE //  // dxc 2015-03-10
#define HOTHEAD_3 0   ////
#define HOTHEAD_4 0   //

#ifdef BOARD_M301_Pro_S
    #define TEMPERATURE_SAMPLING_PERIOD 1
#else
    #define TEMPERATURE_SAMPLING_PERIOD 3    //Temperature sampling period
#endif

#define TEMPERATURE_SENSOR 1   //B3950


#define TEMPERATURE_ADJUST_RANGE  10   //PID  Adjustment range

//
#define TRAGE_TEMPERATURE_BED  70.0   //dxc   before is 60.0


#define MIN_TEMPERATURE_BED   0.0//  Extruder minimum temperature

#define MIN_TEMPERATURE_NO_1  0.0//  Extruder minimum temperature
#define MIN_TEMPERATURE_NO_2  0.0//
#define MIN_TEMPERATURE_NO_3  0.0//
#define MIN_TEMPERATURE_NO_4  0.0//


#ifdef BOARD_M301_Pro_S
    #define MAX_TEMPERATURE_BED   130.0
#else
#define MAX_TEMPERATURE_BED   120.0
#endif


#ifdef BOARD_M301_Pro_S
    #define MAX_TEMPERATURE_NO_1  270.0
#elif  BOARD_A30_MINI_S
    #define MAX_TEMPERATURE_NO_1  250.0 //Extrusion head maximum temperature
#elif   BOARD_E180_MINI_S
    #define MAX_TEMPERATURE_NO_1  240.0 //Extrusion head maximum temperature
#endif


#define MAX_TEMPERATURE_NO_2  270.0//230.0
#define MAX_TEMPERATURE_NO_3  270.0//230.0
#define MAX_TEMPERATURE_NO_4  270.0//230.0


//PID  parameter
#ifdef BOARD_M301_Pro_S
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
#else
#define KP_BED  369.610


#ifdef BOARD_A30_MINI_S
	#define KP_NO_1 500.045288//M301 H0 P1600.045288,M301 H0 P2011.045288
#elif   BOARD_E180_MINI_S
	#define KP_NO_1 1600.045288//M301 H0 P1600.045288,M301 H0 P2011.045288
#endif


#define KP_NO_2 608.045288//170.012
#define KP_NO_3 608.045288//170.012
#define KP_NO_4 170.012
#define KI_BED 54.132


#ifdef BOARD_A30_MINI_S
	#define KI_NO_1 11.66539 //M301 H0 I21.21.179
#elif   BOARD_E180_MINI_S
	#define KI_NO_1 105.630539//M301 H0 I21.21.179
#endif


#define KI_NO_2 36.630539//18.665
#define KI_NO_3 36.630539//18.665
#define KI_NO_4 18.665
#define KD_BED 602.870


#ifdef BOARD_A30_MINI_S
	#define KD_NO_1 1800.1924 //M301 H0 D3009.194824,M301 H0 D9509.194824
#elif   BOARD_E180_MINI_S
	#define KD_NO_1 1109.194824//M301 H0 D3009.194824,M301 H0 D9509.194824
#endif


#define KD_NO_2 2709.194824//389.381
#define KD_NO_3 2709.194824//389.381
#define KD_NO_4 389.381
#endif


//Print range definition
#ifdef BOARD_M301_Pro_S
#define PREHEAT_PLA_CONF_FAN         0
#define PREHEAT_PLA_CONF_NOZZLE      200
#define PREHEAT_PLA_CONF_BED         70
#define PREHEAT_ABS_CONF_FAN         0
#define PREHEAT_ABS_CONF_NOZZLE      240
#define PREHEAT_ABS_CONF_BED         95
#endif


#ifdef BOARD_A30_MINI_S
    #define X_MIN_POSITION	            0.0
    #define Y_MIN_POSITION	            0.0
    #define Z_MIN_POSITION	            0.0
    #define X_MAX_POSITION             320.0
    #define Y_MAX_POSITION             320.0
    #define Z_MAX_POSITION             420.0 //159.0
#elif   BOARD_E180_MINI_S
    #define X_MIN_POSITION	            0.0
    #define Y_MIN_POSITION	            0.0
    #define Z_MIN_POSITION	            0.0
    #define X_MAX_POSITION             130.0
    #define Y_MAX_POSITION             130.0
    #define Z_MAX_POSITION             132.0 //159.0
#elif   BOARD_M301_Pro_S
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
#endif






#define X_MAX_LENGTH (X_MAX_POSITION - X_MIN_POSITION)
#define Y_MAX_LENGTH (Y_MAX_POSITION - Y_MIN_POSITION)
#define Z_MAX_LENGTH (Z_MAX_POSITION - Z_MIN_POSITION)


#ifdef BOARD_A30_MINI_S
	#define HOMEX                   MINENDSTOP  //MINENDSTOP      //MAXENDSTOP
	#define HOMEY                   MINENDSTOP      //MINENDSTOP      //MAXENDSTOP//////////////////////////////
	#define HOMEZ                   MINENDSTOP   // MAXENDSTOP
	#define DIR_X                   POSITIVE//OPPOSITE//OPPOSITE
	#define DIR_Y                   POSITIVE///////////////////////////////////////////////////////
	#define DIR_Z                   POSITIVE  
	#define DIR_E                   POSITIVE             ///////////////////2016.6.6 mark
	#define DIR_E1                  OPPOSITE
	#define DIR_E2                  POSITIVE
#elif   BOARD_E180_MINI_S
	#define HOMEX                   MINENDSTOP      //MAXENDSTOP
	#define HOMEY                   MAXENDSTOP      //MINENDSTOP      //MAXENDSTOP//////////////////////////////
	#define HOMEZ                   MAXENDSTOP
	#define DIR_X                   OPPOSITE//OPPOSITE
	#define DIR_Y                   POSITIVE///////////////////////////////////////////////////////
	#define DIR_Z                   POSITIVE  
	#define DIR_E                   POSITIVE             ///////////////////2016.6.6 mark
	#define DIR_E1                  OPPOSITE
	#define DIR_E2                  POSITIVE
#elif   BOARD_M301_Pro_S
#define HOMEX                   MAXENDSTOP      //MAXENDSTOP
#define HOMEY                   MAXENDSTOP      //MAXENDSTOP
#define HOMEZ                   MAXENDSTOP      //MAXENDSTOP
#define DIR_X                   POSITIVE//OPPOSITE
#define DIR_Y                   POSITIVE
#define DIR_Z                   OPPOSITE
#define DIR_E                   OPPOSITE//POSITIVE
#define DIR_E1                 OPPOSITE// POSITIVE
#define DIR_E2                 OPPOSITE// POSITIVE
#endif



#ifdef BOARD_M301_Pro_S
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
#else
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
#endif



#define NUM_AXIS                  4
#define EXT_NUM 3

#define EXTRUDE_MAXLENGTH         10.0


#ifdef BOARD_A30_MINI_S
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
#elif   BOARD_E180_MINI_S
	#define STEPS_PER_mm_FOR_X       80
	#define STEPS_PER_mm_FOR_Y       80
	#define STEPS_PER_mm_FOR_Z       400.000
	#define STEPS_PER_mm_FOR_E       88

	#define MAX_FEEDRAT_FOR_X       550 
	#define MAX_FEEDRAT_FOR_Y       550

	#define MAX_FEEDRAT_FOR_Z        35 

	#define HOME_SPEED_X   300 
	#define HOME_SPEED_Y   300
	#define HOME_SPEED_Z   60
#elif   BOARD_M301_Pro_S
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
#endif



#define MAX_TRAVEL_FEEDRATE       50//mm/s 
#define MIN_TRAVEL_FEEDRATE       5 //mm/s 

#define MIN_FEEDRATE              5
#define EXTRUDE_MULTIPLY          100 //
#define EXTRUDE_MIN_TEMP          170 //
#define MIN_SEGMENT_STEPS         8//
#define MIN_SEGMENT_TIME          80000 //

#define ACCELERATION_CHANGE_DISTANCE 5//mm


#ifdef   BOARD_M301_Pro_S
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
#else
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





//============================= Bed Auto Leveling =========================

#ifdef BOARD_A30_MINI_S
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

#endif
