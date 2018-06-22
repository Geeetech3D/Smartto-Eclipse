#ifndef _XZK_RULE_H_
#define _XZK_RULE_H_

#include "stm32f10x.h"


enum Axis_Enum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3, E1_AXIS=4,E2_AXIS=5};
 enum Hotend_Enum { NOZZLE0=0, NOZZLE1=1, NOZZLE2=2, BED=3 };// standard delicate bulky bed prop, 
enum Filament_Enum { PLA = 0, ABS = 1};
enum Endstop_Enum {X_MIN=0, X_MAX =1, Y_MIN,Y_MAX,Z_MIN,Z_MAX};

#define __PI 3.1415926

//#define DISABLE 0    //fdwong
//#define ENABLE 1

#define HOTHEAD_ENABLE true
#define HOTHEAD_DISABLE false

#define HOTBED_ENABLE true
#define HOTBED_DISABLE false

#define AXIS_ENABLE 0
#define AXIS_DISABLE 1

#define MINENDSTOP 0
#define MAXENDSTOP 1

#define NORMALCLOSE_LOW 1     
#define NORMALCLOSE_HIGH 2
#define NORMALOPEN_LOW 3
#define NORMALOPEN_HIGH 4

#define CLOCKWISE true
#define ANTICLOCKWISE false

#define INCH false
#define MM true

#define ABSOLUTE_COORDINATE true
#define RELATIVE_COORDINATE false

#define OPEN true
#define CLOSE false

#define Heating true
#define Cooling false

#define POSITIVE true
#define OPPOSITE false

#define HIGH true
#define LOW false

#define Localization(A,B,C) ((A)<(B)?(B):((A)>(C)?(C):(A)))

#define MAX(A,B) (((A) > (B)) ? (A) : (B))
#define MIN(A,B) (((A) < (B)) ? (A) : (B))

#define sq(A) (A*A)

#define Open_Global_Interrupt() 1
#define Close_Global_Interrupt() 0

#define No_Interrupt_Status() 1
#define Restore_All_Interrupt() 0

#define Open_Step_Motor_Interrupt() 1
#define Close_Step_Motor_Interrupt() 0
	 
#endif
