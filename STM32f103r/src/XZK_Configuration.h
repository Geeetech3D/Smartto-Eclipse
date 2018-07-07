#ifndef __XZK_Configuration_H__
#define __XZK_Configuration_H__
#include "Configuration_Select_Printer.h"

#define BLOCK_BUFFER_SIZE 64//just 8 16 32 64 128


#if defined(BOARD_A30_MINI_S)
  #include "Configuration_GTM32_MINI_A30.h"

#elif defined(BOARD_E180_MINI_S)
  #include "Configuration_GTM32_MINI_E180.h"

#elif defined(BOARD_M301_Pro_S)
  #include "Configuration_GTM32_Pro_M301.h"

#elif defined(BOARD_A30S_MINI_S)
  #include "Configuration_GTM32_MINI_A30S.h"
#endif



#endif
