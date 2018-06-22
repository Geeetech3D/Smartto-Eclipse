#ifndef _ROCKER_H_
#define _ROCKER_H_

#include <stm32f10x.h>



enum phase_status{
    phase_no_move = 0,
    phase_dec,
    phase_inc,     
    phase_bad,
    phase_press
};

void Rocker_Init(void);
void Rocker_Disable(void);
void Rocker_Enable(void);

void SPI_SD_GPIO_Config(void);
void SPI_SD_EXTI_Config(void);



#endif
