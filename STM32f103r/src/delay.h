#ifndef _DELAY_H
#define _DELAY_H

#include "stm32f10x.h"

#define PAI_PER_US    SystemCoreClock/8000000

void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_s(u8 ns);
void Delay_us(uint32_t nus);

#endif