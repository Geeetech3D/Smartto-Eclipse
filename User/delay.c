#include "delay.h"

void delay_us(uint32_t nus)
{
   
	u32 temp;
	SysTick->LOAD = PAI_PER_US*nus;
	SysTick->VAL = 0X00;
	SysTick->CTRL = 0X01;
	do
	{
		temp = SysTick->CTRL;
	}
	while((temp&0x01) && (!(temp&(1<<16))));
	SysTick->CTRL = 0X00;
	SysTick->VAL = 0X00;
}

void delay_ms(uint16_t nms)
{
	u32 temp;
	SysTick->LOAD = PAI_PER_US*nms*1000;
	SysTick->VAL = 0X00;
	SysTick->CTRL = 0X01;
	do
	{
		temp = SysTick->CTRL;
	}
	while((temp&0x01) && (!(temp&(1<<16))));
	SysTick->CTRL = 0X00;
	SysTick->VAL = 0X00;
}
void delay_s(u8 ns)
{
    u8 i;
    for(i=0;i<ns;i++)
    {
        delay_ms(1000);
    }
}





