#ifndef __SPI_H
#define __SPI_H

#include "stm32f10x_spi.h"
 				  	    													  
void SPI2_Init(void);			 //initialization spi
void SPI2_SetSpeed(u8 SpeedSet); //set spi speed
u8 SPI2_ReadWriteByte(u8 TxData);//SPI read one byte data
 		 
#endif


