#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
void USART1_NVIC_Configuration(void);
void NewUSART3_Config(void);
void NewUSART3_NVIC_Configuration(void);
void USART3_printf(char *Data);
int fputc(int ch, FILE *f);	 
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...);
void USART_Transmit(unsigned char data);
void char_log(unsigned char ch);
void printf_string(uint8_t *ch);


#endif /* __USART1_H */
