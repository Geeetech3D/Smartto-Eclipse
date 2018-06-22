

#include "usart1.h"
#include <stdarg.h>
#include "Configuration_GiantArm_S200.h"
/**********************************************************
***Function:     USART1_Config
***Description: USART1 initialization
***Input:  
***Output: 
***Return:
***********************************************************/
void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* config USART1 clock */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
        
	/* USART1 GPIO config */       
        /* Configure USART1 Tx (PA.9) as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);    
        /* Configure USART1 Rx (PA.10) as input floating */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//
        GPIO_Init(GPIOA, &GPIO_InitStructure);
	  
	/* USART1 mode config */    
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); 
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
       //USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
        USART_ITConfig(USART1, USART_IT_TXE, DISABLE);//USART_IT_IDLE
        USART_Cmd(USART1, ENABLE);
}


void USART1_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
}


/**********************************************************
***Function:     NewUSART3_Config
***Description: USART3 initialization
***Input:  
***Output: 
***Return:
***********************************************************/
void NewUSART3_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
     
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO,ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);  
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
        

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure); 
	  
	/* USART1 mode config */    
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); 
	
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
       //USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
        USART_ITConfig(USART3, USART_IT_TXE, DISABLE);//USART_IT_IDLE
        USART_Cmd(USART3, ENABLE);
                    
}


void NewUSART3_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
}
void USART3_printf(char *Data)
{
//	printf("DDD:%s\r\n",Data);
#ifndef BOARD_M301_Pro_S   
  while ( *Data != 0) 
  {
        USART_SendData(USART3, *Data);
	while( USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET );
	Data++;
  }
#endif
}


/**********************************************************
***Function:     fputc
***Description: Redirection  function printf()
***Input:  
***Output: 
***Return:
***********************************************************/

int fputc(int ch, FILE *f)
{
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}

/**********************************************************
***Function:     itoa
***Description: Convert integers to strings
***Input:  
***Output: 
***Return:
***********************************************************/
static char *itoa(int value, char *string, int radix)
{
    int     i, d;
    int     flag = 0;
    char    *ptr = string;

    /* This implementation only works for decimal numbers. */
    if (radix != 10)
    {
        *ptr = 0;
        return string;
    }

    if (!value)
    {
        *ptr++ = 0x30;
        *ptr = 0;
        return string;
    }

    /* if this is a negative value insert the minus sign. */
    if (value < 0)
    {
        *ptr++ = '-';

        /* Make the value positive. */
        value *= -1;
    }

    for (i = 10000; i > 0; i /= 10)
    {
        d = value / i;

        if (d || flag)
        {
            *ptr++ = (char)(d + 0x30);
            value -= (d * i);
            flag = 1;
        }
    }

    /* Null terminate the string. */
    *ptr = 0;

    return string;

} /* NCL_Itoa */



/***********************************************************************

***********************************************************************/
void USART1_DMA_RxConfig(uint32_t BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1);

  /*!< DMA2 Channel1 disable */
  DMA_Cmd(DMA2_Channel1, DISABLE);	

  /*!< DMA2 Channel1 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR; 
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST; 
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	
  DMA_InitStructure.DMA_BufferSize = BufferSize >> 2; 
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	  
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;			 
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;	
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;			 
  DMA_Init(DMA2_Channel1, &DMA_InitStructure);

  /*!< DMA2 Channel1 enable */			  
  DMA_Cmd(DMA2_Channel1, ENABLE); 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

  USART_Cmd(USART1, DISABLE);
  USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
  USART_Cmd(USART1, ENABLE);

}






void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
  int d;   
  char buf[16];

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     
	{				                          
		if ( *Data == 0x5c )  
		{									  
			switch ( *++Data )
			{
				case 'r':							       
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							         
					USART_SendData(USARTx, 0x0a);	
					Data ++;
					break;
				
				default:
					Data ++;
				    break;
			}			 
		}
		else if ( *Data == '%')
		{									  //
			switch ( *++Data )
			{				
				case 's':										  
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										
          d = va_arg(ap, int);
          itoa(d, buf, 10);
          for (s = buf; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;
				 default:
						Data++;
				    break;
			}		 
		} /* end of else if */
		else USART_SendData(USARTx, *Data++);
		while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
	}
}

/************************************************************/
void USART_Transmit(unsigned char data)
{
  while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
  USART_SendData(USART1, data); 
  //while(USART_GetFlagStatus(USART, USART_FLAG_TC) == RESET);
  
}


/************************************************************/
//uchar USART_Receive(void)
//{
  //while(!UCSR0A_RXC0);
  //return UDR0;
//}

/************************************************************/
void char_log(unsigned char ch)
{
  USART_Transmit(ch);
}

void printf_string(uint8_t *ch)
{


while(*ch != '\0')
  {
    USART_Transmit(*ch);
    ch++;
  }
  printf("\r\n");
  
  
}
/******************* (C) COPYRIGHT 2012 WildFire Team *****END OF FILE************/
