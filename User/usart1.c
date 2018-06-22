

#include "usart1.h"
#include <stdarg.h>

//extern char DMA_cmdbuf[1024];
/*
 * 函数名：USART1_Config
 * 描述  ：USART1 GPIO 配置,工作模式配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
        //DMA_InitTypeDef DMA_InitStructure;

	/* config USART1 clock */
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);        
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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
}

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
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_Init(&NVIC_InitStructure);
}
void USART3_printf(char *Data)
{
//	printf("DDD:%s\r\n",Data);
  while ( *Data != 0) 
  {
        USART_SendData(USART3, *Data);
	while( USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET );
	Data++;
  }
}

/*
 * 函数名：fputc
 * 描述  ：重定向c库函数printf到USART1
 * 输入  ：无
 * 输出  ：无
 * 调用  ：由printf调用
 */
int fputc(int ch, FILE *f)
{
/* 将Printf内容发往串口 */
  USART_SendData(USART1, (unsigned char) ch);
  while (!(USART1->SR & USART_FLAG_TXE));
 
  return (ch);
}

/*
 * 函数名：itoa
 * 描述  ：将整形数据转换成字符串
 * 输入  ：-radix =10 表示10进制，其他结果为0
 *         -value 要转换的整形数
 *         -buf 转换后的字符串
 *         -radix = 10
 * 输出  ：无
 * 返回  ：无
 * 调用  ：被USART1_printf()调用
 */
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

  DMA_ClearFlag(DMA2_FLAG_TC1 | DMA2_FLAG_TE1 | DMA2_FLAG_HT1 | DMA2_FLAG_GL1);//清标志

  /*!< DMA2 Channel1 disable */
  DMA_Cmd(DMA2_Channel1, DISABLE);	

  /*!< DMA2 Channel1 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;  //外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST; //目标地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//外设为原地址
  DMA_InitStructure.DMA_BufferSize = BufferSize >> 2;  //1/4缓存大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//使能外设地址不自增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	  //使能存储目标地址自增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //外设数据大小为字节，8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//外设数据大小为字节，8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;			   //循环，循环模式主要用在adc上
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;	 //通道优先级最高
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;			 //非 存储器至存储器模式
  DMA_Init(DMA2_Channel1, &DMA_InitStructure);

  /*!< DMA2 Channel1 enable */			   //不设置dma中断？
  DMA_Cmd(DMA2_Channel1, ENABLE); 
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);

  USART_Cmd(USART1, DISABLE);
  USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
  USART_Cmd(USART1, ENABLE);

}






/*
 * 函数名：USART1_printf
 * 描述  ：格式化输出，类似于C库中的printf，但这里没有用到C库
 * 输入  ：-USARTx 串口通道，这里只用到了串口1，即USART1
 *		     -Data   要发送到串口的内容的指针
 *			   -...    其他参数
 * 输出  ：无
 * 返回  ：无 
 * 调用  ：外部调用
 *         典型应用USART1_printf( USART1, "\r\n this is a demo \r\n" );
 *            		 USART1_printf( USART1, "\r\n %d \r\n", i );
 *            		 USART1_printf( USART1, "\r\n %s \r\n", j );
 */
void USART1_printf(USART_TypeDef* USARTx, uint8_t *Data,...)
{
	const char *s;
  int d;   
  char buf[16];

  va_list ap;
  va_start(ap, Data);

	while ( *Data != 0)     // 判断是否到达字符串结束符
	{				                          
		if ( *Data == 0x5c )  //'\'
		{									  
			switch ( *++Data )
			{
				case 'r':							          //回车符
					USART_SendData(USARTx, 0x0d);
					Data ++;
					break;

				case 'n':							          //换行符
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
				case 's':										  //字符串
					s = va_arg(ap, const char *);
          for ( ; *s; s++) 
					{
						USART_SendData(USARTx,*s);
						while( USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET );
          }
					Data++;
          break;

        case 'd':										//十进制
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
