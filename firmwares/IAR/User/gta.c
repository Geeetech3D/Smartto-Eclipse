
#include "gta.h"
#include "XZK_Configuration.h"


u8 ERx_buf[CMD_EXT_BUF_MAX][CMD_EXT_LEN_MAX];
u8 read_index = 0; 
u8 write_index = 0; 
//static uint8_t rx_row = 0;
//static uint8_t rx_index = 0;
//static uint8_t rx_len = CMD_EXT_LEN_MAX;


extern float Current_Temperature[5];
extern float Current_Width[3];
extern u8 level_switch;


void USART2_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  /* config USART2 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 , ENABLE);        
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA , ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);    
  /* Configure USART2 Rx (PA.3) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  NVIC_InitTypeDef NVIC_InitStructure;


  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

  NVIC_Init(&NVIC_InitStructure);       
    

  /* USART2 mode config */    
  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART2, &USART_InitStructure); 
  USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
  USART_Cmd(USART2, ENABLE);
        
}

void USART2_TxByte(u8 ch)
{
  USART_SendData(USART2,ch);
  while( USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET );
}

void USART2_TxString(u8 *string)
{
#ifdef WIFI_MODULE
  while(*string != '\0')
    USART2_TxByte(*string ++);
#endif
}




