

#include "stm32f10x.h"
#include "delay.h"
#include "rocker.h"
//#include "usart1.h"


__IO u16 Rocker_value[2] = {0,0};

u8 Encoder_press;

const u8 table_Phase[16]=
{
	phase_no_move,phase_dec,phase_inc,phase_bad,
	phase_inc,phase_no_move,phase_bad,phase_dec, 
	phase_dec,phase_bad,phase_no_move,phase_inc,
	phase_bad,phase_inc,phase_dec,phase_no_move,
};

__IO u8 PhaseShift;




void SPI_SD_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;       
        
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
        
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
         
        GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOE,&GPIO_InitStructure);              
        
}



u8 Encoder_Scan(void)
{
	static s8 PhaseCount = 0;
  //printf("encoder press : %d\r\n",Encoder_press);
	if((Encoder_press != 0) && (GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==0))
		return phase_no_move;
	else
		Encoder_press = 0;
	
	PhaseShift <<= 2;
	if((GPIOD->IDR & GPIO_Pin_9)==0)  PhaseShift |= 0x01;
	if((GPIOD->IDR & GPIO_Pin_10)==0) PhaseShift |= 0x02;
	PhaseShift &= 0x0f;
	if(table_Phase[PhaseShift] == phase_inc) PhaseCount++;
	if(table_Phase[PhaseShift] == phase_dec) PhaseCount--;
	if(PhaseCount > 3)
	{
	  PhaseCount -= 4;
	  return phase_inc;
	}
	else if(PhaseCount < -3)
	{
	  PhaseCount += 4;
	  return phase_dec;
	}
	if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==0)
	{
		delay_ms(10);
		if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_6)==0)
		{
			Encoder_press = 1;
			return phase_press;
		}
	}
	return phase_no_move;
}



void SPI_SD_EXTI_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure; 
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); 

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource11);  
  
    EXTI_InitStructure.EXTI_Line = EXTI_Line11;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; 
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);  
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;  
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure);
                       
}


    

