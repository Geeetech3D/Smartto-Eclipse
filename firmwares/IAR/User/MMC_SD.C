
#include "mmc_sd.h"			   
#include "spi.h"	
#include "sd.h"
#include "LCD2004.h"
#include "variable.h"
					   					   
u8  SD_Type=0;//SD type 

extern char SD_Driver_Number[5];
extern char SD_Path[512]; 
extern u8 SD1_flag;
extern u8 SD_detec_flag;

/**********************************************************
***Function:     SD_SPI_ReadWriteByte
***Description:  spi read and write data
***Input:  data:write data
***Output: 
***Return: read data
***********************************************************/
u8 SD_SPI_ReadWriteByte(u8 data)
{
	return SPI2_ReadWriteByte(data);
}	  
//SD set low speed
void SD_SPI_SpeedLow(void)
{
 	SPI2_SetSpeed(SPI_BaudRatePrescaler_256);//set low speed mode
}
//SD set high speed
void SD_SPI_SpeedHigh(void)
{
 	SPI2_SetSpeed(SPI_BaudRatePrescaler_2);//set high speed mode
}
//SPI  Pin initialization
void SD_SPI_Init(void)
{
  	GPIO_InitTypeDef  GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	 

    	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;			
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_SetBits(GPIOD,GPIO_Pin_8);
			 
	SPI2_Init();
	SD_CS_H;
}
//Cancel chip select
void SD_DisSelect(void)
{
	SD_CS_H;
 	SD_SPI_ReadWriteByte(0xff);//Provides an additional 8 clock pulses
}

/**********************************************************
***Function:     SD_Select
***Description:  chip select SD card  low
***Input:  
***Output: 
***Return:  0:ok  ;1:fail
***********************************************************/
u8 SD_Select(void)
{
	SD_CS_L;
	if(SD_WaitReady()==0)return 0;//waited sucess
	SD_DisSelect();
	return 1;//fail
}



/**********************************************************
***Function:     SD_WaitReady
***Description:  Waiting for the SD card to be ready
***Input:  
***Output: 
***Return:  0:ok  ;1:fail
***********************************************************/
u8 SD_WaitReady(void)
{
	u32 t=0;
	do
	{
		if(SD_SPI_ReadWriteByte(0XFF)==0XFF)return 0;//OK
		t++;		  	
	}while(t<0XFFFFFF);//µÈ´ý 
	return 1;
}

/**********************************************************
***Function:     SD_GetResponse
***Description:  Wait for SD card response,
***Input:    
***Output: Response:The desired response value
***Return:  0:ok  ;other:error
***********************************************************/
u8 SD_GetResponse(u8 Response)
{
	u16 Count=0xFFFF;//waited count					  
	while ((SD_SPI_ReadWriteByte(0XFF)!=Response)&&Count)Count--;//Detection response  
	if (Count==0)return MSD_RESPONSE_FAILURE;//time out
	else return MSD_RESPONSE_NO_ERROR;//ok
}



/**********************************************************
***Function:     SD_RecvData
***Description:  Read the contents of a packet from the SD card
***Input:    len:Data length
***Output: buf: Data storage buffer
***Return:  0:ok  ;1:error
***********************************************************/
u8 SD_RecvData(u8*buf,u16 len)
{			  	  
	if(SD_GetResponse(0xFE))return 1;//Wait for the SD card to respond to the data in fact the token 0xFE
    while(len--)//start receive datas
    {
        *buf=SPI2_ReadWriteByte(0xFF);
        buf++;
    }
    //dummy CRC
    SD_SPI_ReadWriteByte(0xFF);
    SD_SPI_ReadWriteByte(0xFF);									  					    
    return 0;//read sucess
}

/**********************************************************
***Function:     SD_SendBlock
***Description:  write the contents of a packet (512 byte) to the SD card
***Input:    cmd: SD command  buf: Data  buffer
***Output: 
***Return:  0:ok  ;other:error
***********************************************************/
u8 SD_SendBlock(u8*buf,u8 cmd)
{	
	u16 t;		  	  
	if(SD_WaitReady())return 1;//Judging whether it is ready
	SD_SPI_ReadWriteByte(cmd);
	if(cmd!=0XFD)//Judging whether it is end command
	{
		for(t=0;t<512;t++)SPI2_ReadWriteByte(buf[t]);//write data
	    SD_SPI_ReadWriteByte(0xFF);//Ignoring CRC
	    SD_SPI_ReadWriteByte(0xFF);
		t=SD_SPI_ReadWriteByte(0xFF);//Accept response
		if((t&0x1F)!=0x05)return 2;//response erroe								  					    
	}						 									  					    
    return 0;//write sucess
}


/**********************************************************
***Function:     SD_SendCmd
***Description:  Send a command to the SD card
***Input:    cmd: SD command  arg: Command parameters  crc:CRC check value
***Output: 
***Return:  response 
***********************************************************/
u8 SD_SendCmd(u8 cmd, u32 arg, u8 crc)
{
    u8 r1;	
	u8 Retry=0; 
	SD_DisSelect();//Cancel chip select
	if(SD_Select())return 0XFF;//waited cancel chip select ok
	//send command byte
    SD_SPI_ReadWriteByte(cmd | 0x40);
    SD_SPI_ReadWriteByte(arg >> 24);
    SD_SPI_ReadWriteByte(arg >> 16);
    SD_SPI_ReadWriteByte(arg >> 8);
    SD_SPI_ReadWriteByte(arg);	  
    SD_SPI_ReadWriteByte(crc); 
	if(cmd==CMD12)SD_SPI_ReadWriteByte(0xff);//Skip a stuff byte when stop reading
    //Waiting for a response
	Retry=0X1F;
	do
	{
		r1=SD_SPI_ReadWriteByte(0xFF);
	}while((r1&0X80) && Retry--);	 
	//Return status value
    return r1;
}		    																			  


/**********************************************************
***Function:     SD_GetCID
***Description:  Get CID information
***Input:    cid_data:Get data storage buffer , At least 16 bytes
***Output: 
***Return:  0:ok  ;    1:error;
***********************************************************/
u8 SD_GetCID(u8 *cid_data)
{
    u8 r1;	   
    //sed CMD10 , read CID data
    r1=SD_SendCmd(CMD10,0,0x01);
    if(r1==0x00)
	{
		r1=SD_RecvData(cid_data,16);//get 16 byte data
    }
	SD_DisSelect();//cancel chip select
	if(r1)return 1;
	else return 0;
}																				  


/**********************************************************
***Function:     SD_GetCSD
***Description:  Get CSD information  ,Including capacity and speed
***Input:    csd_data:Get data storage buffer , At least 16 bytes
***Output: 
***Return:  0:ok  ;    1:error;
***********************************************************/
u8 SD_GetCSD(u8 *csd_data)
{
    u8 r1;	 
    r1=SD_SendCmd(CMD9,0,0x01);//send CMD9 command,read CSD data
    if(r1==0)
	{
    	r1=SD_RecvData(csd_data, 16);//get 16 byte data
    }
	SD_DisSelect();//cancel chip select
	if(r1)return 1;
	else return 0;
}  


			
/**********************************************************
***Function:     SD_GetSectorCount
***Description:  Get SD Get the total number of SD cards sector
***Input:    no
***Output: 
***Return:  sector num
***********************************************************/
u32 SD_GetSectorCount(void)
{
    u8 csd[16];
    u32 Capacity;  
    u8 n;
	u16 csize;  					    
	//Get CSD information 
    if(SD_GetCSD(csd)!=0) return 0;	    
    //Determine the type of card
    if((csd[0]&0xC0)==0x40)	 //V2.00 card 
    {	
		csize = csd[9] + ((u16)csd[8] << 8) + 1;
		Capacity = (u32)csize << 10;//get sector num	   
    }else//V1.XX card
    {	
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((u16)csd[7] << 2) + ((u16)(csd[6] & 3) << 10) + 1;
		Capacity= (u32)csize << (n - 9);//sector num
    }
    return Capacity;
}
/**********************************************************
***Function:     SD_Initialize
***Description:  SD Initialize
***Input:    no
***Output: 
***Return:   
***********************************************************/
u8 SD_Initialize(void)
{
	return 0;
}


/**********************************************************
***Function:     SD_ReadDisk
***Description:  read SD card 
***Input:    buf:data buffer ;sector :sector ;cnt:sector num 
***Output: 
***Return:   0:ok;  other :fail
***********************************************************/
u8 SD_ReadDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 r1;
	if(SD_Type!=SD_TYPE_V2HC)sector <<= 9;//Convert to byte address
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD17,sector,0X01);//read command
		if(r1==0)//command send sucess
		{
			r1=SD_RecvData(buf,512);//receive 512 byte data
		}
	}else
	{
		r1=SD_SendCmd(CMD18,sector,0X01);//Continuous read commands
		do
		{
			r1=SD_RecvData(buf,512);//receive 512 byte data
			buf+=512;  
		}while(--cnt && r1==0); 	
		SD_SendCmd(CMD12,0,0X01);	//send stop command
	}   
	SD_DisSelect();//cancel chip select
	return r1;//
}



/**********************************************************
***Function:     SD_WriteDisk
***Description:  write SD card 
***Input:    buf:data buffer ;sector :sector ;cnt:sector num 
***Output: 
***Return:   0:ok;  other :fail
***********************************************************/
u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 r1;
	if(SD_Type!=SD_TYPE_V2HC)sector *= 512;//Convert to byte address
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD24,sector,0X01);//write command
		if(r1==0)//command send sucess
		{
			r1=SD_SendBlock(buf,0xFE);//write 512 byte data   
		}
	}else
	{
		if(SD_Type!=SD_TYPE_MMC)
		{
			SD_SendCmd(CMD55,0,0X01);	
			SD_SendCmd(CMD23,cnt,0X01);//send command
		}
 		r1=SD_SendCmd(CMD25,sector,0X01);//Continuous write commands
		if(r1==0)
		{
			do
			{
				r1=SD_SendBlock(buf,0xFC);//write 512 byte data
				buf+=512;  
			}while(--cnt && r1==0);
			r1=SD_SendBlock(0,0xFD);
		}
	}   
	SD_DisSelect();
	return r1;
}	   

void SD_Select_Init(void)
{
    strcpy(SD_Driver_Number, "SD3:");
    strcpy(SD_Path, "SD3:");
    system_infor.sd_status = !SD_OK;
    if(GET_SD_CD == 0)
    {             
      strcpy(SD_Driver_Number, "SD1:");
      strcpy(SD_Path, "SD1:");              
      system_infor.sd_status=SD_Init();       
    }      
}
#ifndef BOARD_M301_Pro_S
void SD_Card_detect(void)
{ 
	if(SD1_flag != GET_SD_CD || (SD1_flag == 0 && system_infor.sd_status != SD_OK))
	{
		SD_detec_flag = 1;
                SD1_flag = GET_SD_CD;

        return;
	}  
}
#else
void SD_Card_detect(void)
{ 
      if(SD1_flag != GET_SD_CD)
      {
        sd_plug_detect();
        SD1_flag = GET_SD_CD;
        return;
      }  
}
#endif
















	  






