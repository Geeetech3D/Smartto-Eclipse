#ifndef _MMC_SD_H_
#define _MMC_SD_H_		 
 			
#include "stm32f10x.h"
#include "XZK_Configuration.h"

// Types of SD 
#define SD_TYPE_ERR     0X00
#define SD_TYPE_MMC     0X01
#define SD_TYPE_V1      0X02
#define SD_TYPE_V2      0X04
#define SD_TYPE_V2HC    0X06	   
// SD card Instruction List   
#define CMD0    0       //card reset
#define CMD1    1
#define CMD8    8       //  SEND_IF_COND
#define CMD9    9       //read CSD data
#define CMD10   10      //read CID data
#define CMD12   12      // Stop data transmission
#define CMD16   16      //set  Sector Size    return 0x00
#define CMD17   17      //read one sector
#define CMD18   18      //read Multi sector
#define CMD23   23      //erase sector   N block
#define CMD24   24      //write  sector
#define CMD25   25      //write  Multi sector
#define CMD41   41      //return 0x00
#define CMD55   55      //return  0x01
#define CMD58   58      //read OCR data
#define CMD59   59      //enable or disable CRC£¬return 0x00
//Data write response word meaning
#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF
//SD card Response marker
#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF
 							   						 	 

#define	SD_CS_H  GPIO_SetBits(GPIOD,GPIO_Pin_8) //SD Chip select pin	---high
#define	SD_CS_L  GPIO_ResetBits(GPIOD,GPIO_Pin_8) //SD Chip select pin  ---low

  #define GET_SD_CD GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)

#define GET_SD_CD GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_7)
extern u8  SD_Type;//SD card type

u8 SD_SPI_ReadWriteByte(u8 data);
void SD_SPI_SpeedLow(void);
void SD_SPI_SpeedHigh(void);
u8 SD_WaitReady(void);							//Waiting for the SD card to be ready
u8 SD_GetResponse(u8 Response);					//Get answers
u8 SD_Initialize(void);							//SD  Initialize
u8 SD_ReadDisk(u8*buf,u32 sector,u8 cnt);		//read block
u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt);		//write write
u32 SD_GetSectorCount(void);   					//get sector count 
u8 SD_GetCID(u8 *cid_data);                     //read CID
u8 SD_GetCSD(u8 *csd_data);                     //read CSD
void SD_Select_Init(void);
void SD_Card_detect(void);
 
#endif




