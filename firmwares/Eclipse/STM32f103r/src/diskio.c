/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
//#include "usbdisk.h"	/* Example: USB drive control */
//#include "atadrive.h"	/* Example: ATA drive control */
#include "sd.h"		/* Example: MMC/SDC contorl */
#include "mmc_sd.h"
//#include "usart1.h"
/* Definitions of physical drive number for each media */

#define RAM   0
#define NAND  1
#define CF    2
#define SD1   3
#define SD2   4
#define USB1  5
#define USB2  6
#define USB3  7

__IO unsigned char SD_Status;
/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (BYTE pdrv)
{
  unsigned char sd_status=0;
  
 // while(SD_Status == BUSY);

	switch (pdrv) 
	{
  	case RAM :return STA_NOINIT;
   case NAND:return STA_NOINIT;
   case CF:return STA_NOINIT;
  	case SD1 :SD_Status = BUSY;
     /*if(SD_HARD_SLOT() == 1)
  		          {
  	             print("Insert SD card not in the card slot!!");
  	             Return;
  	             return STA_NODISK;
  	           }
  	           else
  	           {*/
  	             sd_status = SD_Init();
  	             switch(sd_status)
  	             {
  	                 case 0: 
			       //printf("sd init OK!!\r\n");
				SD_Status = FREE; 
				return RES_OK; 
  	                      
  	                 default:
			       //printf("sd init fail!!\r\n");
				SD_Status = FREE; 
				return STA_NOINIT; 
  	             }
  	           //}
  	           
           
  	             	
  	           	

        
        
        
  	case USB1 :return STA_NOINIT;
  	case USB2 :return STA_NOINIT;
  	case USB3 :return STA_NOINIT;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{/*
	DSTATUS stat;
	int result;

	switch (pdrv) 
	{
  	  case RAM :return RES_OK;
     case NAND:return RES_OK;
     case CF:return RES_OK;
  	  case SD1 :return RES_OK;
  	  case SD2 :return RES_OK; 
    	case USB1 :return RES_OK;
    	case USB2 :return RES_OK;
    	case USB3 :return RES_OK;
	}*/
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/


DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
 SD_Error sd_state;
 u8 res=0;
 u32 time_out = 0x1000;
 if(count == 0)
   return RES_PARERR;
 //while(SD_Status == BUSY);
	switch (pdrv) 
	{
            case RAM :return RES_ERROR;
            case NAND:return RES_ERROR;
            case CF:return RES_ERROR;
            case SD1 :
                  SD_Status = BUSY;
                  if(count > 1)
                  {
                      if(SD_ReadMultiBlocks(buff,sector<<9,512,count) == SD_OK)
                      {
                          if((sd_state = SD_WaitReadOperation()) != SD_OK)
                          {
                        	  my_printf("Read error is %d\r\n",(u8)sd_state);   //dxc
                          }
                          else
                              while(SD_GetStatus()==SD_TRANSFER_BUSY)
                              {
							      time_out--;
								  if(time_out == 0)
									  return RES_ERROR;
							  }
                          
                          SD_Status = FREE;
                          return RES_OK;
                      }
                      else
                      {
                          //print("read multi fail");
                          // Return;
                          SD_Status = FREE;
                          return RES_ERROR;
                      }
                  }
                  else
                  {
                      if(SD_ReadBlock(buff,sector<<9,512) == SD_OK)
                      {
                          if((sd_state = SD_WaitReadOperation()) != SD_OK)
                          {
                              //printf("\r\nRead error is %d\r\n",sd_state);   //dxc
                          }
                          else
                          {
                              while(SD_GetStatus()==SD_TRANSFER_BUSY)
                              {
                                  time_out--;
								  if(time_out == 0)
									  return RES_ERROR;
                              }
                              //TFT_Show(0,0,CHOOSE,BACKGROUND,"  ");
                          }
                          //print("read single ok");
                          // Return;
                          SD_Status = FREE;
                          return RES_OK;
                      }
                      
                      else
                      {
                          //print("read single fail");
                          //Return;
                          SD_Status = FREE;
                          return RES_ERROR;
                      }
                  }

  	case USB1 :return RES_ERROR;
  	case USB2 :return RES_ERROR;
  	case USB3 :return RES_ERROR;
	}
	return RES_ERROR;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
    u8 res=0;
	u32 time_out = 0x1000;
    if(count == 0)  return RES_PARERR;
// while(SD_Status == BUSY);
	switch (pdrv) 
	{
	  case RAM :return RES_ERROR;
   case NAND:return RES_ERROR;
   case CF:return RES_ERROR;

	  case SD1 :SD_Status = BUSY;
              if(count > 1)
	            {
	              if(SD_WriteMultiBlocks((uint8_t *)buff,sector<<9,512,count) == SD_OK)
	              {
	                if(SD_WaitWriteOperation() == SD_DATA_TIMEOUT)
	                	{
                          //printf("\r\nWrite Operation Data Timeout!!\r\n");   //dxc
	                	}
                  else
      	            while(SD_GetStatus()==SD_TRANSFER_BUSY)
					{
                        time_out--;
						if(time_out == 0)
						     return RES_ERROR;
                    }
                 //print("write multi ok");
                 //Return;
                 SD_Status = FREE;
                 return RES_OK;
	              }
	              else
	              {
	                //print("write multi fail");
	                //Return;
	                SD_Status = FREE;
		               return RES_ERROR;
	  	           }
             }
             else
             {
               if(SD_WriteBlock((uint8_t *)buff,sector<<9,512) == SD_OK)
               {
                 if(SD_WaitWriteOperation() == SD_DATA_TIMEOUT)
                 	{
                     //printf("\r\nWrite Operation Data Timeout!!\r\n");   //dxc
                 	}
               else
      	           while(SD_GetStatus()==SD_TRANSFER_BUSY)
				   {
                        time_out--;
						if(time_out == 0)
						     return RES_ERROR;
                   }
                // print("write single ok");
                 //Return;
                 SD_Status = FREE;
                 return RES_OK;
               }
               else
               {
                // print("write single fail");
                // Return;
                 SD_Status = FREE;
                 return RES_ERROR;
               }
             }


                     
  	  case USB1 :return RES_ERROR;
  	  case USB2 :return RES_ERROR;
  	  case USB3 :return RES_ERROR;
	}
	return RES_ERROR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{

	switch (pdrv) 
	{
	  case RAM :return RES_OK ;
          case NAND:return RES_OK ;
          case CF:return RES_OK ;
	  case SD1 :return RES_OK ;
	  case SD2 :return RES_OK ; 
  	  case USB1 :return RES_OK ;
  	  case USB2 :return RES_OK ;
  	  case USB3 :return RES_OK ;
	}
	return RES_OK ;
}
#endif



