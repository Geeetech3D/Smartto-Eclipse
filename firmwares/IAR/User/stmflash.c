#include "stmflash.h"
 
 

/**********************************************************
***Function:     STMFLASH_ReadHalfWord
***Description: read halfword
***Input:         read addr position  (faddr%2==0)
***Output: 
***Return:       read data
***********************************************************/
u16 STMFLASH_ReadHalfWord(u32 faddr)
{
	return *(vu16*)faddr; 
}


#if STM32_FLASH_WREN	//write enable


/**********************************************************
***Function:     STMFLASH_Write_NoCheck
***Description: No Check read halfword
***Input:         WriteAddr:write addr position  (faddr%2==0) pBuffer:write data  NumToWrite:write data len
***Output: 
***Return:      no
***********************************************************/
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)   
{ 			 		 
	u16 i;
	for(i=0;i<NumToWrite;i++)
	{
		FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	        WriteAddr+=2;//addr +2.
	}  
} 



#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else 
#define STM_SECTOR_SIZE	2048
#endif		 
u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节



/**********************************************************
***Function:     STMFLASH_Write
***Description: read halfword
***Input:         WriteAddr:write addr position  (faddr%2==0) pBuffer:write data  NumToWrite:write data len
***Output: 
***Return:      no
***********************************************************/
void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
	u32 secpos;	   //Sector address
	u16 secoff;	   //Sector offset address
	u16 secremain; //Sector remaining space
 	u16 i;    
	u32 offaddr;   //
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))
		return;//Illegal address
	FLASH_Unlock();						//flash unlock
	offaddr=WriteAddr-STM32_FLASH_BASE;		//Actual offset address
	secpos=offaddr/STM_SECTOR_SIZE;			// Sector address    0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//Sector offset address
	secremain=STM_SECTOR_SIZE/2-secoff;		//Sector remaining space
	if(NumToWrite<=secremain)
		secremain=NumToWrite; 
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//Read entire sector contents
		for(i=0;i<secremain;i++)//Check data
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)
				break;//Erase 
		}
		if(i<secremain)//Erase
		{
			FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//Erase sector
			for(i=0;i<secremain;i++)
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//write entire sector contents
		}else STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//			   
		if(NumToWrite==secremain)
			break;//Write end
		else//Write not ended
		{
			secpos++;				//sector address + 1
			secoff=0;				//sector offset = 0
		   	pBuffer+=secremain;  	//Pointer offset
			WriteAddr+=secremain;	
		   	NumToWrite-=secremain;	
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;
			else secremain=NumToWrite;
		}	 
	};	
	FLASH_Lock();//lock
}

/**********************************************************
***Function:     STMFLASH_Write2
***Description: write halfword
***Input:         WriteAddr:write addr position  (faddr%2==0) pBuffer:write data  NumToWrite:write data len
***Output: 
***Return:      no
***********************************************************/
void STMFLASH_Write2(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)	
{
 	u16 i;    

	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))
		return;//error address
	
	FLASH_Unlock();						
	for(i=0;i<NumToWrite;i++)
	{
		 FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
	   	 WriteAddr+=2;
	}  
	FLASH_Lock();
}
/**********************************************************
***Function:     STMFLASH_Erase
***Description: Erase 
***Input:         WriteAddr:write addr position  (faddr%2==0)  NumToWrite:write data len
***Output: 
***Return:      no
***********************************************************/
void STMFLASH_Erase(u32 WriteAddr,u16 NumToWrite)	
{
	u32 secpos;	   //sector address
	u16 secoff;	   //sector offset
	u16 secremain; //Sector remaining space
 	u16 i;    
	u32 offaddr;   
	if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))
		return;//error address
	FLASH_Unlock();	
	offaddr=WriteAddr-STM32_FLASH_BASE;		//Actual offset address
	secpos=offaddr/STM_SECTOR_SIZE;			// Sector address    0~127 for STM32F103RBT6
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//Sector offset address
	secremain=STM_SECTOR_SIZE/2-secoff;		//Sector remaining space
	if(NumToWrite<=secremain)
		secremain=NumToWrite;
	while(1) 
	{	
		STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//Read entire sector contents
		for(i=0;i<secremain;i++)//check data
		{
			if(STMFLASH_BUF[secoff+i]!= 0XFFFF )
				break;//erase
		}
		if(i<secremain)//erase
		{
			FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//erase this sector
			for(i=0;i<secremain;i++)//copy
			{
				STMFLASH_BUF[i+secoff]=0xFFFF;	  
			}
			STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);
		}			   
		if(NumToWrite==secremain)
			break;//write end
		else//write not ended
		{
			secpos++;				
			secoff=0;				
			WriteAddr+=secremain;	  
		   	NumToWrite-=secremain;	
			if(NumToWrite>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;
			else secremain=NumToWrite;
		}	 
	};	
	FLASH_Lock();//上锁

	
}

#endif

/**********************************************************
***Function:     STMFLASH_Read
***Description: read data
***Input:         ReadAddr:read addr position  (faddr%2==0) pBuffer:read data  NumToWrite:read data len
***Output: 
***Return:      no
***********************************************************/
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)   	
{
	u16 i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);
		ReadAddr+=2;
	}
}

void Test_Write(u32 WriteAddr,u16 WriteData)   	
{
	STMFLASH_Write(WriteAddr,&WriteData,1);
}
















