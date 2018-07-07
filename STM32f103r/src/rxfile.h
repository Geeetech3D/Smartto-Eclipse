#ifndef _RXFILE_H_
#define _RXFILE_H_
#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)  

#define S_HEAD   0
#define W_HEAD  1
#define S_DATA   2
#define W_DATA  3
#define S_END     4
#define W_END    5
#define O_TIME    6


extern u16 Rx_File_Len;
extern u8 Firmware_Updata_Flag;
u16 Get_RxData_Size(void);
 u16  crc16( char *d,  u16  len);
u8 Analysis_RxData(void);
u8 get_File_Data(void);
u8  Sent_Boot_Binfile(void);
void clear_Uart(void);
u8 finish_boot_send(void);

void Send_Data_BIN(char *data,u16 size);

void AddData_To_ConfigFile(void);
u8 AddData_To_File(char *data,u16 len,char *filename,long long offset);

#endif

