
extern void delay(int count);
//·¢ËÍÃüÁî
extern unsigned long SPI_ReadID();
extern int SPI_WriteContents(unsigned long address , unsigned char* contents);
extern void read_SPI_Registers();
extern unsigned long SPI_ReadSR();
extern int SPI_WriteContents(unsigned long address , unsigned char* contents);
extern unsigned long ReadFlashData(unsigned long address , unsigned char reciveData[]);
extern unsigned long Program_Page(unsigned long address , unsigned char* data , int length);
extern void CMD_RDSCUR();
extern void Chip_Erase();