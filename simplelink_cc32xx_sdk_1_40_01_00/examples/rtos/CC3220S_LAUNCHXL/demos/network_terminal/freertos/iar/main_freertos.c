/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== main_freertos.c ========
 */
#include <stdint.h>

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include "FreeRTOS.h"
#include "task.h"

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>

/* Example/Board Header files */
#include "Board.h"
#include "uart_term.h"
#include <ti/drivers/net/wifi/simplelink.h>
//#include <Spi.h>
#include <ti/devices/cc32xx/driverlib/spi.h>
#include <ti/devices/cc32xx/driverlib/rom.h>
#include <ti/devices/cc32xx/driverlib/rom_map.h>
//#include <Hw_memmap.h>
#include <ti/devices/cc32xx/inc/hw_memmap.h>
#include <ti/devices/cc32xx/driverlib/prcm.h>
#include <ti/devices/cc32xx/inc/hw_mcspi.h>
#include <ti/devices/cc32xx/driverlib/hwspinlock.h>
#include <ti/devices/cc32xx/inc/hw_types.h>
#include <ti/devices/cc32xx/driverlib/pin.h>
#include <ti/devices/cc32xx/driverlib/gpio.h>
//#include "gpio_if.h"
#include <ti/devices/cc32xx/driverlib/uart.h>
#include <ti/devices/cc32xx/inc/hw_uart.h>

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100
#define UART_BUF_SIZE    4096


extern void Network(void *arg);

/* Stack size in bytes */
#define THREADSTACKSIZE   4096


#define _u8  unsigned char
#define _i8  signed char
#define _u16 unsigned short
#define _i16 signed short
#define _u32 unsigned long
#define _i32 signed long

#define _volatile volatile
#define _const    const



char *paraes_SPI_FLASH_WORK=NULL;
char *paraes_UART_MCU_CC3220=NULL;
char *paraesNetwork=NULL;
//int a , b , c;
int UART_Print_flg=0;
unsigned long DeviceID = 0;
unsigned char reciveData[256];
unsigned char sendData[]={0xab,0xab,0xab,0xab,0xab};

char*           DeviceFileName = "MyFile.txt";
unsigned long   MaxSize = 63 * 1024; //62.5K is max file size
unsigned long   Offset = 0;
unsigned char   InputBuffer[100];

int sl_start_flag=0;


UART_Handle uart_1;

#define APPLICATION_NAME        "FILE OPERATIONS"
#define APPLICATION_VERSION     "1.1.1"

#define SL_MAX_FILE_SIZE        64L*1024L       /* 64KB file */
#define BUF_SIZE                2048
#define USER_FILE_NAME          "fs_demo.txt"

extern void UDP_TASK();
extern void TCP_TASK();
extern void WRITE_FILE_TASK();
extern TaskHandle_t xHandle_UDP_TASK;
extern TaskHandle_t xHandle_TCP_TASK;
extern TaskHandle_t xHandle_RAW_TASK;
TaskHandle_t xHandle_WRITE_FILE_TASK;
TaskHandle_t netWork;

/* Application specific status/error codes */
typedef enum{
    // Choosing this number to avoid overlap w/ host-driver's error codes
    FILE_ALREADY_EXIST = -0x7D0,
    FILE_CLOSE_ERROR = FILE_ALREADY_EXIST - 1,
    FILE_NOT_MATCHED = FILE_CLOSE_ERROR - 1,
    FILE_OPEN_READ_FAILED = FILE_NOT_MATCHED - 1,
    FILE_OPEN_WRITE_FAILED = FILE_OPEN_READ_FAILED -1,
    FILE_READ_FAILED = FILE_OPEN_WRITE_FAILED - 1,
    FILE_WRITE_FAILED = FILE_READ_FAILED - 1,

    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
//unsigned char gaucCmpBuf[BUF_SIZE];
/*const unsigned char gaucOldMacDonald[] = "Old MacDonald had a farm,E-I-E-I-O, \
And on his farm he had a cow, \
E-I-E-I-O, \
With a moo-moo here, \
And a moo-moo there, \
Here a moo, there a moo, \
Everywhere a moo-moo. \
Old MacDonald had a farm, \
E-I-E-I-O. \
Old MacDonald had a farm, \
E-I-E-I-O, \
And on his farm he had a pig, \
E-I-E-I-O, \
With an oink-oink here, \
And an oink-oink there, \
Here an oink, there an oink, \
Everywhere an oink-oink. \
Old MacDonald had a farm, \
E-I-E-I-O. \
Old MacDonald had a farm, \
E-I-E-I-O, \
And on his farm he had a duck, \
E-I-E-I-O, \
With a quack-quack here, \
And a quack-quack there, \
Here a quack, there a quack, \
Everywhere a quack-quack. \
Old MacDonald had a farm, \
E-I-E-I-O. \
Old MacDonald had a farm, \
E-I-E-I-O, \
And on his farm he had a horse, \
E-I-E-I-O, \
With a neigh-neigh here, \
And a neigh-neigh there, \
Here a neigh, there a neigh, \
Everywhere a neigh-neigh. \
Old MacDonald had a farm, \
E-I-E-I-O. \
Old MacDonald had a farm, \
E-I-E-I-O, \
And on his farm he had a donkey, \
E-I-E-I-O, \
With a hee-haw here, \
And a hee-haw there, \
Here a hee, there a hee, \
Everywhere a hee-haw. \
Old MacDonald had a farm, \
E-I-E-I-O. \
Old MacDonald had a farm, \
E-I-E-I-O, \
And on his farm he had some chickens, \
E-I-E-I-O, \
With a cluck-cluck here, \
And a cluck-cluck there, \
Here a cluck, there a cluck, \
Everywhere a cluck-cluck. \
Old MacDonald had a farm, \
E-I-E-I-O.";
*/


void PinMuxConfig(void)
{
    //
    // Enable Peripheral Clocks 
    //
    //MAP_PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);
    //MAP_PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    //
    // Configure PIN_55 for UART0 UART0_TX
    //
    //MAP_PinTypeUART(PIN_03, PIN_MODE_7);
    //MAP_PinTypeUART(PIN_01, PIN_MODE_7);
    MAP_PinTypeUART(PIN_58, PIN_MODE_6);

    //
    // Configure PIN_57 for UART0 UART0_RX
    //
    //MAP_PinTypeUART(PIN_04, PIN_MODE_7);
    //MAP_PinTypeUART(PIN_02, PIN_MODE_7);
    MAP_PinTypeUART(PIN_59, PIN_MODE_6);
}

/*

//*****************************************************************************
//
//!  This funtion includes the following steps:
//!  -open a user file for writing
//!  -write "Old MacDonalds" child song 37 times to get just below a 64KB file
//!  -close the user file
//!
//!  /param[out] ulToken : file token
//!  /param[out] lFileHandle : file handle
//!
//!  /return  0:Success, -ve: failure
//
//*****************************************************************************
long WriteFileToDevice(unsigned long *ulToken)
{
    long lRetVal = -1;
    int iLoopCnt = 0;
    long lFileHandle;
    //
    //  create a user file
    //
    lFileHandle = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                        SL_FS_CREATE|SL_FS_OVERWRITE|SL_FS_CREATE_MAX_SIZE( MaxSize ),
                        ulToken);
    if(lFileHandle < 0)
    {
        //创建失败
		return -10;
    }
    
    //
    // write "Old MacDonalds" child song as many times to get just below a 64KB file
    //
    for (iLoopCnt = 0; 
            iLoopCnt < (SL_MAX_FILE_SIZE / sizeof(gaucOldMacDonald)); 
            iLoopCnt++)
    {
        lRetVal = sl_FsWrite(lFileHandle,
                    (unsigned int)(iLoopCnt * sizeof(gaucOldMacDonald)), 
                    (unsigned char *)gaucOldMacDonald, sizeof(gaucOldMacDonald));
        if (lRetVal < 0)
        {
            lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
            //ASSERT_ON_ERROR(FILE_WRITE_FAILED);
			//写入失败
			return -11;
        }
    }
    
    //
    // close the user file
    //
    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != lRetVal)
    {
        //ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
		//关闭失败
		return -12;
    }

    return 0;//success
}

//*****************************************************************************
//
//!  This funtion includes the following steps:
//!    -open the user file for reading
//!    -read the data and compare with the stored buffer
//!    -close the user file
//!
//!  /param[in] ulToken : file token
//!  /param[in] lFileHandle : file handle
//!
//!  /return 0: success, -ve:failure
//
//*****************************************************************************
long ReadFileFromDevice(unsigned long ulToken)
{
    long lRetVal = -1;
    int iLoopCnt = 0;
    long lFileHandle;

    //
    // open a user file for reading
    //
    lFileHandle = sl_FsOpen((unsigned char *)USER_FILE_NAME,
                        SL_FS_READ,
                        &ulToken);
    if(lFileHandle < 0)
    {
        //打开失败
		return -10;
    }

    //
    // read the data and compare with the stored buffer
    //
    for (iLoopCnt = 0;
            iLoopCnt < (SL_MAX_FILE_SIZE / sizeof(gaucOldMacDonald));
            iLoopCnt++)
    {
        lRetVal = sl_FsRead(lFileHandle,
                    (unsigned int)(iLoopCnt * sizeof(gaucOldMacDonald)),
                     gaucCmpBuf, sizeof(gaucOldMacDonald));
        if ((lRetVal < 0) || (lRetVal != sizeof(gaucOldMacDonald)))
        {
            lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
            //ASSERT_ON_ERROR(FILE_READ_FAILED);
			return -11;
        }

        lRetVal = memcmp(gaucOldMacDonald,
                         gaucCmpBuf,
                         sizeof(gaucOldMacDonald));
        if (lRetVal != 0)
        {
            //ASSERT_ON_ERROR(FILE_NOT_MATCHED);
			return -14;
        }
    }

    //
    // close the user file
    //
    lRetVal = sl_FsClose(lFileHandle, 0, 0, 0);
    if (SL_RET_CODE_OK != lRetVal)
    {
        //ASSERT_ON_ERROR(FILE_CLOSE_ERROR);
		return -12;
    }

    return 0;
}

*/


unsigned char recvBuffFromMCU[UART_BUF_SIZE]; //数据接收
_u16 index = 0;
_u16 RecvLength = 0;
int interruptFlag = 0;
int writeIndex = 0;
int readIndex = 0;
static int lRetVal = 0;
#if 0
unsigned char warningdata[26];
extern int UDP_Recv_Flag;
extern int UDP_RUN_Flag;
extern SlSockAddrIn_t Addr_UDP;
extern _i16 Sd_UDP;
void MakeWaringdata2APP()
{
  //打包头部
  warningdata[0]=warningdata[1]=warningdata[2]=warningdata[3]=0xaa;//起始标志
  warningdata[4]=recvBuffFromMCU[(readIndex +8)%UART_BUF_SIZE];warningdata[5]=recvBuffFromMCU[(readIndex +9)%UART_BUF_SIZE];warningdata[6]=0;//消息长度
  //响应标志
  //序列号
  warningdata[10]=recvBuffFromMCU[(readIndex +10)%UART_BUF_SIZE];warningdata[11]=recvBuffFromMCU[(readIndex + 11)%UART_BUF_SIZE];//服务类型消息类型
  //if(warningdata[10] == MSG_WARNING_REQ)
  //{
    warningdata[8]=warningdata[9]=0;//序列号
  //}
  //else
  //{
    //warningdata[7] = 2;//回复
    //warningdata[8]=warningdata[9]=0;//序列号
  //}
  //打包体
  int length = warningdata[4] + warningdata[5]*256;
  if(length > 16)
  {
    for(int i = 16 ; i < length ; i++)
      warningdata[i] =  recvBuffFromMCU[(readIndex + i)%UART_BUF_SIZE];
    unsigned int crcValue = GetCrc32(&warningdata[16] , length -16);
    warningdata[12]=crcValue&0xff;
    warningdata[13]=(crcValue>>8)&0xff;
    warningdata[14]=(crcValue>>16)&0xff;
    warningdata[15]=(crcValue>>24)&0xff;//CRC校验
  }
  else if(length == 16)
  {
    warningdata[12]=0xbb;
    warningdata[13]=0xbb;
    warningdata[14]=0xbb;
    warningdata[15]=0xbb;//CRC校验
  }
  else
  {
  
  }
}
#endif

void UARTIntHandler()
{
    interruptFlag=1;
    lRetVal = MAP_UARTIntStatus(UARTA1_BASE, 1);
    if(lRetVal & UART_INT_OE) //FIFO 溢出中断
    {
       MAP_UARTIntClear(UARTA1_BASE, UART_INT_OE);
     }
     if(lRetVal & UART_INT_RT) //串口空闲中断，（已使能此中断，但一直不进）
     {
       if((writeIndex >= readIndex) && ((writeIndex - readIndex) < UART_BUF_SIZE))
       {
         while(UARTCharsAvail(UARTA1_BASE))
         {
            recvBuffFromMCU[writeIndex%UART_BUF_SIZE] = MAP_UARTCharGet(UARTA1_BASE);
            writeIndex++;
         }
       }
     }
     if(lRetVal & UART_INT_RX) //串口接收中断
     {
       while(UARTCharsAvail(UARTA1_BASE))
       {
         if((writeIndex >= readIndex) && ((writeIndex - readIndex) < UART_BUF_SIZE))
         {
            recvBuffFromMCU[writeIndex%UART_BUF_SIZE] = MAP_UARTCharGet(UARTA1_BASE);
            writeIndex++;
         }
         else
         {
            MAP_UARTCharGet(UARTA1_BASE);//丢掉数据
         }
         
       }
     }
    MAP_UARTIntClear(UARTA1_BASE, UART_INT_RT|UART_INT_RX|UART_INT_BE|UART_INT_FE);
    interruptFlag = 0;
}

/*                       
void UART_MCU_CC3220(void *arg)
{
  while(1)
  {
    Message("\r\n UART_MCU_CC3220!");
    
    if(readIndex < writeIndex)
      {
        if((recvBuffFromMCU[readIndex%4096] == 0xcc)&&(recvBuffFromMCU[(readIndex+1)%4096] == 0xcc)&&(recvBuffFromMCU[(readIndex+2)%4096] == 0xcc)&&(recvBuffFromMCU[(readIndex+3)%4096] == 0xcc))
        {
          if(((writeIndex - readIndex) > 9) && ((writeIndex - readIndex - (recvBuffFromMCU[(readIndex + 8)%4096] + recvBuffFromMCU[(readIndex+9)%4096]*256)) > 0))//有了一针完整的数据
          {
            int length = recvBuffFromMCU[(readIndex + 8)%4096] + recvBuffFromMCU[(readIndex+9)%4096]*256;
            Message("\r\n Received data:");
            for(int i = readIndex ; i < readIndex + length ; i++)
              UART_PRINT("%2x,",recvBuffFromMCU[i%4096]&0xff);
            readIndex = readIndex + length; //移动指针
            vTaskDelay(30);
          }
        }
        else
          readIndex++;//移动指针
      }
    
    //vTaskDelay(30);
  }
}
*/
/*
 *  ======== main ========
 */
TaskHandle_t netWork;
int main(void)
{
    /* Call board init functions */
    //Board_initGeneral();
    //Board_initGPIO();
    Board_initSPI();
    //UART_init();
    PinMuxConfig();
    InitTerm();
    xTaskCreate(Network,"network",1024,(void *)&paraesNetwork,4,&netWork);
    
    
    char *paraes_UDP_TASK=NULL;
    xTaskCreate(UDP_TASK,"UDP_TASK",1024,(void *)&paraes_UDP_TASK,2,&xHandle_UDP_TASK);
    
    char *paraes_TCP_TASK=NULL;
    xTaskCreate(TCP_TASK,"TCP_TASK",1024,(void *)&paraes_TCP_TASK,3,&xHandle_TCP_TASK);
    
    char *paraes_WRITE_FILE_TASK=NULL;
    xTaskCreate(WRITE_FILE_TASK,"WRITE_FILE_TASK",1024,(void *)&paraes_WRITE_FILE_TASK,3,&xHandle_WRITE_FILE_TASK);
    
    
    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();
    
    while(1)
    {
      Message("\r\n main process!");
    }
    
    return (0);
}

//*****************************************************************************
//
//! \brief Application defined malloc failed hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationMallocFailedHook()
{
    /* Handle Memory Allocation Errors */
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {
    }
}

void vApplicationTickHook(void)
{
    /*
     * This function will be called by each tick interrupt if
     * configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
     * added here, but the tick hook is called from an interrupt context, so
     * code must not attempt to block, and only the interrupt safe FreeRTOS API
     * functions can be used (those that end in FromISR()).
     */
}

void vPreSleepProcessing(uint32_t ulExpectedIdleTime)
{

}

//*****************************************************************************
//
//! \brief Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook(void)
{
    /* Handle Idle Hook for Profiling, Power Management etc */
}

//*****************************************************************************
//
//! \brief 	Overwrite the GCC _sbrk function which check the heap limit related
//! 		to the stack pointer.
//!			In case of freertos this checking will fail.
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
#if defined (__GNUC__)
void * _sbrk ( uint32_t delta )
{
	extern char _end; /* Defined by the linker */
	extern char __HeapLimit;
	static char *heap_end;
	static char *heap_limit;
	char *prev_heap_end;

  if (heap_end == 0) {
    heap_end = &_end;
    heap_limit = &__HeapLimit;
  }

  prev_heap_end = heap_end;
  if (prev_heap_end+delta > heap_limit) {
	  return (void *) -1L;
  }
  heap_end += delta;
  return (void *) prev_heap_end;
}
#endif
