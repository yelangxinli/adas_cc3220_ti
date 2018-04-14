#ifndef _GETTIME_H_
#define	_GETTIME_H_

#include <stdint.h>
#include <stdlib.h>
/* POSIX Header files */
#include <pthread.h>
/* RTOS header files */
#include "FreeRTOS.h"
#include "task.h"
/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
/* Example/Board Header files */
#include "Board.h"
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/devices/cc32xx/driverlib/spi.h>
#include <ti/devices/cc32xx/driverlib/rom.h>
#include <ti/devices/cc32xx/driverlib/rom_map.h>
#include <ti/devices/cc32xx/inc/hw_memmap.h>
#include <ti/devices/cc32xx/driverlib/prcm.h>
#include <ti/devices/cc32xx/inc/hw_mcspi.h>
#include <ti/devices/cc32xx/driverlib/hwspinlock.h>
#include <ti/devices/cc32xx/inc/hw_types.h>
#include <ti/devices/cc32xx/driverlib/pin.h>
#include <ti/devices/cc32xx/driverlib/gpio.h>
#include <ti/devices/cc32xx/driverlib/uart.h>
#include <ti/devices/cc32xx/inc/hw_uart.h>
/* Example/Board Header files */
#include "network_terminal.h"
#include "cmd_parser.h"
#include "wlan_cmd.h"
#include "netapp_cmd.h"
#include "socket_cmd.h"
#include "transceiver_cmd.h"

//#include <user.h>
#include <ti/drivers/net/wifi/porting/user.h>

/* Application defines */
#define SIX_BYTES_SIZE_MAC_ADDRESS  (17)
#define RESPONSE_TEXT "Example text to be displayed in browser"
#define         get_TIME       0
#define         set_TIME        1

typedef struct {
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int weekdays;
} my_tm;

int32_t showAvailableCmd();
int32_t cmd_prompt(void *arg);
int32_t cmdClearcallback(void *arg);
int32_t printClearUsage(void *arg);
int32_t cmdHelpCallback(void *arg);
int32_t printHelpUsage(void *arg);
int32_t initAppVariables();
int32_t sem_wait_timeout(sem_t *sem, uint32_t Timeout);
void   *MainThread(void *arg);

#endif //_GETTIME_H_

