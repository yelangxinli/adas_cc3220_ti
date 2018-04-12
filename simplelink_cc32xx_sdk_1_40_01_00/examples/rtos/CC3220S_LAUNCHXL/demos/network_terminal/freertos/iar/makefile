SIMPLELINK_CC32XX_SDK_INSTALL_DIR ?= $(abspath ../../../../../../..)

include $(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/imports.mak

KERNEL_BUILD := $(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/kernel/freertos/builds/CC3220S_LAUNCHXL/release

CC = "$(IAR_ARMCOMPILER)/bin/iccarm"
LNK = "$(IAR_ARMCOMPILER)/bin/ilinkarm"

OBJECTS = uart_term.obj main_freertos.obj cmd_parser.obj netapp_cmd.obj network_terminal.obj radio_tool.obj socket_cmd.obj str.obj transceiver_cmd.obj wlan_cmd.obj CC3220S_LAUNCHXL.obj

NAME = network_terminal

CFLAGS = -I../.. "-I$(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/source" "-I$(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/kernel/freertos/posix" "-I$(FREERTOS_INSTALL_DIR)/FreeRTOS/Source/include" "-I$(FREERTOS_INSTALL_DIR)/FreeRTOS/Source/portable/IAR/ARM_CM3" "-I$(KERNEL_BUILD)" --cpu=Cortex-M4 --debug --silent -e --aeabi --thumb --diag_suppress=Pa050 "-I$(IAR_ARMCOMPILER)/inc/c"

LFLAGS = "$(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/source/ti/display/lib/display.arm4" "$(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/source/ti/drivers/lib/drivers_cc32xx.arm4" "$(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/source/third_party/fatfs/lib/fatfs.arm4" "$(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/source/ti/drivers/net/wifi/iar/rtos/simplelink.a" "$(KERNEL_BUILD)/iar/freertos.lib" "$(SIMPLELINK_CC32XX_SDK_INSTALL_DIR)/source/ti/devices/cc32xx/driverlib/iar/Release/driverlib.a" --config ../../freertos/iar/CC3220S_LAUNCHXL_FREERTOS.icf --map "$(NAME).map" --silent --cpu=Cortex-M4 --redirect _Printf=_PrintfSmall --redirect _Scanf=_ScanfSmall --semihosting=iar_breakpoint

all: $(NAME).out

$(KERNEL_BUILD)/iar/freertos.lib:
	@ $(ECHOBLANKLINE)
	@ echo $(abspath $@) is not built.
	@ echo You can build it by issuing $(MAKE) in $(abspath $(dir $@)).
	@ $(ECHOBLANKLINE)

uart_term.obj: ../../uart_term.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

main_freertos.obj: ../../freertos/main_freertos.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

cmd_parser.obj: ../../cmd_parser.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

netapp_cmd.obj: ../../netapp_cmd.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

network_terminal.obj: ../../network_terminal.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

radio_tool.obj: ../../radio_tool.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

socket_cmd.obj: ../../socket_cmd.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

str.obj: ../../str.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

transceiver_cmd.obj: ../../transceiver_cmd.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

wlan_cmd.obj: ../../wlan_cmd.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

CC3220S_LAUNCHXL.obj: ../../CC3220S_LAUNCHXL.c
	@ echo Building $@
	@ $(CC) $(CFLAGS) $< -o $@

$(NAME).out: $(OBJECTS) $(KERNEL_BUILD)/iar/freertos.lib
	@ echo linking...
	@ $(LNK)  $(OBJECTS) $(LFLAGS) -o $(NAME).out

clean:
	@ echo Cleaning...
	@ $(RM) $(OBJECTS) > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).out > $(DEVNULL) 2>&1
	@ $(RM) $(NAME).map > $(DEVNULL) 2>&1
