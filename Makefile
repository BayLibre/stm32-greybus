PREFIX ?= arm-none-eabi-
CFLAGS = -mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=soft -mno-unaligned-access \
	 -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-builtin \
	 -w -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wpointer-arith \
	 -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal  -g3 -ggdb -DSTM32F407xx -std=gnu11
LDFLAGS = -T "STM32F407VGTx_FLASH.ld" -nostartfiles -nodefaultlibs -nostdlib -Xlinker \
	  --gc-sections -Wl,-Map,"stm32-greybus.map" --specs=nano.specs

TARGET = stm32-greybus

BUILD_DIR ?= build

SRC_PATH = Src
SRC_FILES =  gpio.c \
	     libc_stubs.c \
	     stm32f4xx_hal_msp.c \
	     usart.c \
	     usb_device.c \
	     main.c \
	     stm32f4xx_it.c \
	     i2c.c \
	     spi.c \
	     tim.c \
	     usbd_desc.c \
	     usbd_conf.c \
	     usbd_greybus.c \
	     nunchuk.c
GREYBUS_PATH = Src/greybus
GREYBUS_FILES = gpio.c \
		light.c \
	     	spi.c \
	     	control.c \
	     	i2c.c \
	     	pwm.c \
	     	connection.c \
	     	hid.c \
	     	manifest.c \
	     	svc.c
USB_PATH = Middlewares/ST/STM32_USB_Device_Library/Core/Src
USB_FILES = usbd_core.c  usbd_ctlreq.c  usbd_ioreq.c
CMSIS_PATH = Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates
CMSIS_FILES = system_stm32f4xx.c
CMSIS_GCC_PATH = Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc
CMSIS_GCC_FILES = startup_stm32f407xx.S
HAL_PATH = Drivers/STM32F4xx_HAL_Driver/Src
HAL_FILES = stm32f4xx_hal.c \
	    stm32f4xx_hal_cortex.c \
	    stm32f4xx_hal_dma.c \
	    stm32f4xx_hal_dma_ex.c \
	    stm32f4xx_hal_flash.c \
	    stm32f4xx_hal_flash_ex.c \
	    stm32f4xx_hal_flash_ramfunc.c \
	    stm32f4xx_hal_gpio.c \
	    stm32f4xx_hal_i2c.c \
	    stm32f4xx_hal_i2c_ex.c \
	    stm32f4xx_hal_pcd.c \
	    stm32f4xx_hal_pcd_ex.c \
	    stm32f4xx_hal_pwr.c \
	    stm32f4xx_hal_pwr_ex.c \
	    stm32f4xx_hal_rcc.c \
	    stm32f4xx_hal_rcc_ex.c \
	    stm32f4xx_hal_rtc.c \
	    stm32f4xx_hal_rtc_ex.c \
	    stm32f4xx_hal_spi.c \
	    stm32f4xx_hal_tim.c \
	    stm32f4xx_hal_tim_ex.c \
	    stm32f4xx_hal_uart.c \
	    stm32f4xx_ll_usb.c

INCLUDE_PATHS = Inc \
		Drivers/STM32F4xx_HAL_Driver/Inc \
		Drivers/CMSIS/Include \
		Drivers/CMSIS/Device/ST/STM32F4xx/Include \
		Middlewares/ST/STM32_USB_Device_Library/Core/Inc

######################################

CC = $(PREFIX)gcc

BUILD_DIRS = $(BUILD_DIR)/$(SRC_PATH) \
	     $(BUILD_DIR)/$(GREYBUS_PATH) \
	     $(BUILD_DIR)/$(USB_PATH) \
	     $(BUILD_DIR)/$(CMSIS_PATH) \
	     $(BUILD_DIR)/$(CMSIS_GCC_PATH) \
	     $(BUILD_DIR)/$(HAL_PATH)

INCLUDES = $(INCLUDE_PATHS:%=-I%)

CFILES = $(SRC_FILES:%=$(SRC_PATH)/%) \
	 $(GREYBUS_FILES:%=$(GREYBUS_PATH)/%) \
	 $(USB_FILES:%=$(USB_PATH)/%) \
	 $(CMSIS_FILES:%=$(CMSIS_PATH)/%) \
	 $(HAL_FILES:%=$(HAL_PATH)/%)
SFILES = $(CMSIS_GCC_FILES:%=$(CMSIS_GCC_PATH)/%) \

OBJS = $(CFILES:%.c=$(BUILD_DIR)/%.o) $(SFILES:%.S=$(BUILD_DIR)/%.o)

all: $(BUILD_DIRS) $(TARGET)

$(TARGET): $(OBJS)
	@echo LD $@
	@$(CC) $(CFLAGS) $(LDFLAGS) $(OBJS) -o $@
	@-$(PREFIX)objcopy -O ihex $@  $@.hex
	@-$(PREFIX)size --format=berkeley $@

$(BUILD_DIRS):
	@echo MKDIR $@
	@mkdir -p $@

$(BUILD_DIR)/%.o: %.c
	@echo CC $@
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(BUILD_DIR)/%.o: %.S
	@echo CC $@
	@$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

.PHONY: clean help all

clean:
	@echo RM $(BUILD_DIR) $(TARGET)
	@-rm -fr $(BUILD_DIR) $(TARGET)

help:
	@echo Use PREFIX to pass cross compiler PATH
	@echo Use BUILD_DIR to change build directory
	@echo all: build all targets
	@echo $(TARGET): build binary
	@echo clean: clean build directory
