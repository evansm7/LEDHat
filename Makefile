CROSS_COMPILE=/usr/local/gcc-arm-embedded/bin/arm-none-eabi-
CC=$(CROSS_COMPILE)gcc
LD=$(CROSS_COMPILE)ld
OBJCOPY=$(CROSS_COMPILE)objcopy
SIZE=$(CROSS_COMPILE)size

CMSIS=../STM32F0xx_StdPeriph_Lib_V1.0.0/Libraries/CMSIS
STMLIB=../STM32F0xx_StdPeriph_Lib_V1.0.0/Libraries/STM32F0xx_StdPeriph_Driver
INCLUDES = -I$(CMSIS)/ST/STM32F0xx/Include
INCLUDES += -I$(CMSIS)/Include
INCLUDES += -I$(CMSIS)/Device/ST/STM32F0xx/Include
INCLUDES += -I$(STMLIB)/inc
INCLUDES += -I.

CFLAGS=-mthumb -mcpu=cortex-m0 -std=c99 -Os -g -ggdb $(INCLUDES) -ffreestanding -msoft-float -DUSE_STDPERIPH_DRIVER=1
CFLAGS += -fdata-sections -ffunction-sections
LIBS=-lgcc
# Note:  The -march/-mthumb items are must-haves, to select v6m crtX.o and
# libgcc.a!
LINKFLAGS=-Wl,--gc-sections -march=armv6-m -mthumb --specs=nano.specs

OBJECTS = me_startup_stm32f0xx.o system_stm32f0xx.o stm32f0xx_rcc.o stm32f0xx_tim.o stm32f0xx_gpio.o stm32f0xx_dma.o stm32f0xx_misc.o stm32f0xx_i2c.o
OBJECTS += main.o uart.o time.o ws2812b.o i2c.o vector.o

all:	main.fl.bin

clean:
	rm -f *.o *~ *.bin *.elf

%.bin:	%.elf
	$(OBJCOPY) $< -O binary $@

main.fl.elf: $(OBJECTS)
	$(CC) $(LINKFLAGS) -Wl,-T stm32_flash.ld $^ $(LIBS) -o $@
	$(SIZE) $@

%.o:	%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o:	%.s
	$(CC) $(CFLAGS) -c $< -o $@
%.o:	%.S
	$(CC) $(CFLAGS) -c $< -o $@

%.o:	$(CMSIS)/Device/ST/STM32F0xx/Source/Templates/%.c
	$(CC) $(CFLAGS) -c $< -o $@
%.o:	$(STMLIB)/src/%.c
	$(CC) $(CFLAGS) -c $< -o $@

prog:	main.fl.bin
	st-flash --reset write $< 0x08000000
