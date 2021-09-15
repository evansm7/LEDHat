# Makefile for LEDHat firmware
#
# This makefile builds two things: firmware for the MCU and a 'sim'
# binary that's an OpenGL app for testing.
#
# Copyright (c) 2015, 2018, 2021 Matt Evans
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#

# This project uses QFPLIB, fetched from:
QFPLIB_VER=20200617
QFPLIB_BUNDLE=qfplib-$(QFPLIB_VER)
QFPLIB_ARC=$(QFPLIB_BUNDLE).tar.gz
QFPLIB_URL=https://www.quinapalus.com/$(QFPLIB_ARC)

ifeq ($(V), 1)
	VERBOSE=
else
	VERBOSE=@
endif

CROSS_COMPILE ?= /usr/local/gcc-arm-embedded/bin/arm-none-eabi-
STM_LIB ?= /Users/matt/code/STM32F0/STM32F0xx_StdPeriph_Lib_V1.0.0/

################################################################################

CC = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
OBJCOPY = $(CROSS_COMPILE)objcopy
SIZE = $(CROSS_COMPILE)size

CMSIS = $(STM_LIB)/Libraries/CMSIS
STMLIB = $(STM_LIB)/Libraries/STM32F0xx_StdPeriph_Driver
INCLUDES = -I$(CMSIS)/ST/STM32F0xx/Include
INCLUDES += -I$(CMSIS)/Include
INCLUDES += -I$(CMSIS)/Device/ST/STM32F0xx/Include
INCLUDES += -I$(STMLIB)/inc
INCLUDES += -I.

CFLAGS = -mthumb -mcpu=cortex-m0 -Os -std=c99 -Wall
CFLAGS += -g -ggdb -ffreestanding -msoft-float
CFLAGS += -DUSE_STDPERIPH_DRIVER=1 $(INCLUDES)
CFLAGS += -fdata-sections -ffunction-sections
LIBS = -lgcc
# Note:  The -march/-mthumb items are must-haves, to select v6m crtX.o and
# libgcc.a!
LINKFLAGS = -Wl,--gc-sections -march=armv6-m -mthumb --specs=nano.specs
LINKFLAGS += -Wl,-T stm32f030x4.ld

OBJECTS = me_startup_stm32f0xx.o system_stm32f0xx.o stm32f0xx_rcc.o stm32f0xx_tim.o stm32f0xx_gpio.o stm32f0xx_dma.o stm32f0xx_misc.o stm32f0xx_i2c.o stm32f0xx_exti.o stm32f0xx_syscfg.o
OBJECTS += main.o uart.o time.o ws2812b.o i2c.o adxl345.o
OBJECTS += qfplib.s

################################################################################

.PHONY: all
all:	main.fl.bin

.PHONY: clean
clean:
	rm -rf *.o *~ *.bin *.elf rgb_clut.h qfplib.* $(QFPLIB_BUNDLE)

%.bin:	%.elf
	@$(SIZE) $<
	@echo "[OBJ] $@"
	$(VERBOSE)$(OBJCOPY) $< -O binary $@

main.fl.elf: $(OBJECTS)
	@echo "[LINK]  $@"
	$(VERBOSE)$(CC) $(LINKFLAGS) $^ $(LIBS) -o $@
	$(SIZE) $@

main.o:	main.c rgb_clut.h qfplib.s qfplib.h
	@echo "[CC]  $@"
	$(VERBOSE)$(CC) $(CFLAGS) -c $< -o $@
%.o:	%.c
	@echo "[CC]  $@"
	$(VERBOSE)$(CC) $(CFLAGS) -c $< -o $@
%.o:	%.s
	@echo "[AS]  $@"
	$(VERBOSE)$(CC) $(CFLAGS) -c $< -o $@
%.o:	%.S
	@echo "[AS]  $@"
	$(VERBOSE)$(CC) $(CFLAGS) -c $< -o $@

%.o:	$(CMSIS)/Device/ST/STM32F0xx/Source/Templates/%.c
	@echo "[CC]  $@"
	$(VERBOSE)$(CC) $(CFLAGS) -c $< -o $@
%.o:	$(STMLIB)/src/%.c
	@echo "[CC]  $@"
	$(VERBOSE)$(CC) $(CFLAGS) -c $< -o $@

rgb_clut.h:	./mkclut.pl
	./mkclut.pl > $@

# Gubbins for fetching QFPLIB:
$(QFPLIB_ARC):
	wget $(QFPLIB_URL)
$(QFPLIB_BUNDLE): $(QFPLIB_ARC)
	tar xvf $(QFPLIB_ARC)
$(QFPLIB_BUNDLE)/qfplib.s:	$(QFPLIB_BUNDLE)
$(QFPLIB_BUNDLE)/qfplib.h:	$(QFPLIB_BUNDLE)
qfplib.s:	$(QFPLIB_BUNDLE)/qfplib.s
	ln -sf $(QFPLIB_BUNDLE)/qfplib.s .
qfplib.h:	$(QFPLIB_BUNDLE)/qfplib.h
	ln -sf $(QFPLIB_BUNDLE)/qfplib.h .

prog:	main.fl.bin
	st-flash --reset write $< 0x08000000
