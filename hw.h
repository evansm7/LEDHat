/* Hardware defines, init, for LEDHat
 *
 * Copyright (C) 2015, 2021 Matt Evans
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#ifndef hw_H
#define hw_H

#include <stm32f0xx_gpio.h>


#define B(x) (1 << (x))
#define B2(x, y) ((x) << ((y)*2))

/* PA4 is LED.
 * PA9/10 are wired to UART header but are only pins available for I2C; PA9=SCL, PA10=SDA.
 * PA2/3 alternates for UART...?
 */

#define UART1_BAUD 	115200
#define SYS_CLK		48000000

/* PA4 for LED out, PA1 for button in */
static void	setup_io(void) __attribute__((unused));
static void	setup_io(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

static int  button_pressed(void)  __attribute__((unused));
static int  button_pressed(void)
{
	return !(GPIOA->IDR & B(1));
}

static void led(int o) __attribute__((unused));
static void led(int o)
{
	if (o)
		GPIOA->BSRR = GPIO_Pin_4;
	else
		GPIOA->BRR = GPIO_Pin_4;
}


#endif
