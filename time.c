/* Simple SysTick timekeeping for STM32F0
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

#include <inttypes.h>
#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include "time.h"
#include "hw.h"

/* This rolls over every 46 weeks.  For *this application*, there's no point
 * using a 64-bit type (but for something that's not explicitly expected to be
 * switched off every day, this is not a good assumption).
 */
static volatile uint32_t global_time = 0;

void	delay_ms(int d)
{
	uint32_t tn = global_time;

	while ((global_time - tn) < d) {
		__WFI();
	}
}

void	delay_us(int d)
{
	d *= 48/3;
	__asm__ volatile ("1: sub %0, #1 ; cmp %0, #0; bgt 1b" : : "r"(d));
}

/* WFI until the given tick occurs, returning the time at that point (which will
 * either be the given tick, or if that was in the past, the current time)
 */
uint32_t wait_till_tick(uint32_t tick)
{
	uint32_t tn;
	while ((tn = global_time) < tick) {
		__WFI();
	}
	return tn;
}

void SysTick_Handler(void)
{
	global_time++;
}

uint32_t time_get(void)
{
        return global_time;
}

void	time_init(void)
{
	SysTick_Config(SYS_CLK/1000);	/* 1KHz */
}
