/* Simple UART/debug IO routines for STM32F0
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
#include <stm32f0xx_gpio.h>

#include "uart.h"
#include "hw.h"

void uart_putch(char c)
{
	while ((USART1->ISR & USART_ISR_TXE) == 0) {}

	USART1->TDR = c;
}

char uart_getch(void)
{
	return USART1->RDR;
}

int uart_ch_rdy(void)
{
	return !!(USART1->ISR & USART_ISR_RXNE);
}

void uart_phex32(unsigned int w)
{
	char hexbuff[9];
	int i;
	for (i = 0; i < 8; i++) {
		unsigned int b = (w >> ((7-i) * 4)) & 0xf;
		hexbuff[i] = (b > 9) ? b + 'a' - 10 : b + '0';
	}
	hexbuff[8] = 0;
	uart_pstr(hexbuff);
}

void uart_init(void)
{
	/* Use PA2=TX, PA3=RX as PA9/PA10 are for I2C.  USART1 = AF1. */
	GPIO_InitTypeDef  	GPIO_InitStructure;

	RCC->AHBENR |= RCC_AHBPeriph_GPIOA;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_1);

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	USART1->CR1 = 0; /* 8N */
	USART1->BRR = SYS_CLK/UART1_BAUD;
	USART1->CR2 = 0; /* 1 stop */
	USART1->CR1 |= USART_CR1_UE;
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
}
