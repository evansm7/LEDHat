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
#ifndef UART_H
#define UART_H

void uart_putch(char c);
char uart_getch(void);
int uart_ch_rdy(void);
static inline void uart_pstr(char *s)
{
	while (*s)
		uart_putch(*s++);
}
void uart_phex32(unsigned int w);

void uart_init(void);


#endif
