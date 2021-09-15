/* Simple vector routines for LEDHat
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
#ifndef VECTOR_H
#define VECTOR_H

#include <inttypes.h>
#include "uart.h"

typedef struct {
	int16_t x,y,z;
} vec16_t;

/* a - b */
static inline vec16_t	vec_sub(vec16_t a, vec16_t b)
{
	vec16_t r = { a.x-b.x, a.y-b.y, a.z-b.z };
	return r;
}

/* a + b */
static inline vec16_t	vec_add(vec16_t a, vec16_t b)
{
	vec16_t r = { a.x+b.x, a.y+b.y, a.z+b.z };
	return r;
}

static inline void	print_vector(vec16_t *in)
{
	uart_phex32(in->x); uart_putch(':');
	uart_phex32(in->y); uart_putch(':');
	uart_phex32(in->z);
}

#endif
