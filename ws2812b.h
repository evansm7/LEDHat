/* DMA-driven WS2812B LED driver for STM32F0 microcontrollers
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
#ifndef WS2812_H
#define WS2812_H

/* Call this first, to initialise: */
void	ws2812_init(void);

/* Call this to update the display asynchronously:
 * - Buffer byte order is Green, Red, Blue, Green, Red, Blue
 * - Pass number of three-byte colour tuples (i.e. number of LEDs)
 */
void	ws2812_display(uint8_t *buffer, int num);

/* Returns true when the update is done */
int	ws2812_display_done(void);

#endif
