# LEDHat firmware

15 September 2021

OK, so, er, I built a hat with LEDs on it but we all make questionable decisions from time to time.

Should you want to make similarly bold fashion statements, this code might help.

Using a cheap STM32F031 microcontroller, a cheap ADXL345 accelerometer, and a horizontal hoop of WS2812B LEDs, this code implements a kind of "spirit level"/air bubble effect:  a flash of light follows the highest part of the brim as the hat is tilted.

Bits you can steal:

   * STM32F0 timekeeping, UART stuff
   * DMA-driven WS2812B code: high-performance and background updates, and zero-flicker
   * Vector/trig code for measuring tilt direction using an accelerometer

Have fun!

https://axio.ms/


## License

### Main code, WS2812/vector stuff
Copyright (C) 2015, 2021 Matt Evans

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

### Other parts/credits

   * stm32f030x4.ld is hacked from an original from https://github.com/antongus/stm32-ld-scripts (MIT-licenced)
   * Uses qfplib from https://www.quinapalus.com/qfplib-m0-tiny.html
   * `i2c.c` uses a handful of code from the ST STM32 StdPeriphLib examples
