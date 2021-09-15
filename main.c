/* Main application for LEDHat
 *
 * This program performs the crucial task of reading accelerometer values and
 * translating movement into rad patterns on a WS2812B LED strip... on a hat.
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
#include <string.h>
#include <limits.h>
#include "qfplib.h"

#include "hw.h"
#include "time.h"
#include "uart.h"
#include "ws2812b.h"
#include "i2c.h"
#include "adxl345.h"
#include "vector.h"

/*
  NOTES:

  STM32F030F4

  Accel data read takes 210us.  Data coming in at ~22Hz therefore uses 4.5ms/s,
  0.5% CPU time.

  ~10us to refill half buffer
  Half buffer is 16 LEDs, each LED takes 30us, i.e. DMA IRQs every 480us
*/

/******************************************************************************/
/* Types */

/******************************************************************************/
/* Globals */

#define NUM_LEDS 	38

#define MIN_RADIUS	16

static uint8_t framebuffer[2][3*NUM_LEDS];
static uint8_t backbuffer[3*NUM_LEDS];

static vec16_t rest_vector;

float	rest_ang_a;	/* Rest vector angle to Z axis in XZ plane */
float	rest_ang_b;	/* Rest vector angle to Z axis in YZ plane */

#define PI	3.14159265359

/* #define DEBUG */

/******************************************************************************/
/* Misc utils */

void    setup_clocks(void)
{
        RCC_ClocksTypeDef RCC_Clocks;
        RCC_GetClocksFreq(&RCC_Clocks);

        /* Without an external crystal, use the internal 8MHz RC then PLL that
	 * up x 6.
         *
         * Set HSI as clock source:
         */
        /* Enable Prefetch Buffer and set Flash Latency */
        FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
        /* HCLK = SYSCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
        /* PCLK = HCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;
        /* PLL configuration = HSI * 12 = 48 MHz */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC |
                                            RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
        RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 |
                                RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL12);
        /* Enable PLL */
        RCC->CR |= RCC_CR_PLLON;
        /* Wait till PLL is ready */
        while((RCC->CR & RCC_CR_PLLRDY) == 0) { }

        /* Select PLL as system clock source */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

        /* Wait till PLL is used as system clock source */
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL) { }
}

void	measure_vec_at_rest(vec16_t *out)
{
	const int i = 2000;
	int sx = 0, sy = 0, sz = 0;
	vec16_t	m;

	for (int j = 0; j < i; j++) {
		accel_read(&m);
		/* Note: Longer-term, average post-filtered signal to remove
		 * glitchy/high-freq components */
		sx += m.x;
		sy += m.y;
		sz += m.z;
		delay_ms(2);
	}
	out->x = sx/i;
	out->y = sy/i;
	out->z = sz/i;
}

/* qfp_fatan2 gives results in the range -pi to +pi.  Fix this to give full
 * range of 0 to 2pi.
 */
float	my_qfp_fatan2(float y, float x)
{
	float r = qfp_fatan2(y, x);

	if (qfp_fcmp((float)0.0, r) > 0) {
		r = qfp_fadd((float)2*PI, r);
	}
	return r;
}

#ifdef DEBUG
static void	test_atan(void)
{
	uint32_t t1 = qfp_float2fix(my_qfp_fatan2((float)1.0, (float)1.0), 24);
	uint32_t t2 = qfp_float2fix(my_qfp_fatan2((float)1.0, (float)-1.0), 24);
	uint32_t t3 = qfp_float2fix(my_qfp_fatan2((float)-1.0, (float)-1.0), 24);
	uint32_t t4 = qfp_float2fix(my_qfp_fatan2((float)-1.0, (float)1.0), 24);

	uart_pstr("atan test: ");
	uart_phex32(t1);
	uart_putch(',');
	uart_phex32(t2);
	uart_putch(',');
	uart_phex32(t3);
	uart_putch(',');
	uart_phex32(t4);
	uart_pstr("\r\n");

	if (t1 != 0x00c90fda ||	/* Expect pi/4 */
	    t2 != 0x025b2f90 || /* Expect 3/4 pi */
	    t3 != 0x03ed4f48 || /* Expect 5/4 pi */
	    t4 != 0x057f6f00) { /* Expect 7/4 pi */
		uart_pstr("BAD ATAN RESULT, compiler bug?\r\n");
		while (1) {};
	}
}
#endif

void	display_pattern(uint32_t p)
{
	for (int i = 0; i < NUM_LEDS; i++) {
		framebuffer[0][(i*3)+0] = i & ((p >> 16) & 0xff);
		framebuffer[0][(i*3)+1] = i & ((p >> 8) & 0xff);
		framebuffer[0][(i*3)+2] = i & (p & 0xff);
	}
	while (!ws2812_display_done()) {}
	ws2812_display(framebuffer[0], NUM_LEDS);
}

/* The LEDs are 0 to 38ish, clockwise.  When moving from one position to
 * another, this finds the 'shortest' route; returns -1 for subtract from a to
 * get to b, +1 for add to a to get to b.
 */
static int dir(int a, int b)
{
	int x = b - a;
	if (x < 0)
		x += NUM_LEDS;
	if (x > NUM_LEDS/2)
		return -1;
	else
		return 1;
}

static const uint8_t rgb_clut[] = {
#include "rgb_clut.h"
};

static uint32_t clut(uint8_t c)
{
        if (c >= 0xff) {
                return 0xffffff;
        } else {
                // rgbR,GBrg,bRGB,rgbR,...
                // c % 4 =
                // 0: 3 offset 0
                // 1: 1 offset 4 + 2 offset 0
                // 2: 2 offset 2 + 1 offset 0
                // 3: 3 offset 1
                uint32_t r = rgb_clut[c*3+0];
                uint32_t g = rgb_clut[c*3+1];
                uint32_t b = rgb_clut[c*3+2];
                return r | (g << 8) | (b << 16);
        }
}

/* */
static void 	fill_line(uint8_t *fb, int from, int to, int radius)
{
	int i = from;
	int n = dir(from, to);
	while (i != to) {
		uint32_t c = clut(radius);
		/* G, R, B */
		fb[(i*3) + 0] = (c >> 8) & 0xff;
		fb[(i*3) + 1] = (c) & 0xff;
		fb[(i*3) + 2] = (c >> 16) & 0xff;

		i += n;
		if (i >= NUM_LEDS) {
			i = 0;
		} else if (i < 0) {
			i += NUM_LEDS;
		}
	}
}

int	main(void)
{
	setup_clocks();
	setup_io();

	time_init();	/* 1KHz */

	uart_init();
	uart_pstr("UART inited, hello das world!\r\n");

	i2c_init();
	accel_init();

	ws2812_init();
	delay_ms(1);

	__enable_irq();
	led(0);

	uart_pstr("Waiting to measure... ");
	display_pattern(0x000400);		/* Red zebra */
	delay_ms(1000*5);

	uart_pstr("measuring at-rest vector... ");
	display_pattern(0x010101);		/* White zebra */

	measure_vec_at_rest(&rest_vector);
	print_vector(&rest_vector);
	uart_pstr(" Done.\r\n");

	/* Calc atan for angles to vertical normal in XY plane and in XZ
	 * plane */
	float vx, vy, vz;
	vx = qfp_int2float(rest_vector.x);
	vy = qfp_int2float(rest_vector.y);
	vz = qfp_int2float(rest_vector.z);

	rest_ang_a = my_qfp_fatan2(vx, vz);
	rest_ang_b = my_qfp_fatan2(vy, vz);

#ifdef DEBUG
	uart_phex32(qfp_float2fix(rest_ang_a, 24));
	uart_putch(',');
	uart_phex32(qfp_float2fix(rest_ang_b, 24));
	uart_pstr("\r\n");

	test_atan();
#endif

	volatile int timeleft __attribute__((unused));
	timeleft = 0;
	uint8_t disp_buf = 0;
	uint32_t tick = time_get();
	int effect = 0;
	int last_val = 0;

#ifdef DEBUG
	int minx = INT_MAX, miny = INT_MAX, minz = INT_MAX;
	int maxx = INT_MIN, maxy = INT_MIN, maxz = INT_MIN;
#endif

	while(1) {
		/*************************************************************/
		/* Throttle frame rate; sleep until we need to do something. */
#ifdef DEBUG
		const int frame_period = 100;
#else
		/* Aim for a few ms/frame. */
		const int frame_period = 7;
#endif
		uint32_t ptick = time_get();
		uint32_t ntick = wait_till_tick(tick + frame_period);

		if (ntick > (tick + frame_period)) {
			/* Indicate frame overrun to a debugger */
			timeleft = -(ntick-(tick + frame_period));
		} else {
			timeleft = ntick-ptick;
		}
		tick = ntick;

		/*************************************************************/
		/* Accelerometer sample on regular tick with (hopefully) little
		 * jitter: */
		vec16_t m;
		accel_read(&m);

		/*************************************************************/
		/* This should never be true, given the wait above. */
		while (!ws2812_display_done()) {}
		/* Display of 38 LEDs = 1.14ms */
		ws2812_display(framebuffer[disp_buf], NUM_LEDS);

		/* Render NEXT frame into other buffer */
		disp_buf ^= 1;

#ifdef DEBUG
		print_vector(&m);
		uart_putch(' ');
#endif
		/*************************************************************/
		/* Calculate orientation */
		vx = qfp_int2float(m.x);
		vy = qfp_int2float(m.y);
		vz = qfp_int2float(m.z);

		/* Measured angle from true vertical */
		float theta = qfp_fsub(my_qfp_fatan2(vx, vz), rest_ang_a);
		float rho = qfp_fsub(my_qfp_fatan2(vy, vz), rest_ang_b);

		if (qfp_fcmp(theta, (float)0.0) < 0) {
			theta = qfp_fadd(theta, (float)2*PI);
		}
		if (qfp_fcmp(rho, (float)0.0) < 0) {
			rho = qfp_fadd(rho, (float)2*PI);
		}

#ifdef DEBUG
		uart_pstr(" [");
		uart_phex32(qfp_float2fix(theta, 24));
		uart_putch(',');
		uart_phex32(qfp_float2fix(rho, 24));
		uart_pstr("] ");
#endif

		/* Project the measured vector into the XY plane to get a
		 * bearing (e.g.  phi here).  We can use |m| to give brightness.
		 */
		float sintheta = qfp_fsin(theta);
		float sinrho = qfp_fsin(rho);
		float phi = my_qfp_fatan2(sintheta, sinrho);

		/* And the 'length' of that 2D vector when viewed from the top is
		 * qfp_fsqrt[_fast](qfp_fadd(qfp_fmul(sintheta, sintheta),
		 *                  qfp_fmul(sinrho, sinrho)));
		 * (on a scale of 0.0 to sqrt(2))
		 *
		 * Now, this can be used to see how much the board is tilting
		 * from straight vertical (which is very noisy) and to scale the
		 * LED brightness with a threshold/cut-off if the hat's too
		 * vertical -- this removes jittery straight-up vector noise.
		 */
		float frad = qfp_fsqrt(qfp_fadd(qfp_fmul(sintheta, sintheta),
						qfp_fmul(sinrho, sinrho)));
		int radius = qfp_float2int(qfp_fmul(181.019335983756,
						    frad)); /* 256/sqrt(2) */

		/*************************************************************/
		/* Display patterns/graphics */

		/* Basic:  Convert from radians to numleds: */
		int led = qfp_float2int(qfp_fdiv(qfp_fmul(phi, (float)NUM_LEDS),
						 (float)(PI * 2)));

		if (effect == 0) {
			/* LEDs go the other way round... */
			led = NUM_LEDS-1 - led;

			if (led >= NUM_LEDS)
				led = NUM_LEDS - 1; /* JIC */
			/* Rotate 90 degrees... crudely */
			led -= (NUM_LEDS/4);
			if (led < 0)
				led += NUM_LEDS;

			led = NUM_LEDS - 1 - led;
			if (radius > MIN_RADIUS) {
				radius *= 1.5;
				if (radius > 255)
					radius = 255;

				if (led != last_val) {
					fill_line(backbuffer, last_val, led, radius);
					last_val = led;
				}
			}
		} else {
			backbuffer[((led % NUM_LEDS)*3)+2] = 16;
		}
#ifdef DEBUG
		uart_phex32(led);

		if (m.x < minx)		minx = m.x;
		if (m.x > maxx)		maxx = m.x;
		if (m.y < miny)		miny = m.y;
		if (m.y > maxy)		maxy = m.y;
		if (m.z < minz)		minz = m.z;
		if (m.z > maxz)		maxz = m.z;

		uart_pstr(" {");
		uart_phex32(minx);
		uart_putch(',');
		uart_phex32(maxx);
		uart_putch(' ');
		uart_phex32(miny);
		uart_putch(',');
		uart_phex32(maxy);
		uart_putch(' ');
		uart_phex32(minz);
		uart_putch(',');
		uart_phex32(maxz);
		uart_pstr("}\r\n");
#endif

		/* And fade out (again): */
		for (int i = 0; i < NUM_LEDS; i++) {
			uint8_t *c;
			c = &backbuffer[(i*3)+0];
			if (*c != 0) (*c)--;
			c = &backbuffer[(i*3)+1];
			if (*c != 0) (*c)--;
			c = &backbuffer[(i*3)+2];
			if (*c != 0) (*c)--;
		}

		/* The display is double-buffered.  We maintain backbuffer so
		 * that we can have a single framebuffer in which to do
		 * accumulation/fade effects.
		 */
		memcpy(framebuffer[disp_buf], backbuffer, NUM_LEDS*3);
	}
}
