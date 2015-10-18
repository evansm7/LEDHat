#include <inttypes.h>
#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>

#include "hw.h"
#include "time.h"
#include "uart.h"
#include "ws2812b.h"
#include "i2c.h"

/*
  NOTES:

STM32F030F4

*/

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

const uint8_t colours[] = { 0xaa, 0x55, 0x11,
			    0xff, 0x00, 0xff };

int	main(void)
{
	setup_clocks();
	setup_io();

	time_init();	/* 1KHz */

	uart_init();
	uart_pstr("UART inited, hello das world!\r\n");

	i2c_init();
	
	ws2812_init();
	delay_ms(1);

	__enable_irq();
	led(0);
	while(1) {
		delay_ms(1000);
		ws2812_display(colours, 2);
		/* for (int i = 1; i < 100; i++) { */
		/* 	for (int j = 0; j < 100-i; j++) { */
		/* 		delay_ms(i); */
		/* 		led(1); */
		/* 		delay_ms(i); */
		/* 		led(0); */
		/* 	} */
		/* } */
	}
}
