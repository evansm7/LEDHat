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

	uart_pstr("Hello das world!\n");
}
