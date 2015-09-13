#include <stm32f0xx_gpio.h>
#include <inttypes.h>
#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_dma.h>

#include "hw.h"
#include "ws2812b.h"

/* PB1: TIM1_CH3N (AF2)
   WS2812B:  1.25us cycle, either 0.35 high + 0.9 low (0) or 0.9 high + 0.35 low (1)
   1.25us +/- 0.15 = 800MHz.

   So 10 ticks total at 8MHz, either 3+7 or 7+3?
   
*/

#define BUFFER_FULL_LEN		20
#define BUFFER_HALF		(BUFFER_FULL_LEN/2)

static uint8_t buffer[BUFFER_FULL_LEN] = {0};
static int transfer_bottntop;
static int input_buffer_pos;

static int total_transfer_size = 20;

static inline uint8_t input_buffer_val(uint8_t *buffer, unsigned int pos)
{
	// TBD
	return pos;
}

static void ws_timer_init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	/* 24MHz sysclock -> 800KHz period: */
	TIM_TimeBaseStructure.TIM_Period = 30;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_ARRPreloadConfig(TIM1, ENABLE);

	/* Channel 3 output configuration */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	/* This is CCR3, time high until low: */
	TIM_OCInitStructure.TIM_Pulse = 10; //22 or 8
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	/* When DMA occurs to the TIM1->DMAR address, it hits the CCR3 reg. */
	TIM_DMAConfig(TIM1, TIM_DMABase_CCR3, TIM_DMABurstLength_1Transfer);
	TIM_DMACmd(TIM1, TIM_DMA_CC3, ENABLE);
	
	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	
	/* TIM1 counter enable */
//	TIM_Cmd(TIM1, ENABLE);
}

static void	ws_irq_init(uint32_t enable)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = enable;
	NVIC_Init(&NVIC_InitStructure);
}

static void	ws_dma_init(void)
{
	DMA_InitTypeDef  DMA_InitStructure;

	RCC->AHBENR |= RCC_AHBPeriph_DMA1;

	/* To write to TIM1_CH3, use DMA1_channel5 */
	DMA_DeInit(DMA1_Channel5);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uintptr_t)&TIM1->CCR3;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uintptr_t)buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	// buffersize
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	// Mode
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_BufferSize = BUFFER_FULL_LEN;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel5, ENABLE);
}

void	ws2812_init(void)
{
	GPIO_InitTypeDef  	GPIO_InitStructure;

	for (int i = 0; i < BUFFER_FULL_LEN; i++) {
		buffer[i] = i;
	}

	RCC->AHBENR |= RCC_AHBPeriph_GPIOB;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/* Select TIM1_CH3N: */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_2);

	ws_timer_init();
	ws_dma_init();
	ws_irq_init(ENABLE);

	led(0);

	
	/* This technique deals with 'half-buffers', for which we get an
	 * interrupt just after output.  There are these possibilities for total
	 * output buffer sizes:
	 *
	 *   |           |           |           |
	 *   +-----+-----+-----+-----+-----+-----+-----+- ...
	 *   |0    |1    |2    |3    |4    |5    |     |
	 *   +-----+-----+-----+-----+-----+-----+-----+-- ...
	 * A |==|
	 * B |========|
	 * C |===================|000000000000000
	 *
	 * A & B are both less than a full buffer, so DMA wouldn't wrap at all
	 * anyway.  For these, a non-circular non-repeating DMA is selected.
	 *
	 * C is greater than a full buffer.  On initialisation, first buffer is
	 * emitted and DMA starts.  After 0 is done, IRQ fills part 2.  Another
	 * IRQ (1 done), IRQ fills 3 but finishes original output data here and
	 * pads to end with zero.  Need to wait for another IRQ (2 done) and the
	 * one after (3 done) by which point buffers 4 & 5 have been submitted
	 * as zero, before stopping DMA.
	 *
	 * +------+------+
	 * |0     |1     |
	 * +^-----+------+ then IRQ 0 finished, fill 2
	 *
	 * +------+------+
	 * |2     |1     |
	 * +------+^-----+ then IRQ 1 finished, fill 3 (pad)
	 * 
	 * +------+------+
	 * |2     |3  000|
	 * +^-----+------+ then IRQ 2 finished, fill 4 (zero)
	 * 
	 * +------+------+
	 * |4 0000|3  000|
	 * +------+^-----+ then IRQ 3 finished, as 4 is starting cancel DMA.
	 *
	 */

	/* Fill initial full-buffer from bottom: */
	transfer_bottntop = 0;
	int initial_amount = (total_transfer_size > BUFFER_FULL_LEN) ? BUFFER_FULL_LEN : total_transfer_size;

	for (int i = 0; i < initial_amount; i++) {
//		buffer[i] = input_buffer_val(0, i);
	}
	input_buffer_pos = initial_amount;

	if (1) {//total_transfer_size > BUFFER_FULL_LEN) {
		/* We'll need multiple buffers.  Do circular version: */
//		DMA_InitStructure.DMA_BufferSize = BUFFER_FULL_LEN;
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//		DMA_Init(DMA1_Channel5, &DMA_InitStructure);

		/* For the 'DMA half transfer' IRQ: */
	} else {
		/* We only need one buffer and no refill.  Linear version: */
		/* DMA_InitStructure.DMA_BufferSize = total_transfer_size; */
		/* DMA_InitStructure.DMA_Mode = DMA_Mode_Normal; */
		/* DMA_Init(DMA1_Channel5, &DMA_InitStructure); */
  
		/* /\* No DMA IRQ: *\/ // FIXME, want an actual completeness one? */
		/* NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE; */
		/* NVIC_Init(&NVIC_InitStructure); */
	}

	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);
}

void DMA1_Channel4_5_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA1_IT_HT5)) {
		led(1);
		/* Clear DMA Half Transfer pending */
		DMA_ClearITPendingBit(DMA1_IT_HT5);
		return;
		/* Have transferred 'BUFFER_HALF' items from either the bottom
		 * or top half of the buffer.  Refill entries into this half (as
		 * the other half is now being clocked out).
		 */
		int startpoint = transfer_bottntop ? 0 : BUFFER_HALF;
		int entries_to_go = total_transfer_size - 1 - input_buffer_pos;		/* Might go negative! */
		for (int i = 0; i < BUFFER_HALF; i++) {
			/* After we finish the input data, pad with zero: */
			uint8_t e = (i < entries_to_go) ?
				input_buffer_val(0, input_buffer_pos) : 0;
			buffer[startpoint + i] = e;
			input_buffer_pos++;
		}
		/* How do I stop circular DMA at a particular point?
		 * Sounds impossible -- wait for next+1 HT after done,
		 * which was padded with zeroes, and switch off then.
		 *
		 * Just to be clear, we have output the final data on
		 * this HT IRQ, padding with zero.  We output zero
		 * padding on the NEXT HT IRQ too, and mark DMA done
		 * (previously-written final data is beginning to be
		 * output).  Then on the third HT IRQ, the data has been
		 * output and we can cease DMA itself.
		 */
		if (input_buffer_pos >= total_transfer_size+BUFFER_FULL_LEN) {
			/* We've carried on and filled two half-buffers (got 2
			 * HT IRQs since the last data filled), so cancel DMA
			 * now: */
			TIM_Cmd(TIM1, DISABLE);
			DMA_Cmd(DMA1_Channel5, DISABLE);
		}
		transfer_bottntop ^= 1;
	}
}

void	ws2812_display(uint8_t *buffer, int num)
{
}

int	ws2812_display_done(void)
{
	return 1;
}