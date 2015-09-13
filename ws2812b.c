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
   1.25us +/- 0.15 = 800KHz.

   So 10 ticks total at 8MHz, either 3+7 or 7+3?
   No.  24MHz.  30 for total period @800KHz.  8 or 22 gives correct bits.
*/

#define BUFFER_FULL_LEN		32
#define BUFFER_HALF		(BUFFER_FULL_LEN/2)

#define TIM_VAL_ZERO		8
#define TIM_VAL_ONE		22

static uint8_t output_buffer[BUFFER_FULL_LEN] = {0};
static int input_buffer_pos;
static volatile int display_complete = 1;
static int total_transfer_size = 48;
static uint8_t *loc_input_buffer;


static inline uint8_t input_buffer_val(unsigned int pos)
{
	/* The input position is actually a bit into the byte-indexed buffer. */
	unsigned int byte_offs = pos / 8;
	unsigned int bit_offs = pos & 0x7;
	unsigned int bitval = (loc_input_buffer[byte_offs] >> bit_offs) & 1;
	return bitval ? TIM_VAL_ONE : TIM_VAL_ZERO;
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
	TIM_OCInitStructure.TIM_Pulse = 0;
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
}

static void	ws_irq_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void	ws_dma_init(int circular, unsigned int len)
{
	DMA_InitTypeDef  DMA_InitStructure;

	RCC->AHBENR |= RCC_AHBPeriph_DMA1;

	/* To write to TIM1_CH3, use DMA1_channel5 */
	DMA_DeInit(DMA1_Channel5);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uintptr_t)&TIM1->CCR3;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uintptr_t)output_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;

	DMA_InitStructure.DMA_BufferSize = len;
	if (circular)
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	else
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;

	DMA_Init(DMA1_Channel5, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel5, ENABLE);

	/* Both circular/linear cases use IRQs to wipe up after. */
	DMA_ITConfig(DMA1_Channel5, DMA_IT_TC | DMA_IT_HT, ENABLE);
	DMA_ClearFlag(DMA1_FLAG_TC5 | DMA1_FLAG_HT5);
}

void	ws2812_init(void)
{
	GPIO_InitTypeDef  	GPIO_InitStructure;

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
	ws_irq_init();
}

/* The parameters are a byte buffer of RGB tuples and a number, which is the
 * number of three-byte tuples supplied. (i.e. the number of LEDs.)
 */
void	ws2812_display(uint8_t *input_buffer, int num_leds)
{
	if (!display_complete)
		return;
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
	 * Actually, the 'plain linear' case (input < buffer size) is a horrible
	 * hack.  The IRQ handler's used to stop the timer (er, otherwise it
	 * never seems to stop!) but the last value gets repeated until this
	 * happens.  (Since DMA's stopped.)  It's hacked around by ensuring the
	 * last value written is zero.  So, there're actually BUFF-1 entries in
	 * this case and a transfer equal to the buffer size falls into the
	 * circular case anyway.
	 */

	loc_input_buffer = input_buffer;
	total_transfer_size = num_leds * 24;

	/* Fill initial full-buffer: */
	input_buffer_pos = 0;
	for (int i = 0; i < BUFFER_FULL_LEN; i++) {
		uint8_t e = (i < total_transfer_size) ?
			input_buffer_val(input_buffer_pos) : 0;
		output_buffer[i] = e;
		input_buffer_pos++;
	}

	if (total_transfer_size >= BUFFER_FULL_LEN) {
		/* We'll need multiple buffers.  Do circular version: */
		ws_dma_init(1, BUFFER_FULL_LEN);
	} else {
		/* We only need one buffer and no refill.  Linear version:
		 * (one extra is transferred to ensure a zero comes last)
		 */
		ws_dma_init(0, total_transfer_size+1);
	}

	led(0);	

	display_complete = 0;
	/* TIM1 counter enable */
	TIM_Cmd(TIM1, ENABLE);
}

void DMA1_Channel4_5_IRQHandler(void)
{
	int just_finished_bottom_buffer;
	if (DMA_GetITStatus(DMA1_IT_HT5)) {
		DMA_ClearITPendingBit(DMA1_IT_HT5);
		just_finished_bottom_buffer = 1;
	} else if (DMA_GetITStatus(DMA1_IT_TC5)) {
		DMA_ClearITPendingBit(DMA1_IT_TC5);
		just_finished_bottom_buffer = 0;
	}
	
	/* Have transferred 'BUFFER_HALF' items from either the bottom or top
	 * half of the buffer.  Refill entries into this half (as the other half
	 * is now being clocked out).  The logic is basically the same for
	 * either half, though this IRQ handler is called for two different
	 * reasons, for HT (after bottom half) and TC (after top half).
	 */
	int startpoint = just_finished_bottom_buffer ? 0 : BUFFER_HALF;
	int entries_to_go = total_transfer_size - input_buffer_pos;		/* Might go negative! */
	for (int i = 0; i < BUFFER_HALF; i++) {
		/* After we finish the input data, pad with zero: */
		uint8_t e = (i < entries_to_go) ?
			input_buffer_val(input_buffer_pos) : 0;
		output_buffer[startpoint + i] = e;
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
		display_complete = 1;
		led(1);
	}
}

int	ws2812_display_done(void)
{
	return display_complete;
}
