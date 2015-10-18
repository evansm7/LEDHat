#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_exti.h>
#include <stm32f0xx_syscfg.h>
#include <inttypes.h>
#include "i2c.h"
#include "uart.h"
#include "adxl345.h"

/* The plan here is to try to use the inbuilt low-pass filtration in the
 * ADXL345, by setting the output sample rate low (e.g. 25Hz) and having it
 * interrupt periodically when a sample is ready.
 *
 * Plan B is to select a high output rate (e.g. 400Hz), use the FIFO to capture
 * bunches of samples, implement a FIR filter in software and end up with
 * filtered output samples.  The question then remains how to present these
 * 'periodically' to software, e.g. reading 8 samples every 50Hz-ish will get
 * out of sync/be bursty with a 100Hz consumer loop, so might need some kind of
 * internal FIFO.
 */

/* Globally-accessible through accel_read(): */
int16_t	accel_read_x;
int16_t	accel_read_y;
int16_t	accel_read_z;

void EXTI0_1_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0)) {
		EXTI_ClearITPendingBit(EXTI_Line0);

		/* Read data/sample */
		uint8_t	data[6];
		if (i2c_read(ADXL_ADDR, ADXL345_FIFO, data, 6) == 0) {
			accel_read_x = data[0] | ((int16_t)data[1] << 8);
			accel_read_y = data[2] | ((int16_t)data[3] << 8);
			accel_read_z = data[4] | ((int16_t)data[5] << 8);
		}
	}
}

static void accel_setup_irq(void)
{
	RCC->AHBENR |= RCC_AHBPeriph_GPIOA;
	RCC->APB2ENR |= RCC_APB2Periph_SYSCFG;

	/* Ensure PA0 is an input/GPIO */
	GPIO_InitTypeDef	GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Interrupt will be active-high: */
	EXTI_InitTypeDef 	EXTI_InitStructure;
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* IRQ setup: *Lower* priority than the LED DMA IRQ (1)! */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void accel_reg_wr(uint8_t reg, uint8_t val)
{
	int r = i2c_write(ADXL_ADDR, reg, &val, 1);
	if (r < 0) {
		uart_pstr("Accel: Can't write reg ");
		uart_phex32(reg);
		uart_pstr("\r\n");
	}
}

int	accel_init(void)
{	
	uint8_t data[8] = {0};
	int r = i2c_read(ADXL_ADDR, ADXL345_DEVID, data, 1);
	if (r == 0 && data[0] == 0xe5) {
		uart_pstr("ADXL found\r\n");
	} else {
		uart_pstr("ADXL not found\r\n");
		return -1;
	}

	accel_reg_wr(ADXL345_DATA_FMT, ADXL_DFMT_FULL_RES | ADXL_RANGE_4G);
	accel_reg_wr(ADXL345_ACT_INACT_CTL, 0);
	accel_reg_wr(ADXL345_TIME_FF, 0);
	accel_reg_wr(ADXL345_BW_RATE, ADXL_RATE_25HZ);	/* LOW_POWER=0 */
	accel_reg_wr(ADXL345_FIFO_CTL, ADXL_FIFO_BYPASS);
	accel_reg_wr(ADXL345_INT_ENABLE, ADXL_IRQ_DATA_READY);
	/* Selected IRQ on INT1 output, all others on INT2 */
	accel_reg_wr(ADXL345_INT_MAP, (uint8_t)~ADXL_IRQ_DATA_READY);

	data[0] = 0x00;				/* Zero offset regs */
	data[1] = 0x00;
	data[2] = 0x00;
	r = i2c_write(ADXL_ADDR, ADXL345_OFSX, data, 3);
	if (r < 0) {
		uart_pstr("Can't write offs\r\n");
		return -1;
	}

	accel_setup_irq();

	accel_reg_wr(ADXL345_POWER_CTL, 0x08);	/* Out of sleep, measure on */

	return 0;
}
