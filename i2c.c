#include <inttypes.h>
#include <stm32f0xx.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_i2c.h>

#include "i2c.h"
#include "hw.h"

#include "uart.h"

//#define DEBUG
#ifdef DEBUG
#define I2CDBG(x)	uart_pstr(x)
#else
#define I2CDBG(x)	do { } while (0)
#endif

#define I2C_TIMEOUT_LOOPS	10000

int i2c_read(uint8_t devaddr, uint8_t regaddr, uint8_t *data, int num)
{   
	unsigned int timeout;
	
	/* Test on BUSY Flag */
	timeout = I2C_TIMEOUT_LOOPS;
	I2CDBG("Wait non-busy\n");
	while(I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET) {
		if(timeout-- == 0)
			return -1;
	}
	I2CDBG("Config address\n");
	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(I2C1, devaddr, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);
  
	/* Wait until TXIS flag is set */
  	timeout = I2C_TIMEOUT_LOOPS;
	I2CDBG("Wait TXIS\n");
	while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET) {
		if(timeout-- == 0)
			return -1;
	}

	I2CDBG("Send regaddr\n");
	/* Send Register address */
	I2C_SendData(I2C1, (uint8_t)regaddr);
  
	/* Wait until TC flag is set */
	timeout = I2C_TIMEOUT_LOOPS;
	while(I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET) {
		if(timeout-- == 0)
			return -1;
	}
  
	/* Configure slave address, nbytes, reload, end mode and start or stop generation */
	I2C_TransferHandling(I2C1, devaddr, num, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

	for (int rxpos = 0; rxpos < num; rxpos++) {
		timeout = I2C_TIMEOUT_LOOPS;
		while(I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET) {
			if(timeout-- == 0)
				return -1;
		}
		data[rxpos] = I2C_ReceiveData(I2C1);
	}    
	/* Wait until STOPF flag is set */
	timeout = I2C_TIMEOUT_LOOPS;
	while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET) {
		if(timeout-- == 0)
			return -1;
	}

	/* Clear STOPF flag */
	I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

	return 0;
}


int i2c_write(uint8_t devaddr, uint8_t regaddr, uint8_t *data, int num)
{   
	unsigned int timeout;
	
	/* Test on BUSY Flag */
	timeout = I2C_TIMEOUT_LOOPS;
	I2CDBG("Wait non-busy\n");
	while(I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET) {
		if(timeout-- == 0)
			return -1;
	}
	I2CDBG("Config address\n");
	I2C_TransferHandling(I2C1, devaddr, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);
  
	/* Wait until TXIS flag is set */
  	timeout = I2C_TIMEOUT_LOOPS;
	I2CDBG("Wait TXIS\n");
	while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET) {
		if(timeout-- == 0)
			return -1;
	}

	I2CDBG("Send regaddr\n");
	/* Send Register address */
	I2C_SendData(I2C1, (uint8_t)regaddr);
  
	/* Wait until TCR flag is set */
	timeout = I2C_TIMEOUT_LOOPS;
	while(I2C_GetFlagStatus(I2C1, I2C_ISR_TCR) == RESET) {
		if(timeout-- == 0)
			return -1;
	}
  
	I2C_TransferHandling(I2C1, devaddr, num, I2C_AutoEnd_Mode, I2C_No_StartStop);

	for (int txpos = 0; txpos < num; txpos++) {
		timeout = I2C_TIMEOUT_LOOPS;
		while(I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET) {
			if(timeout-- == 0)
				return -1;
		}
		I2C_SendData(I2C1, data[txpos]);
	}
	/* Wait until STOPF flag is set */
	timeout = I2C_TIMEOUT_LOOPS;
	while(I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET) {
		if(timeout-- == 0)
			return -1;
	}

	/* Clear STOPF flag */
	I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

	return 0;
}
  
void i2c_init(void)
{
	/* PA9=SCL, PA10=SDA */
	GPIO_InitTypeDef	GPIO_InitStructure;
	I2C_InitTypeDef        	I2C_InitStructure;

	RCC->AHBENR |= RCC_AHBPeriph_GPIOA;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);

	RCC->APB1ENR |= RCC_APB1Periph_I2C1;

	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);

  	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Disable;
	I2C_InitStructure.I2C_DigitalFilter = 0x00;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	/* Prescale 0 = (1+0)*clkperiod
	 * Setup time 2, hold time 1,
	 * SCL high period 0x06, SCL low period 0x06
	 * = 400KHz.
	 */
	I2C_InitStructure.I2C_Timing = 0x00320606;
  
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}
