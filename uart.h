#ifndef UART_H
#define UART_H


void uart_putch(char c);
char uart_getch(void);
int uart_ch_rdy(void);
static inline void uart_pstr(char *s)
{
	while (*s)
		uart_putch(*s++);
}
void uart_phex32(unsigned int w);

void uart_init(void);


#endif
