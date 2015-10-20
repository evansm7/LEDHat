#include "vector.h"
#include "uart.h"

void	print_vector(vec16_t *in)
{
	uart_phex32(in->x); uart_putch(':');
	uart_phex32(in->y); uart_putch(':');
	uart_phex32(in->z);
}
