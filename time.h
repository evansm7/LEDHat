#ifndef TIME_H
#define TIME_H

void	time_init(void);
uint32_t time_get(void);

void	delay_ms(int d);
void	delay_us(int d);

uint32_t wait_till_tick(uint32_t tick);

#endif
