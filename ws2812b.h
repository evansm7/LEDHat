#ifndef WS2812_H
#define WS2812_H

void	ws2812_init(void);
void	ws2812_display(uint8_t *buffer, int num);
int	ws2812_display_done(void);

#endif
