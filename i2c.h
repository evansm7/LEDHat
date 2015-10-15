#ifndef I2C_H
#define I2C_H

void 	i2c_init(void);
int 	i2c_read(uint8_t devaddr, uint8_t regaddr, uint8_t *data, int num);
int 	i2c_write(uint8_t devaddr, uint8_t regaddr, uint8_t *data, int num);

#endif
