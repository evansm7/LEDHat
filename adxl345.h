#ifndef ADXL345_H
#define ADXL345_H

#include "i2c.h"
#include "vector.h"
#include <inttypes.h>

#define ADXL_ADDR (0x53 << 1)

#define ADXL345_DEVID		0x00
#define ADXL345_OFSX		0x1e
#define ADXL345_FIFO		ADXL345_DATAX0
#define ADXL345_ACT_INACT_CTL	0x27
#define ADXL345_TIME_FF		0x29
#define ADXL345_BW_RATE		0x2c
#define ADXL345_POWER_CTL	0x2d
#define ADXL345_INT_ENABLE	0x2e
#define ADXL345_INT_MAP		0x2f
#define ADXL345_DATA_FMT	0x31
#define ADXL345_DATAX0 		0x32
#define ADXL345_FIFO_CTL	0x38

#define ADXL_RATE_400HZ		12
#define ADXL_RATE_200HZ		11
#define ADXL_RATE_100HZ		10
#define ADXL_RATE_50HZ		9
#define ADXL_RATE_25HZ		8
#define ADXL_RATE_12_5HZ	7
#define ADXL_RATE_6_25HZ	6
#define ADXL_RATE_3_13HZ	5

#define ADXL_RANGE_2G		0
#define ADXL_RANGE_4G		1
#define ADXL_RANGE_8G		2
#define ADXL_RANGE_16G		3
#define ADXL_DFMT_FULL_RES	0x08

#define ADXL_FIFO_BYPASS	0x00
#define ADXL_FIFO_FIFO		0x40
#define ADXL_FIFO_STREAM	0x80
#define ADXL_FIFO_TRIGGER	0xc0
#define ADXL_FIFO_SAMPLES(x)	((x) & 0x1f)

#define ADXL_IRQ_DATA_READY	0x80
#define ADXL_IRQ_WATERMARK	0x02

int	accel_init(void);

extern int16_t accel_read_x, accel_read_y, accel_read_z;

static inline void	accel_read(vec16_t *out)
{
	out->x = accel_read_x;
	out->y = accel_read_y;
	out->z = accel_read_z;
}

#endif
