#ifndef VECTOR_H
#define VECTOR_H

#include <inttypes.h>

typedef struct {
	int16_t x,y,z;
} vec16_t;

/* a - b */
static inline vec16_t	vec_sub(vec16_t a, vec16_t b)
{
	vec16_t r = { a.x-b.x, a.y-b.y, a.z-b.z };
	return r;
}

/* a + b */
static inline vec16_t	vec_add(vec16_t a, vec16_t b)
{
	vec16_t r = { a.x+b.x, a.y+b.y, a.z+b.z };
	return r;
}

void	print_vector(vec16_t *in);

#endif
