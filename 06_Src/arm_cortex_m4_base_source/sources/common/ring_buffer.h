#ifndef __RING_BUFFER_H__
#define __RING_BUFFER_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <string.h>

#define RET_RING_BUFFER_OK				(0x01)
#define RET_RING_BUFFER_NG				(0x00)

typedef void (*memcpy_f)(void *dst,const void *str, size_t size);

typedef struct {
	uint8_t tail_index;
	uint8_t head_index;
	uint8_t fill_size;
	uint8_t buffer_size;
	uint8_t element_size;
	uint8_t* buffer;

	memcpy_f memcpy;
} ring_buffer_t;

extern void		ring_buffer_init(ring_buffer_t* ring_buffer, memcpy_f memcpy, void* buffer, uint8_t buffer_size, uint8_t element_size);
extern uint8_t	ring_buffer_availble(ring_buffer_t* ring_buffer);
extern bool		ring_buffer_is_empty(ring_buffer_t* ring_buffer);
extern bool		ring_buffer_is_full(ring_buffer_t* ring_buffer);
extern uint8_t	ring_buffer_put(ring_buffer_t* ring_buffer, void* data);
extern uint8_t	ring_buffer_get(ring_buffer_t* ring_buffer, void* data);

#ifdef __cplusplus
}
#endif

#endif //__RING_BUFFER_H__
