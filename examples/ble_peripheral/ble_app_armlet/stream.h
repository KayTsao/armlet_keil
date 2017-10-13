#ifndef __C_BSTONE_STREAM_H
#define __C_BSTONE_STREAM_H

#include "stdint.h"

typedef struct {
	uint8_t* pbuf;
	uint16_t length;
	uint8_t  remain;
	uint16_t offset;
}sStream;

void     stream_init   (sStream * pstream, uint8_t * pbuf, uint16_t length);
void     stream_putbits(sStream * pstream, uint32_t val, uint8_t bitnum);

#endif

