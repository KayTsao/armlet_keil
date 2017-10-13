#include <string.h>
#include "stream.h"

#define b_min(a,b)	((a<b)?(a):(b))

void stream_init(sStream * pstream, uint8_t * pbuf, uint16_t length)
{
	pstream->pbuf 	= pbuf;
	pstream->length = length;
	pstream->remain = 8;
	pstream->offset = 0;
}

void stream_putbits(sStream * pstream, uint32_t val, uint8_t bitnum)
{
	uint8_t m;
	while(bitnum && pstream->offset < pstream->length)
	{
		val &= (1 << bitnum) - 1;
		if (bitnum >= pstream->remain){
			pstream->pbuf[pstream->offset] |= val >> (bitnum - pstream->remain);
			m = pstream->remain;
		}else{
			pstream->pbuf[pstream->offset] |= val << (pstream->remain - bitnum);
			m = bitnum;
		}
		bitnum -= m;
		if (pstream->remain == m)
		{
			pstream->remain = 8;
			pstream->offset++;
		}
		else
		{
			pstream->remain -= m;
		}
	}
}
