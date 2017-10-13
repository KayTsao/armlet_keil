#ifndef __I2C_STATE_H
#define __I2C_STATE_H

#include "nrf.h"

#define TWI_EVT_OK		1
#define TWI_EVT_ERROR	0

typedef struct _sTwiOper sTwiOper;
typedef bool (*t_twievent)(uint8_t evt, sTwiOper* pNextOper);

struct _sTwiOper{
	uint8_t  addr;
	uint8_t* r_buf;
	uint8_t* w_buf;
	uint8_t  w_len;
	uint8_t  r_len;
	t_twievent f_event;
};

extern void twi_init (void);
extern void twi_uninit(void);
extern uint8_t twi_operate(sTwiOper *pOpr);
extern void twi_checkerror(void);

#endif
