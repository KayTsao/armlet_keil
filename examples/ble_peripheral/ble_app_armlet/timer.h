#ifndef __TIMER_H
#define __TIMER_H


extern uint32_t get_time_ms(void);
extern uint32_t get_time_us(void);
extern uint32_t get_time_tick(void);
extern void     timer_sync(void);

#endif //__TIMER_H

