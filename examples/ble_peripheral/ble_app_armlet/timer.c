#include "app_timer.h"

static uint32_t	    m_mstick_last = 0;
static uint32_t	    m_mstick_val = 0;
static uint32_t	    m_mstick_dt = 0;
static uint32_t		m_ustick_last = 0;
static uint32_t		m_ustick_val = 0;
static uint32_t		m_ustick_dt = 0;

/**@brief Function for
*
* @details get system milisecond time
*/
uint32_t get_time_ms(void)
{
	uint32_t tick,ms;
	__disable_irq();
	app_timer_cnt_get(&tick);
	m_mstick_dt = (tick - m_mstick_last) & 0xFFFFFF;
	if (m_mstick_dt >= 32768){
		m_mstick_dt -= 32768;
		m_mstick_val += 1000;
		m_mstick_last += 32768;
		m_mstick_last &= 0xFFFFFF;
	}
	ms = m_mstick_val + (m_mstick_dt*1000/32768);
	__enable_irq();
	
	return ms;
}

/**@brief Function for
*
* @details get system microsecond time
*/
uint32_t get_time_us(void)
{
	uint32_t tick, us;
	
	__disable_irq();
	app_timer_cnt_get(&tick);
	m_ustick_dt = (tick - m_ustick_last) & 0xFFFFFF;
	if (m_ustick_dt >= 32768){
		m_ustick_dt -= 32768;
		m_ustick_val  += 1000000;
		m_ustick_last += 32768;
		m_ustick_last &= 0xFFFFFF;
	}
	us = m_ustick_val + ((uint64_t)m_ustick_dt*1000000/32768);
	__enable_irq();
	
	return us;
}

/**@brief Function for
*
* @details get system milisecond time
*/
uint32_t get_time_tick(void)
{
  	uint32_t tick;
	app_timer_cnt_get(&tick);
	return tick;
}

/**@brief Function for
*
* @details 
*/
void timer_sync(void)
{
  	uint32_t tick;
	
	__disable_irq();
	app_timer_cnt_get(&tick);
	
	m_mstick_val += (m_mstick_dt*1000/32768);
	m_ustick_val += ((uint64_t)m_ustick_dt*1000000/32768);
	
	m_mstick_dt = 0;
	m_ustick_dt = 0;
	m_ustick_last = tick;
	m_mstick_last = tick;
	
	__enable_irq();
}
