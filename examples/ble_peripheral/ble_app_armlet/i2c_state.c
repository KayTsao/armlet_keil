#include <math.h>

#include "nrf_gpio.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "bsp.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "softdevice_handler_appsh.h"
#include "app_timer_appsh.h"
#include "app_button.h"
#include "app_util_platform.h"
#include "ble_advertising.h"
#include "nrf_drv_twi.h"
#include "i2c_state.h"
#include "nrf_delay.h"

#define TWI_PIPE_MAX	8

#define TWI_TIMEOUT		10

static const nrf_drv_twi_t twi_cha = NRF_DRV_TWI_INSTANCE(0);

static uint8_t  twi_busy = 0;
static sTwiOper twi_queue[TWI_PIPE_MAX];
static uint8_t  twi_front = 0;
static uint8_t  twi_rear  = 0;
static sTwiOper twi_oper;
/*
**
*/
static void twi_call_proc(sTwiOper * proc)
{
	twi_oper = *proc;
	
	//NRF_LOG_PRINTF("twi: r%d %02X\r\n", twi_current, twi_oper.addr);
	if (proc->w_len && proc->r_len)
	{
		nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(
						proc->addr,
						proc->w_buf, proc->w_len,
						proc->r_buf, proc->r_len);
		nrf_drv_twi_xfer(&twi_cha, &xfer, NRF_DRV_TWI_FLAG_TX_NO_STOP);
	}
	else if (proc->w_len)
	{
		nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TX(
						proc->addr,
						proc->w_buf, proc->w_len);
		nrf_drv_twi_xfer(&twi_cha, &xfer, 0);
	}
	else if (proc->r_len)
	{
		nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_RX(
						proc->addr,
						proc->r_buf, proc->r_len);
		
		nrf_drv_twi_xfer(&twi_cha, &xfer, 0);
	}
}

/**
 * 事件处理
 */
static void twi_event(uint8_t evt)
{ 
	uint8_t front;
	sTwiOper oper;
	t_twievent fun_event;
	
	fun_event = twi_oper.f_event;
	
	if (fun_event && fun_event(evt, &oper))
	{
		twi_call_proc(&oper);	
	}
	else 
	{
		__disable_irq();
		if (twi_rear != twi_front)
		{
			front = (twi_front+1) % TWI_PIPE_MAX;
			oper  = twi_queue[front];
			twi_front = front;
			
			__enable_irq();
			twi_call_proc(&oper);
		}
		else
		{
			twi_busy = 0;
			__enable_irq();
		}
	}
}
/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch(p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
			if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
			{
				twi_event(TWI_EVT_OK);
			}
			else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
			{
				twi_event(TWI_EVT_OK);
			}
			else if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX)
			{
				twi_event(TWI_EVT_OK);
			}
            break;
        default:
			twi_event(TWI_EVT_ERROR);
            break;        
    }
}
/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

	const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI0_CONFIG_SCL,
       .sda                = TWI0_CONFIG_SDA,
       .frequency          = TWI0_CONFIG_FREQUENCY,
       .interrupt_priority = TWI0_CONFIG_IRQ_PRIORITY,
    };
   
    err_code = nrf_drv_twi_init(&twi_cha, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&twi_cha);
}

void twi_uninit(void)
{
    nrf_drv_twi_uninit(&twi_cha);
}

/**
 * 启动一次I2C数据采集
 */
uint8_t twi_operate(sTwiOper *pOpr)
{
	uint8_t ret = 1;
	uint8_t rear;
	
	__disable_irq();
	rear = (twi_rear+1) % TWI_PIPE_MAX;
	if (!twi_busy)
	{
	  	twi_busy  = 1;
		__enable_irq();
		twi_call_proc(pOpr);
	}
	else
	{
		if( rear != twi_front)
		{
			twi_queue[rear] = *pOpr;
			twi_rear= rear;
		}
		else
		{
			ret = 0;
			NRF_LOG_PRINTF("twi: fifo full\r\n");
		}
		__enable_irq();
	}
	return ret;
}
