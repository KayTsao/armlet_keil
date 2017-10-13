#include <stdint.h>
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
#include "ble_advertising.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "i2c_state.h"
#include "timer.h"
#include "mag_2_ak09911.h"


#define AK09911_I2C_ADDR1				(0x0C)
#define AK09911_I2C_ADDR2				(0x0D)
#define AK09911_CHIP_ID1     			(0x48)
#define AK09911_CHIP_ID2     			(0x05)

//ID2: AK09911C 0x05
//ID2: AK09918C 0x0C

#define AK09911_USER_WIA1_ADDR			0x00
#define AK09911_USER_WIA2_ADDR			0x01
#define AK09911_USER_INFO1_ADDR			0x02
#define AK09911_USER_INFO2_ADDR			0x03
#define AK09911_USER_ST1_ADDR			0x10
#define AK09911_USER_HXL_ADDR			0x11
#define AK09911_USER_HXH_ADDR			0x12
#define AK09911_USER_HYL_ADDR			0x13
#define AK09911_USER_HYH_ADDR			0x14
#define AK09911_USER_HZL_ADDR			0x15
#define AK09911_USER_HZH_ADDR			0x16
#define AK09911_USER_TMPS_ADDR			0x17
#define AK09911_USER_ST2_ADDR			0x18
#define AK09911_USER_CNTL1_ADDR			0x30
#define AK09911_USER_CNTL2_ADDR			0x31
#define AK09911_USER_CNTL3_ADDR			0x32
#define AK09911_USER_TS1_ADDR			0x33
#define AK09911_USER_ASAX_ADDR			0x60
#define AK09911_USER_ASAY_ADDR			0x61
#define AK09911_USER_ASAZ_ADDR			0x62

#define AK09911_I2C_ADDR     AK09911_I2C_ADDR2


static bool     mag_st_work = false;
static bool     mag_st_sleep = false;
static uint8_t  mag_state = 0;
static uint8_t  mag_busy  = 0;
static uint8_t  mag_buffer[20];
static uint32_t mag_tick = 0;
static uint8_t  mag_register;
static int16_t  mag_rawdat[3] = {0};

/*
** AK09911 read iic data
*/
static uint8_t mag_read_buffer(uint8_t reg, uint8_t len, t_twievent f_event)
{
	sTwiOper oper = {0};
	
	mag_register = reg;
	
	oper.addr  = AK09911_I2C_ADDR;
	oper.w_buf = &mag_register;
	oper.w_len = 1;
	oper.r_buf = mag_buffer;
	oper.r_len = len;
	oper.f_event = f_event;
	
	return twi_operate(&oper);
}

/*
** AK09911 write iic data
*/
static uint8_t mag_write_byte(uint8_t reg, uint8_t val, t_twievent f_event)
{
	sTwiOper oper = {0};
	mag_buffer[0] = reg;
	mag_buffer[1] = val;
	
	oper.addr = AK09911_I2C_ADDR;
	oper.w_buf = mag_buffer;
	oper.w_len = 2;
	oper.f_event = f_event;

	return twi_operate(&oper);
}

/*
** AK09911 process measure datas event
*/
static bool mag_event_measure(uint8_t evt, sTwiOper * pNext)
{
	if (evt == TWI_EVT_OK)
	{
		mag_rawdat[0] = (int16_t)(mag_buffer[0] | (mag_buffer[1]<<8));
		mag_rawdat[1] = (int16_t)(mag_buffer[2] | (mag_buffer[3]<<8));
		mag_rawdat[2] = (int16_t)(mag_buffer[4] | (mag_buffer[5]<<8));		
    }
	mag_busy = 0;
	return false;
}

/*
** AK09911 process init event
*/
static bool mag_event_init(uint8_t evt, sTwiOper * pNext)
{
	if (evt != TWI_EVT_OK)
	{
		mag_state  = 0;
		NRF_LOG_PRINTF("AK0991x(2) error\r\n");
	}
	mag_busy = 0;
	mag_tick = get_time_ms() + 10;
	return false;
}

/*
*/
void mag_2_init(void)
{
//	nrf_gpio_cfg_output(MAG_RESET_PORT);
//	nrf_gpio_pin_set(MAG_RESET_PORT);
}
/*
** AK09911 measure callback
*/
void mag_2_measure(void)
{
	if (!mag_st_work)
	{
		mag_st_work   = 1;
		mag_st_sleep  = 0;
		mag_state  = 0;
	}
	else if (get_time_ms() >= mag_tick && !mag_busy)
	{
		switch (mag_state)
		{
		case 0:
		  	mag_state = 1;
			mag_tick  = get_time_ms() + 10;
			break;
		case 1:
			if ((mag_busy = mag_read_buffer(AK09911_USER_WIA1_ADDR, 1, mag_event_init))!=0)
			{
				mag_state++;
			}
			break;
		case 2:
			NRF_LOG_PRINTF("AK0991x(2) ID1:%02X\r\n", mag_buffer[0]);
			if (mag_buffer[0] != AK09911_CHIP_ID1)
			{
				mag_state = 0;
				mag_tick  = get_time_ms() + 50;
			}
			else if ((mag_busy = mag_read_buffer(AK09911_USER_WIA2_ADDR, 1, mag_event_init))!=0)
			{
				mag_state++;
			}
			break;
		case 3:
			NRF_LOG_PRINTF("AK0991x(2) ID2:%02X\r\n", mag_buffer[0]);
			if (mag_buffer[0] != AK09911_CHIP_ID2)
			{
				mag_state = 0;
				mag_tick  = get_time_ms() + 50;
			}
			else if ((mag_busy = mag_write_byte(AK09911_USER_CNTL2_ADDR, 0x08, mag_event_init))!=0)
			{
				mag_state++;
				NRF_LOG_PRINTF("AK0991x(2) Init Finished\r\n");
			}
			break;
		default:
		  	mag_tick  = get_time_ms() + 10;
		 	mag_busy  = mag_read_buffer(AK09911_USER_HXL_ADDR, 8, mag_event_measure);
		  	break;
		}
	}
}

bool mag_2_sleep(void)
{
	if (!mag_st_sleep)
	{
	  	mag_st_sleep  = 1;
		mag_st_work   = 0;
		mag_state  = 0;
	}
	else if (get_time_ms() >= mag_tick && !mag_busy)
	{
	 	switch(mag_state)
		{
		case 0:
		  	mag_tick  = get_time_ms() + 50;
//			nrf_gpio_pin_clear(MAG_RESET_PORT);
			mag_state = 1;
			break;
		case 1:
//		  	nrf_gpio_pin_set(MAG_RESET_PORT);
		  	mag_state = 2;
//		  	NRF_LOG_PRINTF("imu_ak0991 sleep\r\n");
		default:
		  	return true;
		}		
	}
	return false;
}

void mag_2_getraw(int16_t* pbuf)
{
	memcpy(pbuf, mag_rawdat, 3*2);
}
