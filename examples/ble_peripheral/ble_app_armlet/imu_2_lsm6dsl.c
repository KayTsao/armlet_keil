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

#define WHOAMI			0x0F

#define CTRL1_XL		0x10
#define CTRL2_G			0x11

#define OUTX_L_G		0x22
#define OUTX_H_G		0x23
#define OUTY_L_G		0x24
#define OUTY_H_G		0x25
#define OUTZ_L_G		0x26
#define OUTZ_H_G		0x27
#define OUTX_L_XL		0x28
#define OUTX_H_XL		0x29
#define OUTY_L_XL		0x2A
#define OUTY_H_XL		0x2B
#define OUTZ_L_XL		0x2C
#define OUTZ_H_XL		0x2D

#define LSM6DSL_CHIP_ID	0x6A

/*******************************************/
//#define	I2C_ADDRESS    (0xD4>>1)
#define	I2C_ADDRESS    (0xD6>>1)
/*******************************************/


typedef struct{
	uint8_t reg;
	uint8_t val;
}sIMURegs;

static sIMURegs init_regs[] = {
	{0x12,					0x04},
	{CTRL1_XL,   	  0x74},
	{CTRL2_G,   	  0x7C},
};

static uint8_t  imu_alive = 0;
static uint8_t  imu_busy  = 0;
static uint8_t  imu_tabidx = 0;
static uint8_t  imu_state = 0;
static uint8_t  imu_buffer[20];
static uint32_t imu_tick = 0;
static uint8_t  imu_register;
static int16_t  imu_rawdat[6] = {0};
/*
** 请求一个I2C处理
*/
static uint8_t imu_read_buffer(uint8_t reg, uint8_t len, t_twievent f_event)
{
	sTwiOper oper = {0};
	imu_register = reg;
	
	oper.addr  = I2C_ADDRESS;
	oper.w_buf = &imu_register;
	oper.w_len = 1;
	oper.r_buf = imu_buffer;
	oper.r_len = len;
	oper.f_event = f_event;
	
	return twi_operate(&oper);
}
/*
** 请求一个I2C处理
*/
static uint8_t imu_write_byte(uint8_t reg, uint8_t val, t_twievent f_event)
{
	sTwiOper oper = {0};
	imu_buffer[0] = reg;
	imu_buffer[1] = val;
	
	oper.addr  = I2C_ADDRESS;
	oper.w_buf = imu_buffer;
	oper.w_len = 2;
	oper.f_event = f_event;

	return twi_operate(&oper);
}
/*
** 复位状态机
*/
static void imu_reset_state(void)
{
	imu_busy = 0;
	imu_state = 0;
	imu_tabidx = 0;
	
}

/*
** 测量数据  事件处理
*/
static bool imu_event_measure(uint8_t evt, sTwiOper* pNextOper)
{
	if (evt == TWI_EVT_OK)
	{
		//ACC
		imu_rawdat[0] = (int16_t)(imu_buffer[ 7]<<8 | (imu_buffer[ 6]));
		imu_rawdat[1] = (int16_t)(imu_buffer[ 9]<<8 | (imu_buffer[ 8]));
		imu_rawdat[2] = (int16_t)(imu_buffer[11]<<8 | (imu_buffer[10]));
		//GYRO                                  
		imu_rawdat[3] = (int16_t)(imu_buffer[ 1]<<8 | (imu_buffer[ 0]));
		imu_rawdat[4] = (int16_t)(imu_buffer[ 3]<<8 | (imu_buffer[ 2]));
		imu_rawdat[5] = (int16_t)(imu_buffer[ 5]<<8 | (imu_buffer[ 4]));
		
//		NRF_LOG_PRINTF("IMU(2) %7d %7d %7d\r\n", imu_rawdat[3], imu_rawdat[4], imu_rawdat[5]);
	}
	imu_busy = 0;
	return false;
}
/*
** 初始化  事件处理
*/
static bool imu_event_init(uint8_t evt, sTwiOper* pNextOper)
{
	if (evt != TWI_EVT_OK)
	{
		imu_state = 0;
	}
	imu_busy = 0;
	imu_tick = get_time_ms() + 20;
	return false;
}
#define b_arraysize(a)	(sizeof(a)/sizeof(a[0]))
/*
** 发送初始化 或 测量
*/
void imu_2_measure(void)
{
	if (imu_alive && !imu_busy)
	{		
		imu_busy = imu_read_buffer(OUTX_L_G, 12, imu_event_measure);
	}
	if (!imu_alive && !imu_busy && get_time_ms() >= imu_tick)
	{
		switch (imu_state)
		{
		case 0:
			if ((imu_busy = imu_read_buffer(WHOAMI, 1, imu_event_init))!=0)
			{
				imu_state++;
			}
			break;
		case 1:
			NRF_LOG_PRINTF("LSM6DSL(2) ID:%02X\r\n", imu_buffer[0]);
			if (imu_buffer[0] != LSM6DSL_CHIP_ID)
			{
				imu_state = 0;
				imu_tick  = get_time_ms() + 50;
			}
			else
			{
				imu_state++;
				imu_tabidx = 0;
			}
			break;
		case 2:
			if (imu_tabidx < b_arraysize(init_regs))
			{
				imu_busy = imu_write_byte(
						init_regs[imu_tabidx].reg,
						init_regs[imu_tabidx].val, imu_event_init);
				if (imu_busy){
					imu_tabidx++;
				}
			}else{
				imu_alive = 1;
				imu_reset_state();
				NRF_LOG_PRINTF("LSM6DSL(2) Init Finished\r\n");
			}
			break;
		}
	}
}

void imu_2_init(void)
{
}

void imu_2_getraw(int16_t* pbuf)
{
	memcpy(pbuf, imu_rawdat, 6*2);
}
