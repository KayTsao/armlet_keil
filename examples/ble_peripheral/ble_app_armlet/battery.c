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
#include "nrf_drv_saadc.h"
#include "nrf_delay.h"
#include "i2c_state.h"
#include "timer.h"

//#define DEBUG_LOG(...)	NRF_LOG_PRINTF(__VA_ARGS__)
#define DEBUG_LOG(...)

typedef enum
{
    BATTERY_OK, 	  
	BATTERY_LOW,
	BATTERY_EMPTY
} battery_statue_t;

#define BATTERY_LOW_LEVEL				    (2400)
#define BATTERY_EMPTY_LEVEL				    (2100)

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS       3300 	/**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define DIODE_FWD_VOLT_DROP_MILLIVOLTS      155   	/**< Typical forward voltage drop of the diode (Part no: SD103ATW-7-F) that is connected in series with the voltage supply. This is the voltage drop when the forward current is 1mA. Source: Data sheet of 'SURFACE MOUNT SCHOTTKY BARRIER DIODE ARRAY' available at www.diodes.com. */
#define ADC_RES_10BIT                       1024 	/**< Maximum digital value for 10-bit ADC conversion. */
 
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        (((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT)
 
static nrf_saadc_value_t adc_buf[2];

#define SAMPLES_IN_BUFFER					(1)
#define BATTERY_DETECT_INTERVAL				(2000 / SAMPLES_IN_BUFFER)

static uint32_t BatteryTick = 0;
battery_statue_t battery_status = BATTERY_OK;

static uint16_t AdcBatteryLevel = 3000;

#define BATTERY_LED_CYCLE	        (6)


/**@brief Function for handling the ADC interrupt.
*
* @details  This function will fetch the conversion result from the ADC, convert the value into
*           percentage and send it to peer.
*/
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint32_t          err_code;
		uint16_t			adc_total = 0;
		uint8_t 			t;
 
        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);
		
		for(t = 0; t < SAMPLES_IN_BUFFER; t++)
			adc_total += p_event->data.done.p_buffer[t];
        adc_result = adc_total / SAMPLES_IN_BUFFER;		
	
        AdcBatteryLevel = 2*(ADC_RESULT_IN_MILLI_VOLTS(adc_result) + DIODE_FWD_VOLT_DROP_MILLIVOLTS);
		//m_send_battery = (AdcBatteryLevel - 1000) / 10;

//		DEBUG_LOG("m_send_battery = %d%\r\n", m_send_battery);
		
        nrf_drv_saadc_uninit();
        NVIC_ClearPendingIRQ(SAADC_IRQn);
    
        if ((err_code != NRF_SUCCESS)&&(err_code != NRF_ERROR_INVALID_STATE)
            &&(err_code != BLE_ERROR_NO_TX_PACKETS)&&(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}

/**@brief Function for battery init.
*
* @details
*/
static void battery_init(void)
{
	ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);
 
    nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN2);
    err_code = nrf_drv_saadc_channel_init(0,&config);
    APP_ERROR_CHECK(err_code);
 
    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
 
    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for battery sampling.
*
* @details
*/
static void battery_sampling(void)
{
	ret_code_t err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for battery sampling.
*
* @details Battery Measure
*/
void bat_measure(void)
{
    if(get_time_ms() >= BatteryTick)
    {
        battery_init();
        battery_sampling();
        BatteryTick = get_time_ms() + BATTERY_DETECT_INTERVAL;
    }
}

/**@brief Function for battery sampling.
*
* @details Get Battery Level
*/
uint8_t GetAdcBatteryLevel(void) 
{
    return AdcBatteryLevel;
}

/**@brief Function for battery sampling.
*
* @details Get Battery Level percents
*/
uint8_t bat_level_percent(void) 
{
    uint8_t  battery_level;
    battery_level = 100 * AdcBatteryLevel / ADC_REF_VOLTAGE_IN_MILLIVOLTS;
    return battery_level;
}
