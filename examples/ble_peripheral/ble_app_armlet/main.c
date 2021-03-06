/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "boards.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "device_manager.h"
#include "pstorage.h"
#include "app_trace.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "sensorsim.h"
#include "nrf_gpio.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_armlet.h"
#include "i2c_state.h"
#include "stream.h"
#include "nrf_delay.h"

#include "imu_lsm6dsl.h"
#include "mag_ak09911.h"
#include "attitudeSensor.h" 

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "IQIYI Armlet"                             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "IQIYI"                                    /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          6                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(7.5, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(15, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                    0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                 MSEC_TO_UNITS(4000, UNIT_10_MS)            /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY    APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)/**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT     3                                          /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                        0xDEADBEEF                                 /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

static ble_armlet_t                      m_armlet; 

//int16_t imu_raw[6];
//int16_t mag_raw[3];

APP_TIMER_DEF(m_sensor_timer_id);                                                   /**< Sensor timer. */
APP_TIMER_DEF(m_calibrator_timer_id);				   /**< mag calibration timer. **/

static bool AppTimerFlag;

//static SampleSlideBuf GlobalAccSampleSlideBuf_1 ;
//static SampleSlideBuf GlobalGyrSampleSlideBuf_1 ;
static AttitudeSensor SensorNode1, SensorNode2;
static MagSensorCalibrator Mag_calibrator;
static bool isCalibration;


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

                                   
/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for Loading Sensor measurement into AttitudeSensor Structure.
 *
 * @details This function will be called in the Sensor timeout handler (after we got the latest sensor measurement).
 *
 * @param[in] rawIMU  Latest 6-axis IMU(gyr+acc) measurement.
 *            rawMag  Latest 3-axis Magnetometer measurement.
 */
static bool LoadRawData(int id, int16_t* rawIMU, int16_t* rawMag)
{
	AttitudeSensor *sensor;
	switch(id){
		case 1:
			sensor = &SensorNode1;
		break;
		case 2:
			sensor = &SensorNode2;
	} 
	sensor->ax_raw = (float)rawIMU[0] * accRatio * (-1.0f); 
	sensor->ay_raw = (float)rawIMU[1] * accRatio * (-1.0f);
	sensor->az_raw = (float)rawIMU[2] * accRatio; 
	 
	sensor->gx_raw = (float)rawIMU[3] * gyroRatio * (-1.0f);
	sensor->gy_raw = (float)rawIMU[4] * gyroRatio * (-1.0f);
	sensor->gz_raw = (float)rawIMU[5] * gyroRatio;
	
	sensor->mx_raw = (float)rawMag[0] * magRatio_9911;
	sensor->my_raw = (float)rawMag[1] * magRatio_9911;
	sensor->mz_raw = (float)rawMag[2] * magRatio_9911 * (-1.0f); 
	//sensor->cur_t_us = get_time_us(); 
	
	uint32_t dt ;
	sensor->ts_cur_ms = get_time_ms();
	
	if(sensor->ts_prev_ms == 0)
    { 
		sensor->ts_prev_ms = get_time_ms();
        dt = 0;
    }
	else
	{		
		if(sensor->ts_prev_ms > sensor->ts_cur_ms) //prev is larger than current
		{
			dt = 0;
			sensor->ts_prev_ms = sensor->ts_cur_ms;
			//dt = 4294967296.0 - sensor->ts_prev_us + sensor->ts_cur_us; 
		}
		else
		{	
			dt = sensor->ts_cur_ms - sensor->ts_prev_ms;
		}
	} 
	sensor->ts_prev_ms = sensor->ts_cur_ms;	
    sensor->SamplePeriod = 0.001f * dt;
	return 0;
}

/**@brief Function for handling the Sensor measurement timer timeout.
 *
 * @details This function will be called each time the Sensor Raw Data measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void sensor_timeout_handler(void * p_context)
{ 
	int16_t imu_raw[6];
	int16_t mag_raw[3];         

//	int16_t imu_2_raw[6];	int16_t mag_2_raw[3]; 
 
  UNUSED_PARAMETER(p_context);

	imu_measure();
	mag_measure();
	
	imu_getraw(imu_raw);
	mag_getraw(mag_raw);
 
 
	//KK load latest measurement
	LoadRawData(1, imu_raw, mag_raw); 
	 
	UpdateReady = true; 
	countNo++ ; 
	
	return;  	
}

static void calibrator_timeout_handler(void * p_context)
{ 
//	if(Mag_calibrator.idx == MaxGoodMagSamplesCount)
	if(Mag_calibrator.idx == 50)
	{  
		//关闭其他中断timer
		app_timer_stop(m_calibrator_timer_id); 
		//调用计算 
		uint32_t t_start = get_time_us();
		calcMagParam( &Mag_calibrator );
		//对sensorNode赋值或其他操作记录参数
		uint32_t t_stop = get_time_us();
		uint32_t diff_us = t_stop -t_start; 
		NRF_LOG_PRINTF("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
		NRF_LOG_PRINTF("Calculate time(us): %d %d %d\n",t_start ,t_stop, diff_us); 
		NRF_LOG_PRINTF("Out func offset: %f\t%f\t%f\nScale: %f\t%f\t%f\n",Mag_calibrator.mag_bias_x,
		Mag_calibrator.mag_bias_y,Mag_calibrator.mag_bias_z,Mag_calibrator.mag_scale_x,Mag_calibrator.mag_scale_y,Mag_calibrator.mag_scale_z);
		//计算完毕恢复正常
		//isCalibration = false;
		//initCalibrator(&Mag_calibrator);
		ApplyMagParam2Sensor(&Mag_calibrator, &SensorNode1);		
	}
	else
	{
		if(AddMagSample(&Mag_calibrator , &SensorNode1))
		{ 	
			NRF_LOG_PRINTF("AddMagSample: %d\n", Mag_calibrator.idx);
		//-----------------------------------------SendUartForTest
			static uint8_t id = 0;
			int32_t timestamp, out_x, out_y, out_z;
			uint8_t buffer[20] = {0};
			sStream stream;
			timestamp = get_time_ms();
 	
		
			memset(buffer, 0, sizeof(buffer));
			stream_init(&stream, buffer, sizeof(buffer));
			stream_putbits(&stream, 2,  	 2);		//shoudler(2)
			stream_putbits(&stream, timestamp,  	 9);		//时间戳(0~511) //128
			stream_putbits(&stream, id++, 	 3);		//保号(0~31)
			
			
			out_x = (int16_t) (SensorNode1.mx_raw * 10000.0f);
			out_y = (int16_t) (SensorNode1.my_raw * 10000.0f);
			out_z = (int16_t) (SensorNode1.mz_raw * 10000.0f);
			
			stream_putbits(&stream, out_x, 10);	//地磁X
			stream_putbits(&stream, out_y, 10);	//地磁Y
			stream_putbits(&stream, out_z, 10);	//地磁Z
			stream_putbits(&stream, 0, 4);
			
			//Shoulder imu y -x z  Mag x y z
			out_x = (int16_t) (SensorNode1.ax_raw * 10000.0f);
			out_y = (int16_t) (SensorNode1.ay_raw * 10000.0f);
			out_z = (int16_t) (SensorNode1.az_raw * 10000.0f);
			
			stream_putbits(&stream, out_x, 16);	//ACC X
			stream_putbits(&stream, out_y, 16);	//ACC Y
			stream_putbits(&stream, out_z, 16);	//ACC Z
			
		
			out_x = (int32_t) (SensorNode1.gx_raw * 1000.0f);
			out_y = (int32_t) (SensorNode1.gy_raw * 1000.0f);
			out_z = (int32_t) (SensorNode1.gz_raw * 1000.0f);
			
			//NRF_LOG_PRINTF("\nMAG: %f %f %f \n",SensorNode1.gx_raw , SensorNode1.gy_raw , SensorNode1.gz_raw );
			stream_putbits(&stream, out_x, 16);	//GYRO X
			stream_putbits(&stream, out_y, 16);	//GYRO Y
			stream_putbits(&stream, out_z, 16);	//GYRO Z

			if(1)//bleSendErrCode == 0)
			{
				app_uart_put(0xFE);
				app_uart_put(0xEF); 
				int i;
				for(i =0; i <20; i++)
				{
					app_uart_put(buffer[i]);
				} 
				app_uart_put(0x0d);
				app_uart_put(0x0a);
			} 
		} 
	}	
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.

    uint32_t err_code;
    err_code = app_timer_create(&m_sensor_timer_id, APP_TIMER_MODE_REPEATED, sensor_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_calibrator_timer_id, APP_TIMER_MODE_REPEATED, calibrator_timeout_handler);
	APP_ERROR_CHECK(err_code);
	
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
    APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the YYY Service events. 
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service, 
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. \r\n", p_evt->params.char_xx.value.p_str);
            break;
        
        default:
            // No implementation needed.
            break;
    }
}*/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    /* YOUR_JOB: Add code to initialize the services used by the application.*/
    uint32_t                           err_code;
    ble_armlet_init_t                     armlet_init;

    // Initialize XXX Service.
    memset(&armlet_init, 0, sizeof(armlet_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN     (&armlet_init.sensor_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN     (&armlet_init.sensor_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&armlet_init.sensor_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&armlet_init.sensor_report_read_perm);

    armlet_init.evt_handler          = NULL;
    
    err_code = ble_armlet_init(&m_armlet, &armlet_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
*/
static void application_timers_start(void)
{
	uint32_t err_code;

	err_code = app_timer_start(m_sensor_timer_id, APP_TIMER_TICKS(2, APP_TIMER_PRESCALER), NULL);  ///used to be 2
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_start(m_calibrator_timer_id, APP_TIMER_TICKS(200, APP_TIMER_PRESCALER), NULL);
	APP_ERROR_CHECK(err_code);
	//isCalibration = true; 
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
	return;
	
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
            {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    dm_ble_evt_handler(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
	ble_armlet_on_ble_evt(&m_armlet, p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in] sys_evt  System stack event.
 */
static void sys_evt_dispatch(uint32_t sys_evt)
{
    pstorage_sys_event_handler(sys_evt);
    ble_advertising_on_sys_evt(sys_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}


/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    advdata.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


static uint32_t SendPKG()
{
		static uint8_t id = 0;
		int32_t timestamp, out_x, out_y, out_z;
		uint8_t buffer[20] = {0};
		sStream stream;
		timestamp = get_time_ms();
 	
	
		memset(buffer, 0, sizeof(buffer));
		stream_init(&stream, buffer, sizeof(buffer));
		stream_putbits(&stream, 2,  	 2);		//shoudler(2)
		stream_putbits(&stream, timestamp,  	 9);		//时间戳(0~511) //128
		stream_putbits(&stream, id++, 	 3);		//保号(0~31)
		
		
		out_x = (int16_t) (SensorNode1.mx_raw * 10000.0f);
		out_y = (int16_t) (SensorNode1.my_raw * 10000.0f);
		out_z = (int16_t) (SensorNode1.mz_raw * 10000.0f);
		
		stream_putbits(&stream, out_x, 10);	//地磁X
		stream_putbits(&stream, out_y, 10);	//地磁Y
		stream_putbits(&stream, out_z, 10);	//地磁Z
		stream_putbits(&stream, 0, 4);
		
		//Shoulder imu y -x z  Mag x y z
		out_x = (int16_t) (SensorNode1.ax_raw * 10000.0f);
		out_y = (int16_t) (SensorNode1.ay_raw * 10000.0f);
		out_z = (int16_t) (SensorNode1.az_raw * 10000.0f);
		
		stream_putbits(&stream, out_x, 16);	//ACC X
		stream_putbits(&stream, out_y, 16);	//ACC Y
		stream_putbits(&stream, out_z, 16);	//ACC Z
		
	
		out_x = (int32_t) (SensorNode1.gx_raw * 1000.0f);
		out_y = (int32_t) (SensorNode1.gy_raw * 1000.0f);
		out_z = (int32_t) (SensorNode1.gz_raw * 1000.0f);
		
		//NRF_LOG_PRINTF("\nMAG: %f %f %f \n",SensorNode1.gx_raw , SensorNode1.gy_raw , SensorNode1.gz_raw );
		stream_putbits(&stream, out_x, 16);	//GYRO X
		stream_putbits(&stream, out_y, 16);	//GYRO Y
		stream_putbits(&stream, out_z, 16);	//GYRO Z

		if(1)//bleSendErrCode == 0)
		{
			app_uart_put(0xFE);
			app_uart_put(0xEF); 
			int i;
			for(i =0; i <20; i++)
			{
				app_uart_put(buffer[i]);
			} 
			app_uart_put(0x0d);
			app_uart_put(0x0a);
		}  




	/*
	static uint8_t id = 0;
	int32_t timestamp, out_x, out_y, out_z, out_w;
	uint8_t buffer[25] = {0};
	
	sStream stream;
	timestamp = get_time_ms();
	
	memset(buffer, 0, sizeof(buffer));
	stream_init(&stream, buffer, sizeof(buffer));
	stream_putbits(&stream, 2,  	 2);		//shoudler(2)
	stream_putbits(&stream, timestamp,  	 9);		//时间戳(0~511) //128
	stream_putbits(&stream, id++, 	 3);		//保号(0~31)
	
	out_x = (int16_t) (SensorNode1.mx_raw * 10000.0f);
	out_y = (int16_t) (SensorNode1.my_raw * 10000.0f);
	out_z = (int16_t) (SensorNode1.mz_raw * 10000.0f);
	
	stream_putbits(&stream, out_x, 10);	//地磁X
	stream_putbits(&stream, out_y, 10);	//地磁Y
	stream_putbits(&stream, out_z, 10);	//地磁Z
	stream_putbits(&stream, 0, 4);
	
	
	out_x = (int16_t) (SensorNode1.ax_raw * 10000.0f);
	out_y = (int16_t) (SensorNode1.ay_raw * 10000.0f);
	out_z = (int16_t) (SensorNode1.az_raw * 10000.0f);
	
	//test processraw Variance
//	out_x = (int32_t) (SensorNode1.Acc_SlideStable[0] * 10000.0f);
//	out_y = (int32_t) (SensorNode1.Acc_SlideStable[1] * 10000.0f);
//	out_z = (int32_t) (SensorNode1.Acc_SlideStable[2] * 10000.0f);
	stream_putbits(&stream, out_x, 16);	//ACC X
	stream_putbits(&stream, out_y, 16);	//ACC Y
	stream_putbits(&stream, out_z, 16);	//ACC Z
	
	
	out_x = (int32_t) (SensorNode1.gx_raw * 1000.0f);
	out_y = (int32_t) (SensorNode1.gy_raw * 1000.0f);
	out_z = (int32_t) (SensorNode1.gz_raw * 1000.0f);
//	out_x = (int32_t) (SensorNode1.Acc_SlideStable[0] * 1000.0f);
//	out_y = (int32_t) (SensorNode1.Acc_SlideStable[1] * 1000.0f);
//	out_z = (int32_t) (SensorNode1.Acc_SlideStable[2] * 1000.0f);
	stream_putbits(&stream, out_x, 16);	//GYRO X
	stream_putbits(&stream, out_y, 16);	//GYRO Y
	stream_putbits(&stream, out_z, 16);	//GYRO Z  
	
	uint32_t bleSendErrCode = 0;
	
	//bleSendErrCode = ble_armlet_send(&m_armlet, buffer, 20);

	if(bleSendErrCode == 0)
	{
		app_uart_put(0xFE);
		app_uart_put(0xEF); 
		int i;
		for(i =0; i <20; i++)
		{
			app_uart_put(buffer[i]);
		} 
		app_uart_put(0x0d);
		app_uart_put(0x0a); 
	} 
*/ 	
	//................第二包数据 算法输出测试.............................//
	//uint8_t buffer[25] = {0};	
 	memset(buffer, 0, sizeof(buffer));
	stream_init(&stream, buffer, sizeof(buffer));
	stream_putbits(&stream, 0,  	 2);				//Test(0)
	stream_putbits(&stream, timestamp,  	 9);		//时间戳(0~511) 
	stream_putbits(&stream, id, 	 5);				//保号(0~8)   //(0~31) 
	
	int32_t testW, testX, testY, testZ;

	testW = (int32_t) (SensorNode1.Ori.w * 10000.0f);
	testX = (int32_t) (SensorNode1.Ori.x * 10000.0f);
 	testY = (int32_t) (SensorNode1.Ori.y * 10000.0f);
 	testZ = (int32_t) (SensorNode1.Ori.z * 10000.0f);  	
	stream_putbits(&stream, testW, 32);	//Ori X
	stream_putbits(&stream, testX, 32);	//Ori X
 	stream_putbits(&stream, testY, 32);	//Ori Y
 	stream_putbits(&stream, testZ, 32);	//Ori Z 

	if(1)
	{
		app_uart_put(0xFE);
		app_uart_put(0xEF); 
		int i;
		for(i =0; i <20; i++)
		{
			app_uart_put(buffer[i]);
		} 
		app_uart_put(0x0d);
		app_uart_put(0x0a); 
	}  
	return 0;	
}


static uint32_t SendPKG_Q()
{
	static uint8_t id = 0;
	int32_t timestamp, out_x, out_y, out_z;
	uint8_t buffer[20] = {0};
	sStream stream;
	timestamp = get_time_ms(); 
	//................第二包数据 算法输出测试.............................//
	//uint8_t buffer[25] = {0};	
 	memset(buffer, 0, sizeof(buffer));
	stream_init(&stream, buffer, sizeof(buffer));
	stream_putbits(&stream, 0,  	 2);				//Test(0)
	stream_putbits(&stream, timestamp,  	 9);		//时间戳(0~511) 
	stream_putbits(&stream, id, 	 5);				//保号(0~8)   //(0~31) 
	
	int32_t testW, testX, testY, testZ;

	testW = (int32_t) (SensorNode1.Ori.w * 10000.0f);
	testX = (int32_t) (SensorNode1.Ori.x * 10000.0f);
 	testY = (int32_t) (SensorNode1.Ori.y * 10000.0f);
 	testZ = (int32_t) (SensorNode1.Ori.z * 10000.0f);  	
	stream_putbits(&stream, testW, 32);	//Ori X
	stream_putbits(&stream, testX, 32);	//Ori X
 	stream_putbits(&stream, testY, 32);	//Ori Y
 	stream_putbits(&stream, testZ, 32);	//Ori Z 

	//if(1)
	//{
	//	app_uart_put(0xFE);
	//	app_uart_put(0xEF); 
	//	int i;
	//	for(i =0; i <20; i++)
	//	{
	//		app_uart_put(buffer[i]);
	//	} 
	//	app_uart_put(0x0d);
	//	app_uart_put(0x0a); 
	//}  
	return 0;	
}
/**@brief main loop.
 */
static void main_loop()
{
	 power_manage();
	//if(!isCalibration)
	//{
		if(UpdateReady == true)
		{
			processData(&SensorNode1); 
			if(countNo > 0)  //5->85fps)
			{
				if(SendPKG() == 0)
				{
				countNo = 0;
				} 
			}
			UpdateReady = false;		
		}
	//}
}



/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool erase_bonds;
//	  float gx_cali = 0;
//	  float gy_cali = 0;
//	  float gz_cali = 0;

    // Initialize.
    timers_init();
    buttons_leds_init(&erase_bonds);
    ble_stack_init();
    device_manager_init(erase_bonds);
    gap_params_init();
    advertising_init();
    services_init();
    conn_params_init();
	
		NRF_LOG_INIT();
		NRF_LOG_PRINTF("Hello\r\n");

		twi_init();
		mag_init();
		imu_init();

    // Start execution.
    application_timers_start();
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
	
    initAlgoParam(&SensorNode1);
	initCalibrator(&Mag_calibrator);
 
	// Enter main loop.
	for(; ;)
	{ 
		main_loop();
	}
}

/**
 * @}
 */
