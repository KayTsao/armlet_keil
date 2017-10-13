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
#include "ble_bas.h"
#include "ble_dis.h"
#ifdef BLE_DFU_APP_SUPPORT
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#endif // BLE_DFU_APP_SUPPORT
#include "nrf_delay.h"

#include "i2c_state.h"
#include "stream.h"
#include "timer.h"
#include "battery.h"

#include "imu_1_lsm6dsl.h"
#include "imu_2_lsm6dsl.h"
#include "mag_1_ak09911.h"
#include "mag_2_ak09911.h"
#include "attitudeSensor.h" 

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define CENTRAL_LINK_COUNT               0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT            1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                      "IQIYI Armlet"                             /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                "IQIYI"                                    /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                        "IQIYI-Armlet"                               /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                  0x1122334455                               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                    0x667788  


#define APP_ADV_INTERVAL                 300                                        /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS       180                                        /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER              0                                          /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE          6                                          /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL                MSEC_TO_UNITS(7.5, UNIT_1_25_MS)           /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL                MSEC_TO_UNITS(10, UNIT_1_25_MS)           /**< Maximum acceptable connection interval (0.2 second). */
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
#ifdef BLE_DFU_APP_SUPPORT
#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT

static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */

static uint16_t                          m_conn_handle = BLE_CONN_HANDLE_INVALID;   /**< Handle of the current connection. */

static ble_armlet_t                      m_armlet;  
static ble_bas_t                         m_bas;                                     /**< Structure used to identify the battery service. */
#ifdef BLE_DFU_APP_SUPPORT
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
#endif // BLE_DFU_APP_SUPPORT

APP_TIMER_DEF(m_sensor_timer_id);                                                   /**< Sensor timer. */
APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */

static bool AppTimerFlag;

static SampleSlideBuf GlobalAccSampleSlideBuf ;
static SampleSlideBuf GlobalGyrSampleSlideBuf ;
static AttitudeSensor SensorNode1, SensorNode2;


// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =  {{BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
									{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}}; /**< Universally unique service identifiers. */

                                   
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
	sensor->ax_raw = (float)rawIMU[0] * accRatio; 
	sensor->ay_raw = (float)rawIMU[1] * accRatio;
	sensor->az_raw = (float)rawIMU[2] * accRatio; 
	 
	sensor->gx_raw = (float)rawIMU[3] * gyroRatio;
	sensor->gy_raw = (float)rawIMU[4] * gyroRatio;
	sensor->gz_raw = (float)rawIMU[5] * gyroRatio;
	
	sensor->mx_raw = (float)rawMag[0] * magRatio_9918;
	sensor->my_raw = (float)rawMag[1] * magRatio_9918;
	sensor->mz_raw = (float)rawMag[2] * magRatio_9918; 
	sensor->cur_t_us = get_time_us(); 
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
	int16_t imu_1_raw[6];
	int16_t mag_1_raw[3];         

//	int16_t imu_2_raw[6];	int16_t mag_2_raw[3]; 
 
  UNUSED_PARAMETER(p_context);
	
	bat_measure();
	imu_1_measure();
	mag_1_measure();	

	//imu_2_measure();	mag_2_measure();
	
	imu_1_getraw(imu_1_raw);
	mag_1_getraw(mag_1_raw);	

	//imu_2_getraw(imu_2_raw);	mag_2_getraw(mag_2_raw);
 
	//KK load latest measurement
	LoadRawData(1, imu_1_raw, mag_1_raw);
	//LoadRawData(2, imu_2_raw, mag_2_raw);
	 
	UpdateReady = true; 
	
	return;  	
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information (context) from the
 *                        app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    uint32_t err_code;
	
	err_code = ble_bas_battery_level_update(&m_bas, bat_level_percent());
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
	uint32_t err_code;
    
	// Initialize timer module.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);

    // Create timers.
    err_code = app_timer_create(&m_sensor_timer_id, APP_TIMER_MODE_REPEATED, sensor_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_create(&m_battery_timer_id,APP_TIMER_MODE_REPEATED, battery_level_meas_timeout_handler);
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



#ifdef BLE_DFU_APP_SUPPORT
/**@brief Function for stopping advertising.
 */
static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for loading application-specific context after establishing a secure connection.
 *
 * @details This function will load the application context and check if the ATT table is marked as
 *          changed. If the ATT table is marked as changed, a Service Changed Indication
 *          is sent to the peer if the Service Changed CCCD is set to indicate.
 *
 * @param[in] p_handle The Device Manager handle that identifies the connection for which the context
 *                     should be loaded.
 */
static void app_context_load(dm_handle_t const * p_handle)
{
    uint32_t                 err_code;
    static uint32_t          context_data;
    dm_application_context_t context;

    context.len    = sizeof(context_data);
    context.p_data = (uint8_t *)&context_data;

    err_code = dm_application_context_get(p_handle, &context);
    if (err_code == NRF_SUCCESS)
    {
        // Send Service Changed Indication if ATT table has changed.
        if ((context_data & (DFU_APP_ATT_TABLE_CHANGED << DFU_APP_ATT_TABLE_POS)) != 0)
        {
            err_code = sd_ble_gatts_service_changed(m_conn_handle, APP_SERVICE_HANDLE_START, BLE_HANDLE_MAX);
            if ((err_code != NRF_SUCCESS) &&
                (err_code != BLE_ERROR_INVALID_CONN_HANDLE) &&
                (err_code != NRF_ERROR_INVALID_STATE) &&
                (err_code != BLE_ERROR_NO_TX_PACKETS) &&
                (err_code != NRF_ERROR_BUSY) &&
                (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
            {
                APP_ERROR_HANDLER(err_code);
            }
        }

        err_code = dm_application_context_delete(p_handle);
        APP_ERROR_CHECK(err_code);
    }
    else if (err_code == DM_NO_APP_CONTEXT)
    {
        // No context available. Ignore.
    }
    else
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/** @snippet [DFU BLE Reset prepare] */
/**@brief Function for preparing for system reset.
 *
 * @details This function implements @ref dfu_app_reset_prepare_t. It will be called by
 *          @ref dfu_app_handler.c before entering the bootloader/DFU.
 *          This allows the current running application to shut down gracefully.
 */
static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(50);
}
/** @snippet [DFU BLE Reset prepare] */
#endif // BLE_DFU_APP_SUPPORT

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    /* YOUR_JOB: Add code to initialize the services used by the application.*/
   uint32_t           err_code;
	ble_bas_init_t    bas_init;
    ble_dis_init_t    dis_init;
    ble_armlet_init_t armlet_init;
	ble_dis_sys_id_t  sys_id;

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
	
	// Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
	
	// Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     MODEL_NUM);

    sys_id.manufacturer_id            = MANUFACTURER_ID;
    sys_id.organizationally_unique_id = ORG_UNIQUE_ID;
    dis_init.p_sys_id                 = &sys_id;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [DFU BLE Service initialization] */
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
#endif // BLE_DFU_APP_SUPPORT
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

	err_code = app_timer_start(m_sensor_timer_id, APP_TIMER_TICKS(6, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_start(m_battery_timer_id, APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
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
#ifdef BLE_DFU_APP_SUPPORT
    /** @snippet [Propagating BLE Stack events to DFU Service] */
    ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */
#endif // BLE_DFU_APP_SUPPORT
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

#ifdef BLE_DFU_APP_SUPPORT
    ble_enable_params.gatts_enable_params.service_changed = 1;
#endif // BLE_DFU_APP_SUPPORT
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


/**@brief Function for application main entry.
 */

static void initAlgoParam()
{
	UpdateReady = false;
	SensorNode1.ax_raw = SensorNode1.ay_raw = SensorNode1.az_raw = 0.0f;
	SensorNode1.gx_raw = SensorNode1.gy_raw = SensorNode1.gz_raw = 0.0f;
	SensorNode1.mx_raw = SensorNode1.my_raw = SensorNode1.mz_raw = 0.0f;


	SensorNode2.ax_raw = SensorNode2.ay_raw = SensorNode2.az_raw = 0.0f;
	SensorNode2.gx_raw = SensorNode2.gy_raw = SensorNode2.gz_raw = 0.0f;
	SensorNode2.mx_raw = SensorNode2.my_raw = SensorNode2.mz_raw = 0.0f;

}

static void processRawData()
{
	
}
static void SendPKG()
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
	
	
	out_x = (int16_t) (SensorNode1.ax_raw * 10000.0f);
	out_y = (int16_t) (SensorNode1.ay_raw * 10000.0f);
	out_z = (int16_t) (SensorNode1.az_raw * 10000.0f);
	
	stream_putbits(&stream, out_x, 16);	//ACC X
	stream_putbits(&stream, out_y, 16);	//ACC Y
	stream_putbits(&stream, out_z, 16);	//ACC Z
	
	
	out_x = (int32_t) (SensorNode1.gx_raw * 1000.0f);
	out_y = (int32_t) (SensorNode1.gy_raw * 1000.0f);
	out_z = (int32_t) (SensorNode1.gz_raw * 1000.0f);
	
	stream_putbits(&stream, out_x, 16);	//GYRO X
	stream_putbits(&stream, out_y, 16);	//GYRO Y
	stream_putbits(&stream, out_z, 16);	//GYRO Z


	uint32_t bleSendErrCode;
	//bleSendErrCode = ble_armlet_send(&m_armlet, buffer, 20);

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
	memset(buffer, 0, sizeof(buffer));
	stream_init(&stream, buffer, sizeof(buffer));
	stream_putbits(&stream, 3,  	 2);		//shoudler(2)
	stream_putbits(&stream, timestamp,  	 9);		//时间戳(0~511) //128
	stream_putbits(&stream, id++, 	 3);		//保号(0~31)
	
	out_x = (int16_t) (SensorNode2.mx_raw * 10000.0f);
	out_y = (int16_t) (SensorNode2.my_raw * 10000.0f);
	out_z = (int16_t) (SensorNode2.mz_raw * 10000.0f);
	
	stream_putbits(&stream, out_x, 10);	//地磁X
	stream_putbits(&stream, out_y, 10);	//地磁Y
	stream_putbits(&stream, out_z, 10);	//地磁Z
	stream_putbits(&stream, 0, 4);
	
	
	out_x = (int16_t) (SensorNode2.ax_raw * 10000.0f);
	out_y = (int16_t) (SensorNode2.ay_raw * 10000.0f);
	out_z = (int16_t) (SensorNode2.az_raw * 10000.0f);
	
	stream_putbits(&stream, out_x, 16);	//ACC X
	stream_putbits(&stream, out_y, 16);	//ACC Y
	stream_putbits(&stream, out_z, 16);	//ACC Z
	
	
	out_x = (int32_t) (SensorNode2.gx_raw * 1000.0f);
	out_y = (int32_t) (SensorNode2.gy_raw * 1000.0f);
	out_z = (int32_t) (SensorNode2.gz_raw * 1000.0f);
	
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

	*/
	
	
	
	
	
}

static void main_loop()
{
	if(UpdateReady == true)
	{
		processRawData();
		SendPKG();
		UpdateReady = false;		
	}
	  
}



int main(void)
{
	uint32_t err_code;
	bool erase_bonds;
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
	mag_1_init();  
	mag_2_init();
	imu_1_init();	 
	imu_2_init();
    
	// Start execution.
	application_timers_start();
  err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
	
	
	initAlgoParam();
	// Enter main loop.
	for(; ;)
	{
		//power_manage();
		main_loop();
	}
     
} 
/**
 * @}
 */
