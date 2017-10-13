/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/* Attention! 
*  To maintain compliance with Nordic Semiconductor ASA’s Bluetooth profile 
*  qualification listings, this section of source code must not be modified.
*/

#include "ble_armlet.h"
#include <string.h>
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_armlet_t * p_bas, ble_evt_t * p_ble_evt)
{
    p_bas->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_armlet_t * p_bas, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_bas->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_bas       Battery Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_armlet_t * p_bas, ble_evt_t * p_ble_evt)
{
	ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (
		(p_evt_write->handle == p_bas->sensor_handles.cccd_handle)
		&&
		(p_evt_write->len == 2)
	   )
	{
		// CCCD written, call application event handler
		if (p_bas->evt_handler != NULL)
		{
			ble_armlet_evt_t evt;

			if (ble_srv_is_notification_enabled(p_evt_write->data))
			{
				evt.evt_type = BLE_ARMLET_EVT_NOTIFICATION_ENABLED;
			}
			else
			{
				evt.evt_type = BLE_ARMLET_EVT_NOTIFICATION_DISABLED;
			}

			p_bas->evt_handler(p_bas, &evt);
		}
	}
}


void ble_armlet_on_ble_evt(ble_armlet_t * p_bas, ble_evt_t * p_ble_evt)
{
    if (p_bas == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_bas, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_bas, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_bas, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the Battery Level characteristic.
 *
 * @param[in]   p_bas        Battery Service structure.
 * @param[in]   p_bas_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t sensor_char_add(ble_armlet_t * p_bas, const ble_armlet_init_t * p_bas_init)
{
	//00000001-1000-1000-8000-00805F9B34FB
	//00000002-1000-1000-8000-00805F9B34FB
	//00000003-1000-1000-8000-00805F9B34FB

	uint8_t init_dat[20] = {0};
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_gatts_attr_md_t attr_md;
	ble_uuid_t          ble_uuid;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    cccd_md.write_perm = p_bas_init->sensor_char_attr_md.cccd_write_perm;

    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.indicate = 0;
	char_md.char_props.notify   = 1;
	char_md.char_props.write    = 0;
	char_md.char_props.read     = 0;
    char_md.p_char_user_desc    = NULL;
    char_md.p_char_pf           = NULL;
    char_md.p_user_desc_md      = NULL;
    char_md.p_cccd_md           = &cccd_md;
    char_md.p_sccd_md           = NULL;
	
	BLE_UUID_BLE_ASSIGN(ble_uuid, 0x0001);

    memset(&attr_md, 0, sizeof(attr_md));

    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.read_perm  = p_bas_init->sensor_char_attr_md.read_perm;
    attr_md.write_perm = p_bas_init->sensor_char_attr_md.write_perm;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = 20;
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = 20;
    attr_char_value.p_value   = init_dat;

    return sd_ble_gatts_characteristic_add(p_bas->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_bas->sensor_handles);
}

uint32_t ble_armlet_init(ble_armlet_t * p_bas, const ble_armlet_init_t * p_bas_init)
{
    if (p_bas == NULL || p_bas_init == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_bas->evt_handler               = p_bas_init->evt_handler;
    p_bas->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add service
    BLE_UUID_BLE_ASSIGN(ble_uuid, 0xFFF1);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_bas->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add characteristic
    return sensor_char_add(p_bas, p_bas_init);
}


uint32_t ble_armlet_send(ble_armlet_t * p_bas, uint8_t *pbuf, uint8_t len)
{
	uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;
	
    if (p_bas == NULL)
    {
        return NRF_ERROR_NULL;
    }
    
	// Initialize value struct.
	memset(&gatts_value, 0, sizeof(gatts_value));

	gatts_value.len     = len;
	gatts_value.offset  = 0;
	gatts_value.p_value = pbuf;

	// Send value if connected and notifying.
	if (p_bas->conn_handle != BLE_CONN_HANDLE_INVALID)
	{
		ble_gatts_hvx_params_t hvx_params;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_bas->sensor_handles.value_handle;
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = gatts_value.offset;
		hvx_params.p_len  = &gatts_value.len;
		hvx_params.p_data = gatts_value.p_value;

		err_code = sd_ble_gatts_hvx(p_bas->conn_handle, &hvx_params);
	}
	else
	{
		err_code = NRF_ERROR_INVALID_STATE;
	}

    return err_code;
}
