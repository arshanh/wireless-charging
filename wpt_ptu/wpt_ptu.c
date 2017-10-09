#include "sdk_common.h"

#include <stdlib.h> 

#include "ble.h"
#include "ble_hci.h"
#include "wpt_ptu.h"
#include "wpt_pru.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "bsp.h"
#include "bsp_btn_ble.h"

#include "wpt_service.h"


void wpt_ptu_scan_start(const ble_gap_scan_params_t * p_scan_params)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_scan_start(p_scan_params);
    APP_ERROR_CHECK(err_code);

		err_code = bsp_indication_set(BSP_INDICATE_SCANNING);				// LEDs blink to indicate scanning
    APP_ERROR_CHECK(err_code);
}

/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function uses the Handle Value Notification of the WPT PRU Alert characteristic
 *             sent the peer, update the alert buffer, and handle the alert
 *
 * @param[in] p_ble_wpt_ptu Pointer to the WPT PTU instance

 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void wpt_ptu_alert_handler(ble_wpt_ptu_t * p_ble_wpt_ptu, const ble_evt_t * p_ble_evt)
{
		p_ble_wpt_ptu->wpt_service.pru_alert = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
	
		NRF_LOG_INFO("Received PRU alert: %02X\n\r", p_ble_wpt_ptu->wpt_service.pru_alert);
}

void ble_wpt_ptu_on_ble_evt(ble_wpt_ptu_t * p_ble_wpt_ptu, const ble_evt_t * p_ble_evt)
{
    // Check for invalid parameters
		if ((p_ble_wpt_ptu == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

		// Check that a valid connection exists and that it generated the event 
    if ( (p_ble_wpt_ptu->conn_handle != BLE_CONN_HANDLE_INVALID)
       &&(p_ble_wpt_ptu->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
       )
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
						// If HVX received from PRU Alert Characteristic call PTU alert handler
				    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_wpt_ptu->handles.pru_alert_handle)
						{
								wpt_ptu_alert_handler(p_ble_wpt_ptu, p_ble_evt);
            }
						break;	
				case BLE_GATTC_EVT_READ_RSP:
				{
					const ble_gattc_evt_read_rsp_t * read_rsp = &p_ble_evt->evt.gattc_evt.params.read_rsp;
					// Call PTU data handler to handle response
					wpt_ptu_data_handler(p_ble_wpt_ptu, read_rsp);
				} 
						break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_wpt_ptu->conn_handle)
            {
								NRF_LOG_INFO("Disconnected...\n");
								NRF_LOG_PROCESS();
								// Update PTU instance to reflect disconnection 
                p_ble_wpt_ptu->conn_handle = BLE_CONN_HANDLE_INVALID;
								p_ble_wpt_ptu->mode = WPT_PTU_MODE_NO_CONN;
								// Restart scanning
								wpt_ptu_scan_start(&p_ble_wpt_ptu->scan_params);
            }
            break;
    }
}

void ble_wpt_ptu_handles_assign(ble_wpt_ptu_t * p_ble_wpt_ptu)
{	
		if (p_ble_wpt_ptu != NULL) 
		{
			uint16_t base = p_ble_wpt_ptu->handles.wpt_service_handle;
			NRF_LOG_INFO("Assigning handle values from base service handle: %02X\n\r", base);
			NRF_LOG_PROCESS();
			
			// Assign Characteristic handles 
			p_ble_wpt_ptu->handles.pru_control_handle = base + WPT_PRU_CONTROL_HANDLE_OFFSET;
			p_ble_wpt_ptu->handles.ptu_static_handle = base + WPT_PTU_STATIC_HANDLE_OFFSET;
			p_ble_wpt_ptu->handles.pru_alert_handle = base + WPT_PRU_ALERT_HANDLE_OFFSET;	
			p_ble_wpt_ptu->handles.pru_cccd_handle = base + WPT_PRU_CCCD_HANDLE_OFFSET;
			p_ble_wpt_ptu->handles.pru_static_handle = base + WPT_PRU_STATIC_HANDLE_OFFSET;
			p_ble_wpt_ptu->handles.pru_dynamic_handle = base + WPT_PRU_DYNAMIC_HANDLE_OFFSET;
		
		}
}

/**
 * @brief Function for writing to the CCCD.
 *
 * @details   This function writes to the given CCCD handle to either enable 
 * 						or disable the notifications.
 *
 * @param[in] conn_handle 	Connection handle of peer
 * @param[in] cccd_handle		Handle of the CCCD to be written to
 * @param[in] enable				True to enable, False to disable notifications
 */
static uint32_t cccd_configure(uint16_t conn_handle, uint16_t cccd_handle, bool enable)
{
    uint8_t buf[BLE_CCCD_VALUE_LEN];

    buf[0] = enable ? BLE_GATT_HVX_NOTIFICATION : 0;
    buf[1] = 0;

    const ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_REQ,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = cccd_handle,
        .offset   = 0,
        .len      = sizeof(buf),
        .p_value  = buf
    };

    return sd_ble_gattc_write(conn_handle, &write_params);
}

uint32_t ble_wpt_pru_notif_enable(ble_wpt_ptu_t * p_ble_wpt_ptu)
{
    VERIFY_PARAM_NOT_NULL(p_ble_wpt_ptu);

    if ( (p_ble_wpt_ptu->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_wpt_ptu->handles.pru_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
				
    return cccd_configure(p_ble_wpt_ptu->conn_handle,p_ble_wpt_ptu->handles.pru_cccd_handle, true);
}

/**@brief     Function for formatting PTU static parameters packet
 *
 * @details   This function fills the provided byte buffer with values stored in the
 *						static_params struct according to the format defined in the A4WP Specification 
 *
 * @param[out] data							Byte buffer to be filled with formatted packet								
 * @param[in]  static_params		PTU static parameters struct holding control values to be sent
 */
static void ptu_params_format_data(uint8_t *data, const wpt_ptu_static_params_t static_params) {
	
		data[0] = static_params.optional_fields;
		data[1] = static_params.ptu_power;
		data[2] = static_params.ptu_max_src_imp;
		data[3] = static_params.ptu_max_ld_res;
		data[6] = static_params.ptu_class;
		data[7] = static_params.hardware_rev;
		data[8] = static_params.firmware_rev;
		data[9] = static_params.protocol_rev;
		data[10] = static_params.ptu_devices_supported;
		
		// Clear bytes reserved for future use
		memset(data + 4, WPT_PTU_RFU, 2);
		memset(data + 11, WPT_PTU_RFU, 6);
}

uint32_t ble_wpt_send_ptu_static_params(ble_wpt_ptu_t * p_ble_wpt_ptu)
{
    VERIFY_PARAM_NOT_NULL(p_ble_wpt_ptu);

    if (p_ble_wpt_ptu->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }
		
		const wpt_ptu_static_params_t static_params = {
				.optional_fields 			 	= WPT_PTU_OPTIONAL_FIELDS,
				.ptu_power 						  = WPT_PTU_POWER, 
				.ptu_max_src_imp 			  = WPT_PTU_MAX_SRC_IMP,
				.ptu_max_ld_res				  = WPT_PTU_MAX_LD_RES,
				.ptu_class			 			  = WPT_PTU_CLASS,
				.hardware_rev		 			  = WPT_PTU_HARDWARE_REV,
				.firmware_rev					  = WPT_PTU_FIRMWARE_REV,
				.protocol_rev					  = WPT_PTU_PROTOCOL_REV,
				.ptu_devices_supported  = WPT_PTU_DEVICES_SUPPORTED
		};
		
		uint8_t data[20];
		ptu_params_format_data(data, static_params);
		
    const ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = p_ble_wpt_ptu->handles.ptu_static_handle,
        .offset   = 0,
        .len      = sizeof(data),
        .p_value  = data
    };
		
		// Update PTU Instance's Buffer
		p_ble_wpt_ptu->wpt_service.ptu_static = static_params;
		
    return sd_ble_gattc_write(p_ble_wpt_ptu->conn_handle, &write_params);
}

/**@brief     Function for formatting PRU control pacekt
 *
 * @details   This function fills the provided byte buffer with values stored in the
 *						pru_control struct according to the format defined in the A4WP Specification 
 *
 * @param[out] data						Byte buffer to be filled with formatted packet								
 * @param[in]  pru_control		PRU control struct holding control values to be sent
 */
static void pru_control_format_data(uint8_t *data, const wpt_pru_control_t pru_control) {
	
		data[0] = pru_control.enables;
		data[1] = pru_control.permission;
		data[2] = pru_control.time_set;
		
		// Clear bytes reserved for future use
		memset(data + 3, WPT_PTU_RFU, 2);
}


uint32_t ble_wpt_send_pru_control(ble_wpt_ptu_t * p_ble_wpt_ptu, const wpt_pru_control_t pru_control)
{
    VERIFY_PARAM_NOT_NULL(p_ble_wpt_ptu);

    if (p_ble_wpt_ptu->conn_handle == BLE_CONN_HANDLE_INVALID)
    {
        return NRF_ERROR_INVALID_STATE;
    }
		
		uint8_t data[20];
		pru_control_format_data(data, pru_control);
		
    const ble_gattc_write_params_t write_params = {
        .write_op = BLE_GATT_OP_WRITE_CMD,
        .flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE,
        .handle   = p_ble_wpt_ptu->handles.pru_control_handle,
        .offset   = 0,
        .len      = sizeof(data),
        .p_value  = data
    };

    return sd_ble_gattc_write(p_ble_wpt_ptu->conn_handle, &write_params);
}

/**@brief     Function for extracting PRU static parameters from a read response packet
 *
 * @details   This function fills the provided static parameter buffer with values extracted
 *						from the read response packet according to the packet structure defined in the 
 *						A4WP Specification.
 *
 * @param[out] p_static_params 				Pointer to static parameters buffer to be filled
 * @param[in]  read_rsp   						Read response packet
 */
static void pru_static_params_extract(wpt_pru_static_params_t * p_static_params, const ble_gattc_evt_read_rsp_t * read_rsp ) {
		
		if (read_rsp->len == WPT_PRU_STATIC_DATA_LEN) 
			{
				// Assign single byte values to respective field in p_static_params
				p_static_params->optional_fields = read_rsp->data[0];				
				p_static_params->protocol_rev = read_rsp->data[1];					
				// data[2] Reserved for future use
				p_static_params->pru_category = read_rsp->data[3];					
				p_static_params->pru_info = read_rsp->data[4];							
				p_static_params->hardware_rev = read_rsp->data[5];					
				p_static_params->firmware_rev = read_rsp->data[6];					
				p_static_params->p_rect_max = read_rsp->data[7];						

				// Assign multi-byte values to respective fields in p_static_params
				memcpy(&p_static_params->v_rect_min_static, read_rsp->data + 8, sizeof(uint16_t));
				memcpy(&p_static_params->v_rect_high_static, read_rsp->data + 10, sizeof(uint16_t));
				memcpy(&p_static_params->v_rect_set, read_rsp->data + 12, sizeof(uint16_t));
				
				// Assign optional fields if valid
				if (p_static_params->optional_fields & WPT_PRU_STATIC_DELTA_R1)
						memcpy(&p_static_params->delta_r1, read_rsp->data + 14, sizeof(uint16_t));
			
				// Bytes 16 to 20 Reserved for future use
		}
}

/**@brief     Function for extracting PRU dynamic parameters from a read response packet
 *
 * @details   This function fills the provided dynamic parameter buffer with values extracted
 *						from the read response packet according to the packet structure defined in the 
 *						A4WP Specification.
 *
 * @param[out] p_dynamic_params 									Pointer to dynamic parameters buffer to be filled
 * @param[in]  const ble_gattc_evt_read_rsp_t   	Read response packet
 */
static void pru_dynamic_params_extract(wpt_dynamic_params_t * p_dynamic_params, const ble_gattc_evt_read_rsp_t * read_rsp ) {
		
		if (read_rsp->len == WPT_PRU_DYNAMIC_DATA_LEN) 
			{
				// Assign single byte values to respective field in p_dynamic_params
				p_dynamic_params->optional_fields = read_rsp->data[0];
				p_dynamic_params->pru_alert = read_rsp->data[15];
				p_dynamic_params->tester_command = read_rsp->data[16];
				
				// Assign multi byte values to respective field in p_dynamic_params
				memcpy(&p_dynamic_params->V_rect, read_rsp->data + 1, sizeof(uint16_t));
				memcpy(&p_dynamic_params->I_rect, read_rsp->data + 3, sizeof(uint16_t));
				
				// Assign optional fields if valid
				if (p_dynamic_params->optional_fields & WPT_PRU_DYNAMIC_VOUT)
						memcpy(&p_dynamic_params->V_out, read_rsp->data + 5, sizeof(uint16_t));
				
				if (p_dynamic_params->optional_fields & WPT_PRU_DYNAMIC_IOUT)
						memcpy(&p_dynamic_params->I_out, read_rsp->data + 7, sizeof(uint16_t));
				
				if (p_dynamic_params->optional_fields & WPT_PRU_DYNAMIC_TEMP)
						p_dynamic_params->temperature = read_rsp->data[9];
				
				if (p_dynamic_params->optional_fields & WPT_PRU_DYNAMIC_VRECT_MIN)
						memcpy(&p_dynamic_params->V_rect_min, read_rsp->data + 10, sizeof(uint16_t));
				
				if (p_dynamic_params->optional_fields & WPT_PRU_DYNAMIC_VRECT_SET)
						memcpy(&p_dynamic_params->V_rect_set, read_rsp->data + 12, sizeof(uint16_t));
				
				if (p_dynamic_params->optional_fields & WPT_PRU_DYNAMIC_VRECT_HIGH)
						memcpy(&p_dynamic_params->V_rect_high, read_rsp->data + 14, sizeof(uint16_t));
				
				// Bytes 18 to 20 Reserved for future use
		}
}

void wpt_ptu_data_handler(ble_wpt_ptu_t * p_ble_wpt_ptu, const ble_gattc_evt_read_rsp_t * read_rsp)
{
		if (read_rsp->handle == p_ble_wpt_ptu->handles.pru_static_handle) 
		{
				NRF_LOG_INFO("Recieved read response from PRU static params\n");
				// Extract parameters from the read response
				pru_static_params_extract(&p_ble_wpt_ptu->wpt_service.pru_static, read_rsp);
				
			// Indicate completion
				p_ble_wpt_ptu->reg_status.pru_static_complete = true;
			
		}
		else if (read_rsp->handle == p_ble_wpt_ptu->handles.pru_dynamic_handle) 
		{
				NRF_LOG_INFO("Recieved read response from PRU dynamic params\n");
			
				// Extract parameters from the read response
				pru_dynamic_params_extract(&p_ble_wpt_ptu->wpt_service.pru_dynamic, read_rsp);
				
				// Indicate completion if during PRU registration
				if (p_ble_wpt_ptu->mode == WPT_PTU_MODE_REGISTRATION)
						p_ble_wpt_ptu->reg_status.pru_dynamic_complete = true;
		
		}
}

bool wpt_ptu_connection_accepted(ble_wpt_ptu_t * p_ble_wpt_ptu)
{
		// Connection always accepted for now
		return true;
}

