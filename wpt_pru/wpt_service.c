#include <stdint.h>
#include <string.h>
#include "wpt_service.h"
#include "wpt_pru.h"
#include "pca10028.h"
#include "ble_srv_common.h"
#include "app_error.h"
#include "SEGGER_RTT.h"

/**@brief Function for adding our new characterstic to "WPT service"
 *
 * @param[in]   p_wpt_service        WPT Service structure.
 *
 */
static uint32_t wpt_pru_ctrl_add(ble_wpt_service_t * p_our_service)
{
		// Define charactersitic value type
		uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = WPT_PRU_CONTROL_BASE_UUID;
		char_uuid.uuid      = WPT_CHARGING_PRU_CONTROL_UUID;

		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
	
		// Configure Characteristic Metadata
		ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;
		char_md.char_props.write_wo_resp = 1;
	
		// Configue Attribute Metadata
		ble_gatts_attr_md_t attr_md;
		memset(&attr_md, 0, sizeof(attr_md));
		attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
		// Set read/write security levels to characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
		// Configure Characteristic Value Attribute
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
		
		// Set characteristic length in number of bytes
		attr_char_value.max_len 	=	5;
		attr_char_value.init_len	= 5;
		uint8_t value[5]					= {0};
		attr_char_value.p_value		= value;
		
		// Add characteristic to service
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_our_service->char_handles);
		APP_ERROR_CHECK(err_code);
		
		return err_code;
}

/**@brief Function for adding our new characterstic to "WPT service"
 *
 * @param[in]   p_wpt_service        WPT Service structure.
 *
 */
static uint32_t wpt_ptu_static_params_add(ble_wpt_service_t * p_our_service)
{
		// Define charactersitic value type
		uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = WPT_PTU_STATIC_BASE_UUID;
		char_uuid.uuid      = WPT_CHARGING_PTU_STATIC_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
	
		APP_ERROR_CHECK(err_code);
	
		// Configure Characteristic Metadata
		ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;
		char_md.char_props.write_wo_resp = 1;
	
		// Configue Attribute Metadata
		ble_gatts_attr_md_t attr_md;
		memset(&attr_md, 0, sizeof(attr_md));
		attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
		// Set read/write security levels to characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
		// Configure Characteristic Value Attribute
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
		
		// Set characteristic length in number of bytes
		attr_char_value.max_len 	=	17;
		attr_char_value.init_len	= 17;
		uint8_t value[17]					= {0};
		attr_char_value.p_value		= value;
		
		// Add characteristic to service
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_our_service->char_handles);
		APP_ERROR_CHECK(err_code);
		
		return err_code;
}

/**@brief Function for adding our new characterstic to "WPT service"
 *
 * @param[in]   p_wpt_service        WPT Service structure.
 *
 */
static uint32_t wpt_pru_alert_add(ble_wpt_service_t * p_our_service)
{
		// Define charactersitic value type
		uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = WPT_PRU_ALERT_BASE_UUID;
		char_uuid.uuid      = WPT_CHARGING_PRU_ALERT_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);
	
		// Configure Characteristic Metadata
		ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.notify = 1;
	
	  // Configuring Client Characteristic Configuration Descriptor metadata
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
		cccd_md.vloc                = BLE_GATTS_VLOC_STACK;
		char_md.p_cccd_md           = &cccd_md;
		char_md.char_props.notify   = 1;
	
		// Configue Attribute Metadata
		ble_gatts_attr_md_t attr_md;
		memset(&attr_md, 0, sizeof(attr_md));
		attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
		// Set read/write security levels to characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
		// Configure Characteristic Value Attribute
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
		
		// Set characteristic length in number of bytes
		attr_char_value.max_len 	=	1;
		attr_char_value.init_len	= 1;
		uint8_t value[1]					= {0};
		attr_char_value.p_value		= value;
		
		// Add characteristic to service
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_our_service->char_handles);
		APP_ERROR_CHECK(err_code);
		
		return err_code;
}

/**@brief Function for adding our new characterstic to "WPT service"
 *
 * @param[in]   p_wpt_service        WPT Service structure.
 *
 */
static uint32_t wpt_pru_static_params_add(ble_wpt_service_t * p_our_service)
{
		// Define charactersitic value type
		uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = WPT_PRU_STATIC_BASE_UUID;
		char_uuid.uuid      = WPT_CHARGING_PRU_STATIC_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);
	
		// Configure Characteristic Metadata
		ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;
		char_md.char_props.write_wo_resp = 1;
	
		// Configue Attribute Metadata
		ble_gatts_attr_md_t attr_md;
		memset(&attr_md, 0, sizeof(attr_md));
		attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
		// Set read/write security levels to characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
		// Configure Characteristic Value Attribute
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
		
		// Set characteristic length in number of bytes
		attr_char_value.max_len 	=	20;
		attr_char_value.init_len	= 20;
		uint8_t value[20]					= {0};
		attr_char_value.p_value		= value;
		
		// Add characteristic to service
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_our_service->char_handles);
		APP_ERROR_CHECK(err_code);
		
		return err_code;
}

/**@brief Function for adding our new characterstic to "WPT service"
 *
 * @param[in]   p_wpt_service        WPT Service structure.
 *
 */
static uint32_t wpt_pru_dynamic_params_add(ble_wpt_service_t * p_our_service)
{
		// Define charactersitic value type
		uint32_t            err_code;
		ble_uuid_t          char_uuid;
		ble_uuid128_t       base_uuid = WPT_PRU_DYNAMIC_BASE_UUID ;
		char_uuid.uuid      = WPT_CHARGING_PRU_DYNAMIC_UUID;
		err_code = sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
		APP_ERROR_CHECK(err_code);
	
		// Configure Characteristic Metadata
		ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
		char_md.char_props.read = 1;
		char_md.char_props.write = 1;
		char_md.char_props.write_wo_resp = 1;
	
		// Configue Attribute Metadata
		ble_gatts_attr_md_t attr_md;
		memset(&attr_md, 0, sizeof(attr_md));
		attr_md.vloc        = BLE_GATTS_VLOC_STACK;
	
		// Set read/write security levels to characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	
		// Configure Characteristic Value Attribute
		ble_gatts_attr_t    attr_char_value;
		memset(&attr_char_value, 0, sizeof(attr_char_value));    
		attr_char_value.p_uuid      = &char_uuid;
		attr_char_value.p_attr_md   = &attr_md;
		
		// Set characteristic length in number of bytes
		attr_char_value.max_len 	=	20;
		attr_char_value.init_len	= 20;
		uint8_t value[20]					= {0};
		attr_char_value.p_value		= value;
		
		// Add characteristic to service
		err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
                                   &char_md,
                                   &attr_char_value,
                                   &p_our_service->char_handles);
		APP_ERROR_CHECK(err_code);
		
		return err_code;
}



/**@brief Function for adding our new characterstic to "WPT service"
 *
 * @param[in]   p_wpt_service        WPT Service structure.
 *
 */
static void wpt_char_add(ble_wpt_service_t * p_our_service)
{
		wpt_pru_ctrl_add(p_our_service);
		wpt_ptu_static_params_add(p_our_service);
		wpt_pru_alert_add(p_our_service);
		wpt_pru_static_params_add(p_our_service);
		wpt_pru_dynamic_params_add(p_our_service);
	
}
/**@brief Function for initiating our new service.
 *
 * @param[in]   p__wpt_service        WPT Service structure.
 *
 */
void wpt_service_init(ble_wpt_service_t * p_wpt_service)
{
	  uint32_t err_code;
		ble_uuid_t service_uuid;
		service_uuid.uuid = 0xFFFE;
		service_uuid.type = BLE_UUID_TYPE_BLE;
	
		p_wpt_service->conn_handle = BLE_CONN_HANDLE_INVALID;
	
		err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
																				&service_uuid,
																				&p_wpt_service->service_handle);
	
		APP_ERROR_CHECK(err_code);
	
		wpt_char_add(p_wpt_service);
	
}

void wpt_send_pru_alert(uint16_t m_conn_handle, uint8_t *wpt_pru_alert)
{
		
		if (m_conn_handle != BLE_CONN_HANDLE_INVALID && *wpt_pru_alert != WPT_PRU_ALERT_NONE)
		{
				uint16_t						len = sizeof(uint8_t);
				ble_gatts_hvx_params_t hvx_params;
				memset(&hvx_params, 0 , sizeof(hvx_params));
	
				hvx_params.handle = WPT_PRU_ALERT_HANDLE;
				hvx_params.type		= BLE_GATT_HVX_NOTIFICATION;
				hvx_params.offset	= 0;
				hvx_params.p_len 	= &len;
				hvx_params.p_data	= (uint8_t*)wpt_pru_alert;
		
				sd_ble_gatts_hvx(m_conn_handle, &hvx_params);
		}
}
