#ifndef WPT_SERVICE_H
#define WPT_SERVICE_H

#include <stdint.h>
#include "ble.h"
#include "ble_srv_common.h"

/* To add multiple base UUIDs IRAM1 memory start and length need to be adjusted 
 *	The correct values have been set already, but if it needs to be reconfigured the
 * 	propper address and length are output to NRF_LOGGER
 */
#define WPT_CHARACTERISTIC_BASE_UUID 		{{0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64}}
#define WPT_PRU_CONTROL_BASE_UUID 			{{0x67, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64}}
#define WPT_PTU_STATIC_BASE_UUID 				{{0x68, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64}}
#define WPT_PRU_ALERT_BASE_UUID 				{{0x69, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64}}
#define WPT_PRU_STATIC_BASE_UUID 				{{0x6A, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64}}
#define WPT_PRU_DYNAMIC_BASE_UUID 			{{0x6B, 0x9A, 0x0C, 0x20, 0x00, 0x08, 0x96, 0x9E, 0xE2, 0x11, 0x46, 0xA1, 0x70, 0xE6, 0x55, 0x64}}

/* 16-bit UUIDs stay constant because only the base uuid changes */
#define WPT_CHARGING_PRU_CONTROL_UUID		0xE670
#define WPT_CHARGING_PTU_STATIC_UUID		0xE670
#define WPT_CHARGING_PRU_ALERT_UUID			0xE670
#define WPT_CHARGING_PRU_STATIC_UUID		0xE670
#define WPT_CHARGING_PRU_DYNAMIC_UUID		0xE670

#define WPT_SERVICE_UUID								0xFFFE
#define WPT_PRIMARY_SERVICE_HANDLE			0x000C

#define WPT_PRU_DYNAMIC_PARAM_HANDLE		0x17
#define WPT_PRU_ALERT_HANDLE						0x12
#define WPT_PRU_CONTROL_HANDLE					0x000E


/* Options for advertising packet RSSI options*/
#define WPT_RSSI_PRU_PWR_UNKNOWN				0xF8																			// 0b11111000
#define WPT_RSSI_PRU_GAIN_UNKNOWN				0x07																			// 0b00000111

/* Options for advertising packet service data*/
#define WPT_IMP_SHIFT_CAT3							(3<<5)
#define WPT_REBOOT											(0<<4)
#define WPT_OVP_NOT_USED								(0<<3)
#define WPT_TIME_SET_NO_SUPPORT					(0<<2)


typedef struct 
{	
		uint16_t										conn_handle;					
		uint16_t 										service_handle;
		ble_gatts_char_handles_t		char_handles;
} ble_wpt_service_t;


/**@brief Function for initializing the WPT service
 */
void wpt_service_init(ble_wpt_service_t * p_wpt_service);

/**@brief Function for sending PRU alert if they are enabled by PTU
 *
 * @param[in] m_conn_handle	 	Connection handle which needs to be alerted
 * @param[in] *wpt_pru_alert 	Pointer to single byte buffer holding alert data (defined in wpt_params.h
*/
void wpt_send_pru_alert(uint16_t m_conn_handle, uint8_t *wpt_pru_alert);

#endif
