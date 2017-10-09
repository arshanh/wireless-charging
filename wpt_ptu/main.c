/**@file		 main.c
 *
 * @brief    WPT PTU Main 
 *
 * @details  
 *
 * @note     
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "boards.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"

#include "wpt_service.h"
#include "wpt_ptu.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define CENTRAL_LINK_COUNT      1                               /**< Number of central links used by the application. */
#define PERIPHERAL_LINK_COUNT   0                               /**< Number of peripheral links used by the application. */

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE    GATT_MTU_SIZE_DEFAULT           /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 2                               /**< Size of timer operation queues. */

#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. (160)*/
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. (80)*/
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(20, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(160, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                               /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

#define DEBUG										1

static ble_wpt_ptu_t            m_ble_wpt_ptu;                  /**< Instance of WPT PTU. */
					
/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
  {
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
  };
	
/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1, 
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};

/**
 * @brief uuid
 */
static const ble_uuid_t m_wpt_uuid =
  {
    .uuid = WPT_SERVICE_UUID,
    .type = BLE_UUID_TYPE_BLE
  };

/* Function Prototype */
static void wpt_pru_registration_start(void);				
	
/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
		NRF_LOG_ERROR("WPT PTU Error: %s: %d\n\r"); 
		app_error_handler(0xDEADBEEF, line_num, p_file_name);
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

/**@brief Reads an advertising report and checks if a uuid is present in the service list.
 *
 * @details The function is able to search for a 16-bit service UUID of the within an 
 * 					advertisement report's service data 
 *
 * @param[in]   p_target_uuid The uuid to search for
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval      true if the UUID is present in the advertisement report. Otherwise false
 */
static bool is_uuid_present(const ble_uuid_t *p_target_uuid,
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
		uint8_t *p_data = (uint8_t *)p_adv_report->data;
    ble_uuid_t extracted_uuid;
		
		// Extract field type and length from data packet(offset determined by BLE GATT spec)
		uint8_t length			 = p_data[AD_LENGTH_OFFSET];
    uint8_t	field_type   = p_data[AD_TYPE_OFFSET];
	
		// Check length and type of data packet
		if (field_type == BLE_GAP_AD_TYPE_SERVICE_DATA
				&& length >= WPT_SERVICE_DATA_LEN
				) 
		{
				// Extract service UUID from service data packet
				memcpy(&extracted_uuid.uuid, p_data + AD_DATA_OFFSET, UUID16_SIZE);
				return (extracted_uuid.uuid == p_target_uuid->uuid);
		}
    return false;
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
	
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
						
            // If WPT Service UUID is present in advertisement report request to connect to device
						if (is_uuid_present(&m_wpt_uuid, p_adv_report))
            {
								// Send connection request with parameters defined in @ref m_scan_params and @ref m_connection_params
                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
                                              &m_scan_params,
                                              &m_connection_param);

                if (err_code == NRF_SUCCESS)
                {
                    // Extract WPT service handle from advertisement report
										memcpy(&m_ble_wpt_ptu.handles.wpt_service_handle, p_adv_report->data + AD_SERVICE_HANDLE_OFFSET, sizeof(uint16_t));								
										
										// update LEDs to indicate idle (scan is automatically stopped by the connection request)
                    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
                    APP_ERROR_CHECK(err_code);
                    NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x\r\n\n",
                             p_adv_report->peer_addr.addr[0],
                             p_adv_report->peer_addr.addr[1],
                             p_adv_report->peer_addr.addr[2],
                             p_adv_report->peer_addr.addr[3],
                             p_adv_report->peer_addr.addr[4],
                             p_adv_report->peer_addr.addr[5]
                             );
                }
            }
        }break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected to target\r\n");
						// Set LEDs to indicate sucessful connection
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
						// Update Connection Handle 
						m_ble_wpt_ptu.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						nrf_gpio_pin_toggle(LED_2);
						
						// Begin PRU registration
						wpt_pru_registration_start();
						
            break; // BLE_GAP_EVT_CONNECTED
        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                NRF_LOG_INFO("Scan timed out.\r\n");
                wpt_ptu_scan_start(&m_scan_params);
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.\r\n");
            }
            break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_INFO("GATT Client Timeout.\r\n");
            NRF_LOG_PROCESS();
						err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_INFO("GATT Server Timeout.\r\n");
						NRF_LOG_PROCESS();
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
		on_ble_evt(p_ble_evt);															// Dispatch for handling of general BLE Stack events
    bsp_btn_ble_on_ble_evt(p_ble_evt);									// Dispatch to BSP for handling of LEDs
    ble_wpt_ptu_on_ble_evt(&m_ble_wpt_ptu,p_ble_evt);		// Dispatch event for handling by active PTU
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
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
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
            err_code = sd_ble_gap_disconnect(m_ble_wpt_ptu.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}



/**@brief Function for initializing buttons and leds.
 */
static void buttons_leds_init(void)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief     Function for disconnecting from current peer and restarting scanning
 *						for new devices
 */
static void wpt_ptu_disconnect()
{
		uint32_t err_code = sd_ble_gap_disconnect(m_ble_wpt_ptu.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

    if (err_code != NRF_ERROR_INVALID_STATE)
    {
				APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing registration process of new PRU
 *
 * @details This function obtains the required characteristic handles to begin
 *				  the registration process. The PTU is set to regitration mode
 * 					which allows the main loop to enter the registration process. 
 *					All registration progress is reset.
 *
 */
static void wpt_pru_registration_start()
{
		// Assign characteristic handles of the WPT service
		ble_wpt_ptu_handles_assign(&m_ble_wpt_ptu);
	
		// Set PTU to Registration Mode
		m_ble_wpt_ptu.mode = WPT_PTU_MODE_REGISTRATION;
		
		// Reset Progress
		m_ble_wpt_ptu.reg_status.pru_static_complete 				= false;
		m_ble_wpt_ptu.reg_status.ptu_static_complete 				= false;
		m_ble_wpt_ptu.reg_status.pru_dynamic_complete 			= false;
		m_ble_wpt_ptu.reg_status.pru_control_complete 			= false;
}


/**@brief Function for reading PRU Static Parameters Characteristic
 *
 * @details This function sends a read request for the
 *					PRU Static Parameters Characteristic
 */
static void wpt_read_pru_static_params() 
{
		uint32_t err_code; 
			
		// Send read request
		do 	 
		{
				err_code = sd_ble_gattc_read(m_ble_wpt_ptu.conn_handle, m_ble_wpt_ptu.handles.pru_static_handle, 0);
		} while (err_code == NRF_ERROR_BUSY);		//Request may fail if device another read request is pending
		
		APP_ERROR_CHECK(err_code);
		if (err_code == NRF_SUCCESS)
				NRF_LOG_INFO("PRU Static Parameters requested\n\r");
}

/**@brief Function for writing to PTU Static Parameters Characteristic
 *
 * @details This function writes the PTU's static parameters to the 
 *					PTU Static Parameters Characteristic.
 */
static void wpt_write_ptu_static_params() 
{
		uint32_t err_code; 	
		
		// Write static parameters
		do 
		{
				err_code = ble_wpt_send_ptu_static_params(&m_ble_wpt_ptu);
		} 	while (err_code == NRF_ERROR_BUSY); //Request may fail if another read request is pending
		
		APP_ERROR_CHECK(err_code);	
		if (err_code == NRF_SUCCESS)
		{
				NRF_LOG_INFO("PTU Static Parameters written\n");
				// Indicate Progress
				m_ble_wpt_ptu.reg_status.ptu_static_complete = true;
		}
}

/**@brief Function for reading PRU Dynamic Parameters Characteristic
 *
 * @details This function sends a read request for the
 *					PRU Dynamic Parameters Characteristic
 */
static void wpt_read_pru_dynamic_params() 
{
		uint32_t err_code; 	
		
		// Send read request
		do
		{
				err_code = sd_ble_gattc_read(m_ble_wpt_ptu.conn_handle, m_ble_wpt_ptu.handles.pru_dynamic_handle, 0);
			
		} while (err_code == NRF_ERROR_BUSY); //Request may fail if another read request is pending
		APP_ERROR_CHECK(err_code);
		if (err_code == NRF_SUCCESS)
			NRF_LOG_INFO("PTU Dynamic Parameters request\n");
}

/**@brief Function for reading PRU Dynamic Parameters Characteristic
 *
 * @details This function sends initial command during registration to PRU by writing to
 *					PRU Control Characteristic. PRU output will not be enabled until after
 *					registration is complete.
 */
static void wpt_pru_control_init() 
{
		uint32_t err_code; 	
		const wpt_pru_control_t pru_control = 
		{	
				.enables 			 	= WPT_PRU_OUTPUT_DISABLE,
				.permission 		= WPT_PTU_POWER, 
				.time_set				= WPT_PTU_NO_TIME_SET
		};
	
		do
		{
				err_code = ble_wpt_send_pru_control(&m_ble_wpt_ptu, pru_control);
		} while (err_code == NRF_ERROR_BUSY); //Request may fail if device another read request is pending
			
		APP_ERROR_CHECK(err_code);
		
		// Indicate completetion 
		if (err_code == NRF_SUCCESS)
		{
				m_ble_wpt_ptu.reg_status.pru_control_complete = true;
				NRF_LOG_INFO("PRU Control Characteristic written\n\r");
		}
}

/**@brief Function for enabling WPT charging
 *
 * @details This function enables PRU notifications which allows PRU Alerts to be
 *					transmitted. Then the PRU output is enabled by writing to the 
 *					PRU Control Characteristic.
 */
static void wpt_ptu_charging_enable()
{
		uint32_t err_code;
		
		// Write to PRU Alert CCCD to enable PRU notifications
		do 
		{
				err_code = ble_wpt_pru_notif_enable(&m_ble_wpt_ptu);
		} while (err_code == NRF_ERROR_BUSY); //Request may fail if device another read request is pending
			APP_ERROR_CHECK(err_code);
	
		const wpt_pru_control_t pru_control = 
		{
				.enables 			 	= WPT_PRU_OUTPUT_ENABLE,
				.permission 	  = WPT_PTU_POWER, 
				.time_set				= WPT_PTU_NO_TIME_SET
		};

		// Write to the PRU Control Characteristic to enable charging
		do 
		{
				err_code = ble_wpt_send_pru_control(&m_ble_wpt_ptu, pru_control);
		} while (err_code == NRF_ERROR_BUSY);
			APP_ERROR_CHECK(err_code);
		
		// If command sent successfully set PTU to charging mode
		if (err_code == NRF_SUCCESS)
				m_ble_wpt_ptu.mode = WPT_PTU_MODE_CHARGING;
}
/**@brief Function for performing the WPT Charging procedure
 *
 * @details This function reads the PRU dynamic parameter characteristic
 *					and clears command buffer of waiting commands.
 */
static void wpt_ptu_charge_continue()
{
		uint32_t err_code;
		// Send read request for PRU dynamic parameters
		wpt_read_pru_dynamic_params();
	
		// Send command if one is waiting
		if (m_ble_wpt_ptu.cmd_waiting)
		{	
				do 
				{
						ble_wpt_send_pru_control(&m_ble_wpt_ptu, m_ble_wpt_ptu.wpt_service.pru_control);
				} while (err_code == NRF_ERROR_BUSY);
		}
}

/**@brief Function to perform PRU registration process
 *
 * @details This function is called from the main loop as a result of
 *					the PTU entering Registration Mode. Registration mode is 
 *					set by wpt_pru_registration_start(). Once the PTU and PRU
 * 					have exchanged parameters if the connection is accepted
 * 					charging is enabled.
 * 					
 * @note		This function uses the @ref WAIT_FOR_COMPLETION() macro to wait 
						for the completion status of the reads and writes. This macro
 * 					is meant to avoid infinite loops if no resposne is received
 */
static void wpt_pru_registration()
{
		// Read PRU Static Parameters Characteristic
		if (!m_ble_wpt_ptu.reg_status.pru_static_complete)
		{
				wpt_read_pru_static_params();
				WAIT_FOR_COMPLETION(m_ble_wpt_ptu.reg_status.pru_static_complete); 
		}
		
		// Write to PTU Static Parameters Characteristic
		if (!m_ble_wpt_ptu.reg_status.ptu_static_complete) 
		{
				wpt_write_ptu_static_params();
				WAIT_FOR_COMPLETION(m_ble_wpt_ptu.reg_status.ptu_static_complete); 
		}
		
		// Read PRU Dynamic Parameters Characteristic
		if (!m_ble_wpt_ptu.reg_status.pru_dynamic_complete)
		{
				wpt_read_pru_dynamic_params();
				WAIT_FOR_COMPLETION(m_ble_wpt_ptu.reg_status.pru_dynamic_complete); 
		}
		
		// Write PRU Control Characteristic (initial control value does not enable charge output)
		if (!m_ble_wpt_ptu.reg_status.pru_control_complete)
		{		
				wpt_pru_control_init();
				WAIT_FOR_COMPLETION(m_ble_wpt_ptu.reg_status.pru_control_complete);
		}
		
		// If registration complete and connection accepted enable charging
		if (wpt_ptu_connection_accepted(&m_ble_wpt_ptu))
				wpt_ptu_charging_enable();
		else
				wpt_ptu_disconnect();
}


int main(void)
{
		uint32_t err_code;
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    // Initialize NRF Logger Module
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    buttons_leds_init();							// Initialize Buttons and LEDs
    ble_stack_init();		 							// Initialize BLE Stack

    // Start scanning for peripherals and initiate connection with devices that advertise NUS UUID.
    wpt_ptu_scan_start(&m_scan_params);
		NRF_LOG_INFO("WPT PTU Scan started\r\n");
	
		memset(&m_ble_wpt_ptu, 0, sizeof(m_ble_wpt_ptu));
		m_ble_wpt_ptu.scan_params = m_scan_params;

    // Main Loop
		for (;;)
    {
				if (m_ble_wpt_ptu.mode == WPT_PTU_MODE_CHARGING) 
				{
						// Continue performing charging procedure
						wpt_ptu_charge_continue();
				}
				else if (m_ble_wpt_ptu.mode == WPT_PTU_MODE_REGISTRATION)
				{		
						// Continue Registration Process
						wpt_pru_registration();
				}
				
				NRF_LOG_PROCESS();				
				power_manage();
    }
}
