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
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_twi.h"
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
#include "fstorage.h"
#include "fds.h"
#include "peer_manager.h"

#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_gpio.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"

#include "wpt_service.h"
#include "wpt_pru.h"

#include "nrf_delay.h"

#define NRF_LOG_MODULE_NAME "APP"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT 1     /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define TWI_INSTANCE_ID     0

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT                       /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "WPT PRU"                           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "NordicSemiconductor"                       /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                300                                         /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout in units of seconds. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.1 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(160, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (0.2 second). */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                            /**< Handle of the current connection. */

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for dynamic params. */
static wpt_dynamic_params_t m_wpt_dynamic_params;

// Declare all services structure your application is using
static ble_wpt_service_t											m_wpt_service;

// Declare PRU alert
static uint8_t wpt_pru_alert = WPT_PRU_ALERT_NONE;

static wpt_pru_control_t m_pru_command = {0};

// Declare PRU control data buffer
static uint8_t m_wpt_pru_control_buf[WPT_PRU_CTRL_DATA_LEN] = {0};

static void advertising_start(void);

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


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    // Peer Manager events ignored
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

    /* 
       uint32_t err_code;
       err_code = app_timer_create(&m_app_timer_id, APP_TIMER_MODE_REPEATED, timer_timeout_handler);
       APP_ERROR_CHECK(err_code); */
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

		err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_UNKNOWN);
    APP_ERROR_CHECK(err_code);																	

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
		wpt_service_init(&m_wpt_service);  
	
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
 *				Reserved for eventual use of hardware timers instead of software delays
 */
static void application_timers_start(void)
{
    /* uint32_t err_code;
       err_code = app_timer_start(m_app_timer_id, TIMER_INTERVAL, NULL);
       APP_ERROR_CHECK(err_code); */
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
            NRF_LOG_INFO("Fast advertising\r\n");
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
    uint32_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
						m_conn_handle = BLE_CONN_HANDLE_INVALID;
						m_wpt_service.conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.\r\n");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						m_wpt_service.conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

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
    /** The Connection state module has to be fed BLE events in order to function correctly
     * Remember to call ble_conn_state_on_ble_evt before calling any ble_conns_state_* functions. */
    ble_conn_state_on_ble_evt(p_ble_evt);
    pm_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);

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
    // Dispatch the system event to the fstorage module, where it will be
    // dispatched to the Flash Data Storage (FDS) module.
    fs_sys_event_handler(sys_evt);

    // Dispatch to the Advertising module last, since it will check if there are any
    // pending flash operations in fstorage. Let fstorage process system events first,
    // so that it can report correctly to the Advertising module.
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

		// Allocate 5 base uuids for each characteristic 
		ble_enable_params.common_enable_params.vs_uuid_count   = 5;
    
		// Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
		ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
		err_code = softdevice_enable(&ble_enable_params);
				
		APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the Peer Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Peer Manager.
 */
static void peer_manager_init(bool erase_bonds)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    if (erase_bonds)
    {
        err_code = pm_peers_delete();
        APP_ERROR_CHECK(err_code);
    }

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
static void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist();
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
	
		uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;
	
    uint8_t data[4]; // Our data to adverise
		
		ble_advdata_service_data_t			service_data;
		service_data.service_uuid 					= WPT_SERVICE_UUID;
	
		uint16_t wpt_serv_handle						= WPT_PRIMARY_SERVICE_HANDLE;
		memcpy(data, &wpt_serv_handle, sizeof(wpt_serv_handle));
	
		uint8_t wpt_rssi 										= (WPT_RSSI_PRU_PWR_UNKNOWN | WPT_RSSI_PRU_GAIN_UNKNOWN);
		memcpy(data+sizeof(wpt_serv_handle), &wpt_rssi, sizeof(wpt_rssi));
		
		uint8_t wpt_adv_data									 =  WPT_IMP_SHIFT_CAT3
																						| WPT_REBOOT				
																						| WPT_OVP_NOT_USED						
																						| WPT_TIME_SET_NO_SUPPORT;	
	
		memcpy(data+sizeof(wpt_serv_handle)+sizeof(wpt_rssi), &wpt_adv_data, sizeof(wpt_adv_data));
	
		service_data.data.p_data = data;
		service_data.data.size = sizeof(data);

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
		advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    //advdata.flags                   = BLE_GAP_ADV_FLAG_LE_LIMITED_DISC_MODE;
		
		advdata.p_service_data_array		= &service_data;
		advdata.service_data_count 			= 1;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
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


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
}

__STATIC_INLINE void pru_alert_handler(wpt_dynamic_params_t *wdp, ble_wpt_service_t * p_service) {	
	
		// Check for alerts
	
		if (wdp->V_rect > WPT_VOLTAGE_MAX)
			wpt_pru_alert |= WPT_PRU_ALERT_OVER_VOLTAGE;
		else
			wpt_pru_alert &= ~(WPT_PRU_ALERT_OVER_VOLTAGE);			// Clear Alert
	
		if (wdp->V_rect > WPT_CURRENT_MAX)
			wpt_pru_alert |= WPT_PRU_ALERT_OVER_CURRENT;
		else
			wpt_pru_alert &= ~(WPT_PRU_ALERT_OVER_CURRENT);			// Clear Alert
		
		if (wdp->V_rect > WPT_TEMP_MAX)
			wpt_pru_alert |= WPT_PRU_ALERT_OVER_TEMP;
		else
			wpt_pru_alert &= ~(WPT_PRU_ALERT_OVER_TEMP);				// Clear Alert
	
		// Update alert field of PRU dynamic parameters
		wdp->pru_alert = wpt_pru_alert;
		
	/*NRF_LOG_INFO("Struct size: %d\r\n", sizeof(*wdp));
		NRF_LOG_INFO("Connection handle: %d\r\n", m_conn_handle);
		NRF_LOG_INFO("Data validity: %0X\r\n", wdp->data_valid);
		NRF_LOG_INFO("Optional Fields: %0X\r\n", wdp->optional_fields);
		NRF_LOG_INFO("Vrect =  %d\r\n", wdp->V_rect);
		NRF_LOG_INFO("Irect =  %d\r\n", wdp->I_rect);
		NRF_LOG_INFO("Vout =  %d\r\n", wdp->V_out);
		NRF_LOG_INFO("Iout =  %d\r\n", wdp->I_out);
		NRF_LOG_INFO("Temperature =  %d\r\n", wdp->temperature);
		NRF_LOG_INFO("Vrect min =  %d\r\n", wdp->V_rect_min);
		NRF_LOG_INFO("Vrect set =  %d\r\n", wdp->V_rect_set);
		NRF_LOG_INFO("Vrect high=  %d\r\n", wdp->V_rect_high);
		NRF_LOG_INFO("PRU alert =  %0X\r\n", wdp->pru_alert);
		NRF_LOG_INFO("Tester Command =  %0X\r\n", wdp->tester_command);
		
		NRF_LOG_INFO("\r\n");	*/
}


/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                nrf_gpio_pin_toggle(LED_4);
								pru_alert_handler(&m_wpt_dynamic_params, &m_wpt_service);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief twi initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = ARDUINO_SCL_PIN,
       .sda                = ARDUINO_SDA_PIN,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}

static void wpt_read_pru_dynamic_params()
{
		ret_code_t err_code;
	    // Write SRAM memory address to be read 
    uint16_t sram_mem_addr = WPT_SRAM_DYN_MEM_ADDR	;
    err_code = nrf_drv_twi_tx(&m_twi, WPT_SRAM_SLAVE_ADDR, (uint8_t*)&sram_mem_addr, sizeof(sram_mem_addr), false);
		APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);	

		m_xfer_done = false;
		nrf_delay_ms(1);
	
    // Read from the specified address 
    err_code = nrf_drv_twi_rx(&m_twi, WPT_SRAM_SLAVE_ADDR, (uint8_t*)&m_wpt_dynamic_params, sizeof(m_wpt_dynamic_params));
		APP_ERROR_CHECK(err_code);
		while (m_xfer_done == false);	
}

static void wpt_twi_send_command()
{
		nrf_delay_ms(3);
		ret_code_t err_code;
	    // Write SRAM memory address to be read
		uint16_t sram_mem_addr = WPT_SRAM_CMD_MEM_ADDR;
		err_code = nrf_drv_twi_tx(&m_twi, WPT_SRAM_SLAVE_ADDR, (uint8_t*)&sram_mem_addr, sizeof(sram_mem_addr), false);
		for (int i = 0; i < 10000; i++)
			if (m_xfer_done)
				break;
			
		m_xfer_done = false;
		nrf_delay_ms(1);
	
    // Write to the specified address 
    err_code = nrf_drv_twi_tx(&m_twi, WPT_SRAM_SLAVE_ADDR, (uint8_t*)&m_pru_command, sizeof(m_pru_command), false);
		APP_ERROR_CHECK(err_code);
		for (int i = 0; i < 10000; i++)
			if (m_xfer_done)
				break;	
			
	
}

static void wpt_update_pru_dynamic_params(ble_gatts_value_t *newData) 
{
		uint32_t err_code;
	
		if (m_wpt_dynamic_params.data_valid) {
				nrf_gpio_pin_toggle(LED_2);
				err_code = sd_ble_gatts_value_set(m_conn_handle, WPT_PRU_DYNAMIC_PARAM_HANDLE, newData);
		}
		
		APP_ERROR_CHECK(err_code);
}

static void wpt_pru_control_init() 
{
		ble_gatts_value_t value;
		memset(&value, 0, sizeof(value));
		value.len = 5;
		value.p_value = m_wpt_pru_control_buf;
		value.offset = 0;
		
		sd_ble_gatts_value_set(m_conn_handle, WPT_PRU_CONTROL_HANDLE, &value);	
}

static void wpt_update_pru_control()
{
		uint32_t err_code;
		ble_gatts_value_t pru_gatts_value;
		memset(&pru_gatts_value, 0, sizeof(pru_gatts_value));
		uint8_t buf[5] = {0};
		pru_gatts_value.p_value = buf;
		pru_gatts_value.len = WPT_PRU_CTRL_DATA_LEN;
		
		// Get current value of PRU Control Characteristic
		err_code = sd_ble_gatts_value_get(BLE_CONN_HANDLE_INVALID, WPT_PRU_CONTROL_HANDLE, &pru_gatts_value);
		APP_ERROR_CHECK(err_code);
		
		// if control characteristic has changed send new command to chip
		if (memcmp(m_wpt_pru_control_buf, pru_gatts_value.p_value, 3) != 0) {
				// First three bytes of buffer hold control values
				m_pru_command.enables = pru_gatts_value.p_value[0];
				m_pru_command.permission = pru_gatts_value.p_value[1];
				m_pru_command.time_set = pru_gatts_value.p_value[2];
			
				// Update static global control buffer for next comparison	
				memcpy(m_wpt_pru_control_buf, pru_gatts_value.p_value, pru_gatts_value.len);

				wpt_twi_send_command();
				nrf_gpio_pin_toggle(LED_3);
		}
		
}


/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
    bool     erase_bonds;

    // Initialize.
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    // Initialize timers for eventual replacement of software delays
		timers_init();
		// LEDS used for Debugging
    buttons_leds_init(&erase_bonds);
    
		// Configure SoftDevice and event handlers
		ble_stack_init();
	
		// Initialize peer manager to handle encryption and authorization if requested (no security by default)
    peer_manager_init(erase_bonds);

		// Initialize GAP params (device name, appearance, and connection parameters)
		gap_params_init();
		
		// Initialize WPT Service
		services_init();
		
		// Initialize and contruct advertising payload per A4WP Spec
		advertising_init();
		
		//Initialize connection params module
    conn_params_init();
		
		// Initialize TWI (I2C) driver instance
		twi_init();

    NRF_LOG_INFO("WPT PRU started\r\n");
		application_timers_start();

		// Being advertising
		advertising_start();
		
		// Clear PRU control characteristic existing values
		wpt_pru_control_init();
		
		// Declare and Initialize local data buffer for updating Dynamic Parameters Characteristic
		ble_gatts_value_t wpt_dynamic_gatts_value;
		memset(&wpt_dynamic_gatts_value, 0, sizeof(wpt_dynamic_gatts_value));
		wpt_dynamic_gatts_value.offset = 0;
		wpt_dynamic_gatts_value.len = sizeof(wpt_dynamic_params_t);			
		// Set this buffer's data pointer to point to static global buffer m_wpt_dynamic_params
		wpt_dynamic_gatts_value.p_value = (uint8_t*)&m_wpt_dynamic_params; 
		
    // Enter main loop.
    for (;;)
    {
				nrf_delay_ms(500);
				
				// Read pru dynamic params from SRAM using I2C (updates static global buffer m_wpt_dynamic_params)
				wpt_read_pru_dynamic_params();
				
				// Check alert flags (set by pru_alert_handler) and send alert if a flag is set 
				wpt_send_pru_alert(m_conn_handle, &wpt_pru_alert);
			
				// Update pru dynamic params characteristic to reflect most recent reading
				wpt_update_pru_dynamic_params(&wpt_dynamic_gatts_value);
			
				// Check if pru control characteristic has been written to. If so write command to SRAM using I2C
				wpt_update_pru_control();
			
        if (NRF_LOG_PROCESS() == false)
        {
            power_manage();
        }
    }
}


/**
 * @}
 */
