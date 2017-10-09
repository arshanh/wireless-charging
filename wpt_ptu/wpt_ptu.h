/**@file		 wpt_ptu.h
 *
 * @brief    WPT PTU Module
 *
 * @details  
 *
 * @note     
 *
 */


#ifndef WPT_PTU_H
#define WPT_PTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wpt_pru.h"
	
	/* PTU Static Parameters (Currently Placeholder Values)  */
#define WPT_PTU_OPTIONAL_FIELDS					0xFF			/**< Optional Fields Validity */
#define WPT_PTU_POWER										0xFF			/**< PTU Power */
#define WPT_PTU_MAX_SRC_IMP							0xFF			/**< PTU Maximum Source Impedance */
#define WPT_PTU_MAX_LD_RES							0xFF			/**< PTU Maximum Load Resistance */
#define WPT_PTU_CLASS										0xFF			/**< PTU Class */
#define WPT_PTU_HARDWARE_REV						0xFF			/**< Hardware Revision */
#define WPT_PTU_FIRMWARE_REV						0xFF			/**< Firmware Revision */
#define WPT_PTU_PROTOCOL_REV						0x00			/**< A4WP Supported Revision */
#define WPT_PTU_DEVICES_SUPPORTED				0x01			/**< PTU Max Number of Devices Supported */
#define WPT_PTU_RFU											0x00			/**< Reserved for future use (Don't Care) */

/* Length of 16-bit Bluetooth SIG assigned WPT Service UUID (in octets) */
#define WPT_UUID_LENGTH 								2

/* Size and offsets of PRU Advertising Data*/
#define AD_FLAGS_STRUCT_SIZE						3
#define AD_LENGTH_OFFSET								AD_FLAGS_STRUCT_SIZE
#define AD_TYPE_OFFSET									AD_LENGTH_OFFSET + 1
#define AD_DATA_OFFSET									AD_TYPE_OFFSET + 1
#define AD_SERVICE_HANDLE_OFFSET				AD_DATA_OFFSET + 2
	
/* WPT Handle offsets as provided by A4WP specification  */	
#define WPT_PRU_CONTROL_HANDLE_OFFSET		2
#define WPT_PTU_STATIC_HANDLE_OFFSET		4
#define WPT_PRU_ALERT_HANDLE_OFFSET			6
#define WPT_PRU_CCCD_HANDLE_OFFSET			7
#define WPT_PRU_STATIC_HANDLE_OFFSET		9
#define WPT_PRU_DYNAMIC_HANDLE_OFFSET		11

/* BLE GAP Service Data Type identifier */
#define GAP_SERVICE_DATA_TYPE_UUID_16		0x16

/* PTU Modes of Operation:  */
#define WPT_PTU_MODE_NO_CONN						0							/**<   Initial state no PRU connection */
#define WPT_PTU_MODE_REGISTRATION				1							/**<  PRU Registration underway */
#define WPT_PTU_MODE_CHARGING						2							/**<  Registration Complete/Charging enabled */

/**@brief Macro to wait for the completion status of the input parameter. Uses
 *				a for loop to avoid infinte loop in event of failure.
 * @details This loop is used to track completion status of the reads
 *					and writes required for PRU registration. In the event that
 *					a response is lost and the an an event never occurs the for
 *					loop will terminate and the registration process will be
 *					restarted in the for loop. 
 *
 * @param[in] PARAM boolean parameter to be polled 
 */
#define WAIT_FOR_COMPLETION(PARAM)                                     \
    do                                                                 \
    {                                                                  \
				for (int i=0; i < 1000000; i++)																 \
						if (PARAM) 																								 \
								break;																								 \
				if (!PARAM)																										 \
						return;																										 \
		} while (0)
		
/**
 *@brief Handles for each of the characteristics of the WPT service
 */
typedef struct 
{
		uint16_t                wpt_service_handle;      /**< Handle of the WPT Service as provided by advertising service data. */
    uint16_t                pru_control_handle;      /**< Handle of the PRU control characteristic. */
    uint16_t                ptu_static_handle;			 /**< Handle of the PTU static parameters characteristic. */
    uint16_t                pru_alert_handle;      	 /**< Handle of the PRU alert characteristic. */
    uint16_t               	pru_cccd_handle;      	 /**< Handle of the CCCD for the PRU alert characteristic. */
    uint16_t                pru_static_handle;       /**< Handle of the PRU static parameters characteristic. */
		uint16_t                pru_dynamic_handle;      /**< Handle of the PRU dynamic parameters characteristic. */
} ble_wpt_ptu_handles_t;

/**
 *@brief PTU static parameters contains information on the constant values of the PTU
 */
typedef struct 
{
		uint8_t 							optional_fields;					/**< Optional Fields Validity */
		uint8_t 							ptu_power;								/**< PTU Power */
		uint8_t 							ptu_max_src_imp;					/**< PTU Maximum Source Impedance */
		uint8_t 							ptu_max_ld_res;						/**< PTU Maximum Load Resistance */
		uint8_t								ptu_class;								/**< PTU Class */
		uint8_t 							hardware_rev;							/**< Hardware Revision */
		uint8_t 							firmware_rev;							/**< Firmware Revision */
		uint8_t 							protocol_rev;							/**< A4WP Supported Revision */
		uint8_t 							ptu_devices_supported;		/**< PTU Max Number of Devices Supported */
} wpt_ptu_static_params_t;

/** @brief WPT Service struct: holds current values for WPT Service Characteristics
 * 	
 *  @details Acts as buffer holding most recently read values of the PRU control 
 * 					 and static/dynamic parameters. Also holds outgoing commands which 
 * 					 are placed within the pru_control field. 
 */
typedef struct
{
		wpt_pru_control_t 					pru_control;			/**< PRU ON/OFF Control  */
		wpt_ptu_static_params_t			ptu_static;				/**< Static characteristics of PTU */
		uint8_t											pru_alert;				/**< PRU Alert conditions */
		wpt_pru_static_params_t			pru_static;				/**< Static characteristics of PRU */
		wpt_dynamic_params_t				pru_dynamic;			/**< Dynamic characteristics of PRU */
}	wpt_service_t;


/** @brief PTU Status Flags Struct: used to track registration process
 * 	
 *  @details One of the 4 fields is set for each completed step in the Registration Process
 * 					 Once all 4 fields are set PTU is ready to begin charging
 */
typedef struct
{
		bool							pru_static_complete;				/**< PRU Static Parameters Read */
		bool							pru_dynamic_complete;				/**< PRU Dynamic Parameters Read */
		bool							ptu_static_complete;				/**< PTU Static Parameters Written */
		bool							pru_control_complete;				/**< Initial PRU Control Command Written*/
} wpt_reg_status_t;

/**
 * @brief WPT PTU structure. Represents an instance of a PTU connection with a PRU
 */
typedef struct
{
    uint8_t                 	uuid_type;          /**< UUID type. */
    uint16_t                	conn_handle;        /**< Handle of the current connection. */
		ble_gap_scan_params_t 		scan_params;				/**< GAP Scan parameters of PTU. */
		ble_wpt_ptu_handles_t     handles;            /**< Handles on the connected peer device needed to interact with it. Set with @ref ble_wpt_ptu_handles_assign when connected.*/
		uint8_t										mode;								/**< PTU Mode of Operation. */
		wpt_service_t							wpt_service;				/**< Struct holding WPT characteristic values for the current connection. */
		wpt_reg_status_t					reg_status;					/**< Struct for tracking PRU registration process for current connection. */
		uint8_t										cmd_waiting;				/**< Flag to indicate if new command needs to be sent to PRU.*/
} ble_wpt_ptu_t;

/**@brief Function to start scanning.
 *
 * @details Parameters for scanning defined in @ref m_scan_params
 * 
 * @param[in] p_scan_params Pointer to BLE GAP scan parameters struct
 */
void wpt_ptu_scan_start(const ble_gap_scan_params_t  * scan_params);

/**@brief     Function for handling the PTU's BLE stack events sent from the soft device
 *
 * @details   This function handles BLE stack events that are initiated by or relate to
 * 						the current PTU connection.
 *
 * @param[in] p_ble_wpt_ptu Pointer to the WPT PTU instance
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
void ble_wpt_ptu_on_ble_evt(ble_wpt_ptu_t * p_ble_wpt_ptu, const ble_evt_t * p_ble_evt);

/**@brief     Function for assigning 16-bit Characteristic Handles of the WPT Service Characteristics
 *
 * @details   This function uses the Primary Service handle extracted from the Advertising Service Data
 *						as a base value from which the Characteristic handles are calculated. The handle offsets
 *						are determined by the A4WP spec.
 * 					
 * @param[in] p_ble_wpt_ptu Pointer to the WPT PTU instance
 */
void ble_wpt_ptu_handles_assign(ble_wpt_ptu_t * p_ble_wpt_ptu);

/**@brief     Function for handling read responses received from the peer (PRU)
 *
 * @details   This function is called upon a BLE_GATTC_READ_RSP event and determines
 *						what data has been sent by the peer then updates the corresponding
 *						buffer in the PTU instance. Progress is also updated 
 * 						if new device is being registered.
 *
 * @param[in] p_ble_wpt_ptu Pointer to the NUS Client structure.
 * @param[in] read_rsp  		Pointer to the read response recieved.
 */
void wpt_ptu_data_handler(ble_wpt_ptu_t * p_ble_wpt_ptu, const ble_gattc_evt_read_rsp_t * read_rsp);

/**@brief     Function for writing the PTU's Static Parameters to the PRU 
 *
 * @details   This function writes to the PTU static parameters defined as Macros above
 * 						to the PTU Static Parameters Characteristic using the Write Command
 *						soft device function. The PTU instance's corresponding buffer is also
 *						updated to reflect the written static parameters.
 *
 * @param[in] p_ble_wpt_ptu Pointer to the WPT PTU structure.
 * @retval 		see return values of @ref sd_ble_gattc_write()
 */
uint32_t ble_wpt_send_ptu_static_params(ble_wpt_ptu_t * p_ble_wpt_ptu);

/**@brief     Function for writing a command to the PRU 
 *
 * @details   This function writes a command to the PRU by writing to the PTU
 * 						Static Parameters Characteristic using the Write Command soft device
 *						function call. 
 *
 * @param[in] p_ble_wpt_ptu Pointer to the WPT PTU structure.
 * @param[in] pru_control		WPT PTU Control Struct holding control values to be written
 * @retval 		see return values of @ref sd_ble_gattc_write()
 */
uint32_t ble_wpt_send_pru_control(ble_wpt_ptu_t * p_ble_wpt_ptu, const wpt_pru_control_t pru_control);

/**@brief     Function for enabling alert notifications from the PRU
 *
 * @details   This function enables notifications from the PRU Alert Characteristic
 * 						by writing to CCCD of the PRU Alert Characteristic
 *
 * @param[in] p_ble_wpt_ptu Pointer to the WPT PTU structure.
 * @retval 		see return values of @ref cccd_configure()
 */
uint32_t ble_wpt_pru_notif_enable(ble_wpt_ptu_t * p_ble_wpt_ptu);

/**@brief     Function for determining whether the PTU accepts a PRU for Wireless Power Transfer
 *
 * @details   NOT IMPLEMENTED
 *						This function determines if the PTU can initiate Wireless Power Transfer
 * 						with a PRU that attempts to register based on PRU's static and dynamic
 * 						parameter and the state of the PTU itself. 
 *
 * @param[in] p_ble_wpt_ptu Pointer to the WPT PTU structure.
 * @retval 		Currently this function always returns true
 */
bool wpt_ptu_connection_accepted(ble_wpt_ptu_t * p_ble_wpt_ptu);

#ifdef __cplusplus
}
#endif

#endif

/** @} */
