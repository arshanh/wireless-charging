#ifndef WPT_PRU_H
#define WPT_PRU_H

#include "wpt_service.h"

/* I2C Addresses */
#define WPT_SRAM_SLAVE_ADDR												(0x90U >> 1)	/**> I2C Slave address. */
#define WPT_SRAM_DYN_MEM_ADDR											0xEFBE				/**> SRAM Memory address of dynamic parameters to be read. */
#define WPT_SRAM_CMD_MEM_ADDR											0xADDE				/**> SRAM Memory address for commands to be written */

/* Length (in octets) of  WPT Characteristic Data Fields*/
#define WPT_PRU_CTRL_DATA_LEN												5
#define WPT_PRU_STATIC_DATA_LEN											20
#define WPT_PTU_STATIC_DATA_LEN											17
#define WPT_PRU_DYNAMIC_DATA_LEN										20

/* PRU Alert Bit Masks for both PRU Dynamic Parameters and PRU Alert Char.*/
#define WPT_PRU_ALERT_NONE												(0x00)				/**< No Alert */
#define WPT_PRU_ALERT_OVER_VOLTAGE								(1<<7)				/**< Over Voltage */
#define WPT_PRU_ALERT_OVER_CURRENT								(1<<6)				/**< Over Current */
#define WPT_PRU_ALERT_OVER_TEMP										(1<<5)				/**< Over Temperature */
#define WPT_PRU_ALERT_SELF_PROTECTION							(1<<4)				/**< Indication of PRU Self-Protection */
#define WPT_PRU_ALERT_CHARGE_COMPLETE							(1<<3)				/**< Charge Complete */
#define WPT_PRU_ALERT_WIRED_DETECT								(1<<2)				/**< PRU Wired Charger Detected */
	// Only Present in PRU Alert field of PRU Dynamic Char.  
#define WPT_PRU_ALERT_CHARGE_PORT									(1<<1)				/**< PRU Charge Port */
#define WPT_PRU_ALERT_APR													(1<<0)				/**< Adjust Power Response */
	// Only Present in PRU Alert Char.
#define WPT_PRU_ALERT_MODE_TRANS_1								(1<<1)				/**< Indicates pending Mode Transition  */
#define WPT_PRU_ALERT_MODE_TRANS_0								(1<<0)				/**< Indicates pending Mode Transition  */

// Only 
/* PRU Control Commands (written by PTU) */
	// Enables
#define WPT_PRU_OUTPUT_DISABLE										(0x00)
#define WPT_PRU_OUTPUT_ENABLE											(1<<7)
#define WPT_PRU_INDICATOR_ENABLE									(1<<6)
#define WPT_PRU_POWER_MAX													(0<<4)
#define WPT_PRU_POWER_66_PERCENT_OF_MAX						(1<<4)
#define WPT_PRU_POWER_33_PERCENT_OF_MAX						(2<<4)
#define WPT_PRU_POWER_2_5_WATTS										(3<<4)
	// Permission
#define WPT_PRU_PERMITTED														0x00
#define WPT_PRU_PERMITTED_WITH_WAIT									0x01
#define WPT_PRU_DENIED_CROSS_CONN										0x80
#define WPT_PRU_DENIED_LIM_AVAIL_POWER							0x81
#define WPT_PRU_DENIED_NUM_OF_DEVICES								0x82
#define WPT_PRU_DENIED_LIM_CLASS_SUP								0x83
#define WPT_PRU_DENIED_HIGH_TEMP										0x84
	//	Time Set
#define WPT_PTU_NO_TIME_SET													0x00
#define WPT_PTU_TIME_SET_10_MS											0x01	
#define WPT_PTU_TIME_SET_20_MS											0x02	
#define WPT_PTU_TIME_SET_30_MS											0x03	
#define WPT_PTU_TIME_SET_40_MS											0x04	
#define WPT_PTU_TIME_SET_50_MS											0x05	
#define WPT_PTU_TIME_SET_60_MS											0x06	
#define WPT_PTU_TIME_SET_70_MS											0x07	
#define WPT_PTU_TIME_SET_80_MS											0x08	

/* PRU Static Optional Fields */
#define WPT_PRU_STATIC_DELTA_R1											(1<<7)

/* PRU Dynamic Optional Fields */
#define WPT_PRU_DYNAMIC_VOUT												(1<<7)
#define WPT_PRU_DYNAMIC_IOUT												(1<<6)
#define WPT_PRU_DYNAMIC_TEMP												(1<<5)
#define WPT_PRU_DYNAMIC_VRECT_MIN										(1<<4)
#define WPT_PRU_DYNAMIC_VRECT_SET										(1<<3)
#define WPT_PRU_DYNAMIC_VRECT_HIGH									(1<<2)

/* Maximum dynamic parameter values */
#define WPT_VOLTAGE_MAX				10
#define WPT_CURRENT_MAX				10
#define WPT_TEMP_MAX						10


typedef struct 
{
		uint8_t	data_valid;										// Lock which is set if SRAM is mid-write
		uint8_t optional_fields;
		uint16_t V_rect;
		uint16_t I_rect;
		uint16_t V_out;
		uint16_t I_out;
		uint8_t temperature;
		uint16_t V_rect_min;
		uint16_t V_rect_set;
		uint16_t V_rect_high;
		uint8_t pru_alert;
		uint8_t tester_command;
} wpt_dynamic_params_t;

/** 
 * 	@brief PRU static parameters contains information on the constant values of the PRU
 */
typedef struct {
		uint8_t 							optional_fields;			/**< Optional Fields Validity */
		uint8_t 							protocol_rev;					/**< A4WP Supported Revision */
		uint8_t 							pru_category;					/**< PRU Category */
		uint8_t 							pru_info;							/**< PRU Information. */
		uint8_t								hardware_rev;					/**< Hardware Revision */
		uint8_t 							firmware_rev;					/**< Firmware Revision */
		uint8_t 							p_rect_max;						/**< Maximum rated P_rect power */
		uint16_t 							v_rect_min_static;		/**< Minimum V_rect voltage */
		uint16_t 							v_rect_high_static;		/**< Maximum V_rect voltage */
		uint16_t 							v_rect_set;						/**< PRU desired V_rect voltage */
		uint16_t 							delta_r1;							/**< PRU Delta R1 */
} wpt_pru_static_params_t;

/** 
 * 	@brief PRU control contains fields used to send commands to the PRU
 */
typedef struct
{
		uint8_t 							enables;					/**< Enables field: PTU instruction for power control. */
		uint8_t								permission;				/**< Permision: Power availability reason codes. */
		uint8_t 							time_set;					/**< Time set: time for PRU to create vaild load variation */
} wpt_pru_control_t;


/**@brief Function for initializing the WPT service
 */
void wpt_service_init(ble_wpt_service_t * p_wpt_service);

/**@brief Function for sending PRU alert if they are enabled by PTU
 *
 * @param[in] m_conn_handle	 	Connection handle which needs to be alerted
 * @param[in] wpt_pru_alert 	Pointer to single byte buffer holding alert data (defined in wpt_ptu.h
*/
void wpt_send_pru_alert(uint16_t m_conn_handle, uint8_t *wpt_pru_alert);

#endif
