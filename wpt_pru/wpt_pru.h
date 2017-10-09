#ifndef WPT_PRU_H
#define WPT_PRU_H

#define WPT_SRAM_SLAVE_ADDR												(0x90U >> 1)
#define WPT_SRAM_DYN_MEM_ADDR											0xEFBE				// Memory location of dynamic parameters in SRAM
#define WPT_SRAM_CMD_MEM_ADDR											0xADDE				// Memory location of command buffer in SRAM

#define WPT_PRU_CTRL_DATA_LEN											5


/* PRU Alerts */
#define WPT_PRU_ALERT_NONE												(0x00)
#define WPT_PRU_ALERT_OVER_VOLTAGE								(1<<7)
#define WPT_PRU_ALERT_OVER_CURRENT								(1<<6)
#define WPT_PRU_ALERT_OVER_TEMP										(1<<5)
#define WPT_PRU_ALERT_SELF_PROTECTION							(1<<4)
#define WPT_PRU_ALERT_CHARGE_COMPLETE							(1<<3)
#define WPT_PRU_ALERT_WIRED_DETECT								(1<<2)
#define WPT_PRU_ALERT_CHARGE_PORT									(1<<1)
#define WPT_PRU_ALERT_APR													(1<<0)

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

typedef struct
{
		uint8_t enables;
		uint8_t permission;
		uint8_t time_set;
		// uint16_t RFU; // Reserved for Future Use
} wpt_pru_control_t;


#endif
