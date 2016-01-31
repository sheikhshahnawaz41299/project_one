#ifndef _LINUX_FTS_I2C_H_
#define _LINUX_FTS_I2C_H_

#include <linux/wakelock.h>

#define	TOUCHSCREEN_NORMAL  0
#define  TOUCHSCREEN_SLEEP   1
#define  TOUCHSCREEN_GESTURE 2

#define FTS_POWER_ON     1
#define FTS_POWER_OFF    0

#define BBK_DCLICK_WAKE
#define DRIVER_NAME	"FTS"
#define FTS_COORDS_ARR_SIZE	4
#define MAX_BUTTONS 4


/*
 * Uncomment to use polling mode insead of interrupt mode.
 *
 * #define FTS_USE_POLLING_MODE
 */

/*
   define feature
*/
#define BBK_LARGE_SUPRESSION 1

#define FTS_TS_DRV_NAME                     "fts"
#define FTS_TS_DRV_VERSION                  "1108"
#define FTS_ID0                             0x39
#define FTS_ID1                             0x6c //0x80

#define FTS_FIFO_MAX                        32
#define FTS_EVENT_SIZE                      8

#define X_AXIS_MAX                          1440//720//800		//CHANGE while product change
#define X_AXIS_MIN                          0
#define Y_AXIS_MAX                          2560//1280			//CHANGE while product change
#define Y_AXIS_MIN                          0

#define PRESSURE_MIN                        0
#define PRESSURE_MAX                        127

#define FINGER_MAX                          10
#define STYLUS_MAX                          1
#define TOUCH_ID_MAX                        (FINGER_MAX + STYLUS_MAX)

#define AREA_MIN                            PRESSURE_MIN
#define AREA_MAX                            PRESSURE_MAX

/*
 * Firmware
 */
#define MODE_RELEASE_ONLY              0
#define MODE_CONFIG_ONLY               1
#define MODE_RELEASE_AND_CONFIG_64     2
#define MODE_MANULLY 		3


/* Delay to be wait for flash command completion */
#define FTS_FLASH_COMMAND_DELAY    3000

/*
 * Events ID
 */
#define EVENTID_NO_EVENT                    0x00
#define EVENTID_ENTER_POINTER               0x03
#define EVENTID_LEAVE_POINTER               0x04
#define EVENTID_MOTION_POINTER              0x05
#define EVENTID_HOVER_ENTER_POINTER         0x07
#define EVENTID_HOVER_MOTION_POINTER        0x08
#define EVENTID_HOVER_LEAVE_POINTER         0x09
#define EVENTID_PROXIMITY_ENTER             0x0B
#define EVENTID_PROXIMITY_LEAVE             0x0C
#define EVENTID_BUTTON_STATUS               0x0E
#define EVENTID_ERROR                       0x0F
#define EVENTID_CONTROLLER_READY            0x10
#define EVENTID_SLEEPOUT_CONTROLLER_READY   0x11
#define EVENTID_STATUS                      0x16
#define EVENTID_GESTURE                     0x22
#define EVENTID_PEN_ENTER                   0x23
#define EVENTID_PEN_LEAVE                   0x24
#define EVENTID_PEN_MOTION                  0x25
#define EVENTID_LAST                        (EVENTID_PEN_MOTION+1)

/*
 * Commands
 */
#define INT_ENABLE                          0x41
#define INT_DISABLE                         0x00
#define READ_STATUS                         0x84
#define READ_ONE_EVENT                      0x85
#define READ_ALL_EVENT                      0x86
#define SLEEPIN                             0x90
#define SLEEPOUT                            0x91
#define SENSEOFF                            0x92
#define SENSEON                             0x93
#define SELF_SENSEON                        0x95
#define LP_TIMER_CALIB                        0x97
#define GESTURE_OFF                         0x9C// Abb by qiuguifu ,from datasheet.
#define KEY_OFF                             0x9A// Abb by liukangfei ,from harry.
#define KEY_ON                              0x9B// Abb by qiuguifu ,from datasheet.

#define CHARGE_IN		0xA8
#define CHARGE_OUT		0xAB

#define GESTURE_ON                          0x9D
#define GLOVE_MODE_OPEN			            0x9F
#define GLOVE_MODE_CLOSE			        0x9E
#define SYSTEM_RESET			            0xA0
#define FLUSHBUFFER                         0xA1
#define FORCECALIBRATION                    0xA2
#define CX_TUNING                           0xA3
#define SELF_TUNING                         0xA4
#define ITO_CHECK                           0xA7

#define HOVER_ON                            0x95
#define CHARGER_PLUGGED                     0xA7
#define TUNING_BACKUP                       0xFC
#define CONFIG_BACKUP                       0xFB
#define ENTER_GESTURE_MODE 		            0xAD

/*
 * Firmware
 */
#define MODE_RELEASE_ONLY              0
#define MODE_CONFIG_ONLY               1
#define MODE_RELEASE_AND_CONFIG_128    2

/* Flash programming */
#define FLASH_LOAD_FIRMWARE_LOWER_64K       0xF0
#define FLASH_LOAD_FIRMWARE_UPPER_64K       0xF1
#define FLASH_PROGRAM                       0xF2
#define FLASH_ERASE                         0xF3
#define FLASH_READ_STATUS                   0xF4
#define FLASH_UNLOCK                        0xF7
#define FLASH_LOAD_INFO_BLOCK               0xF8
#define FLASH_ERASE_INFO_BLOCK              0xF9
#define FLASH_PROGRAM_INFO_BLOCK            0xFA

#define FLASH_LOAD_FIRMWARE_OFFSET         0x0000
#define FLASH_LOAD_INFO_BLOCK_OFFSET       0xE800

#define FLASH_SIZE_F0_CMD                  (64  * 1024)
#define FLASH_SIZE_FW_CONFIG			   (124 * 1024)
#define FLASH_SIZE_CXMEM				   (4   * 1024)

#define FLASH_UNLOCK_CODE_0                 0x74
#define FLASH_UNLOCK_CODE_1                 0x45

#define FLASH_STATUS_UNKNOWN                (-1)
#define FLASH_STATUS_READY                  (0)
#define FLASH_STATUS_BUSY                   (1)

#define FLASH_LOAD_CHUNK_SIZE               (2048)
#define FLASH_LOAD_COMMAND_SIZE             (FLASH_LOAD_CHUNK_SIZE + 3)

#define FILE_HEADER_SIZE					32
#define FILE_FW_VER_OFFSET					4
#define FILE_CONFIG_VER_OFFSET				(FILE_HEADER_SIZE+1024*122+1) 

/*
 * Gesture direction
 */
#define GESTURE_RPT_LEFT                    1
#define GESTURE_RPT_RIGHT                   2
#define GESTURE_RPT_UP                      3
#define GESTURE_RPT_DOWN                    4

/*
 * Configuration mode
 */
#define MODE_NORMAL                         0
#define MODE_HOVER                          1
#define MODE_PROXIMITY                      2
#define MODE_HOVER_N_PROXIMITY              3
#define MODE_GESTURE                        4
#define MODE_GESTURE_N_PROXIMITY            5
#define MODE_GESTURE_N_PROXIMITY_N_HOVER    6

/*
 * Status Event Field:
 *     id of command that triggered the event
 */
#define FTS_STATUS_MUTUAL_TUNE              0x01
#define FTS_STATUS_SELF_TUNE                0x02
#define FTS_FLASH_WRITE_CONFIG              0x03
#define FTS_FLASH_WRITE_COMP_MEMORY         0x04
#define FTS_FORCE_CAL_SELF_MUTUAL           0x05
#define FTS_FORCE_CAL_SELF                  0x06
#define FTS_WATER_MODE_ON                   0x07
#define FTS_WATER_MODE_OFF                  0x08

#define FTS_FW_ADDR_L			0x04
#define FTS_FW_ADDR_H			0x05
#define FTS_CONFIG_ADDR_H		0x1e822 		//0xf821:64k
#define FTS_CONFIG_ADDR_L		0x1e821		//0xf822

#define PHONE_KEY
/* Production  Test*/
#define READ_CNT_ITO           40
#define READ_CNT               300
#define CMD_STR_LEN            32

#define CNRL_RDY_CNT           50
#define INIT_FLAG_CNT		3
#define EVENTID_COMP_DATA_READ              0x13

#define K_COFF  1
#define L_COFF	1
#define M_COFF	1
#define N_COFF	1

/*
 * forward dec
 */
struct fts_ts_info;
/*
 * Dispatch event handler
 */
typedef unsigned char * (*event_dispatch_handler_t)
				(struct fts_ts_info *info, unsigned char *data);
				
/*  I2C cmd Read  Write Funtion
 */				
#define CMD_RESULT_STR_LEN                  512
#define TSP_BUF_SIZE                       1024				

struct fts_cap_button_map {
	unsigned char nbuttons;
	unsigned char *map;
};

struct fts_i2c_platform_data {
	struct i2c_client *client;
	struct regulator *vcc_ana;
	struct regulator *vcc_i2c;
	int (*power) (bool on);
	u32 reset_gpio_flags;
	int attn_gpio;
	u32 irq_gpio_flags;
	int rst_gpio;
	
	//#if defined(TP_GPIO_CTRL)
	u32 vdd1_gpio_flags;
	int vdd1_gpio;
	//#else
	//#endif
	
	bool x_flip;
	bool y_flip;
	bool swap_axes;
	bool regulator_en;
	int reset_gpio;	//=rst_gpio
	int irq_gpio;	//=attn_gpio
	#if 1//defined(TP_GPIO_CTRL)
	int vdd_gpio;
	u32 vdd_gpio_flags;
	int vcc_gpio;	//vdd1_gpio
	u32 vcc_gpio_flags;	//vdd1_gpio_flags
	#endif
	unsigned long irq_flags;
	//unsigned int panel_x;
	//unsigned int panel_y;
	unsigned int reset_delay_ms;
	
	//chenyunzhe add BEG-----------
	int suspend_resume_methods;
	int fixed_key_type; 
	int ts_dimension_by_lcm;
	
	int lcd_dimension_x;
	int lcd_dimension_y;
	
	int ts_dimension_x;
	int ts_dimension_y;
	
	int  dclick_trip_x_area;
	int  dclick_trip_y_area;
	int  dclick_two_finger_x_area;
	int  dclick_two_finger_y_area;
	
	const char *virtual_key_string;
	 //chenyunzhe add END-----------
	
	int (*gpio_config)(int gpio, bool configure);
	struct fts_cap_button_map *cap_button_map;
};

/*
 * struct fts_ts_info - FTS capacitive touch screen device information
 * @dev:                  Pointer to the structure device
 * @client:               I2C client structure
 * @input_dev             Input device structure
 * @work                  Work thread
 * @event_wq              Event queue for work thread
 * @cmd_done              Asyncronous command notification
 * @event_dispatch_table  Event dispatch table handlers
 * @fw_version            Firmware version
 * @attrs                 SysFS attributes
 * @mode                  Device operating mode
 * @touch_id              Bitmask for touch id (mapped to input slots)
 * @buttons               Bitmask for buttons status
 * @timer                 Timer when operating in polling mode
 * @early_suspend         Structure for early suspend functions
 * @power                 Power on/off routine
 */
struct fts_ts_info {
	struct device            *dev;
	struct i2c_client        *client;
	struct input_dev         *input_dev;

	struct work_struct        work;
	struct workqueue_struct  *event_wq;
	struct completion         cmd_done;
	struct kobject kobject_debug;// bbk add -- qiuguifu
	struct kobject kobject_guest;	//add for guest guester
#if defined(BBK_DCLICK_WAKE)
	struct timer_list dclick_timer;
	struct work_struct dclick_timer_work;
#endif
	//struct mutex input_slot_mutex;//add by qiuguifu bbk.
	unsigned int interrupt_enable;
	event_dispatch_handler_t *event_dispatch_table;
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#endif
	//unsigned int              fw_version;
	//unsigned int              config_version;
	//unsigned int	host_config_version;

	unsigned int ic_fw_config_version;
	unsigned int ic_fw_base_version;
	unsigned int driver_fw_config_version;
	unsigned int driver_fw_base_version;
	
	struct attribute_group    attrs;
	struct device             *i2c_cmd_dev;
	char cmd_read_result[CMD_RESULT_STR_LEN];
	char cmd_wr_result[CMD_RESULT_STR_LEN];
	char cmd_write_result[20];
	unsigned int    log_switch;
	unsigned int              mode;
	unsigned long             touch_id;
	unsigned int              buttons;
	int start ;//  1--start,0---store.
	//add for i2c timeout start 2014.10.9
	struct work_struct irq_err_work;
	struct workqueue_struct *irq_err_workqueue;
	//add for i2c timeout end 2014.10.9
#if defined(BBK_DCLICK_WAKE)

	unsigned int dclick_switch;		//  0---off .1 -- on
	unsigned int has_lcd_shutoff;	//  1:lcd off  0:lcd on
	unsigned int swipe_switch;
	unsigned int gesture_switch;
	unsigned int ts_dclick_simulate_switch;
	atomic_t ts_state;
	int need_change_to_dclick;
	int has_tp_suspend;
/*simulate dclick start*/
	int finger_state;//0--lift.1--down.move
	int pre_state;
	int num_fingers;
	int first_press_x;
	int first_press_y;
	int second_press_x;
	int second_press_y;
	int pressed_count;
	int release_count;
	bool is_dclick_valide;
	bool has_dclick_timer_start;
	int dclick_dimension_x_min;
	int dclick_dimension_x_max;
	int dclick_dimension_y_min;
	int dclick_dimension_y_max;
/*Simulate dclick end*/
#endif
#ifdef FTS_USE_POLLING_MODE
	struct hrtimer timer;
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int glove_mode_switch;// 1 -- open 0 -- close
	int (*power)(bool on);
	int power_state;
	
//liukangfei add for user defined gesture
	u8 gesture_index;
	u8 gesture_status;
	u8 custom_gesture_template_data[62];	//gesture template data.30 points include x and y

	struct timer_list home_timer;
	struct timer_list menu_timer;
	struct timer_list back_timer;
	struct work_struct home_work;
	struct work_struct menu_work;
	struct work_struct back_work;
	int is_0D_key_enable;

	
	long long aa_up_time;
	long long aa_2_2d_diff_time;

	int aa_current_touch_num;
	int touch_flag;

	struct wake_lock wakelock;

	int user_define_gesture_switch;
	int is_template_in_chip;

	/* user mode finger print */
	unsigned int finger_press_record[10][3];

	/* charger flag */
	int usb_charger_flag;		//_1:charge 0:no charge
	struct work_struct charger_work;
	int is_probe_complete;	//_1:finish 0:no finish
	/* add for glove mode while charger */
	int glove_mode_state_save_in_charging;

	/* input lock */
	struct mutex input_report_mutex;

	/* iic and reset lock */
	struct mutex i2c_reset_mutex;

	/* for quick draw gesture wakeup 2times problom */
	unsigned long new_gesture_time;
	unsigned long old_gesture_time;

	/* large press touch */
	int large_press_touch_id;
	int is_large_press_mode;

	/* firmware update flag */
	bool firmware_updating_flag;

	/* edge suppress switch*/
	int edge_suppress_switch;


	unsigned int is_3_finger_touch;
};


struct touchscreen_driver_sysfs_entry {
	struct attribute attr;
	ssize_t (*show)(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf);
	ssize_t (*store)(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count);
};

typedef enum
{
    ERR_ITO_NO_ERR,                 ///< 0 No ITO Error
    ERR_ITO_PANEL_OPEN_FORCE,       ///< 1 Panel Open Force
    ERR_ITO_PANEL_OPEN_SENSE,       ///< 2 Panel Open Sense
    ERR_ITO_F2G,                    ///< 3 Force short to ground
    ERR_ITO_S2G,                    ///< 4 Sense short to ground
    ERR_ITO_F2VDD,                  ///< 5 Force short to VDD
    ERR_ITO_S2VDD,                  ///< 6 Sense short to VDD
    ERR_ITO_P2P_FORCE,              ///< 7 Pin to Pin short (Force)
    ERR_ITO_P2P_SENSE,              ///< 8 Pin to Pin short (Sense)
}errItoSubTypes_t;

extern int fts_interrupt_set(struct fts_ts_info *info, int enable);
extern int fts_command(struct fts_ts_info *info, unsigned char cmd);
extern int fts_write_reg(struct fts_ts_info *info, unsigned char *reg, unsigned short len);
extern int fts_read_reg(struct fts_ts_info *info, unsigned char *reg, int cnum, unsigned char *buf, int num);

extern void  fts_power_reset(struct fts_ts_info *info);
extern void release_point_and_key(struct fts_ts_info *info);
extern int fts_systemreset(struct fts_ts_info *info);
extern int  fts_init_flash_reload(struct fts_ts_info *info);

#endif



