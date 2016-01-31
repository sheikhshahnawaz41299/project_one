
#include <linux/device.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <fts.h>
#include <linux/completion.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/kernel.h>
#include <linux/lockdep.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#ifdef CONFIG_BBK_DRIVER_INFO
#include <linux/bbk_drivers_info.h>
#endif
#include <linux/list.h>
#include <linux/fb.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/time.h>
#include <linux/kthread.h>

#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>

//#include "bbk_ts_node.h"
#include "fts.h"

#include <asm/string.h>
#include <linux/string.h>

static int fts_write_template(struct fts_ts_info *info, const char* buf, int count)
{
	int ret;
	unsigned char start_regAdd[3] = { 0xC3, 0x10, 0x11 };
	unsigned char add_data_1_regAdd[25] = {0};
	unsigned char add_data_2_regAdd[25] = {0};
	unsigned char add_data_3_regAdd[25] = {0};
	unsigned char finish_regAdd[3] = { 0xC3, 0x12, 0x11 };

	add_data_1_regAdd[0] = 0xc3;
	add_data_1_regAdd[1] = 0x11;
	add_data_1_regAdd[2] = 0x11;
	add_data_1_regAdd[3] = 20;
	add_data_1_regAdd[4] = 0;
	
	add_data_2_regAdd[0] = 0xc3;
	add_data_2_regAdd[1] = 0x11;
	add_data_2_regAdd[2] = 0x11;
	add_data_2_regAdd[3] = 20;
	add_data_2_regAdd[4] = 10;
	
	add_data_3_regAdd[0] = 0xc3;
	add_data_3_regAdd[1] = 0x11;
	add_data_3_regAdd[2] = 0x11;
	add_data_3_regAdd[3] = 20;
	add_data_3_regAdd[4] = 20;

	//judge if data is 60 bytes
	if(count != 60) {
		VIVO_TS_LOG_ERR("gesture template data bytes is err! data bytes = %d\n", (int)count);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF("step 1:copy template to buffer!\n");
	if(NULL == memcpy(&add_data_1_regAdd[5], buf, 20)) {
		VIVO_TS_LOG_ERR("copy gesture template data first 20 bytes err!\n");
		return -EINVAL;
	}
	if(NULL == memcpy(&add_data_2_regAdd[5], &buf[20], 20)) {
		VIVO_TS_LOG_ERR("copy gesture template data second 20 bytes err!\n");
		return -EINVAL;
	}
	if(NULL == memcpy(&add_data_3_regAdd[5], &buf[40], 20)) {
		VIVO_TS_LOG_ERR("copy gesture template data third 20 bytes err!\n");
		return -EINVAL;
	}
	
	//Start adding custom gesture
	VIVO_TS_LOG_INF("step 2:send start commond!\n");
	ret = fts_write_reg(info, start_regAdd, sizeof(start_regAdd));
	if (ret) {
		VIVO_TS_LOG_INF("Cannot write commond in RAM\n");
		return -EINVAL;
	}	

	//write data to chip
	VIVO_TS_LOG_INF("step 3:write template to chip!\n");
	ret = fts_write_reg(info, add_data_1_regAdd, 25);
	if (ret) {
		VIVO_TS_LOG_INF("Cannot write 0~19 bytes fail!\n");
		return -EINVAL;
	}
	ret = fts_write_reg(info, add_data_2_regAdd, 25);
	if (ret) {
		VIVO_TS_LOG_INF("Cannot write 20~39 bytes fail!\n");
		return -EINVAL;
	}
	ret = fts_write_reg(info, add_data_3_regAdd, 25);
	if (ret) {
		VIVO_TS_LOG_INF("Cannot write 40~59 bytes fail!\n");
		return -EINVAL;
	}

	//Finish adding custom gesture
	VIVO_TS_LOG_INF("step 4:write finish commond!\n");
	ret = fts_write_reg(info, finish_regAdd, sizeof(finish_regAdd));
	if (ret) {
		VIVO_TS_LOG_INF("Cannot write commond in RAM\n");
		return -EINVAL;
	}	

	info->is_template_in_chip = 1;		//template is wirte to chip,if no remove,and check template return unvalid but is_template_should_in_chip is 1 we should write template to chip
	VIVO_TS_LOG_INF( "step 5:gesture template data write success\n");
	
	return 0;
}

ssize_t fts_custom_gesture_template_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_guest);	//attention!!!!!!!!
	int ret;
	
	if(!info) {
		VIVO_TS_LOG_INF("FTS Failed to get info\n");
		return  -EINVAL;
	}

	VIVO_TS_LOG_INF("gesture_template data count=%d\n", (int)count);

	//judge if data is 60 bytes
	if(count != 60) {
		VIVO_TS_LOG_ERR("gesture template data bytes is err! data bytes = %d\n", (int)count);
		return -EINVAL;
	}
	
	//copy template to driver's buffer to save
	if(NULL == memcpy(info->custom_gesture_template_data, buf, 60)) {
		VIVO_TS_LOG_ERR("copy gesture template data to driver fail!\n");
		return -EINVAL;
	}

	//write template to chip
	ret = fts_write_template(info, buf, count);
	if(ret<0) {
		VIVO_TS_LOG_INF("FTS Failed fts_write_template\n");
		return -EINVAL;
	}

	return 60;
}
ssize_t fts_custom_gesture_template_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_guest);

	if(!info)
	{
		VIVO_TS_LOG_INF("FTS Failed to get info\n");
		return  -EINVAL;
	}

	if(NULL == memcpy(buf, info->custom_gesture_template_data, 60)) {
		VIVO_TS_LOG_ERR("copy gesture template data err!\n");
		return -EINVAL;
	}
	
	return 60;//sprintf(buf, "%s", __func__);
}


//remove custom gesture template
ssize_t fts_custom_gesture_remove_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_guest);

	int val, err = 0;
	unsigned char regAdd[3] = { 0xC3, 0x13, 0x11 };

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR( "%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	if(val != 1) {
		VIVO_TS_LOG_ERR("Invalide number of parameters passed\n");
		return -EINVAL;
	}

	err = fts_write_reg(info, regAdd, sizeof(regAdd));
	if (err) {
		VIVO_TS_LOG_ERR("%s Cannot write reg\n",__func__);
		return -EINVAL;
	}

	info->is_template_in_chip = 0;	//template is not in chip

	VIVO_TS_LOG_INF( "gesture template data remove success\n");

	return count;
}

ssize_t fts_custom_gesture_remove_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "echo 1 to remove gesture template\n");
}

#if 0
//数据读取参考

	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	fts_write_reg(info, regAdd, sizeof(regAdd));
	msleep(30);
	error = fts_read_reg(info, &regAdd3,sizeof(regAdd3), val, FTS_EVENT_SIZE);
	if (error) {
		info->config_version = 0;
		VIVO_TS_LOG_DBG("Cannot read config version\n");
		return -ENODEV;
	} else {
		info->config_version = (val[3] << 8) | val[4];
		VIVO_TS_LOG_INF("read config version = %d\n", info->config_version);
	}
	fts_interrupt_set(info, INT_ENABLE);

#endif

//Check Custom Gesture Valid
ssize_t fts_custom_gesture_valid_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_guest);

	int err = 0;
	unsigned char regAdd[3] = { 0xC3, 0x14, 0x11 };
	unsigned char addr;
	unsigned char data[8];
	unsigned char is_valid = 0;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}

	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	msleep(30);
	
	err = fts_write_reg(info, regAdd, sizeof(regAdd));
	if (err) {
		VIVO_TS_LOG_ERR("%s Cannot write reg\n",__func__);
		err = -EINVAL;
		goto error;		
	}
	msleep(10);
	
	addr = READ_ONE_EVENT;
	err = fts_read_reg(info, &addr, 1, data, FTS_EVENT_SIZE);
	if (err) 
	{
		VIVO_TS_LOG_ERR("read : Cannot read device info\n");
		err = -ENODEV;
		goto error;
	}

	fts_interrupt_set(info, INT_ENABLE);

	VIVO_TS_LOG_INF( "event:%x %x %x %x %x %x %x %x\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
	if(data[0]==0x22 && data[1]==0x04 && data[2]==0x11 &&data[3]==0x14) {
		VIVO_TS_LOG_INF("[%s]get template valid event!\n", __func__);
		is_valid = data[4];		
		if(is_valid == 0) 
			is_valid = 3;
	}

	if(is_valid == 0x01) {
		VIVO_TS_LOG_INF( "Custom template is initialized\n");

	}else if(is_valid == 0x02) {
		is_valid = 0;
		VIVO_TS_LOG_INF( "Custom template is not initialized\n");

	}else if(is_valid == 0x03){
		is_valid = 0;
		VIVO_TS_LOG_INF( "read event err\n");
	}else{
		is_valid = 0;
		VIVO_TS_LOG_INF( "Custom template has never write to chip\n");

	}
		
	return sprintf(buf, "%d  valid state:[1]write succeed [2]not write [3]read err\n", is_valid);
	
error:
	msleep(10);
	fts_interrupt_set(info, INT_ENABLE);
 	
	return sprintf(buf, "tms:%d\n", -1);
}
ssize_t fts_custom_gesture_valid_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
	return -EINVAL;
}

//custom gesture switch
ssize_t touchscreen_user_defined_gesture_enable_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf) {
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	
	return sprintf(buf, "%d", info->user_define_gesture_switch);
}

ssize_t touchscreen_user_defined_gesture_enable_store(struct kobject *kobj, struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR( "%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	if(val<0 || val>1) {
		VIVO_TS_LOG_ERR( "%s Invalide number of parameters passed\n",__func__);
		return  -EINVAL;
	}

	VIVO_TS_LOG_INF("user_defined_gesture:%d\n", val);
	info->user_define_gesture_switch = val;

	return count;
}

/*
 * setting_recheck - while reset or other except,the high layer's set such as gesture switch not change,but the set in touchscreen
 * chip is been reset.so we must rewrite these set from driver to chip,make chip work well.
 * 
 * which need rewrite
 * 1.default gestures set include double click			//phone must in suspend and lcd state is off
 * 2.user define gesture's switch and gesture template	//phone must in suspend and lcd state is off
 * 3.glove mode switch set.							//phone must in resume and lcd state is on
 * 4.charger status.								//phone in all state should write
 *
 * which time we need rewrite
 * 1.while chip reset (ESD...)
 * 2.while power on 
 * 3.after upgrade firmware by hand
 **/
//_1.default gestures set include double click and user define gesture's switch
int fts_gesture_switch_rewrite_tochip(struct fts_ts_info *info) {
	int error;
	
	//gesture enable and disable setting data
	u8 reg1[6] = {0xc3,0x01,0x00,0x00,0x00,0x00};	//enable
	u8 reg2[6] = {0xc3,0x02,0xFF,0xFF,0xFF,0xFF};	//disable	

	VIVO_TS_LOG_INF("[%s]rewrite current gestures setting include double click and user define gesture's switch.\n", __func__);
	if(info->dclick_switch == 1){
		reg1[2]|= 0x02;			   	//reg[2]|=0x02;
	}
	if(info->gesture_switch & 0x02){		//UP
		reg1[3]|= 0x04;			 	//reg[2]|=0x80;
	}
	if(info->gesture_switch & 0X01){		//right
		reg1[2]|= 0x80;				//reg[2]|=0x80;
	}
	if(//gesture_id == GESTURE_SWIPE_WEST_LEFT
		(info->gesture_switch & 0X01)){
		reg1[3]|= 0x01;			  //reg[2]|=0x80;
	}
	if(//gesture_id == GESTURE_SWIPE_SOUTH_DOWN
		(info->swipe_switch == 1)){
		reg1[3]|= 0x02;		   //	reg[2]|=0x80;
	}
	if(//gesture_id == GESTURE_LETTER_C
		(info->gesture_switch & 0X40)){
		reg1[2]|= 0x08;			      //reg[2]|=0x20;
	}
	if(//(gesture_id == GESTURE_LETTER_M ||gesture_id == GESTURE_LETTER_m)
		(info->gesture_switch & 0X10)){
		reg1[2]|= 0x10;				//reg[2]|=0x10;
	}
	if(//gesture_id == GESTURE_LETTER_O
		(info->gesture_switch & 0X04)){
		reg1[2]|= 0x04;    			        //reg[2]|=0x40;
	}
	if(//gesture_id == GESTURE_LETTER_W
		(info->gesture_switch & 0X08)){
		reg1[2]|= 0x20;			//	reg[2]|=0x08;
	}
	if(//gesture_id == GESTURE_LETTER_e
		(info->gesture_switch & 0X20)){
		reg1[2]|= 0x40;				//reg[2]|=0x04;
	}
	if(info->user_define_gesture_switch==1) {	//enable custom gesture
		reg1[4] |=0x02;
	}
	error = fts_write_reg(info,reg2,sizeof(reg2));  		    
	if (error) {
		VIVO_TS_LOG_ERR("[%s]Cannot write reg2,disable gesture.\n",__func__);
		return -1;
	}
	mdelay(5);
	error = fts_write_reg(info,reg1,sizeof(reg1));	            
	if (error) {
		VIVO_TS_LOG_ERR("[%s]Cannot write reg,enable gesture.\n",__func__);
		return -1;
	}
	VIVO_TS_LOG_INF("[%s]FTS fts gesture enable bit : %02X %02X %02X %02X %02X %02X\n", __func__,
		reg1[0], reg1[1], reg1[2], reg1[3], reg1[4], reg1[5]); 

	return 0;		//0:write succeed  not 0:fail
}

/* 2.gesture template */
int fts_custom_gesture_template_rewrite_tochip(struct fts_ts_info *info) 
{
	int ret = 0;

	VIVO_TS_LOG_INF("[%s]rewrite user define gestures template.\n", __func__);

	if(info->user_define_gesture_switch == 0) {
		VIVO_TS_LOG_INF("[%s]user define gestures is off, no need to rewrite template to chip.\n", __func__);
		return 0;
	}	

	ret = fts_write_template(info, info->custom_gesture_template_data, 60);
	if(ret<0) {
		VIVO_TS_LOG_INF("FTS Failed fts_write_template\n");
		return -1;
	}

	VIVO_TS_LOG_INF("[%s]rewrite  user define gestures template succeed.\n", __func__);	

	return 0;	
}

/** _3.glove mode switch set.	
 * when rewrite glove mode setting
 * 1.resume process
 * 2.reset process
 * 3.esd reset must
 *
 */
int fts_glove_mode_switch_rewrite_tochip(struct fts_ts_info *info) 
{
	int error = 0;
	u8 reg_glove_mode[2] = {0};
	
	VIVO_TS_LOG_INF("[%s]rewrite glove mode setting.glove_mode_switch=%d\n", __func__, info->glove_mode_switch);
	if(info->glove_mode_switch == 1 ) {//&& info->has_lcd_shutoff==1) {	//must lcd on and glove_switch on
		
		reg_glove_mode[0] = 0XC1;		//set enable data
		reg_glove_mode[1] = 0X01;
		error = fts_write_reg(info, reg_glove_mode, sizeof(reg_glove_mode));
		if(error != 0) {
			VIVO_TS_LOG_ERR("[%s]Fail to write reg,rewrite gloves mode on.\n", __func__);
			return -1;
		}
	} else {
		reg_glove_mode[0] = 0XC2;		//set disable data
		reg_glove_mode[1] = 0X01;
		error = fts_write_reg(info, reg_glove_mode, sizeof(reg_glove_mode));
		if(error != 0) {
			VIVO_TS_LOG_ERR("[%s]Fail to write reg,rewrite gloves mode off.\n", __func__);
			return -1;
		}
	}	
	VIVO_TS_LOG_INF("[%s]rewrite glove mode setting succeed.\n", __func__);

	return 0;
}


int bbk_rewrite_usb_charger_flag_tochip(struct fts_ts_info *info)
{
	int ret = 0;

	VIVO_TS_LOG_INF("[%s]write charge flag to chip.ts_state:%d,usb_charge_flag=%d\n",
		__func__, atomic_read(&info->ts_state), info->usb_charger_flag);
	//just write in normal mode
	if(atomic_read(&info->ts_state) == TOUCHSCREEN_NORMAL) {//ts work normal,need to write charger 
		if(info->usb_charger_flag == 1) {
			/* sent charger cmd */
			ret += fts_command(info, CHARGE_IN);
			
			/* add for glove mode while charger */
			ret += fts_glove_mode_switch_rewrite_tochip(info);
		} else if(info->usb_charger_flag == 0) {
			/* send no charger cmd */
			ret += fts_command(info, CHARGE_OUT);

			/* add for glove mode while charger */			
			ret += fts_glove_mode_switch_rewrite_tochip(info);
		}
	}

	if(ret < 0) {
		ret = -1;
	}

	return ret;
}

int bbk_rewrite_edge_suppress_switch_tochip(struct fts_ts_info *info)
{
	//unsigned char reg_read[3] = {0xb2, 0x02, 0x8c};
	unsigned char reg_write[4] = {0xB0, 0x02, 0x8c, 0x00};
	//unsigned char state_reg;
	int ret = 0;

	VIVO_TS_LOG_INF("[%s]write edge restrain switch to chip.ts_state:%d,edge_restrain_switch=%d\n",
		__func__, atomic_read(&info->ts_state), info->edge_suppress_switch);
/*	
	ret = fts_read_reg(info, reg_read, sizeof(reg_read), &state_reg, sizeof(state_reg));
	if (ret) {
		VIVO_TS_LOG_ERR("[%s]Cannot read crc status.\n", __func__);
		return -1;
	}
	VIVO_TS_LOG_INF("[%s]read supress reg:%x\n", __func__, state_reg);
*/	
	if(info->edge_suppress_switch == 1)
		reg_write[3] = 0xff;
	else if(info->edge_suppress_switch == 0)
		reg_write[3]= 0;
 
	ret = fts_write_reg(info, reg_write, sizeof(reg_write));
	if(ret) {
		VIVO_TS_LOG_ERR("[%s]send  edge restrain switch cmd fail.\n", __func__);
	}


	return ret;
}


ssize_t touchscreen_edge_suppress_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "%d\n", info->edge_suppress_switch);
}

ssize_t touchscreen_edge_suppress_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	int ret = 0;

	if(!info) {
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR( "[%s]Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF( "[%s]paramater is %d.\n", __func__, val);
	if (val == 0) {
		info->edge_suppress_switch= 0;
	}else if (val == 1) {
		info->edge_suppress_switch = 1;
	}else{
		VIVO_TS_LOG_ERR("[%s]Invalide parameter passed(%d)\n", __func__, val);
		return -EINVAL;
	}

	ret = bbk_rewrite_edge_suppress_switch_tochip(info);
	if(ret) {
		VIVO_TS_LOG_ERR("[%s]bbk_rewrite_edge_suppress_switch_tochip fail.\n", __func__);
	}

	return count;
}


