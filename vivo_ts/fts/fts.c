/*
 * fts.c
 *
 * FTS Capacitive touch screen controller (FingerTipS)
 *
 * Copyright (C) 2012, 2013 STMicroelectronics Limited.
 * Authors: AMS(Analog Mems Sensor)
 *        : Victor Phay <victor.phay@st.com>
 *        : Li Wu <li.wu@st.com>
 *        : Giuseppe Di Giore <giuseppe.di-giore@st.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define DEBUG
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
#include <asm/string.h>
#include <linux/time.h>
#include <linux/kthread.h>

#include <linux/wakelock.h> 

#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>
#include "fts.h"
#include "bbk_ts_node.h"
#include "sensor_test.h"



/* firmware header file for every product */
static char fts_firmware_try_try[] = {
	#include "PD1516A_fw.h"
};
#define NUM_TX  0X0F
#define NUM_RX  0X1A

/*qiuguifu add for each Project end*/


#if 0
//64kb
static int fts_fw_size[] = {
	(62 * 1024),        /* 62Kb */
	( 2 * 1024),        /*  2Kb */
	(32 + (64 * 1024))  /* 32b header + 64Kb firmware */
};
#endif
//128kb
static int fts_fw_size[] = {
	(122 * 1024),        	/* 122Kb */
	( 2 * 1024),        	/*  2Kb */
	(32 + (128 * 1024))  	/* 32b header + 128Kb firmware */
};


extern int qup_i2c_suspended; // add for i2c timeout  0:active 1:i2c bus suspend

 /*  I2C cmd Read  Write Funtion
 */
static struct class *i2c_cmd_class;
static int fts_power_on(bool);
struct fts_ts_info *info_gl;
struct fts_i2c_platform_data pdata_gl ={
	.power = fts_power_on,
};
static int fts_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
static int fts_check_into_gesture_mode(struct fts_ts_info *info) ;
/*
 * Event installer helpers
 */
#define event_id(_e)     EVENTID_##_e
#define handler_name(_h) fts_##_h##_event_handler

#define install_handler(_i, _evt, _hnd) \
do { \
	_i->event_dispatch_table[event_id(_evt)] = handler_name(_hnd); \
} while (0)


/*
 * Asyncronouns command helper
 */
#define WAIT_WITH_TIMEOUT(_info, _timeout, _command) \
do { \
	if (wait_for_completion_timeout(&_info->cmd_done, _timeout) == 0) { \
		dev_warn(_info->dev, "Waiting for %s command: timeout\n", \
		#_command); \
	} \
} while (0)

/*
 * product info
 */
static const char *global_product_name = NULL;
// struct fts_ts_info *global_fts_ts_info = NULL;

void touch_callback(unsigned int status)
{
	/* Empty */
}

/* forward declarations */
static int  fts_suspend(struct i2c_client *client, pm_message_t mesg);
static int  fts_resume(struct i2c_client *client);
static void fts_interrupt_enable(struct fts_ts_info *info);
static void fts_interrupt_disable(struct fts_ts_info *info);
static int  fts_fw_upgrade_and_init(struct fts_ts_info *info, int mode,char name[]);
static int  fts_init_hw(struct fts_ts_info *info);

/* sysfs routines */
static ssize_t fts_fw_control_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	return scnprintf(buf, PAGE_SIZE, "%04x\n", info->ic_fw_base_version);
}


static ssize_t fts_fw_control_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int  mode;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	/* reading out firmware upgrade mode */
	sscanf(buf, "%d", &mode);
	VIVO_TS_LOG_INF("[%s]mode = %d\n", __func__, mode);
	fts_fw_upgrade_and_init(info, mode,NULL);
	//after upgrade fw,must rewrite setting
	fts_glove_mode_switch_rewrite_tochip(info);
	bbk_rewrite_usb_charger_flag_tochip(info);
	
	return count;
}


//////////////////////////////production test////////////////////////////
static int fts_get_ic_fw_base_version(struct fts_ts_info *info);
static int fts_get_ic_fw_config_version(struct fts_ts_info *info);


static int cx_crc_check(struct fts_ts_info *info)
{
	unsigned char regAdd1[3] = {0xB6, 0x00, 0x86};
	unsigned char val[4];
	unsigned char crc_status;
	unsigned int error;
	
	error = fts_read_reg(info, regAdd1, sizeof(regAdd1), val, sizeof(val));
	if (error) {
		VIVO_TS_LOG_ERR("[%s]Cannot read crc status.\n", __func__);
		return -1;
	}

	crc_status = val[1] & 0x02;
	if(crc_status != 0) // CRC error
	{
		VIVO_TS_LOG_ERR("[%s]CRC check fail,status = %d \n", __func__, crc_status);
	}

	return crc_status;	/* 0:no error  not 0:error */
}	

/* get AFE config version,if same,no need to auto tune,else need auto tune */
static int fts_get_init_status(struct fts_ts_info *info)
{
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char regAdd[4] = {0xB2, 0x07, 0x29, 0x04};
	unsigned char buff_read[17];
	unsigned char regAdd1 =0;
	unsigned char event_id = 0;
	unsigned char ms_tune_version      = 0;
	unsigned char chip_ms_tune_version = 0;
	unsigned char ss_tune_version = 0;
	unsigned char chip_ss_tune_version = 0;
	unsigned char error ;
	unsigned int retry ;
	int	address_offset = 0,
		start_tx_offset = 0;
	unsigned char tune_version_same = 0x0;
	
	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, FLUSHBUFFER);
	
	/********************Reading MS tune version*****************************/
	fts_write_reg(info, regAdd, sizeof(regAdd)); //READ Mutual Tune version
	for(retry = 0; retry <= 40; retry++) {
		regAdd1 = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd1,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			dev_err(info->dev, "Cannot read device info\n");
			return -1;
		}
		#ifdef DEBUG
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);
		#endif
		event_id = data[0];
		if (event_id == 0x12){
			ms_tune_version = data[3];
			break;
		}else{
			msleep(10);
		}
	}

	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x02;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry <= READ_CNT; retry++) // poll with time-out 3000ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			VIVO_TS_LOG_ERR("fts_read_mutual_tune_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == EVENTID_COMP_DATA_READ) && (data[1] == 0x02)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				VIVO_TS_LOG_INF("fts_read_mutual_tune_show : TIMEOUT ,Cannot read completion event for MS Touch compensation \n");
			}
		}
	}	

	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);
		VIVO_TS_LOG_INF("FTS Read Offset Address for Compensation1: %02X %02X %02X %02X\n",
				buff_read[0], buff_read[1], buff_read[2],buff_read[3]);	
	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);//if first byte is dummy byte 
	address_offset  = start_tx_offset + 0x10;
	VIVO_TS_LOG_INF("address offset=%d\n", address_offset);

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	VIVO_TS_LOG_INF("FTS  Read Offset Address for f_cnt and s_cnt: %02X %02X %02X %02X  %02X %02X %02X %02X %02X %02X\n",buff_read[0], buff_read[1], buff_read[2],buff_read[3],buff_read[4], buff_read[5], buff_read[6],buff_read[7],buff_read[8],buff_read[9]);	

	chip_ms_tune_version = buff_read[9];
	VIVO_TS_LOG_INF("[%s]chip_ms_tune_version=%d ms_tune_version=%d\n", __func__, chip_ms_tune_version, ms_tune_version);	
	/********************Reading MS tune version-ENDs*****************************/
	if(chip_ms_tune_version == ms_tune_version )
	{
		tune_version_same = 0x1;
		#ifdef DEBUG
		VIVO_TS_LOG_INF("fts MS Tune version is same\n");
		#endif
	}else{
		tune_version_same = 0x0;
		VIVO_TS_LOG_INF("fts MS Tune version not the same\n");
		goto  exit_init ;//return error if ms tune version no matches
	}
	
	/********************Reading SS tune version*****************************/
	//regAdd[4] = { 0xB2, 0x07, 0x51, 0x04};
	regAdd[0] = 0xB2;
	regAdd[1] = 0x07;
	regAdd[2] = 0x4e;	//change by harry
	regAdd[3] = 0x04;
	fts_write_reg(info, regAdd, sizeof(regAdd)); //READ Self Tune version
	for(retry = 0; retry <= 40; retry++) {
		regAdd1 = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd1,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			VIVO_TS_LOG_ERR("Cannot read device info\n");
			return -1;
		}
		#ifdef DEBUG
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);
		#endif
		event_id = data[0];
		if (event_id == 0x12){
			ss_tune_version = data[3];
			break;
		}else{
			msleep(10);
		}
	}
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry <= READ_CNT; retry++) // poll with time-out 3000ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			VIVO_TS_LOG_ERR("%s : I2c READ ERROR : Cannot read device info\n",__func__);
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == EVENTID_COMP_DATA_READ) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				VIVO_TS_LOG_INF("%s : TIMEOUT ,Cannot read completion event for SS Touch compensation \n",__func__);
			}
		}
	}	

	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);
	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);//if first byte is dummy byte 
	address_offset  = start_tx_offset + 0x10;

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	VIVO_TS_LOG_INF("FTS  Read Offset Address for f_cnt and s_cnt: %02X %02X %02X %02X  %02X %02X %02X %02X\n",buff_read[0], buff_read[1], buff_read[2],buff_read[3],buff_read[4], buff_read[5], buff_read[6],buff_read[7]);	

	chip_ss_tune_version = buff_read[9];
	/********************Reading SS tune version-ENDs*****************************/
	if(chip_ss_tune_version == ss_tune_version )
	{
		tune_version_same = 0x1;
		#ifdef DEBUG
		VIVO_TS_LOG_INF("fts SS Tune version is same\n");
		#endif
	}else{
		tune_version_same = 0x0;
		VIVO_TS_LOG_INF("fts SS Tune version not the same\n");
	}
exit_init:
	fts_interrupt_set(info, INT_ENABLE);

	if(tune_version_same == 0) {	
		VIVO_TS_LOG_DBG("fts initialization status error\n");
		return -1;
	}
	else
	{
		VIVO_TS_LOG_INF("fts initialization status OK\n");
		return 0;
	}
}

static ssize_t fts_firmware_change_show(struct device *dev, struct device_attribute *attr, char *buf)
{

	unsigned char regAdd[4] = { 0xB0, 0x00, 0x01, 0x08 };

	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

      VIVO_TS_LOG_INF("FTS ====change_Config_ID Start\n");
	//fts_systemreset(info);
	//msleep(200);
	//fts_command(info, SLEEPOUT);
	//msleep(10);
	//fts_interrupt_set(info, INT_DISABLE);
	//msleep(10);
	fts_write_reg(info, regAdd, sizeof(regAdd));
	msleep(20);
	fts_command(info, CONFIG_BACKUP);
    msleep(50);
	fts_command(info, TUNING_BACKUP);
    msleep(100);
	//fts_systemreset(info);
	//msleep(200);
	fts_command(info, SYSTEM_RESET);
	msleep(200);
	//fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEON);
	//fts_command(info, FORCECALIBRATION);
	//msleep(10);
	fts_command(info, KEY_ON);//key on.
	//msleep(10);
	//fts_interrupt_set(info, INT_ENABLE);


   	VIVO_TS_LOG_INF("FTS ====fts_get_ic_fw_base_version\n");
	fts_get_ic_fw_base_version(info);

	VIVO_TS_LOG_INF("FTS ====fts_get_ic_fw_config_version\n");
	fts_get_ic_fw_config_version(info);


    return sprintf(buf, "%04x\n",info->ic_fw_base_version);

}

static ssize_t fts_autotune_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char event_id = 0;
	unsigned char tune_flag = 0;
	unsigned int retry = 0;
	unsigned char error ;
	unsigned char mutual_check_error = 0;
	unsigned char self_check_error = 0;
	unsigned char tuningvalue_save_error = 0;
	unsigned char crc_check_error = 0;
	unsigned char regAdd =0;

	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	fts_systemreset(info);
	msleep(200);
	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	msleep(5);
	fts_command(info, CX_TUNING);
	msleep(400);

	for (retry = 0; retry < 40; retry++) {
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]Cannot read device info.\n", __func__);
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],
			data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x16) && (tune_flag == 0x01)) {
			if((data[3] == 0x00) && (data[4] == 0x00)){
				mutual_check_error = 0;
				VIVO_TS_LOG_INF("fts autotune check mutual ok \n");
				break;
			}else{
				mutual_check_error = 1;
				VIVO_TS_LOG_ERR("fts autotune check mutual fail \n");
			}
		}else{
			msleep(10);
		}
	}
	fts_command(info, SELF_TUNING);
	msleep(100);

	for (retry = 0; retry < 40; retry++) {
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]Cannot read device info.\n", __func__);
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],
			data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x16) && (tune_flag == 0x02)) {
			if((data[3] == 0x00) && (data[4] == 0x00)){
				self_check_error = 0;
				VIVO_TS_LOG_INF("fts autotune check self ok \n");
				break;
			}else{
				self_check_error = 1;
				VIVO_TS_LOG_ERR("fts autotune check self fail \n");
			}
		}else{
      		msleep(5);
		}
	}

	if ((self_check_error == 0) && (self_check_error == 0)) {
		VIVO_TS_LOG_INF("fts start to save tuning value\n");
		fts_command(info, FLUSHBUFFER);
		msleep(5);
		fts_command(info, TUNING_BACKUP);
		msleep(100);

		for (retry = 0; retry < 40; retry++) {
			regAdd = READ_ONE_EVENT;
			error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
			if (error) {
				VIVO_TS_LOG_ERR("[%s]Cannot read device info.\n", __func__);
				return -ENODEV;
			}
			VIVO_TS_LOG_INF("FTS fts statu event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
 				data[4], data[5], data[6], data[7]);

			event_id = data[0];
			tune_flag = data[1];

			if (event_id == 0x16){
				if(tune_flag == 0x04){
					tuningvalue_save_error = 0;
					VIVO_TS_LOG_INF("fts tuning value save ok \n");
					break;
				}else{
					tuningvalue_save_error = 1;
					VIVO_TS_LOG_ERR("fts tuning value save fail \n");
				}
			}else{
				msleep(5);
			}
		}
	}

	disable_irq(info->client->irq);
	fts_command(info, FLUSHBUFFER);
	msleep(10);
	VIVO_TS_LOG_INF("fts CRC check \n");
	fts_systemreset(info);
	msleep(200);
	fts_interrupt_set(info, INT_DISABLE);

	for (retry = 0; retry < 10; retry++) {
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]Cannot read device info.\n", __func__);
			return -ENODEV;
		}

		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x0F) && (tune_flag == 0x04)) {
 			crc_check_error = 1;
			VIVO_TS_LOG_ERR("fts crc_check_error check  fail \n");
			break;
		}else{
			msleep(10);
		}
	}

	enable_irq(info->client->irq);

	VIVO_TS_LOG_INF("fts restart TP \n");
	fts_systemreset(info);
	msleep(200);
	fts_interrupt_set(info, INT_ENABLE);
	msleep(10);
	fts_command(info, SENSEON);

	return sprintf(buf, "%d\n", (tuningvalue_save_error+self_check_error+mutual_check_error+crc_check_error));

}



static ssize_t fts_ito_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char event_id = 0;
	unsigned char tune_flag = 0;
	unsigned int retry = 0;
	unsigned char i = 0;
	unsigned char error ;
	unsigned char regAdd = 0;
	unsigned char ito_check_error= 0;
	unsigned int ito_check_status[11]={0,0,0,0,0,0,0,0,0,0,0};

	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);

	fts_systemreset(info);
	msleep(200);
	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	msleep(5);
	fts_command(info, ITO_CHECK);
	msleep(200);
	VIVO_TS_LOG_INF("fts ITO Check Start \n");

	for (retry = 0; retry < 40; retry++) {
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			VIVO_TS_LOG_ERR("[%s]Cannot read device info.\n", __func__);
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS ITO event : %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x0F) && (tune_flag == 0x05)) {
			if((data[2] == 0x00) && (data[3] == 0x00)){
				ito_check_status[0] = 0;
				ito_check_error= 0;
				VIVO_TS_LOG_INF("[%s]ITO check ok.\n", __func__);
				break;
			}else{
				ito_check_status[1] = 1;
				ito_check_error= 1;
				VIVO_TS_LOG_INF("[%s]ITO check fail.\n", __func__);
				for(i = 0;i<10;i++){
					if(ito_check_status[i+1] == 0){
 						ito_check_status[i+1] = (data[2] << 8) | data[3];
 						break;
					}
				}
			}
		}else{
			msleep(5);
		}
	}

	memcpy(buf,ito_check_status,sizeof(ito_check_status));
	VIVO_TS_LOG_INF("the ito test data is: ");
	for(i = 0; i < 11; i++){
		VIVO_TS_LOG_INF("%d ",*((int*)(buf+i*4)));
	}
	VIVO_TS_LOG_INF("fts restart TP ++++++++++++ito_check_status = %d\n", ito_check_status[0]);
	fts_systemreset(info);
	msleep(200);
	fts_interrupt_set(info, INT_ENABLE);
	msleep(10);
	fts_command(info, SENSEON);
	msleep(10);
	fts_command(info, KEY_ON);

	return snprintf(buf, PAGE_SIZE, "%x:%x:%x:%x:%x:%x:%x:%x:%x:%x:%x\n",ito_check_error,
		ito_check_status[0],ito_check_status[1],ito_check_status[2],ito_check_status[3],ito_check_status[4],
		ito_check_status[5],ito_check_status[6],ito_check_status[7],ito_check_status[8],ito_check_status[9]);

}

static DEVICE_ATTR(update_fw, 0644, fts_fw_control_show, fts_fw_control_store);
static DEVICE_ATTR(autotune_test,0644, fts_autotune_test_show, NULL);
static DEVICE_ATTR(configid_change,0644, fts_firmware_change_show, NULL);
static DEVICE_ATTR(ito_test,0644, fts_ito_test_show, NULL);

static struct attribute *fts_attr_group[] = {
	&dev_attr_update_fw.attr,
	&dev_attr_autotune_test.attr,
	&dev_attr_configid_change.attr,
	&dev_attr_ito_test.attr,
	NULL,
};

// add by bbk qiuguifu
void release_point_and_key(struct fts_ts_info *info)
{
	int i;
	
	VIVO_TS_LOG_INF("[%s]release point and 0d key.\n", __func__);
	mutex_lock(&info->input_report_mutex);
	for(i = 0;i < FINGER_MAX;i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);
	}
	input_sync(info->input_dev);

	//release key
	input_event(info->input_dev,EV_KEY, KEY_BACK,0);
	input_event(info->input_dev,EV_KEY, KEY_MENU,0);
	input_event(info->input_dev,EV_KEY, KEY_HOMEPAGE,0);
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
	info->buttons = 0;	//must clean for wake,else 0d key will be no function
	info->touch_flag = 0;	//must clean for wake,else 0d key will be no function
}
void release_point(struct fts_ts_info *info)
{
	int i;
	
	VIVO_TS_LOG_DBG("[%s]release point.\n", __func__);
	mutex_lock(&info->input_report_mutex);
	for(i = 0;i < FINGER_MAX;i++) {
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 0);
	}
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
}


unsigned int data[512] = {0};
unsigned char pAddress_i2c[512] = {0};
int byte_count_read = 0 ;
static char Out_buff[512];
static ssize_t fts_i2c_wr_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int i ;
	char buff[16];
	memset(Out_buff, 0x00, ARRAY_SIZE(Out_buff));
	if(byte_count_read == 0)
	{
		snprintf(Out_buff, sizeof(Out_buff), "{FAILED}");
		return snprintf(buf, TSP_BUF_SIZE, "{%s}\n", Out_buff);
	}
#ifdef DEBUG
	 VIVO_TS_LOG_INF("%s:DATA READ {", __func__);
	for(i=0;i<byte_count_read;i++)
	{
		VIVO_TS_LOG_INF(" %02X",(unsigned int )info->cmd_wr_result[i]);
		if(i < (byte_count_read-1))
		{
			VIVO_TS_LOG_INF(" ");
		}
	}
	VIVO_TS_LOG_INF("}\n");
#endif
	snprintf(buff, sizeof(buff), "{");
	strncat(Out_buff, buff, 512);
	for (i = 0; i < byte_count_read; i++) {
		snprintf(buff, sizeof(buff), "%02X", info->cmd_wr_result[i]);
		strncat(Out_buff, buff, 512);
		if(i < (byte_count_read-1))
		{
			snprintf(buff, sizeof(buff), " ");
			strncat(Out_buff, buff, 512);
		}
	}
	snprintf(buff, sizeof(buff), "}");
	strncat(Out_buff, buff, 512);
	return snprintf(buf, TSP_BUF_SIZE, "%s\n", Out_buff);
}
static ssize_t fts_i2c_wr_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char pAddress[8] = {0};
	unsigned int byte_count =0 ;
	int i;
	//mutex_lock(&(info->input_slot_mutex));//ADD BY QIUGUIF BBK
	unsigned int data[8] = {0};
	memset(info->cmd_wr_result, 0x00, ARRAY_SIZE(info->cmd_wr_result));
	sscanf(buf, "%x %x %x %x %x %x %x %x ", (data+7), (data),(data+1),(data+2),(data+3),(data+4),(data+5),(data+6));
	byte_count = data[7];
#ifdef DEBUG
	VIVO_TS_LOG_INF(" \n");
	VIVO_TS_LOG_INF("%s: Input Data 1:",__func__);
	for(i =0 ; i <7; i++)
	{
		 VIVO_TS_LOG_INF(" %02X",data[i]);
		pAddress[i] = (unsigned char)data[i];
	}
	VIVO_TS_LOG_INF("\n");
#else
	for(i =0 ; i <7; i++)
	{
		pAddress[i] = (unsigned char)data[i];
	}
#endif
	byte_count_read = data[byte_count-1];
	ret = fts_write_reg(info,pAddress,3);
	msleep(20);
	ret = fts_read_reg(info,&pAddress[3], (byte_count-4), info->cmd_wr_result ,byte_count_read );
	if (ret)
		VIVO_TS_LOG_ERR("[%s]Unable to read register.\n", __func__);

#ifdef DEBUG
	 VIVO_TS_LOG_INF("%s:DATA READ {", __func__);
	for(i=0;i<byte_count_read;i++)
	{
		VIVO_TS_LOG_INF(" %02X",(unsigned int )info->cmd_wr_result[i]);
	}
	VIVO_TS_LOG_INF("}\n");
#endif

	return count;
}
static ssize_t fts_i2c_read_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int i ;
	char buff[16];
	memset(Out_buff, 0x00, ARRAY_SIZE(Out_buff));
	if(byte_count_read == 0)
	{
		snprintf(Out_buff, sizeof(Out_buff), "{FAILED}");
		return snprintf(buf, TSP_BUF_SIZE, "{%s}\n", Out_buff);
	}
#ifdef DEBUG
	VIVO_TS_LOG_INF("%s:DATA READ {", __func__);
	for(i=0;i<byte_count_read;i++)
	{
		VIVO_TS_LOG_INF("%02X",(unsigned int )info->cmd_read_result[i]);
		if(i < (byte_count_read-1))
		{
			VIVO_TS_LOG_INF(" ");
		}
	}
	VIVO_TS_LOG_INF("}\n");
#endif
	snprintf(buff, sizeof(buff), "{");
	strncat(Out_buff, buff, 512);
	for (i = 0; i < byte_count_read; i++) {
		snprintf(buff, sizeof(buff), "%02X", info->cmd_read_result[i]);
		strncat(Out_buff, buff, 512);
		if(i < (byte_count_read-1))
		{
			snprintf(buff, sizeof(buff), " ");
			strncat(Out_buff, buff, 512);
		}
	}
	snprintf(buff, sizeof(buff), "}");
	strncat(Out_buff, buff, 512);
	return snprintf(buf, TSP_BUF_SIZE, "%s\n", Out_buff);
}
static ssize_t fts_i2c_read_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned char pAddress[8] = {0};
	unsigned int byte_count =0 ;
	int i ;
	unsigned int data[8] = {0};
	byte_count_read = 0;
	memset(info->cmd_read_result, 0x00, ARRAY_SIZE(info->cmd_read_result));
	sscanf(buf, "%x %x %x %x %x %x %x %x ", (data+7), (data),(data+1),(data+2),(data+3),(data+4),(data+5),(data+6));
	byte_count = data[7];
	if(byte_count >7 )
	{
#ifdef DEBUG
		VIVO_TS_LOG_INF("%s : Byte count is more than 7\n",__func__);
#endif
		return count;
	}
#ifdef DEBUG
	VIVO_TS_LOG_INF(" \n");
	VIVO_TS_LOG_INF("%s: Input Data 1:",__func__);
	for(i =0 ; i < byte_count; i++)
	{
		 VIVO_TS_LOG_INF(" %02X",data[i]);
		pAddress[i] = (unsigned char)data[i];
	}
	VIVO_TS_LOG_INF(" \n");
#else
	for(i =0 ; i < byte_count; i++)
	{
		pAddress[i] = (unsigned char)data[i];
	}
#endif
	byte_count_read = data[byte_count-1];
	ret = fts_read_reg(info,pAddress, (byte_count-1), info->cmd_read_result ,byte_count_read );
	if (ret)
		VIVO_TS_LOG_ERR("[%s]Unable to read register.\n", __func__);
	
#ifdef DEBUG
	 VIVO_TS_LOG_INF("%s:DATA READ {", __func__);
	for(i=0;i<byte_count_read;i++)
	{
		VIVO_TS_LOG_INF("%02X",(unsigned int )info->cmd_read_result[i]);
		if(i < (byte_count_read-1))
		{
			VIVO_TS_LOG_INF(" ");
		}
	}
	VIVO_TS_LOG_INF("}\n");
#endif
	
	return count;
}
static ssize_t fts_i2c_write_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	return snprintf(buf, TSP_BUF_SIZE, "%s", info->cmd_write_result);
}
static ssize_t fts_i2c_write_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	struct i2c_client *client = to_i2c_client(dev);
	struct fts_ts_info *info = i2c_get_clientdata(client);
	unsigned short byte_count =0 ;
	int i ;
	memset(pAddress_i2c, 0x00, ARRAY_SIZE(pAddress_i2c));
	memset(info->cmd_write_result, 0x00, ARRAY_SIZE(info->cmd_write_result));
	sscanf(buf, "%x ", data);
	byte_count = data[0];
	if(byte_count <= 512)
	{
		for(i=1; i <= byte_count ;i++)
		{
			sscanf(&buf[3*i] , "%x ", (data+(i-1)));
		}
	}else
	{
#ifdef DEBUG
		VIVO_TS_LOG_INF("%s : message size is more than allowed limit of 512 bytes\n",__func__);
#endif
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write NOT OK}\n");
	}
#ifdef DEBUG
	VIVO_TS_LOG_INF(" \n");
	//printk("%s: Byte_count=  %02d| Count = %02d | size of buf:%02d\n",__func__,byte_count,count, (int)(unsigned long)sizeof(buf));
	VIVO_TS_LOG_INF("%s: Input Data 1:",__func__);
	for(i =0 ; i < byte_count; i++)
	{
		VIVO_TS_LOG_INF(" %02X",data[i]);
		pAddress_i2c[i] = (unsigned char)data[i];
	}
	VIVO_TS_LOG_INF(" \n");
#else
	for(i =0 ; i < byte_count; i++)
	{
		pAddress_i2c[i] = (unsigned char)data[i];
	}
#endif
	if((pAddress_i2c[0] == 0xb3)&&(pAddress_i2c[3] == 0xb1))
	{
#ifdef DEBUG
		VIVO_TS_LOG_INF("%s : B3B1 write\n",__func__);
#endif
		ret = fts_write_reg(info, pAddress_i2c, 3);
		msleep(20);
		ret = fts_write_reg(info,&pAddress_i2c[3], byte_count-3);
	}else
	{
#ifdef DEBUG
		VIVO_TS_LOG_INF("%s : B6B2B0 write\n",__func__);
#endif
		ret = fts_write_reg(info,pAddress_i2c, byte_count);
	}
#ifdef DEBUG
	VIVO_TS_LOG_INF("%s:DATA :", __func__);
	for(i=0;i<byte_count;i++)
	{
		VIVO_TS_LOG_INF(" %02X",(unsigned int )pAddress_i2c[i]);
	}
	VIVO_TS_LOG_INF(" byte_count: %02X\n",byte_count);
#endif
	if (ret)
	{
		dev_err(dev, "{Write NOT OK}\n");
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write NOT OK}\n");
	}else
	{
		snprintf(info->cmd_write_result, sizeof(info->cmd_write_result), "{Write OK}\n");
#ifdef DEBUG
		VIVO_TS_LOG_INF("%s : {Write OK}\n",__func__);
#endif
	}
	return count;
}
static DEVICE_ATTR(iread,(S_IWUSR|S_IWGRP), NULL, fts_i2c_read_store);
static DEVICE_ATTR(iread_result,(S_IRUGO), fts_i2c_read_show, NULL);
static DEVICE_ATTR(iwr,(S_IWUSR|S_IWGRP), NULL, fts_i2c_wr_store);
static DEVICE_ATTR(iwr_result,(S_IRUGO), fts_i2c_wr_show, NULL);
static DEVICE_ATTR(iwrite,(S_IWUSR|S_IWGRP), NULL, fts_i2c_write_store);
static DEVICE_ATTR(iwrite_result,(S_IRUGO), fts_i2c_write_show, NULL);



static struct attribute *i2c_cmd_attributes[] = {
	&dev_attr_iread.attr,
	&dev_attr_iread_result.attr,
	&dev_attr_iwr.attr,
	&dev_attr_iwr_result.attr,
	&dev_attr_iwrite.attr,
	&dev_attr_iwrite_result.attr,
	NULL,
};
static struct attribute_group i2c_cmd_attr_group = {
	.attrs = i2c_cmd_attributes,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void fts_early_suspend(struct early_suspend *h)
{
	struct fts_ts_info *info;
	struct device *dev;

	info = container_of(h, struct fts_ts_info, early_suspend);
	dev = &info->client->dev;
	dev_info(dev, "FTS Early Suspend entered\n");
	if (fts_suspend(info->client, PMSG_SUSPEND))
		dev_err(&info->client->dev, "Early suspend failed\n");
	dev_info(dev, "FTS Early Suspended\n");
}


static void fts_late_resume(struct early_suspend *h)
{
	struct fts_ts_info *info;
	struct device *dev;

	info = container_of(h, struct fts_ts_info, early_suspend);
	dev = &info->client->dev;
	dev_info(dev, "FTS Early Resume entered\n");
	if (fts_resume(info->client))
		dev_err(&info->client->dev, "Late resume failed\n");
	dev_info(dev, "FTS Early Resumed\n");
	release_point_and_key(info);//add by qiuguifu
}
#endif /* CONFIG_HAS_EARLYSUSPEND */
/*wunandi add start*/
#if 0
static void fts_early_suspend(struct early_suspend *h)
{
	struct fts_ts_info *info;
	struct device *dev;

	info = container_of(h, struct fts_ts_info, early_suspend);
	dev = &info->client->dev;
	dev_info(dev, "FTS Early Suspend entered\n");
	fts_standard_suspend (info);
}

static void fts_late_resume(struct early_suspend *h)
{
	struct fts_ts_info *info;
	struct device *dev;

	info = container_of(h, struct fts_ts_info, early_suspend);
	dev = &info->client->dev;
	dev_info(dev, "FTS Early Suspend entered\n");
	fts_standard_resume(info);
}
#endif
/*wunandi add end*/

int fts_write_reg(struct fts_ts_info *info, unsigned char *reg,
						 unsigned short len)
{
	struct i2c_msg xfer_msg[1];
	int ret = 0;
	int retry_count = 0;
	/*
	int error;
	while (len > 255)
	{
		xfer_msg[0].addr = info->client->addr;
		xfer_msg[0].len = 255;
		xfer_msg[0].flags = 0;
		xfer_msg[0].buf = reg;
		error = (i2c_transfer(info->client->adapter, xfer_msg, 1) != 1);
		if(error)
			return error;
		len -= 255;
		reg += 255;
	}*/
	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = len;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;
	while(qup_i2c_suspended > 0)
	{
		VIVO_TS_LOG_INF("[%s]fts iic is on suspend.delay %dms.\n", __func__, (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 5) {
			VIVO_TS_LOG_INF("[%s]after 60ms delay , fts iic is still  on suspend.\n", __func__);
			return -1;
		}
	}

	/* iic and chip reset mutex,must do. */
	mutex_lock(&info->i2c_reset_mutex);
	ret = (i2c_transfer(info->client->adapter, xfer_msg, 1) != 1);// 0--ok,1---fail
	mutex_unlock(&info->i2c_reset_mutex);	
	
	if(ret == 1) {
		ret = -1;
	}
	return ret;
}


int fts_read_reg(struct fts_ts_info *info, unsigned char *reg, int cnum,
						unsigned char *buf, int num)
{
	struct i2c_msg xfer_msg[2];
	int ret = 0;
	int retry_count = 0;

	xfer_msg[0].addr = info->client->addr;
	xfer_msg[0].len = cnum;
	xfer_msg[0].flags = 0;
	xfer_msg[0].buf = reg;

	xfer_msg[1].addr = info->client->addr;
	xfer_msg[1].len = num;
	xfer_msg[1].flags = I2C_M_RD;
	xfer_msg[1].buf = buf;
	while(qup_i2c_suspended > 0)
	{
		VIVO_TS_LOG_INF("[%s]fts iic is on suspend.delay %dms.\n", __func__, (retry_count+1)*10);
		retry_count++;
		msleep(10);
		if(retry_count > 5) {
			VIVO_TS_LOG_INF("[%s]after 60ms delay , fts iic is still  on suspend.\n", __func__);
			return -1;
		}
	}

	/* iic and chip reset mutex,must do. */
	mutex_lock(&info->i2c_reset_mutex);
	ret = (i2c_transfer(info->client->adapter, xfer_msg, 2) != 2);
	mutex_unlock(&info->i2c_reset_mutex);	

	if(ret == 1) {
		ret = -1;
	}

	return ret;
}


int fts_command(struct fts_ts_info *info, unsigned char cmd)
{
	unsigned char regAdd;
	int ret;

	regAdd = cmd;
	ret = fts_write_reg(info, &regAdd, sizeof(regAdd));
	VIVO_TS_LOG_DBG("[%s]send command 0x%02x, return value %d\n", __func__, cmd, ret);
	if(ret) {
		VIVO_TS_LOG_INF("[%s]send command 0x%02x fail, return value %d\n", __func__, cmd, ret);
	}
	return ret;
}


int fts_systemreset(struct fts_ts_info *info)
{
	int ret;
	unsigned char regAdd[4] = { 0xB6, 0x00, 0x23, 0x01 };

	VIVO_TS_LOG_INF("[%s]Doing a system reset\n", __func__);

	ret = fts_write_reg(info, regAdd, sizeof(regAdd));
	if(ret) {
		VIVO_TS_LOG_ERR("[%s]fail to write systemreset cmd to chip.\n", __func__);
		return -1;
	}

	ret = fts_wait_controller_ready(info);
	if(ret) {
		VIVO_TS_LOG_ERR("[%s]fail to read controller ready event.\n", __func__);
		return -1;
	}

	return ret;
}

/*
static int fts_get_mode(struct fts_ts_info *info)
{
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char regAdd[4] = { 0xB2, 0x00, 0x02, 0x01 };

	fts_write_reg(info, regAdd, sizeof(regAdd));

	msleep(20);

	regAdd[0] = READ_ONE_EVENT;
	info->mode = fts_read_reg(info, regAdd, 1, data, FTS_EVENT_SIZE) ?
			MODE_NORMAL : data[3];

	return 0;
}
*/

/*

fts_read_frame(TYPE_RAW_TOUCH);    mutual raw
fts_read_frame(TYPE_NORM_TOUCH);   touch strength
fts_read_frame(TYPE_RAW_SELF);     self raw

*/
#define READ_CHUNK_SIZE 	128
#define TYPE_RAW_TOUCH	0
#define TYPE_RAW_SELF		8
unsigned char pFrame[4096];

int fts_read_frame(struct fts_ts_info *info,unsigned char type)
{
	unsigned char pChannelLength[8] = {0xB3, 0x00, 0x00, 0xB1, 0xF8, 0x14, 0x03, 0x00};//{0xB3, 0x00, 0x10, 0xB1, 0x00, 0x14, 0x03, 0x00};
	unsigned char pFrameAddress[8] = {0xD0, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00};
	unsigned int FrameAddress = 0;
	unsigned int writeAddr = 0;
	unsigned int start_addr = 0;
	unsigned int end_addr = 0;
	unsigned int totalbytes = 0;
	unsigned int remained = 0;
	unsigned int readbytes = 0xFF;
	unsigned int dataposition = 0;
	unsigned char pRead[READ_CHUNK_SIZE] = {0};
	unsigned char SenseChannelLength=0;
	unsigned char ForceChannelLength=0;

	int ret = 0;
	int i = 0;

	//首先拿到channel数目
	fts_write_reg(info, &pChannelLength[0], 3);
	ret = fts_read_reg(info, &pChannelLength[3], 3, pRead, pChannelLength[6]);
	if(ret == 0)
	{
		SenseChannelLength = pRead[1];
		ForceChannelLength = pRead[2];
		totalbytes = SenseChannelLength * ForceChannelLength * 2;
	}
	else
	{
		VIVO_TS_LOG_INF("FTS :read failed rc = %d \n", ret);
		return ret;
	}

	VIVO_TS_LOG_INF("fts SenseChannelLength = %d, ForceChannelLength = %d \n", SenseChannelLength, ForceChannelLength);

	switch(type)
	{
		case TYPE_RAW_TOUCH:
			VIVO_TS_LOG_INF("FTS [Raw Touch : 0x%X] \n", start_addr);
			break;
		case TYPE_RAW_SELF:
			VIVO_TS_LOG_INF("FTS [Raw Self : 0x%X] \n", start_addr);
			break;
		 default:
			VIVO_TS_LOG_INF("FTS: type not defined\n");
	}

	//set start addr according the type params
	pFrameAddress[2] = type;

	ret = fts_read_reg(info, &pFrameAddress[0], 3, pRead, pFrameAddress[3]);
	if(ret == 0)
	{
		FrameAddress = pRead[0] + (pRead[1] << 8);
		start_addr = 0xD0000000 + FrameAddress;
		end_addr = 0xD0000000 + FrameAddress + totalbytes;
	}
	else
	{
		VIVO_TS_LOG_INF("FTS read failed rc = %d \n", ret);
		return ret;
	}

	VIVO_TS_LOG_INF("FTS FrameAddress = %X \n", start_addr);
	VIVO_TS_LOG_INF("FTS start_addr = %X, end_addr = %X \n", start_addr, end_addr);

	remained = totalbytes;

	//根据以上信息读取要的data存在pFrame中。
	for(writeAddr = start_addr; writeAddr < end_addr; writeAddr += READ_CHUNK_SIZE)
	{
		pFrameAddress[1] = (writeAddr >> 8) & 0xFF;
		pFrameAddress[2] = writeAddr & 0xFF;

		if(remained >= READ_CHUNK_SIZE)
		{
			readbytes = READ_CHUNK_SIZE;
		}
		else
		{
			readbytes = remained;
		}

		memset(pRead, 0x0, readbytes);
		fts_read_reg(info, &pFrameAddress[0], 3, pRead, readbytes);

		remained -= readbytes;

		for(i = 0; i < readbytes; i += 2)
		{
			pFrame[dataposition++] = pRead[i] + (pRead[i + 1] << 8);
		}

		VIVO_TS_LOG_INF("FTS writeAddr = %X, start_addr = %X, end_addr = %X \n", writeAddr, start_addr, end_addr);

	}

	return 0;
}

static int fts_get_ic_fw_base_version(struct fts_ts_info *info)
{
	unsigned char val[8];
	unsigned char regAdd[3];
	int error;
	/* TS Chip ID */
	regAdd[0] = 0xB6;
	regAdd[1] = 0x00;
	regAdd[2] = 0x07;
	error = fts_read_reg(info, regAdd, 3, (unsigned char *)val, 8);
	if (error) {
             dev_err(info->dev, "Cannot read device id\n");
             return -ENODEV;
   	}
    	VIVO_TS_LOG_INF("FTS %s : %02X%02X%02X =  %02x %02x %02x %02x %02x \n",__func__,
           				regAdd[0], regAdd[1], regAdd[2], val[1], val[2], val[3], val[4],val[5]);
 	 if ((val[1] != FTS_ID0) || (val[2] != FTS_ID1))
  	{
		dev_err(info->dev, "Wrong version id (read 0x%02x%02x, expected 0x%02x%02x)\n",
                                           val[1], val[2], FTS_ID0, FTS_ID1);
   	 	return -ENODEV;
	}

	/* store firmware version */
	info->ic_fw_base_version= (val[5] << 8) | val[4];

	return 0;
}

static int fts_get_ic_fw_config_version(struct fts_ts_info *info)
{
	unsigned char val[8]={0};
	unsigned char regAdd[4] = {0xB2, 0x00, 0x01,0x08};
	unsigned char regAdd3 = READ_ONE_EVENT;
	int error;
	int event_num = 0xfff;
	int retry_count = 0;
	
	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
read_config_retry:	
	fts_command(info, FLUSHBUFFER);
	msleep(30);	//must delay long time else will read false event data
	fts_write_reg(info, regAdd, sizeof(regAdd));
	msleep(30);

	while(event_num >= 0) {
		error = fts_read_reg(info, &regAdd3,sizeof(regAdd3), val, FTS_EVENT_SIZE);
		if (error) {
			info->ic_fw_config_version= 0;
			VIVO_TS_LOG_ERR("[%s]Cannot read event.\n", __func__);
			fts_interrupt_set(info, INT_ENABLE);
			return -ENODEV;
		}

		VIVO_TS_LOG_INF("[%s]event data:%d %d %d %d %d %d %d %d\n", __func__, val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7]);
		
		if (val[0]==18 && val[1]==0 && val[2]==1) {
			info->ic_fw_config_version = (val[4] << 8) | val[3];
			VIVO_TS_LOG_INF("[%s]read ic_fw_config_version = %d\n", __func__, info->ic_fw_config_version);			
			break;
		}

		/* avoid loop forever */
		if(event_num == 0xfff) {
			event_num = val[7];
			/* max event num is 32(FTS_FIFO_MAX),so... */
			if(event_num > FTS_FIFO_MAX) {
				event_num = FTS_FIFO_MAX; 
			}
		}		
		event_num--;		
	}
	if(info->ic_fw_config_version == 0) {
		VIVO_TS_LOG_ERR("[%s]get ic config version fail.retry_time=%d\n", __func__, retry_count);
		if(retry_count < 2) {
			retry_count++;
			goto read_config_retry;
		}
		
	}

	fts_interrupt_set(info, INT_ENABLE);

	return 0;
}

#if 0
static int fts_check_fw_version_and_is_ic_connected(struct fts_ts_info *info)
{
	unsigned char val[8];
	unsigned char regAdd[3];
	int error;
	int i;
	/* TS Chip ID */
	regAdd[0] = 0xB6;
	regAdd[1] = 0x00;
	regAdd[2] = 0x07;
	for(i=0; i<50; i++) {		
		//info->client->addr = 73;		//do not set i2c addr,because dts has set.delete by chenpeng,get addr by dtsi
		error = fts_read_reg(info, regAdd, 3, (unsigned char *)val, 8);
		if(error) {
			VIVO_TS_LOG_ERR("[%s]Cannot read device id. wait time:%dms\n", __func__, i*10);
			msleep(10);
			continue;
		} else {
			break;
		}
	}
	if (error) {
             VIVO_TS_LOG_ERR("[%s]Cannot read device id,And retry 200 times all 2000ms,ic still no response.so exit.\n", __func__);
             return -ENODEV;
   	}
  	VIVO_TS_LOG_INF("[%s]version is :%02x %02x %02x %02x %02x %02x %02x %02x \n", __func__,
           val[0], val[1],val[2], val[3], val[4], val[5], val[6], val[7] );
	return 0;
}
#endif

//get fw's base version in driver
static  unsigned int fts_get_driver_fw_base_version(void)
{
 	unsigned int driver_fw_base_version=0xff;

	if(fts_firmware_try_try !=NULL)
		driver_fw_base_version = (fts_firmware_try_try[FTS_FW_ADDR_L] |( fts_firmware_try_try[FTS_FW_ADDR_H] << 8));
	else
		VIVO_TS_LOG_INF( "FTS  get driver fw base version error! ");

	return driver_fw_base_version;
}
//get fw's config version in driver
static  unsigned int fts_get_driver_fw_config_version(void)
{
 	unsigned int driver_fw_config_version=0xff;

	if(fts_firmware_try_try !=NULL)
		driver_fw_config_version = ((fts_firmware_try_try[FTS_CONFIG_ADDR_H] <<8 ) | fts_firmware_try_try[FTS_CONFIG_ADDR_L]);
	else
		VIVO_TS_LOG_INF( "FTS  get driver fw config version error!\n");

	return driver_fw_config_version;
}

static int fts_flash_status(struct fts_ts_info *info,
				unsigned int timeout, unsigned int steps)
{
	int ret, status;
	unsigned char data;
	unsigned char regAdd[2];
	unsigned int timeout_bk = timeout;

	do {
		regAdd[0] = FLASH_READ_STATUS;
		regAdd[1] = 0;

		msleep(20);

		ret = fts_read_reg(info, regAdd, sizeof(regAdd), &data, sizeof(data));
		if (ret)
			status = FLASH_STATUS_UNKNOWN;
		else
			status = (data & 0x01) ? FLASH_STATUS_BUSY : FLASH_STATUS_READY;

		dev_err(info->dev, "%s : status = %d (1:busy,0:ready )\n",__func__,status);

		if (status == FLASH_STATUS_BUSY) {
			timeout -= steps;
			msleep(steps);
		}

	} while ((status == FLASH_STATUS_BUSY) && (timeout));

	dev_err(info->dev, "%s : use %d times\n",__func__,timeout_bk-timeout);
	return status;
}


static int fts_flash_unlock(struct fts_ts_info *info)
{
	int ret;
	unsigned char regAdd[4] = { FLASH_UNLOCK,
				FLASH_UNLOCK_CODE_0,
				FLASH_UNLOCK_CODE_1,
				0x00 };

	ret = fts_write_reg(info, regAdd, sizeof(regAdd));
	if (ret)
		VIVO_TS_LOG_INF("Cannot unlock flash\n");
	else
		VIVO_TS_LOG_INF("Flash unlocked\n");

	return ret;
}

static int fts_flash_load(struct fts_ts_info *info,
			int cmd, int address, const char *data, int size)
{
	int ret = 0;
	unsigned char *cmd_buf;
	unsigned int loaded;

	cmd_buf = kmalloc(FLASH_LOAD_COMMAND_SIZE, GFP_KERNEL);
	if (cmd_buf == NULL) {
		VIVO_TS_LOG_INF("Out of memory when programming flash\n");
		return -1;
	}

	loaded = 0;
	while (loaded < size) {
		cmd_buf[0] = cmd;
		cmd_buf[1] = (address >> 8) & 0xFF;
		cmd_buf[2] = (address) & 0xFF;

		memcpy(&cmd_buf[3], data, FLASH_LOAD_CHUNK_SIZE);
		ret = fts_write_reg(info, cmd_buf, FLASH_LOAD_COMMAND_SIZE);
		if (ret) {
			VIVO_TS_LOG_INF("Cannot load firmware in RAM\n");
			break;
		}
		VIVO_TS_LOG_INF("FTS LOADED = %d chunk succeed.\n",loaded/FLASH_LOAD_CHUNK_SIZE);
		data += FLASH_LOAD_CHUNK_SIZE;
		loaded += FLASH_LOAD_CHUNK_SIZE;
		address += FLASH_LOAD_CHUNK_SIZE;
	}

	kfree(cmd_buf);

	return (loaded == size) ? 0 : -1;
}



static int fts_flash_erase(struct fts_ts_info *info, int cmd)
{
	int ret;
	unsigned char regAdd = cmd;

	ret = fts_write_reg(info, &regAdd, sizeof(regAdd));

	if (ret)
		VIVO_TS_LOG_ERR("Cannot erase flash\n");
	else
		VIVO_TS_LOG_INF("Flash erased\n");

	return ret;
}


static int fts_flash_program(struct fts_ts_info *info, int cmd)
{
	int ret;
	unsigned char regAdd = cmd;

	ret = fts_write_reg(info, &regAdd, sizeof(regAdd));

	if (ret)
		VIVO_TS_LOG_ERR("Cannot program flash\n");
	else
		VIVO_TS_LOG_INF("Flash programmed\n");

	return ret;
}
/*
static int flashBusy(struct fts_ts_info *info)
{
	int error, busy=1;
	unsigned char pread[1] ={0};
	unsigned char readreg[1] = {0xF4};
	error = fts_read_reg(info, readreg, 1, pread,1);
	if (error) {
		dev_err(info->dev, "Cannot read Flash Status\n");
		return -ENODEV;
	}
	if((pread[0]& 0x01) == 0)
	{
		busy =0;
	}
        return (busy);
}
*/
static int fts_fw_upgrade(struct fts_ts_info *info, int mode,char name[], int crc_err)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	unsigned char *data;
	unsigned int size;
	int status;
	int program_command, erase_command, load_command=0, load_address = 0;
	int request_firmware_tag = 0;
	int cur_fw_base_version = 0;
	int cur_fw_config_version = 0;

	VIVO_TS_LOG_INF("FTS Firmware upgrade is_probe_complete = %d\n",info->is_probe_complete);
	if(info->is_probe_complete == 1)	
	{
		VIVO_TS_LOG_INF("Firmware upgrade...\n");


		ret = request_firmware(&fw, name, info->dev);
		if (ret) {
			VIVO_TS_LOG_INF("Unable to open firmware file '%s'\n",
				name);
			return ret;
		}
		request_firmware_tag = 1;
		VIVO_TS_LOG_INF("fw file size = %d, fw size = %d\n", (int)fw->size, fts_fw_size[mode]);
		if ((fw->size == 0) || (fw->size != fts_fw_size[mode])) {  //mode=2
			VIVO_TS_LOG_INF("Wrong firmware file '%s' in upgrade by hand!\n", name);
			ret = -1;
			goto fw_done;
		}

		VIVO_TS_LOG_INF("Flash programming...\n");

		data = (unsigned char *)fw->data;
		size = fw->size;
	}else{
		//upgrade auto
		VIVO_TS_LOG_INF("Firmware upgrade...start\n");
		data = fts_firmware_try_try;
		size = sizeof(fts_firmware_try_try);

		VIVO_TS_LOG_INF("driver fw size = %d\n", size);
		if ((size == 0) || (size != fts_fw_size[mode])) {	
			VIVO_TS_LOG_ERR("Wrong firmware file in probe auto upgrade!\n"); //65568
			ret = -1;
			goto fw_done;
		}		
	}
	/* get current update firmware version */
	cur_fw_base_version = (data[FTS_FW_ADDR_L] |( data[FTS_FW_ADDR_H] << 8));
	cur_fw_config_version = ((data[FTS_CONFIG_ADDR_H] <<8 ) | data[FTS_CONFIG_ADDR_L]);

	/* if crc err,must not run system_reset,because wait_control_ready will timeout */
	if(crc_err == 0) {
		ret = fts_systemreset(info); 
		if (ret) {
			VIVO_TS_LOG_ERR("[%s]Cannot reset the device\n", __func__);
			goto fw_done;
		}
	}
	switch (mode) {
	case MODE_CONFIG_ONLY:
		program_command = FLASH_PROGRAM;
		erase_command = FLASH_ERASE;
		load_command = FLASH_LOAD_FIRMWARE_UPPER_64K;
		load_address = FLASH_LOAD_INFO_BLOCK_OFFSET;
		break;
	case MODE_RELEASE_AND_CONFIG_128:
		/* skip 32 bytes header */
		data += 32;
		size = size - 32;
		/* fall throug */
	case MODE_RELEASE_ONLY:
		program_command = FLASH_PROGRAM;
		erase_command = FLASH_ERASE;
		load_command = FLASH_LOAD_FIRMWARE_LOWER_64K;
		load_address = FLASH_LOAD_FIRMWARE_OFFSET;
		break;
	default:
		/* should never be here, already checked mode value before */
		break;
	}

	VIVO_TS_LOG_INF("1) checking for status.\n");
	status = fts_flash_status(info, 1000, 100);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		VIVO_TS_LOG_INF("Wrong flash status\n");
		ret = -1;
		goto fw_done;
	}

	VIVO_TS_LOG_INF("2) unlock the flash.\n");
	ret = fts_flash_unlock(info);
	if (ret) {
		VIVO_TS_LOG_INF("Cannot unlock the flash device\n");
		goto fw_done;
	}

	status = fts_flash_status(info, 3000, 100);
	VIVO_TS_LOG_INF("FTS 1 status = %d\n",status);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		VIVO_TS_LOG_ERR("Wrong flash status\n");
		ret = -1;
		goto fw_done;
	}

	VIVO_TS_LOG_INF("3) load the program.\n");

	if(load_command == FLASH_LOAD_FIRMWARE_LOWER_64K) {
		ret = fts_flash_load(info, load_command, load_address, data, FLASH_SIZE_F0_CMD);
		load_command = FLASH_LOAD_FIRMWARE_UPPER_64K;
		if((crc_err == 0)&&(size == (FLASH_SIZE_FW_CONFIG + FLASH_SIZE_CXMEM))) {//only for D2 chip 
			//if size is 128 K, then adjust the size to include only fw and config(124 K)
			size = size - FLASH_SIZE_CXMEM;
		}		
		ret = fts_flash_load(info, load_command, load_address, (data+FLASH_SIZE_F0_CMD), (size-FLASH_SIZE_F0_CMD));
	} else {
		ret = fts_flash_load(info, load_command, load_address, data, size);
	}
	if (ret) {
		VIVO_TS_LOG_INF("Cannot load program to for the flash device\n");
		goto fw_done;
	}

	/* wait for a while */
	status = fts_flash_status(info, 3000, 100);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		VIVO_TS_LOG_ERR("Wrong flash status 3\n");
		ret = -1;
		goto fw_done;
	}

	VIVO_TS_LOG_INF("4) erase the flash.\n");
	ret = fts_flash_erase(info, erase_command);
	if (ret) {
		VIVO_TS_LOG_ERR("Cannot erase the flash device\n");
		goto fw_done;
	}

	/* wait for a while */
	VIVO_TS_LOG_INF("5) checking for status.\n");
	status = fts_flash_status(info, 3000, 100);
	VIVO_TS_LOG_INF("FTS 1 status = %d\n",status);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) {
		VIVO_TS_LOG_ERR("Wrong flash status 4\n");
		ret = -1;
		goto fw_done;
	}

	/* wait for a while */
	VIVO_TS_LOG_INF("6) program the flash.\n");
	ret = fts_flash_program(info, program_command);
	if (ret) {
		VIVO_TS_LOG_ERR("Cannot program the flash device\n");
		goto fw_done;
	}

	/* wait for a while */
	status = fts_flash_status(info, 3000, 100);
	if ((status == FLASH_STATUS_UNKNOWN) || (status == FLASH_STATUS_BUSY)) 
	{
		VIVO_TS_LOG_ERR("Wrong flash status 5\n");
		ret = -1;
		goto fw_done;
	}

	VIVO_TS_LOG_INF("Flash programming: done.\n");

	VIVO_TS_LOG_INF("Perform a system reset\n");
	ret = fts_systemreset(info);
	if (ret) {
		VIVO_TS_LOG_INF("Cannot reset the device\n");
		goto fw_done;
	}

	fts_interrupt_set(info, INT_ENABLE);

	//change chenpeng
	ret = fts_get_ic_fw_base_version(info);
	if (ret) {
		VIVO_TS_LOG_INF("Cannot retrieve firmware base version\n");
		goto fw_done;
	}
	ret = fts_get_ic_fw_config_version(info);
	if (ret) {
		VIVO_TS_LOG_INF("Cannot retrieve firmware config version\n");
		goto fw_done;
	}

	/* if upgrade version not equal driver fw version, it means upgrade fw fail */
	if(info->ic_fw_base_version!= cur_fw_base_version || info->ic_fw_config_version!=cur_fw_config_version) {
		VIVO_TS_LOG_ERR("[%s]current firmware version not equal ic fw version.\n", __func__);
		return -1;
	}

	VIVO_TS_LOG_INF("New firmware base version:0x%04x, config version:0x%04x installed\n", info->ic_fw_base_version, info->ic_fw_config_version);

fw_done:
	/* just request firmware,need to free */
	if(request_firmware_tag == 1) {
		release_firmware(fw);
	}	

	return ret;
}
static int fts_save_tuning_value(struct fts_ts_info *info)
{
            //add crc check after save tuning value by harry
            unsigned char data[FTS_EVENT_SIZE];
            unsigned char event_id = 0;
            unsigned char tune_flag= 0;
            unsigned char error ;
            unsigned int retry ;
            unsigned char regAdd =0;
            unsigned char crc_check_error=0;
            int backup_error = 2;
            unsigned char regAdd1[3] = {0xB6, 0x00, 0x86};
            unsigned char val[4];
            unsigned char crc_status;
 
            fts_interrupt_set(info, INT_DISABLE);
            msleep(10);
            fts_command(info, FLUSHBUFFER);
 
            msleep(5);
            fts_command(info, TUNING_BACKUP);
            msleep(100);
 
            for (retry = 0; retry < 200; retry++)
            {
                        regAdd = READ_ONE_EVENT;
                        error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
                        if (error) {
                                    VIVO_TS_LOG_ERR("Cannot read device info\n");
                                    return -1;
                        }
                        VIVO_TS_LOG_INF("FTS fts statu event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                                    data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);
 
                        event_id = data[0];
                        tune_flag = data[1];
 
                        if ((event_id == 0x16) && (tune_flag == 0x04))
                        {
                                    VIVO_TS_LOG_INF("fts initialization save to flash ok \n");
                                    backup_error = 0;
                                    break;
                        }
                        else
                        {
                                    msleep(10);
                        }
            }
 
            if(backup_error != 0)
            {
                        VIVO_TS_LOG_INF("fts initialization save to flash timeout \n");
                        return -1;
            }
           
 
            fts_command(info, FLUSHBUFFER);
            msleep(10);
            VIVO_TS_LOG_INF("fts CRC check \n");
            fts_systemreset(info);
            VIVO_TS_LOG_INF("fts restart TP \n");
            msleep(200);
 
            // check CRC status
            error = fts_read_reg(info, regAdd1, sizeof(regAdd1), val, sizeof(val));
            if (error)
            {
                        VIVO_TS_LOG_ERR("Cannot read crc status\n");
                        return -1;
            }
 
            crc_status = val[1] & 0x06;
            if(crc_status != 0) // CRC error
            {
                        VIVO_TS_LOG_INF("fts CRC status = %d \n", crc_status);
                        crc_check_error = -1;
            }          
 
            enable_irq(info->client->irq);
 
            fts_interrupt_set(info, INT_ENABLE);
 
            return crc_check_error;
}
int fts_init_flash_reload(struct fts_ts_info *info)
{
	unsigned char data[FTS_EVENT_SIZE];
	int retry, error = 0;
	unsigned char event_id = 0;
	unsigned char tune_flag = 0;
	char lp_check_error = 2;
	char mutual_check_error = 2;
	char self_check_error = 2;
	unsigned char regAdd =0;
	unsigned char crc_check =0;
           
	fts_systemreset(info);
	VIVO_TS_LOG_INF("FTS:%s enter\n", __func__);  
	msleep(500);
 
	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	msleep(5);

	fts_command(info, LP_TIMER_CALIB);
	msleep(800);

	for (retry = 0; retry < READ_CNT; retry++) // poll with timeout 2200ms
	{
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) 
		{
			VIVO_TS_LOG_DBG("Cannot read device info\n");
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if((event_id == 0x16) && (tune_flag == 0x20)) {
			if((data[2] == 0x00) && (data[3] == 0x01)) {
				lp_check_error = 0;
				VIVO_TS_LOG_ERR("Low power Timer Calibration  ok \n");
			} else {
				lp_check_error = 1;
				VIVO_TS_LOG_ERR("Low power Timer Calibration  failed \n");
			}
			break;				
		} else if(retry == READ_CNT) {
			lp_check_error = 2;
		} else {
			msleep(10);
		}
	}

	if(lp_check_error != 0) {
		if(lp_check_error == 2) {
			VIVO_TS_LOG_ERR("mutual initialization timeout \n");
		}
		return -1;		
	}

	// mutual initialization
	fts_command(info, CX_TUNING);
	msleep(800);

	for (retry = 0; retry < READ_CNT; retry++){ // poll with timeout 2200ms
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
		            VIVO_TS_LOG_INF("Cannot read device info\n");
		            return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		                        data[0], data[1], data[2], data[3],
		                        data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x16) && (tune_flag == 0x01)) {
	            if((data[3] == 0x00) && (data[4] == 0x00)) {
	                        mutual_check_error = 0;
	                        VIVO_TS_LOG_INF("fts mutual initialization ok \n");
	            }else{
	                        mutual_check_error = 1;
	                        VIVO_TS_LOG_INF("fts mutual initialization fail \n");
	            }
	            break;
		}else if(retry == READ_CNT) 	{
			mutual_check_error = 2;
		} else{
		     msleep(10);
		}
	}
 
	if(mutual_check_error != 0){
	            if(mutual_check_error == 2){
	                        VIVO_TS_LOG_INF("fts mutual initialization timeout \n");
	            }
	            return -1;          
	}                      
 
	// self initialization
	fts_command(info, SELF_TUNING);
	msleep(300);
 
	for (retry = 0; retry < READ_CNT; retry++) { //poll with timeout
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
		            VIVO_TS_LOG_ERR("Cannot read device info\n");
		            return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
		                        data[0], data[1], data[2], data[3],
		                        data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x16) && (tune_flag == 0x02)) {
			if((data[3] == 0x00) && (data[4] == 0x00)) {
				self_check_error = 0;
				VIVO_TS_LOG_INF("fts self initialization ok \n");
			} else {
				self_check_error = 1;
				VIVO_TS_LOG_INF("fts self initialization fail \n");
			}
			break;  
		} else if(retry == READ_CNT) {
			self_check_error = 2;
		} else {
			msleep(10);
		}
	}

	if(self_check_error != 0){
		if(self_check_error == 2){
		            VIVO_TS_LOG_INF("fts self initialization timeout \n");
		}
		return -1;                      
	}
 
	crc_check = fts_save_tuning_value(info);

	error += fts_command(info, KEY_ON);
	error += fts_command(info, SENSEON);
	error += fts_command(info, FLUSHBUFFER);
 	error += fts_command(info, INT_ENABLE);
	if (error != 0) {
	            VIVO_TS_LOG_ERR("init flash reload err\n");
	            return -ENODEV ;
	} else if (crc_check != 0){
	            return -1;
	} else {
	            return 0;
	}
 
}

static int fts_fw_upgrade_and_init(struct fts_ts_info *info, int mode,char name[])
{
	int retval;
	int count = 0;
	int crc_err = 0;
	
	info->firmware_updating_flag = 1;

	VIVO_TS_LOG_INF("[%s]enter.step 1.update firmware.\n", __func__);  

	crc_err = cx_crc_check(info);
	if(crc_err == -1) {
		VIVO_TS_LOG_ERR("[%s]could not read crc_err from chip.\n", __func__);
		return -1;
	}

	/* try 4 times to update firmware */
	while(count < 4) {
		retval = fts_fw_upgrade(info, 2,name, crc_err);
		if(retval != 0) {
			count++;
			VIVO_TS_LOG_INF("[%s]fireware update times=%d.\n", __func__, count); 
		} else {
			break;
		}
		if(count == 4) {
			VIVO_TS_LOG_ERR("[%s]try update firmware 4 times fail.\n", __func__);
			info->firmware_updating_flag = 0;
			return -1;	//update firmware fail.
		}
	}
	
	count = 0;

	VIVO_TS_LOG_INF("[%s]step 2.judge AFE config version is same or not same.\n", __func__);  
	retval = fts_get_init_status(info); // return value 0 means initialization status correct	
	if(retval != 0) {	// initialization status not correct or after FW update, do initialization.		
		VIVO_TS_LOG_INF("[%s]AFE config version is change,doing auto tune...\n", __func__);  
		fts_chip_initialization(info);
	} else {
		VIVO_TS_LOG_INF("[%s]AFE config version is not change,not do auto tune.\n", __func__);  
	}

	/* reset firmware updating flag */
	info->firmware_updating_flag = 0;

	fts_systemreset(info);
	fts_command(info, SENSEON);
	fts_command(info, KEY_ON);
	fts_command(info, FLUSHBUFFER);
	fts_interrupt_set(info, INT_ENABLE);	
	
	VIVO_TS_LOG_INF("[%s]exit.update and init flash reload success.\n", __func__); 

	return 0;		//return 0 means update success
}

int fts_interrupt_set(struct fts_ts_info *info, int enable)
{
	unsigned char regAdd[4] = { 0xB6, 0x00, 0x1C, enable };

	return fts_write_reg(info, &regAdd[0], 4);
}


/*
 * New Interrupt handle implementation
 */



static inline unsigned char *fts_next_event(unsigned char *evt)
{

	/* Nothing to do with this event, moving to the next one */
	evt += FTS_EVENT_SIZE;
	//printk("FTS %s \n",__func__);

	/* the previous one was the last event ?  */
	return (evt[-1] & 0x1F) ? evt : NULL;
}


/* EventId : 0x00 */
static unsigned char *fts_nop_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	// GIUSEPPE dev_dbg(info->dev, "Doing nothing for event 0x%02x\n", *event);

	return fts_next_event(event);
}

#if defined (BBK_LARGE_SUPRESSION)
#define KEY_TS_LARGE_SUPPRESSION  250
static int is_calling = 0;
static int is_calling_save = 0;
static void large_square_supression(struct fts_ts_info *info)
{
	VIVO_TS_LOG_INF( "FTS: TouchScreen large_square_supression!\n");

	mutex_lock(&info->input_report_mutex);
	input_report_key(info->input_dev, KEY_TS_LARGE_SUPPRESSION, 1);
	mdelay(20);
	input_report_key(info->input_dev, KEY_TS_LARGE_SUPPRESSION, 0);
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
}
#endif

char *get_bbk_board_version(void);

/* EventId : 0x03 */
static unsigned char *fts_enter_pointer_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	unsigned char touchId;
	int x, y, z;

#if defined (BBK_LARGE_SUPRESSION)
	unsigned char touch_size;
	unsigned char touch_num;
#endif

	VIVO_TS_LOG_DBG("[%s]Received event 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
		__func__, event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);

	//step 1.cal event ID from event data
	touchId = event[1] & 0x0F;
	__set_bit(touchId, &info->touch_id);

	//step 2.cal x/y position from event data
	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);
	z = (event[5] & 0x3F);
	if (x == X_AXIS_MAX) {
		x--;
	}
	if (y == Y_AXIS_MAX) {
		y--;
	}

	//step 3.output finger press log
	info->finger_press_record[touchId][0] = x;		//x
	info->finger_press_record[touchId][1] = y;		//y
	//just print log while first report this finger
	info->finger_press_record[touchId][2]++;		//report rate=80,so 80*60*60=288000timers/hour
	if(info->finger_press_record[touchId][2] == 1){
		VIVO_TS_LOG_INF("[%s]TouchInfo[%d][%d][%d][%d]\n", __func__, touchId, 1,
			info->finger_press_record[touchId][0], info->finger_press_record[touchId][1]);
	}

#if defined (BBK_LARGE_SUPRESSION)
	touch_num = (event[1] >>4) & 0x0F;
	info->touch_flag = touch_num;

	//0d to 2d and 2d to 0d
	if(info_gl->buttons > 0) {
		info->aa_current_touch_num = touch_num;
	}
	
	if(touch_num >2){
		VIVO_TS_LOG_DBG("FTS: touch_num=%d\n", touch_num);
		info->is_3_finger_touch = 1;
	} else {
		info->is_3_finger_touch = 0;
	}

	touch_size=(event[5] >>6) & 0x03;
	VIVO_TS_LOG_DBG("[%s]touch_num=%d,touch_size=%d,touch_id=%d\n", 
				__func__, touch_num, touch_size, touchId);

	VIVO_TS_LOG_DBG("large_press_touch_id=%d is_large_press_mode=%d touch_size=%d touch_num=%d\n", info->large_press_touch_id, info->is_large_press_mode, touch_size, touch_num);
	if(touch_size == 0x03 && info->is_large_press_mode==0) {
		release_point_and_key(info);
		info->is_large_press_mode = 1;
		info->large_press_touch_id = touchId;
		VIVO_TS_LOG_INF("[%s]Into large press mode.\n", __func__);		
	}

	VIVO_TS_LOG_DBG("is_calling == %d \n",is_calling);
	if(is_calling== 1)
	{
		VIVO_TS_LOG_INF("is_3_finger_touch == %d \n", info->is_3_finger_touch);
		if(info->is_3_finger_touch == 1 || info->is_large_press_mode)
		{
			VIVO_TS_LOG_INF("report large press event\n");
			large_square_supression(info);
			is_calling = 0;
			return fts_next_event(event);
		}
	}

	if(info->is_large_press_mode == 1) {		
		return fts_next_event(event);
	}
#endif
	mutex_lock(&info->input_report_mutex);
	input_mt_slot(info->input_dev, touchId);
	input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, 1);
	//input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, touchId);	//no tracking is ok?
	if(0 == strcmp(get_bbk_board_version(),"A1")) {
		input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(info->input_dev, ABS_MT_POSITION_Y, Y_AXIS_MAX-y);
	} else {
		input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	}
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
	VIVO_TS_LOG_DBG("FTS board version is %s \n", get_bbk_board_version());
	VIVO_TS_LOG_DBG("FTS %s Event 0x%02x - ID[%d], (x, y, z) = (%3d, %3d, %3d)\n",__func__,
			 *event, touchId, x, y, z);

	return fts_next_event(event);
}


/* EventId : 0x04 */
static unsigned char *fts_leave_pointer_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	unsigned char touchId;
	unsigned char touch_num;
	if(info->log_switch == 1)
	VIVO_TS_LOG_DBG("[%s]Received event 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", 
		__func__, event[0], event[1], event[2], event[3], event[4], event[5], event[6], event[7]);

	touchId = event[1] & 0x0F;
	touch_num = (event[1] >>4) & 0x0F;	

	if(touch_num == 0) {		//while all finger up,record the jiffies
		info->aa_up_time = jiffies;			
	}
	info->touch_flag = touch_num;

	if(touchId == info->large_press_touch_id) {
		info->large_press_touch_id = -1;
		info->is_large_press_mode = 0;
		is_calling = is_calling_save;
		release_point(info);
	}
	/* must release this,else is_calling is always 0 */
	info->is_3_finger_touch = 0;
	is_calling = is_calling_save;

	info->finger_press_record[touchId][2] = 0;		//clear count,report rate=80,so 80*60*60=288000timers/hour
	VIVO_TS_LOG_INF("[%s]TouchInfo[%d][%d][%d][%d]\n", __func__, touchId, 0,
			info->finger_press_record[touchId][0], info->finger_press_record[touchId][1]);
	
	__clear_bit(touchId, &info->touch_id);

	if(touch_num == 0) {
		release_point(info);
	}

	mutex_lock(&info->input_report_mutex);
	input_mt_slot(info->input_dev, touchId);
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
	VIVO_TS_LOG_DBG("FTS Event 0x%02x - release ID[%d]\n", event[0], touchId);

	return fts_next_event(event);
}

/* EventId : 0x05 */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler


/* EventId : 0x07 */
static unsigned char *fts_hover_enter_pointer_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	unsigned char touchId;
	int x, y, z;
	if(info->log_switch == 1)
	VIVO_TS_LOG_DBG("FTS Received event 0x%02x\n", event[0]);

	touchId = event[1] & 0x0F;

	__set_bit(touchId, &info->touch_id);

	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);
#define HOVER_ENTER_Z_VALUE 0
	z = HOVER_ENTER_Z_VALUE;

	if (x == X_AXIS_MAX)
		x--;

	if (y == Y_AXIS_MAX)
		y--;

	mutex_lock(&info->input_report_mutex);
	input_mt_slot(info->input_dev, touchId);
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, touchId);
	input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);	
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
	if(info->log_switch == 1)
		VIVO_TS_LOG_DBG("Event 0x%02x - ID[%d], (x, y, z) = (%3d, %3d, %3d)\n",
				event[0], touchId, x, y, z);

	return fts_next_event(event);
}


/* EventId : 0x08 */
#define fts_hover_leave_pointer_event_handler fts_enter_pointer_event_handler


/* EventId : 0x09 */
#define fts_hover_motion_pointer_event_handler fts_leave_pointer_event_handler


/* EventId : 0x0B */
static unsigned char *fts_proximity_enter_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	unsigned char touchId;
	int x, y, z;

	if(info->log_switch == 1)
	VIVO_TS_LOG_DBG("Received event 0x%02x\n", event[0]);

	touchId = event[1] & 0x0F;

	__set_bit(touchId, &info->touch_id);

	x = X_AXIS_MAX / 2;
	y = Y_AXIS_MAX / 2;
#define PROXIMITY_ENTER_Z_VALUE 0
	z = PROXIMITY_ENTER_Z_VALUE;

	mutex_lock(&info->input_report_mutex);
	input_mt_slot(info->input_dev, touchId);
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, touchId);
	input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);

	if(info->log_switch == 1)
	VIVO_TS_LOG_DBG("Event 0x%02x - ID[%d], (x, y, z) = (%3d, %3d, %3d)\n",
				event[0], touchId, x, y, z);

	return fts_next_event(event);
}


/* EventId : 0x0C */
#define fts_proximity_leave_event_handler fts_leave_pointer_event_handler


static void home_work_handler(struct work_struct *work) {
	VIVO_TS_LOG_INF	("home_work_handler run!\n");
	mutex_lock(&info_gl->input_report_mutex);
	if(info_gl->aa_current_touch_num>0) {//info_gl->buttons & (1 << 9) 
		input_event(info_gl->input_dev,EV_KEY, KEY_HOMEPAGE,-1);
		VIVO_TS_LOG_INF("home key give up\n");
	} else {
		input_event(info_gl->input_dev,EV_KEY, KEY_HOMEPAGE,0);
		VIVO_TS_LOG_INF("home key up\n");
	}
	input_sync(info_gl->input_dev);
	mutex_unlock(&info_gl->input_report_mutex);
	info_gl->aa_current_touch_num = 0;
	info_gl->buttons &= (~(1<<9));
}
static void home_timer_func(unsigned long data) {
	schedule_work(&info_gl->home_work);
}
static void back_work_handler(struct work_struct *work) {
	VIVO_TS_LOG_INF	("back_work_handler run!\n");

	mutex_lock(&info_gl->input_report_mutex);
	if(info_gl->aa_current_touch_num>0)	 {//info_gl->buttons & (1 << 8) || 	
		input_event(info_gl->input_dev,EV_KEY, KEY_BACK,-1);
	} else {
		input_event(info_gl->input_dev,EV_KEY, KEY_BACK, 0);
	}
	input_sync(info_gl->input_dev);
	mutex_unlock(&info_gl->input_report_mutex);
	info_gl->aa_current_touch_num = 0;
	info_gl->buttons &= (~(1<<8));
}
static void back_timer_func(unsigned long data) {
	schedule_work(&info_gl->back_work);
}
static void menu_work_handler(struct work_struct *work) {
	VIVO_TS_LOG_INF	("menu_work_handler run! touch_num=%d \n", info_gl->aa_current_touch_num);
	mutex_lock(&info_gl->input_report_mutex);
	if(info_gl->aa_current_touch_num>0) {
		input_event(info_gl->input_dev,EV_KEY, KEY_MENU,-1);
		VIVO_TS_LOG_INF("menu key give up\n");
	} else {
		input_event(info_gl->input_dev,EV_KEY, KEY_MENU,0);
		VIVO_TS_LOG_INF("meun key up\n");
	}
	input_sync(info_gl->input_dev);
	mutex_unlock(&info_gl->input_report_mutex);
	info_gl->aa_current_touch_num = 0;
	info_gl->buttons &= (~(1<<10));	
}
static void menu_timer_func(unsigned long data) {
	schedule_work(&info_gl->menu_work);
}

/* EventId : 0x0E */
static unsigned char *fts_button_status_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{
	unsigned int buttons, changed;
	long long jiffies_0d_down;

	VIVO_TS_LOG_DBG("Received event 0x%02x\n", event[0]);

	/* get current buttons status */
	buttons = event[1] | (event[2] << 8);
	
	if(buttons ==256 || buttons==512 || buttons==1024) {	//is key down event
		jiffies_0d_down = jiffies;
		info->aa_2_2d_diff_time = jiffies_0d_down - info->aa_up_time;	//cal diff time between aa area release last finger and 0d area button down.
		if(info->aa_2_2d_diff_time < HZ/5 || (info->touch_flag > 0)) {	//diff is to short,
		VIVO_TS_LOG_INF("FTS:button=%x, info_button=%x, %lld-%lld, touch_flag:%d\n", buttons, info->buttons, jiffies_0d_down, info->aa_up_time, info->touch_flag);

			//VIVO_TS_LOG_INF("aa to 0d no report\n");
			return fts_next_event(event);
		}
	}

	VIVO_TS_LOG_INF("time:diff=%lld = %lld-%lld\n", info->aa_2_2d_diff_time, (long long)jiffies, info->aa_up_time);	

	/* check what is changed */
	changed = buttons ^ info->buttons;
	VIVO_TS_LOG_INF("FTS:change=%x, button=%x, info_button=%x\n", changed, buttons, info->buttons);

	if(changed & (1<<9)) {
		if(buttons & (1 << 9)) {	//pressed  home
			VIVO_TS_LOG_INF("home key pressed\n");
			mutex_lock(&info->input_report_mutex);
			input_event(info->input_dev,EV_KEY, KEY_HOMEPAGE,1);
			input_sync(info->input_dev);
			mutex_unlock(&info->input_report_mutex);
			info->buttons |= (1 << 9);		//must set it own bit,void to clean other two 0d key
		} else {
			VIVO_TS_LOG_INF("home key up but not report\n");
			mod_timer(&info->home_timer, jiffies + msecs_to_jiffies(50));
		}
	}
	if(changed & (1<<10)) {
		if(buttons & (1 << 10)) {	//pressed
			VIVO_TS_LOG_INF("menu key pressed\n");
			mutex_lock(&info->input_report_mutex);
			input_event(info->input_dev,EV_KEY, KEY_MENU,1);
			input_sync(info->input_dev);
			mutex_unlock(&info->input_report_mutex);
			info->buttons |= (1 << 10);
		} else {
			VIVO_TS_LOG_INF("menu key up but not report\n");
			mod_timer(&info->menu_timer, jiffies + msecs_to_jiffies(50));	
		}
	}
	if(changed & (1<<8)) { 	
		if(buttons & (1 << 8)) {	//pressed
			VIVO_TS_LOG_INF("back key pressed\n");
			mutex_lock(&info->input_report_mutex);
			input_event(info->input_dev,EV_KEY, KEY_BACK,1);	
			input_sync(info->input_dev);
			mutex_unlock(&info->input_report_mutex);
			info->buttons |= (1 << 8);
		} else {
			VIVO_TS_LOG_INF("back key up but not report\n");
			mod_timer(&info->back_timer, jiffies + msecs_to_jiffies(50));	
		}
	}

	VIVO_TS_LOG_DBG("Event 0x%02x -  SS = 0x%02x, MS = 0x%02x\n",
				event[0], event[1], event[2]);

	return fts_next_event(event);
}

//change name to power on reset
void  fts_power_reset(struct fts_ts_info *info)
{
	int error=0;

	/* while firmware updating, no power opr */
	if(info->firmware_updating_flag) {
		VIVO_TS_LOG_INF("[%s]firmware is updating.no power opr\n", __func__);	
		return ;
	}

	mutex_lock(&info->i2c_reset_mutex);

	//power down and power on
	error = fts_power_on(0);
	if(error){
		VIVO_TS_LOG_ERR("[%s]fts_power_off error\n", __func__);
	}
	msleep(10);
	error = fts_power_on(1);
	if(error){
		VIVO_TS_LOG_ERR("[%s]fts_power_off error\n", __func__);
	}
	msleep(200);

	mutex_unlock(&info->i2c_reset_mutex);
#if 0
	//config rst pin for hw reset
	VIVO_TS_LOG_INF("[%s]step 1.hard reset by reset gpio.", __func__);
	if (gpio_is_valid(data->rst_gpio)) {
		error = gpio_direction_output(data->rst_gpio, 1);
		if (error) {
			VIVO_TS_LOG_ERR("unable to set direction out to 0 for gpio [%d]\n",
				data->rst_gpio);
		}
		msleep(10);

		error = gpio_direction_output(data->rst_gpio, 0);
		if (error) {
			VIVO_TS_LOG_ERR("unable to set direction out to 0 for gpio [%d]\n",
				data->rst_gpio);
		}
		msleep(50);

		error = gpio_direction_output(data->rst_gpio, 1);
		if (error) {
			VIVO_TS_LOG_ERR("unable to set direction out to 1 for gpio [%d]\n",
				data->rst_gpio);
		}
		msleep(200);
	}
#endif

	if (error)
		VIVO_TS_LOG_ERR("[%s]power down reset fail. (#errors = %d)\n", __func__, error);
}

/* send esd protect cmd to change internal circuit */
static int fts_esd_protect(struct fts_ts_info *info)
{
	int ret;
	unsigned char regAdd[2] = {0xc1, 0x10};

	VIVO_TS_LOG_INF("[%s]send  esd protect cmd.\n", __func__);
	ret = fts_write_reg(info, regAdd, sizeof(regAdd));
	if(ret) {
		VIVO_TS_LOG_ERR("[%s]send  esd protect cmd fail.\n", __func__);
	}
	msleep(5);
	
	return ret;
}

/* EventId : 0x0F */
static unsigned char *fts_error_event_handler(struct fts_ts_info *info,
			unsigned char *event)
{

	unsigned char event_err;
	
	int rehandle_esd_count = 0;
	int ret = 0;

	VIVO_TS_LOG_INF("[%s]step 1.get error event data:%02X %02X %02X %02X %02X %02X %02X %02X\n", __func__,
			event[0], event[1], event[2], event[3],
			event[4], event[5], event[6], event[7]);

	/* get event id*/
	event_err = event[1] ;

	VIVO_TS_LOG_INF("[%s]step 2.Prase error event,error event number is:0x%x\n", __func__, event_err);
	if(event_err==0x0A) {	//0x0A is ESD check error
rehandle_esd_error:		
		if(rehandle_esd_count > 0) {
			VIVO_TS_LOG_INF("[%s]rehandle esd error!rehandle_esd_count=%d.ret=%d\n", __func__, rehandle_esd_count, ret);
		}			
		ret = 0;
		VIVO_TS_LOG_INF("[%s]step 2_1.release finger.\n", __func__);
		release_point_and_key( info);
		/* must release large press mode,else tp maybe no function */
		info->large_press_touch_id = -1;	
		info->is_large_press_mode = 0;

		VIVO_TS_LOG_INF("[%s]step 2_2.power off and on.then, do hardware reset by reset pin.\n", __func__);
		msleep(500);
		fts_power_reset(info);
		ret += fts_systemreset(info);
		
		VIVO_TS_LOG_INF("[%s]step 2_3.set chip into right state.ts_state:%d\n", 
			__func__, atomic_read(&info->ts_state));

		//lcd on,rewrite glove mode switch
		if(atomic_read(&info->ts_state)==TOUCHSCREEN_NORMAL) {
			if(info->glove_mode_switch==1) {
				ret += fts_glove_mode_switch_rewrite_tochip(info);
			}
			ret += bbk_rewrite_usb_charger_flag_tochip(info);
			/* no need to rewrite usb charge status */
			ret += fts_command(info, SENSEON);
			msleep(10);
			ret += fts_command(info, KEY_ON);
			msleep(10);			
		}
		
		//lcd off,rewrite gesture switch and custon template,then,enter gesture mode
		if((atomic_read(&info->ts_state)==TOUCHSCREEN_GESTURE)) {
			ret += fts_command(info, SENSEON);
			msleep(10);
			ret += fts_custom_gesture_template_rewrite_tochip(info);
			ret += fts_gesture_switch_rewrite_tochip(info);
			ret += fts_check_into_gesture_mode(info);	
			/* send esd protect cmd to change internal circuit */
			//ret += fts_esd_protect(info);
		}
		ret += fts_interrupt_set(info, INT_ENABLE);	

		if(ret < 0) {
			rehandle_esd_count++;
			if(rehandle_esd_count > 2) {
				VIVO_TS_LOG_ERR("[%s]3 times ESD reset fail.\n", __func__);
			} else {
				msleep(50);
				goto rehandle_esd_error;
			}
		}

		/* release for ESD remain points */
		release_point_and_key( info);

		/* clear event save in driver,aviod to solve event saved in driver after power down reset */
		event[7] = 0;
		VIVO_TS_LOG_INF("[%s]event[7] = %d\n", __func__, event[7]);
	}

	return fts_next_event(event);
}


/* EventId : 0x10 */
static unsigned char *fts_controller_ready_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	VIVO_TS_LOG_DBG("FTS fts Controller ready event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			event[0], event[1], event[2], event[3],
			event[4], event[5], event[6], event[7]);

	info->touch_id = 0;
	info->buttons = 0;
	mutex_lock(&info->input_report_mutex);
	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);
	return fts_next_event(event);
}


/* EventId : 0x11 ,this event given up*/
static unsigned char *fts_sleepout_controller_ready_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	VIVO_TS_LOG_INF("FTS fts sleep out ready event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			event[0], event[1], event[2], event[3],
			event[4], event[5], event[6], event[7]);

	complete(&info->cmd_done);
	return fts_next_event(event);
}


/* EventId : 0x16 */
static unsigned char *fts_status_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			event[0], event[1], event[2], event[3],
			event[4], event[5], event[6], event[7]);

	switch (event[1]) {
	case FTS_STATUS_MUTUAL_TUNE:
	case FTS_STATUS_SELF_TUNE:
	case FTS_FORCE_CAL_SELF_MUTUAL:
		complete(&info->cmd_done);
		break;

	case FTS_FLASH_WRITE_CONFIG:
	case FTS_FLASH_WRITE_COMP_MEMORY:
	case FTS_FORCE_CAL_SELF:
	case FTS_WATER_MODE_ON:
	case FTS_WATER_MODE_OFF:
	default:
		VIVO_TS_LOG_DBG("Received unhandled status event = 0x%02x\n", event[1]);
		break;
	}

	return fts_next_event(event);
}


/* EventId : 0x05 */
#define fts_motion_pointer_event_handler fts_enter_pointer_event_handler

void fts_input_report_key(struct fts_ts_info *info, int key_code) 
{
	mutex_lock(&info->input_report_mutex);
	input_report_key(info->input_dev, key_code, 1);
	input_sync(info->input_dev);	/* must be sync,else upmenu maybe popdown  */
	VIVO_TS_LOG_INF("[%s]keycode:%d down", __func__, key_code);
	//mdelay(10);
	input_report_key(info->input_dev, key_code, 0);
	input_sync(info->input_dev);
	VIVO_TS_LOG_INF("[%s]keycode:%d up", __func__, key_code);
	mutex_unlock(&info->input_report_mutex);	
}

/* EventId : 0x22 */
unsigned short gesture_data[70];     // coordinate date
unsigned char  gesture_index;        // index

static unsigned char *fts_gesture_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	unsigned char touchId;
	unsigned char error;
	unsigned char regAdd[3] = {0xd0,0x00,0x00};
	unsigned char val[121];
	unsigned short buffer;
	int x = 0;
	int y = 0;
	int z = 0;
	static int x_off, y_off;
	int i = 0;
	int j = 0;
	int len = 0;
	int interval = 0;
	int remainder = 0;
	unsigned char *pval = NULL;
	unsigned short max_x = 0, min_x = 0xffff, max_y = 0x00, min_y = 0xffff;
	

	VIVO_TS_LOG_INF("FTS fts gesture event data : %02X %02X %02X %02X %02X %02X %02X\n",
				 event[0], event[1], event[2], event[3], event[4], event[5], event[6]);

	/* get time of this gesture event */
	info->new_gesture_time = jiffies;
	
	wake_lock_timeout(&info->wakelock, 2*HZ);	//get wake_lock

	for(i=0;i<70;i++)
		gesture_data[i]=0xFFFF;

	if(event[1] == 0x03)
	{
		//TODO: event[2] is the gesture id; event[3] mean enable/disable
		
	}
	if(event[1] == 0x04)
	{
		if(event[2] == 0x00 && event[3] == 0x01)
		{
			//event[4] shows the status of cmd enable specified gestures
			VIVO_TS_LOG_INF("event:gesture enable event:0x%x", event[4]);
		}
		if(event[2] == 0x00 && event[3] == 0x02)
		{
			//event[4] shows the status of cmd disable specified gestures
			VIVO_TS_LOG_INF("event:gesture disable event:0x%x", event[4]);
		}
		if(event[3] == 0x10)
		{
			//event[2] shows the gesture id
			//event[4] shows the status of cmd start adding custon gesture
			info->gesture_index = event[2] - 0x11;
			info->gesture_status = event[4];
		}
		if(event[3] == 0x11)
		{
			//event[2] shows the gesture id
			//event[4] shows the status of cmd add custom gesture data
			info->gesture_index = event[2] - 0x11;
			info->gesture_status = event[4];
		}
		if(event[3] == 0x12)
		{
			//event[2] shows the gesture id
			//event[4] shows the status of cmd finish adding custom gesture
			info->gesture_index = event[2] - 0x11;
			info->gesture_status = event[4];
		}
	}
	
	if(event[1] == 0x02) {
  		fts_interrupt_set(info, INT_DISABLE);
  		gesture_index = event[1] + 0x6F;  // Gesture index value
		msleep(2);
		regAdd[1] = event[4];    // Ofset address
		regAdd[2] = event[3];    // Ofset address
		len = event[6];
		len = (len << 8) | event[5];
		error = fts_read_reg(info, regAdd, 3, (unsigned char *)val, 1 + (len<<2));
		if (error) {
			VIVO_TS_LOG_ERR("%s Cannot write reg\n",__func__);
		}
	
		VIVO_TS_LOG_DBG("FTS fts gesture coordinate address : %02X %02X %02X len = %02X\n", regAdd[0], regAdd[1], regAdd[2], len); // add by terry 150312 for default enable user defind gesture

		//------------------------------------------
		pval = val + 1;

		if(event[2] != 0x11) {	//default gesture
			interval = (len / 9)<<1;
			remainder = len % 9;
			
			for(i = 0, j = 0;i < 18;i=i+2 ) {  
				buffer = (pval[j+1] &0x0f);
				gesture_data[i] = (buffer<<8)|pval[j];

				/* add for gesture size judge */
				if(min_x > gesture_data[i]) {
					min_x = gesture_data[i];
				} 
				if(max_x < gesture_data[i]) {
					max_x = gesture_data[i];
				}
				
				buffer = (pval[j+1+len*2] &0x0f);
				gesture_data[i+1] = (buffer<<8)|pval[j+len*2];

				/* add for gesture size judge */
				if(min_y > gesture_data[i+1]) {
					min_y = gesture_data[i+1];
				} 
				if(max_y < gesture_data[i+1]) {
					max_y = gesture_data[i+1];
				}
				
				if((i >> 1) < (remainder+1)) {
					j = j + interval + 2;
				} else {
					j = j + interval;
				}
	        	}
			gesture_data[18] = 0xffff;
			gesture_data[19] = 0xffff;
		}else {
			for(i = 0, j = 0;i < 60;i=i+2,j=j+2 ) {  	//user define gesture point : 60 point 120bytes
				buffer = (pval[j+1] &0x0f);
				gesture_data[i] = (buffer<<8)|pval[j];

				/* add for gesture size judge */
				if(min_x > gesture_data[i]) {
					min_x = gesture_data[i];
				} 
				if(max_x < gesture_data[i]) {
					max_x = gesture_data[i];
				}
				
				buffer = (pval[j+1+len*2] &0x0f);
				gesture_data[i+1] = (buffer<<8)|pval[j+len*2];

				/* add for gesture size judge */
				if(min_y > gesture_data[i+1]) {
					min_y = gesture_data[i+1];
				} 
				if(max_y < gesture_data[i+1]) {
					max_y = gesture_data[i+1];
				}
				
	        	}
			gesture_data[60] = 0xffff;
			gesture_data[61] = 0xffff;
		}
	      fts_interrupt_set(info, INT_ENABLE);
	}
	/* always use touchId zero */
	touchId = 0;
	__set_bit(touchId, &info->touch_id);

	if (++x_off > 20)
		x_off = 0;

	if (++y_off > 20)
		y_off = 0;


	if(event[1] == 0x01 || event[1] == 0x02) {
		/* if gesture size less than 1.5cm, no report gesture.dclick no effect*/
		if(((max_x-min_x<326) && (max_y-min_y<326)) && (event[2]!=0x01)) {
			VIVO_TS_LOG_INF("[%s]gesture size less than 1.5cm,no report.size_x:%d size_y:%d\n", __func__, max_x-min_x, max_y-min_y);
			goto gesture_done;
		}
		
		/* while diff time much than 500ms,report gesture normal
		 * if left of right gesture,because no wakeup and must switch music quickly,so,no need to control it.
		 */
		if((info->new_gesture_time - info->old_gesture_time > HZ/2) || (event[2]  == 0x07) || (event[2]  == 0x08)) {
			info->old_gesture_time = info->new_gesture_time;
			VIVO_TS_LOG_DBG("[%s]gesture report right time.\n", __func__);
		} else {
			event[2] = 0;
		}

		if(event[2]  == 0x02 && info->gesture_switch & 0X04) {		
			/*bbk add 02-->O*/
			fts_input_report_key(info, KEY_O);
			VIVO_TS_LOG_INF("detect gesture \"O\"!\n");
		}
		if(event[2]  == 0x03&& info->gesture_switch & 0X40) {		
			/*bbk add 03-->C*/
			fts_input_report_key(info, KEY_C);
			VIVO_TS_LOG_INF("detect gesture \"C\"!\n");
		}
		if(event[2]  == 0x04&& info->gesture_switch & 0X10) {
			/*bbk add 04-->M*/
			fts_input_report_key(info, KEY_M);
			VIVO_TS_LOG_INF("detect gesture \"M\"!\n");
		}
		if(event[2]  == 0x05&& info->gesture_switch & 0X08) {
			/*bbk add 05-->W*/
			fts_input_report_key(info, KEY_W);
			VIVO_TS_LOG_INF("detect gesture \"W\"!\n");
		}
		if(event[2]  == 0x06&& info->gesture_switch & 0X20) {
			/*bbk add 06-->E*/
			fts_input_report_key(info, KEY_E);
			VIVO_TS_LOG_INF("detect gesture \"E\"!\n");
		}
		if(event[2]  == 0x0a&& info->gesture_switch & 0X02) {
			/*bbk add 0a-->UP*/
			fts_input_report_key(info, KEY_UP);
			VIVO_TS_LOG_INF("detect gesture \"UP\"!\n");
		}
		if(event[2]  == 0x09&& info->swipe_switch == 1) {
			/*bbk add 09-->down*/
			fts_input_report_key(info, KEY_WAKEUP_SWIPE);		
			VIVO_TS_LOG_INF("detect gesture \"DOWN\"!\n");
		}
		if(event[2]  == 0x07&& info->gesture_switch & 0X01) {
			/*bbk add 07-->right */
			fts_input_report_key(info, KEY_RIGHT);
			VIVO_TS_LOG_INF("detect gesture \"RIGHT\"!\n");
		}
		if(event[2]  == 0x08&& info->gesture_switch & 0X01) {
			/*bbk add 08-->left */
			fts_input_report_key(info,  KEY_LEFT);
			VIVO_TS_LOG_INF("detect gesture \"LEFT\"!\n");
		}
		if(event[2]  == 0x01&& info->dclick_switch == 1) {
			/*bbk add 01-->double click*/
			fts_input_report_key(info, KEY_WAKEUP);
			VIVO_TS_LOG_INF("detect gesture \"Double Click\"!\n");
		}
		if(event[2]  == 0x11 && info->user_define_gesture_switch== 1) {
			/*user define gesture 1*/
			fts_input_report_key(info, KEY_CUSTOM_GESTURE);
			VIVO_TS_LOG_INF("detect gesture \"Custom Gesture\"!\n");
		}		

gesture_done:
	/* Done with gesture event, clear bit. */
		__clear_bit(touchId, &info->touch_id);
	}
	VIVO_TS_LOG_INF("[%s]Event 0x%02x - ID[%d], (x, y, z) = (%3d, %3d, %3d)\n", __func__, event[0], touchId, x, y, z);

	return fts_next_event(event);
}


/* EventId : 0x23 */
static unsigned char *fts_pen_enter_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	unsigned char touchId;
	int x, y, z;
	int eraser, barrel;

	VIVO_TS_LOG_DBG("Received event 0x%02x\n", event[0]);

	/* always use last position as touchId */
	touchId = TOUCH_ID_MAX;

	__set_bit(touchId, &info->touch_id);

	x = (event[2] << 4) | (event[4] & 0xF0) >> 4;
	y = (event[3] << 4) | (event[4] & 0x0F);
	z = (event[5] & 0xFF);

	eraser = (event[1] * 0x80) >> 7;
	barrel = (event[1] * 0x40) >> 6;

	if (x == X_AXIS_MAX)
		x--;

	if (y == Y_AXIS_MAX)
		y--;

	mutex_lock(&info->input_report_mutex);
	input_mt_slot(info->input_dev, touchId);
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, touchId);
	input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, z);
	input_report_abs(info->input_dev, ABS_MT_PRESSURE, z);

	input_report_key(info->input_dev, BTN_STYLUS, eraser);
	input_report_key(info->input_dev, BTN_STYLUS2, barrel);
	input_mt_report_slot_state(info->input_dev, MT_TOOL_PEN, 1);

	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);

	VIVO_TS_LOG_DBG("Event 0x%02x - ID[%d], (x, y, z) = (%3d, %3d, %3d)\n",
				event[0], touchId, x, y, z);

	return fts_next_event(event);
}


/* EventId : 0x24 */
static unsigned char *fts_pen_leave_event_handler(
			struct fts_ts_info *info, unsigned char *event)
{
	unsigned char touchId;

	dev_dbg(info->dev, "Received event 0x%02x\n", event[0]);

	/* always use last position as touchId */
	touchId = TOUCH_ID_MAX;

	__clear_bit(touchId, &info->touch_id);

	mutex_lock(&info->input_report_mutex);
	input_report_key(info->input_dev, BTN_STYLUS, 0);
	input_report_key(info->input_dev, BTN_STYLUS2, 0);

	input_mt_slot(info->input_dev, touchId);
	input_report_abs(info->input_dev, ABS_MT_TRACKING_ID, -1);

	input_sync(info->input_dev);
	mutex_unlock(&info->input_report_mutex);

	VIVO_TS_LOG_DBG("Event 0x%02x - release ID[%d]\n",
		event[0], touchId);

	return fts_next_event(event);
}


/* EventId : 0x25 */
#define fts_pen_motion_event_handler fts_pen_enter_event_handler

/*Simulate dclick start */
#define TRIP_X_AREA	50
#define TRIP_Y_AREA	50
#define DCLICK_TWO_FINGER_AREA_X 100
#define DCLICK_TWO_FINGER_AREA_Y 100
static void wakeup_system_dclick(struct  fts_ts_info *sensor)
{
	mutex_lock(&sensor->input_report_mutex);
	input_report_key(sensor->input_dev, KEY_WAKEUP, 1);
	mdelay(10);
	input_report_key(sensor->input_dev, KEY_WAKEUP, 0);
	input_sync(sensor->input_dev);
	mutex_unlock(&sensor->input_report_mutex);
}

static void rmi_f11_dclick_timer_work_func(struct work_struct *work)
{
	struct fts_ts_info *sensor = container_of(work, struct fts_ts_info,
													dclick_timer_work);
	if(!sensor){
		VIVO_TS_LOG_ERR("FTS %s:Fail to get info data.\n",__func__);
		return ;
	}

	if (sensor->is_dclick_valide) {
		if (sensor->pressed_count >= 2 && sensor->release_count >= 2)
			wakeup_system_dclick(sensor);
	}

	sensor->is_dclick_valide = false;
	sensor->pressed_count = 0;
	sensor->release_count = 0;
	sensor->first_press_x = -1;
	sensor->first_press_y = -1;
	sensor->second_press_x = -1;
	sensor->second_press_y = -1;
}

static void rmi_f11_dclick_timer_func(unsigned long data)
{
	struct fts_ts_info *sensor = info_gl;
	if(!sensor){
		VIVO_TS_LOG_ERR("FTS %s:Fail to get info data.\n",__func__);
		return ;
	}

	VIVO_TS_LOG_ERR( "FTS %s: dclick timer time out\n", __func__);
	VIVO_TS_LOG_ERR( "FTS %s: is this time dclick valide -- %s\n",
				__func__, sensor->is_dclick_valide?"yes":"no");
	VIVO_TS_LOG_ERR( "FTS %s: pressed_count(%d) release_count(%d)\n",
				__func__, sensor->pressed_count, sensor->release_count);
	VIVO_TS_LOG_ERR( "FTS %s: first_press_x(%d) first_press_y(%d)\n",
				__func__, sensor->first_press_x, sensor->first_press_y);
	VIVO_TS_LOG_ERR( "FTS %s: second_press_x(%d) second_press_y(%d)\n",
				__func__, sensor->second_press_x, sensor->second_press_y);

	sensor->has_dclick_timer_start = false;
	if(sensor->is_dclick_valide != false){
		schedule_work(&sensor->dclick_timer_work);
	}

}

static void rmi_f11_cancel_dclick_trace(struct  fts_ts_info *sensor)
{
	if(!sensor){
		VIVO_TS_LOG_ERR("FTS %s:Fail to get info data.\n",__func__);
		return ;
	}
	sensor->has_dclick_timer_start = false;
	sensor->is_dclick_valide = false;
	sensor->pressed_count = 0;
	sensor->release_count = 0;
	sensor->first_press_x = -1;
	sensor->first_press_y = -1;
	sensor->second_press_x = -1;
	sensor->second_press_y = -1;
	del_timer_sync(&sensor->dclick_timer);
}

static bool rmi_f11_whether_point_int_dclick_dimension(struct  fts_ts_info *sensor,
				int point_x, int point_y)
{
	if (point_x < sensor->dclick_dimension_x_min
				|| point_x > sensor->dclick_dimension_x_max
				|| point_y < sensor->dclick_dimension_y_min
				|| point_y > sensor->dclick_dimension_y_max) {
		return false;
	}

	return true;
}

static void fts_judge_dclick(struct  fts_ts_info *sensor,int x,int y)
{
	int pre_state = sensor->pre_state;
//	int x, y,z;
	bool whether_point_legal;
	int finger_0_state = sensor->finger_state;
	VIVO_TS_LOG_ERR( "FTS %s: enter\n", __func__);

	if (sensor->num_fingers > 1) {
		VIVO_TS_LOG_INF( "FTS %s: More than one finger pressed on the TP\n", __func__);
		if (sensor->has_dclick_timer_start)
			rmi_f11_cancel_dclick_trace(sensor);
		return;
	}

	if (!pre_state && !finger_0_state) {
		/* Invalide event nothing to do */
		VIVO_TS_LOG_ERR( "FTS %s: nothing to do.\n", __func__);
	} else if (!pre_state && finger_0_state) {
		VIVO_TS_LOG_ERR( "%s: down state.\n", __func__);
		whether_point_legal = rmi_f11_whether_point_int_dclick_dimension(sensor,
										x, y);
		if (!whether_point_legal) {
			if (sensor->has_dclick_timer_start) {
				VIVO_TS_LOG_INF( "FTS %s: The point not in dclick dimension cancel trace\n",
						__func__);
				rmi_f11_cancel_dclick_trace(sensor);
			}else{
				VIVO_TS_LOG_ERR( "FTS %s: The point not in dclick dimension nothing to do\n",
						__func__);
			}

			return;
		}
		/* the first down event of one time press */
		if (!sensor->has_dclick_timer_start) {
			sensor->first_press_x = x;
			sensor->first_press_y = y;
			sensor->pressed_count++;

			VIVO_TS_LOG_ERR( "FTS %s: first press start timer\n", __func__);
			mod_timer(&sensor->dclick_timer,
					jiffies + msecs_to_jiffies(500));
			sensor->has_dclick_timer_start = true;
			sensor->is_dclick_valide = true;
		}else{
			sensor->second_press_x = x;
			sensor->second_press_y = y;
			sensor->pressed_count++;

			VIVO_TS_LOG_ERR( "FTS %s: second press start x(%d), y(%d)\n", __func__, x, y);

			if ((x - sensor->first_press_x < -DCLICK_TWO_FINGER_AREA_X
						|| x - sensor->first_press_x > DCLICK_TWO_FINGER_AREA_X)
					|| (y - sensor->first_press_y < -DCLICK_TWO_FINGER_AREA_Y
						|| y - sensor->first_press_y > DCLICK_TWO_FINGER_AREA_Y)) {
				VIVO_TS_LOG_ERR( "FTS%s: The distance of the two down is too large\n",
								__func__);
				rmi_f11_cancel_dclick_trace(sensor);
			}
		}
	} else {
		/* the pre_state is down event */
		if (finger_0_state) {
			/* down event trace double click */
			if (sensor->pressed_count == 1 && sensor->release_count == 0) {
				if ((x - sensor->first_press_x < -TRIP_X_AREA
							|| x - sensor->first_press_x > TRIP_X_AREA)
						|| (y - sensor->first_press_y < -TRIP_Y_AREA
							|| y - sensor->first_press_y > TRIP_Y_AREA)) {
					VIVO_TS_LOG_ERR(  "FTS %s: finger triped in one time down\n", __func__);
					rmi_f11_cancel_dclick_trace(sensor);
				}
			}else if (sensor->pressed_count == 2 && sensor->release_count == 1) {
				if ((x - sensor->second_press_x < -TRIP_X_AREA
							|| x - sensor->second_press_x > TRIP_X_AREA)
						|| (y - sensor->second_press_y < -TRIP_Y_AREA
							|| y - sensor->second_press_y > TRIP_Y_AREA)) {
					VIVO_TS_LOG_ERR(  "FTS %s: finger triped in one time down\n", __func__);
					rmi_f11_cancel_dclick_trace(sensor);
				}
			}else{
				/* should not happen nothing to do */
			}
		}else{
			sensor->release_count++;
			VIVO_TS_LOG_ERR( "FTS %s: release.\n", __func__);
		}
	}
}
/*Simulate dclick end.*/

/*
 * This handler is called each time there is at least
 * one new event in the FIFO
 */
static void fts_event_handler(struct work_struct *work)
{
	struct fts_ts_info *info;
	int error, error1;
	int left_events;
	unsigned char regAdd;
	unsigned char data[FTS_EVENT_SIZE * (FTS_FIFO_MAX)];
	unsigned char *event;
	unsigned char eventId;
	int x,y;
	event_dispatch_handler_t event_handler;

	info = container_of(work, struct fts_ts_info, work);
	if(!info) {
		VIVO_TS_LOG_ERR("FTS:%s Fail to get info.\n",__func__);
		goto EXIT;
	}

	VIVO_TS_LOG_DBG("[%s]enter.\n",__func__);

	/* zero events buffer,avoid random data,avoid unhandle event */
	memset(data, 0, FTS_EVENT_SIZE * (FTS_FIFO_MAX));
	
	/*
	 * to avoid reading all FIFO, we read the first event and
	 * then check how many events left in the FIFO
	 */
	regAdd = READ_ONE_EVENT;
	error = fts_read_reg(info, &regAdd,
			sizeof(regAdd), data, FTS_EVENT_SIZE);	//read 8byte first to judge this event is which event

	if (!error) {

		left_events = data[7] & 0x1F;
		if(info->log_switch == 1)
			VIVO_TS_LOG_INF("FTS left_events = %d\n",left_events);
		if ((left_events > 0) && (left_events < FTS_FIFO_MAX)) {
			/*
			 * Read remaining events.
			 */
			regAdd = READ_ALL_EVENT;
			error1 = fts_read_reg(info, &regAdd, sizeof(regAdd),
						&data[FTS_EVENT_SIZE],
						left_events * FTS_EVENT_SIZE);

			/*
			 * Got an error reading remining events,
			 * process at least * the first one that was
			 * reading fine.
			 */
			if (error1)
				data[7] &= 0xE0;		
		}

		/* At least one event is available */
		event = data;
		eventId = *event;
		x = (data[2] << 4) | (data[4] & 0xF0) >> 4;
		y = (data[3] << 4) | (data[4] & 0x0F);
	//	printk( "FTS %s:eventId = %d x = %d,y = %d\n", __func__,eventId,x,y);

	//	printk( "FTS %s:data[1] = %x,data[2] = %x,data[3] = %x,data[4] = %x,data[5] = %x,data[6] = %x,data[7] = %x,\n",
	//	__func__,data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
        if(data[0] == 0xDB)
     	{
			VIVO_TS_LOG_INF("FTS fts debug event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],
			data[4], data[5], data[6], data[7]);
        }

		/*Simulate dclick start.*/
		if(eventId == EVENTID_ENTER_POINTER  ||eventId == EVENTID_MOTION_POINTER){
			info->finger_state = 1;
		}else if(eventId == EVENTID_LEAVE_POINTER){
			info->finger_state = 0;
		}else {
			VIVO_TS_LOG_DBG("FTS :Simulate. %s eventId = %d\n",__func__,eventId);
		}
		if(info->ts_dclick_simulate_switch == 1 &&(eventId == EVENTID_LEAVE_POINTER||
			eventId == EVENTID_ENTER_POINTER  ||eventId == EVENTID_MOTION_POINTER) )
		{
			if(eventId > EVENTID_LAST || eventId < 0){
				VIVO_TS_LOG_ERR("FTS : %s eventId > EVENTID_LAST || eventId < 0\n",__func__);
				goto EXIT;
			}
			x = (data[2] << 4) | (data[4] & 0xF0) >> 4;
			y = (data[3] << 4) | (data[4] & 0x0F);
			VIVO_TS_LOG_DBG( "FTS %s: x = %d,y = %d\n", __func__,x,y);
			if (x == X_AXIS_MAX)
				x--;
			if (y == Y_AXIS_MAX)
				y--;
			VIVO_TS_LOG_DBG("FTS S: %s eventId = %d\n",__func__,eventId);
			info->num_fingers = left_events+1;
			fts_judge_dclick(info,x,y);
		}
		if(eventId == EVENTID_ENTER_POINTER  ||eventId == EVENTID_MOTION_POINTER){
			info->pre_state = 1;
		}else if(eventId == EVENTID_LEAVE_POINTER){
			info->pre_state = 0;
		}
		/*Simulate dclick end.*/

		do {
			eventId = *event;
			VIVO_TS_LOG_DBG("[%s]eventId = %x\n",__func__,eventId);
			if(eventId > EVENTID_LAST || eventId < 0) {
				VIVO_TS_LOG_ERR("FTS : %s eventId > EVENTID_LAST || eventId < 0\n",__func__);
				goto EXIT;
			}
			/*add by qiuguifu for bbk end */
			event_handler = info->event_dispatch_table[eventId];
			event = event_handler ?
					event_handler(info, event) :
					fts_next_event(event);
			//input_sync(info->input_dev);
		} while (event);
	}else{
		VIVO_TS_LOG_ERR("FTS  Fail to read FTS_EVENT_SIZE.\n");
	}
EXIT:
	/*
	 * re-enable interrupts
	 */
	fts_interrupt_enable(info);
}

 static void fts_irq_err_work_func(struct work_struct *work)
{
	struct fts_ts_info *info= container_of(work, struct fts_ts_info, irq_err_work);
	int count = 0;

	if (info->has_lcd_shutoff) {
		while(qup_i2c_suspended > 0 && count < 80) {
			msleep(5);
			count++;
		}
		if (count == 80) {
			VIVO_TS_LOG_ERR( "FTS :The i2c bus stilll suspend after 100 times try \n");
			return;
		}

	}

	disable_irq_nosync(info->client->irq);
	queue_work(info->event_wq, &info->work);

}


#ifdef FTS_USE_POLLING_MODE
static enum hrtimer_restart fts_timer_func(struct hrtimer *timer)
{
	struct fts_ts_info *info =
		container_of(timer, struct fts_ts_info, timer);

	queue_work(info->event_wq, &info->work);
	return HRTIMER_NORESTART;
}
#else
static irqreturn_t fts_interrupt_handler(int irq, void *handle)
{
	struct fts_ts_info *info = handle;
	int count = 0;

	VIVO_TS_LOG_DBG("[%s] enter!lcd state:%d,qup_i2c_suspended:%d\n", __func__, !(info->has_lcd_shutoff), qup_i2c_suspended);	

	if (info->has_lcd_shutoff) {
		while(qup_i2c_suspended > 0 && count < 20) {
			msleep(5);
			count++;
		}
		if (count == 20) {
			VIVO_TS_LOG_ERR("FTS :The i2c bus stilll suspend after 100 times try \n");
			queue_work(info->event_wq, &info->irq_err_work);
			return IRQ_HANDLED;
		}
	}

	disable_irq_nosync(info->client->irq);
	queue_work(info->event_wq, &info->work);

	return IRQ_HANDLED;
}
#endif


static int fts_interrupt_install(struct fts_ts_info *info)
{
	int i, error = 0;

	info->event_dispatch_table = kzalloc(
		sizeof(event_dispatch_handler_t) * EVENTID_LAST, GFP_KERNEL);

	if (!info->event_dispatch_table) {
		VIVO_TS_LOG_ERR("OOM allocating event dispatch table\n");
		return -ENOMEM;
	}

	for (i = 0; i < EVENTID_LAST; i++)
		info->event_dispatch_table[i] = fts_nop_event_handler;

	install_handler(info, ENTER_POINTER, enter_pointer);

	install_handler(info, LEAVE_POINTER, leave_pointer);
	install_handler(info, MOTION_POINTER, motion_pointer);

	install_handler(info, BUTTON_STATUS, button_status);

	install_handler(info, HOVER_ENTER_POINTER, hover_enter_pointer);
	install_handler(info, HOVER_LEAVE_POINTER, hover_leave_pointer);
	install_handler(info, HOVER_MOTION_POINTER, hover_motion_pointer);

	install_handler(info, PROXIMITY_ENTER, proximity_enter);
	install_handler(info, PROXIMITY_LEAVE, proximity_leave);

	install_handler(info, ERROR, error);
	install_handler(info, CONTROLLER_READY, controller_ready);
	install_handler(info, SLEEPOUT_CONTROLLER_READY,
					sleepout_controller_ready);
	install_handler(info, STATUS, status);

	install_handler(info, GESTURE, gesture);

	install_handler(info, PEN_ENTER, pen_enter);
	install_handler(info, PEN_LEAVE, pen_leave);
	install_handler(info, PEN_MOTION, pen_motion);

	/* disable interrupts in any case */
	fts_interrupt_set(info, INT_DISABLE);

#ifdef FTS_USE_POLLING_MODE
	VIVO_TS_LOG_DBG("Polling Mode\n");
	hrtimer_init(&info->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	info->timer.function = fts_timer_func;
	hrtimer_start(&info->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
#else
	VIVO_TS_LOG_DBG("Interrupt Mode\n");
	info->client->irq = gpio_to_irq(info->client->irq);// ADD BY QIUGUIFU
	if (request_irq(info->client->irq, fts_interrupt_handler,
			IRQF_TRIGGER_LOW,info->client->name,// "fts_attn",
			info)) {
		VIVO_TS_LOG_ERR("Request irq failed\n");
		kfree(info->event_dispatch_table);
		error = -EBUSY;

	} else
		fts_interrupt_set(info, INT_ENABLE);
	error = enable_irq_wake(info->client->irq);
	if (error) {
		VIVO_TS_LOG_ERR("%s: set_irq_wake failed for irq %d\n", __func__, info->client->irq);
	}
	VIVO_TS_LOG_INF("%s: set_irq_wake success for irq %d\n", __func__, info->client->irq);
#endif

	return error;
}


static void fts_interrupt_uninstall(struct fts_ts_info *info)
{
	fts_interrupt_set(info, INT_DISABLE);

	kfree(info->event_dispatch_table);

#ifdef FTS_USE_POLLING_MODE
	hrtimer_cancel(&info->timer);
#else
	free_irq(info->client->irq, info);
#endif
}

static void fts_interrupt_enable(struct fts_ts_info *info)
{
#ifdef FTS_USE_POLLING_MODE
	hrtimer_start(&info->timer,
			ktime_set(0, 10000000), HRTIMER_MODE_REL);
#else
	enable_irq(info->client->irq);
#endif
}

static void fts_interrupt_disable(struct fts_ts_info *info)
{
#ifdef FTS_USE_POLLING_MODE
	hrtimer_cancel(&info->timer);
#else
	disable_irq(info->client->irq);
#endif
}


static int fts_check_id(struct fts_ts_info *info)
{
	unsigned char val[8];
	unsigned char regAdd[3];
	int error;
	/* TS Chip ID */
	    regAdd[0] = 0xB6;
	    regAdd[1] = 0x00;
	    regAdd[2] = 0x07;
	error = fts_read_reg(info, regAdd, 3, (unsigned char *)val, 5);
	if (error) {
             VIVO_TS_LOG_ERR("Cannot read device id.[%s]\n", __func__);
             return -ENODEV;
   	}
    	VIVO_TS_LOG_INF( "FTS %s : %02X%02X%02X =  %02x %02x %02x %02x \n",__func__,
           regAdd[0], regAdd[1], regAdd[2], val[1], val[2], val[3], val[4]);
    	if ((val[1] != FTS_ID0) || (val[2] != FTS_ID1))
    	{
		VIVO_TS_LOG_ERR( "Wrong version id (read 0x%02x%02x, expected 0x%02x%02x)\n",
                                           val[1], val[2], FTS_ID0, FTS_ID1);
      		return -ENODEV;
	}

/*
	regAdd[0] = 0x80;
	error = fts_write_reg(info, regAdd, sizeof(regAdd));
	if (error) {
		dev_err(info->dev, "Cannot retrieve device id\n");
		return -ENODEV;
	}

	msleep(20);

	regAdd[0] = 0x80;
	error = fts_read_reg(info, regAdd, sizeof(regAdd), val, sizeof(val));
	if (error) {
		dev_err(info->dev, "Cannot read device id\n");
		return -ENODEV;
	}
*/
	/* check for chip id */
//	if ((val[6] != FTS_ID0) || (val[7] != FTS_ID1)) {
//		dev_err(info->dev,
//			"Wrong version id (read 0x%02x%02x, expected 0x%02x%02x)\n",
//				val[6], val[7], FTS_ID0, FTS_ID1);
//		return -ENODEV;
//	}

	/* Chip Id is fine, store also firmware version */
	//info->fw_version = (val[5] << 8) | val[4];

	return 0;
}


static int fts_init(struct fts_ts_info *info)
{
	int error;
	int i = 0;

	for(i=0; i<11; i++) {
		VIVO_TS_LOG_INF("[%s]system reset count:%d\n", __func__, i);
		error = fts_systemreset(info);
		if (error) {
			if(i==10) {
				VIVO_TS_LOG_ERR("[%s]Cannot reset the device.retry:%d\n", __func__, i);
				return -ENODEV;
			}	
			msleep(100);
		} else {
			break;
		}
	} 

	/* check for chip id */
	error = fts_check_id(info);
	if (error) {
		VIVO_TS_LOG_ERR("Cannot initiliaze, wrong device id\n");
		return -ENODEV;
	}

	error = fts_interrupt_install(info);

	if (error)
		VIVO_TS_LOG_ERR("Init (1) error (#errors = %d)\n", error);

	return error ? -ENODEV : 0;
}

#if 0 //change by chenpeng
static int  fts_init_flash_reload(struct fts_ts_info *info)
{
	int error = 0;
    error +=fts_systemreset(info);
	msleep(200);
	//init_completion(&info->cmd_done);
	error += fts_command(info, SLEEPOUT);
	msleep(50);
	//WAIT_WITH_TIMEOUT(info, HZ, SLEEPOUT);

	//init_completion(&info->cmd_done);
	error += fts_command(info, CX_TUNING);
	msleep(1000);
	//WAIT_WITH_TIMEOUT(info, HZ, CX_TUNING);

	//init_completion(&info->cmd_done);
	error += fts_command(info, SELF_TUNING);
	msleep(1000);
	//WAIT_WITH_TIMEOUT(info, HZ, SELF_TUNING);

	//init_completion(&info->cmd_done);
	error += fts_command(info, FORCECALIBRATION);
	msleep(5);
	//WAIT_WITH_TIMEOUT(info, HZ, FORCECALIBRATION);

	//init_completion(&info->cmd_done);
//	error += fts_command(info, TUNING_BACKUP);
//	msleep(200);
	//WAIT_WITH_TIMEOUT(info, HZ, TUNING_BACKUP);

	error += fts_command(info, SENSEON);
	msleep(5);
	error += fts_command(info, KEY_ON);           //add by terry 150316
	msleep(5);
	error += fts_command(info, FLUSHBUFFER);
    msleep(100);
	error += fts_command(info, TUNING_BACKUP);
    msleep(500);

	if (error)
		dev_err(info->dev, "Init (2) error (#errors = %d)\n", error);

	return error ? -ENODEV : 0;
}
#endif

static int fts_init_hw(struct fts_ts_info *info)
{
	int error = 0;

	init_completion(&info->cmd_done);

	error += fts_command(info, SENSEON);
	error += fts_command(info, KEY_ON);
	error += fts_command(info, FLUSHBUFFER);

	if (error)
		VIVO_TS_LOG_INF("init hw err!\n");

	fts_interrupt_set(info, INT_ENABLE);		//open chip's intterupt,so,chip can send int

	return error ? -ENODEV : 0;
}

#ifdef CONFIG_OF
static int fts_parse_dt(struct device *dev, struct fts_i2c_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	u32 temp_val;
	const char *str_val = NULL;
#if defined(TP_GPIO_CTRL_E)	
	int error;
#endif

	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "synaptics,reset-gpio",
				0, &pdata->reset_gpio_flags);
	pdata->rst_gpio = pdata->reset_gpio;
	pdata->irq_gpio = of_get_named_gpio_flags(np, "synaptics,irq-gpio",
				0, &pdata->irq_gpio_flags);
	pdata->attn_gpio = pdata->irq_gpio;

#if defined(TP_GPIO_CTRL_E)	
	pdata->vdd_gpio = of_get_named_gpio_flags(np, "synaptics,vdd-gpio",	//_3.3v
				0, &pdata->vdd_gpio_flags);

	pdata->vcc_gpio = of_get_named_gpio_flags(np, "synaptics,vcc-gpio",		//_1.8
				0, &pdata->vcc_gpio_flags);
	pdata->vdd1_gpio = pdata->vcc_gpio;
	if (gpio_is_valid(pdata->vdd1_gpio)) {
		VIVO_TS_LOG_INF("fts vdd1_gpio is valid\n");
		/* configure touchscreen vdd gpio control gpio */
		error = gpio_request(pdata->vdd1_gpio, "vdd1_ctl");
		if(error){
			VIVO_TS_LOG_ERR("fts vdd1_gpio request fail\n");
			return error;
		}
	}else{
		VIVO_TS_LOG_ERR("fts vdd1_gpio is not valid\n");	
	}
	
	if (gpio_is_valid(pdata->vdd_gpio)) {
		/* configure touchscreen vdd gpio control gpio */
		error = gpio_request(pdata->vdd_gpio, "vdd_ctl");
		if(error){
			VIVO_TS_LOG_ERR("fts vdd_gpio request fail\n");
			return error;
		}
	}
#endif
	
	rc = of_property_read_u32(np, "synaptics,attn_polarity", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG( "Unable to read attn_polarity value\n");
	else if (rc != -EINVAL)
		pdata->irq_flags = (u8) temp_val;


	//chenyunzhe add BEG-----------------------------------------------
	rc = of_property_read_u32(np, "ts-suspend-resume", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-suspend-resume value\n");
	else if (rc != -EINVAL)
		pdata->suspend_resume_methods = temp_val;

	rc = of_property_read_u32(np, "ts-fixed-key-type", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-fixed-key-type value\n");
	else if (rc != -EINVAL)
		pdata->fixed_key_type = temp_val;

    rc = of_property_read_u32(np, "ts-dimension-by-lcm", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-dimension-by-lcm value\n");
	else if (rc != -EINVAL)
		pdata->ts_dimension_by_lcm = temp_val;

	rc = of_property_read_u32(np, "lcd-dimension-x", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read lcd-dimension-x value\n");
	else if (rc != -EINVAL)
		pdata->lcd_dimension_x =  temp_val;

	rc = of_property_read_u32(np, "lcd-dimension-y", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read lcd-dimension-y value\n");
	else if (rc != -EINVAL)
		pdata->lcd_dimension_y =  temp_val;

	rc = of_property_read_u32(np, "ts-dimension-x", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-dimension-x value\n");
	else if (rc != -EINVAL)
		pdata->ts_dimension_x = temp_val;

	rc = of_property_read_u32(np, "ts-dimension-y", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-dimension-y value\n");
	else if (rc != -EINVAL)
		pdata->ts_dimension_y =  temp_val;

	rc = of_property_read_u32(np, "ts-dclick-trip-x-area", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-dclick-trip-x-area value\n");
	else if (rc != -EINVAL)
		pdata->dclick_trip_x_area =  temp_val;

	rc = of_property_read_u32(np, "ts-dclick-trip-y-area", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-dclick-trip-x-area value\n");
	else if (rc != -EINVAL)
		pdata->dclick_trip_y_area =  temp_val;

	rc = of_property_read_u32(np, "ts-dclick-two-fingers-x-area", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-dclick-two-fingers-x-area value\n");
	else if (rc != -EINVAL)
		pdata->dclick_two_finger_x_area =  temp_val;

	rc = of_property_read_u32(np, "ts-dclick-two-fingers-y-area", &temp_val);
	if (rc && (rc != -EINVAL))
		VIVO_TS_LOG_DBG("Unable to read ts-dclick-two-fingers-y-area value\n");
	else if (rc != -EINVAL)
		pdata->dclick_two_finger_y_area =  temp_val;

	rc = of_property_read_string(np,"ts-virt-key",&str_val);
	if (rc) {
		VIVO_TS_LOG_DBG("Unable to read ts-dclick-dimension-y-max value\n");
	} else {
	    pdata->virtual_key_string =  str_val;
	}

	VIVO_TS_LOG_INF("%s:srm(%d) tdbl(%d) fkt(%d) lcd(%d %d) ts(%d %d) dclk(%d %d %d %d)\n",__func__,
	    pdata->suspend_resume_methods,pdata->ts_dimension_by_lcm,pdata->fixed_key_type,pdata->lcd_dimension_x,pdata->lcd_dimension_y,
		pdata->ts_dimension_x,pdata->ts_dimension_y,pdata->dclick_trip_x_area,pdata->dclick_trip_y_area,
		pdata->dclick_two_finger_x_area,pdata->dclick_two_finger_y_area);

	VIVO_TS_LOG_INF("%s:virt-key: %s\n",__func__,pdata->virtual_key_string);
	//chenyunzhe add END-----------------------------------------------

	return 0;
}
#else
static int fts_parse_dt(struct device *dev, struct synaptics_dsx_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#if 0
static int fts_parse_dt(struct device *dev, struct fts_i2c_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	#if defined(TP_GPIO_CTRL_E)
	int error;
	pdata->vdd1_gpio = of_get_named_gpio_flags(np, "fts,vdd1-gpio",
				0, &pdata->vdd1_gpio_flags);
	if (gpio_is_valid(pdata->vdd1_gpio)) {
		/* configure touchscreen vdd gpio control gpio */
		error = gpio_request(pdata->vdd1_gpio, "vdd1_ctl");
	}
	pdata->vdd_gpio = of_get_named_gpio_flags(np, "fts,vdd-gpio",
				0, &pdata->vdd_gpio_flags);
	if (gpio_is_valid(pdata->vdd_gpio)) {
		/* configure touchscreen vdd gpio control gpio */
		error = gpio_request(pdata->vdd_gpio, "vdd_ctl");
		if (error) {
			dev_err(&pdata->client->dev, "unable to request gpio [%d]\n",
						pdata->rst_gpio);
			return error;
		}
	}else{
		dev_err(&pdata->client->dev, "vdd-gpio not provided\n");
		return -EINVAL;
	}
	#else
	#endif
	/* reset, irq gpio info */
	pdata->rst_gpio = of_get_named_gpio_flags(np, "fts,reset-gpio",
				0, &pdata->reset_gpio_flags);
	pdata->attn_gpio = of_get_named_gpio_flags(np, "fts,irq-gpio",
				0, &pdata->irq_gpio_flags);
	/*rc = of_property_read_u32(np, "fts,attn_polarity", &temp_val);
	if (rc && (rc != -EINVAL))
		dev_err(dev, "Unable to read attn_polarity value\n");
	else if (rc != -EINVAL)
		pdata->attn_polarity = (u8) temp_val;
	*/
	return 0;
}
#endif

static int fts_set_power_state( bool onoff)
{
	int rc;
	struct fts_i2c_platform_data *data	 = &pdata_gl;
	if(!data)
	{
		VIVO_TS_LOG_ERR("FTS:%s Fail to get data.\n",__func__);
		return 0;
	}
	if(onoff){
	#if defined(TP_GPIO_CTRL_E)
		if(gpio_is_valid(data->vdd1_gpio)){
			rc = gpio_direction_output(data->vdd1_gpio, 1);
			if (rc) {
				VIVO_TS_LOG_ERR("unable to set vdd1_gpio direction out to 1 for gpio [%d]\n",
					data->vdd1_gpio);
				return rc;
			}
		}
		rc = gpio_direction_output(data->vdd_gpio, 1);
		if (rc) {
			VIVO_TS_LOG_ERR("unable to set vdd_gpio direction out to 1 for gpio [%d]\n",
				data->vdd_gpio);
			return rc;
		}
	#else

		if (regulator_count_voltages(data->vcc_ana) > 0) {
			rc = regulator_set_voltage(data->vcc_ana, 3300000,
								3300000);
			if (rc) {
				VIVO_TS_LOG_ERR("regulator set_vtg failed rc=%d\n", rc);
				regulator_put(data->vcc_ana);
				return rc;
			}
		}
		rc = regulator_enable(data->vcc_ana);
		if (rc) {
			VIVO_TS_LOG_ERR("Regulator vcc_ana enable failed rc=%d\n", rc);
			return rc;
		}
	#endif
	}else{
	#if defined(TP_GPIO_CTRL_E)
		if(gpio_is_valid(data->vdd1_gpio)){
			rc = gpio_direction_output(data->vdd1_gpio, 0);
			if (rc) {
				VIVO_TS_LOG_ERR("unable to set direction out to 0 for gpio [%d]\n",
					data->vdd1_gpio);
				//return rc;
			}
		}
		rc = gpio_direction_output(data->vdd_gpio, 0);
		if (rc) {
			VIVO_TS_LOG_ERR("unable to set direction out to 0 for gpio [%d]\n",
				data->vdd_gpio);
			return rc;
		}
	#else
		rc = regulator_disable(data->vcc_ana);
		if (rc) {
			VIVO_TS_LOG_ERR("Regulator vcc_ana enable failed rc=%d\n", rc);
			return rc;
		}
	#endif
	}		
			if(fts_command(info_gl, SENSEOFF))
			{
				VIVO_TS_LOG_ERR("%s SENSEOFF failed \n", __func__);
			}
			mdelay(10);
			if(fts_command(info_gl, KEY_OFF))
			{
				VIVO_TS_LOG_ERR("%s KEY_OFF failed \n", __func__);
			}
			mdelay(10);
			if(fts_command(info_gl, SENSEON))
			{
				VIVO_TS_LOG_ERR("%s SENSEON failed \n", __func__);
			}
			mdelay(10);
			if(fts_command(info_gl, KEY_ON))
			{
				VIVO_TS_LOG_ERR("%s KEY_ON failed \n", __func__);
			}
			mdelay(10);
			//WAIT_WITH_TIMEOUT(info, HZ, SENSEON);
			//cyttsp5_core_rt_resume(cd->dev);
			release_point_and_key(info_gl);//add by qiuguifu

	return 0;
}

static int fts_power_on( bool onoff)
{
	int rc;
	struct fts_i2c_platform_data *data	 = &pdata_gl;

	/* power on */
	if(onoff){
#if defined(TP_GPIO_CTRL_E)	/* GPIO power control */
		if(gpio_is_valid(data->vdd_gpio)){
			VIVO_TS_LOG_INF("[%s]3.3v is valid.set 3.3v gpio to 1.\n", __func__);
			rc = gpio_direction_output(data->vdd_gpio, 1);
			if (rc) {
				VIVO_TS_LOG_ERR("[%s]unable to set direction out to 1 for vdd3.3_gpio [%d]\n",
					__func__, data->vdd_gpio);
				return rc;
			}
		}
		if(gpio_is_valid(data->vdd1_gpio)){	//_1.8
			VIVO_TS_LOG_INF("[%s]1.8v is valid.set 1.8v gpio to 1.\n", __func__);
			rc = gpio_direction_output(data->vdd1_gpio, 1);
			if (rc) {
				VIVO_TS_LOG_ERR("[%s]unable to set direction out to 1 for vdd1.8_gpio [%d]\n",
					__func__, data->vdd1_gpio);
				return rc;
			}
		}
#else	/* PMIC power control */
		if (regulator_count_voltages(data->vcc_ana) > 0) {
			rc = regulator_set_voltage(data->vcc_ana, 3300000,
								3300000);
			if (rc) {
				VIVO_TS_LOG_ERR("[%s]regulator set_vtg:vcc_ana failed rc=%d\n", __func__, rc);
				regulator_put(data->vcc_ana);
				return rc;
			}
		}
		rc = regulator_enable(data->vcc_ana);
		if (rc) {
			VIVO_TS_LOG_ERR("[%s]Regulator vcc_ana enable failed rc=%d\n", __func__, rc);
			return rc;
		}

		//__1.8v
		if (regulator_count_voltages(data->vcc_i2c) > 0) {
			rc = regulator_set_voltage(data->vcc_i2c, 1800000,
								1800000);
			if (rc) {
				VIVO_TS_LOG_ERR("regulator set_vtg:vcc_i2c failed rc=%d\n", rc);
				regulator_put(data->vcc_i2c);
				return rc;
			}
		}
		rc = regulator_enable(data->vcc_i2c);
		if (rc) {
			VIVO_TS_LOG_ERR("Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}		
#endif
	}else{
	/* power off */
#if defined(TP_GPIO_CTRL_E)		/* GPIO power control */
		/* _1.8v must first power down,else will get big problem in resume and esd event handler */
		if(gpio_is_valid(data->vdd1_gpio)){
			VIVO_TS_LOG_INF("[%s]1.8v gpio is valid.set 1.8v gpio to 0.\n", __func__);
			rc = gpio_direction_output(data->vdd1_gpio, 0);	//_1.8	power down:1.8 must down first
			if (rc) {
				VIVO_TS_LOG_ERR("[%s]unable to set direction out to 0 for vdd1.8_gpio [%d]\n",
					__func__, data->vdd1_gpio);
				return rc;
			}
		}
		/* _3.3v */
		if(gpio_is_valid(data->vdd_gpio)){
			VIVO_TS_LOG_INF("[%s]3.3v gpio is valid.set 3.3v gpio to 0.\n", __func__);
			rc = gpio_direction_output(data->vdd_gpio, 0);
			if (rc) {
				VIVO_TS_LOG_ERR("[%s]unable to set direction out to 0 for vdd3.3_gpio [%d]\n",
						__func__, data->vdd_gpio);
				return rc;
			}
		}
#else	/* PMIC power control */
		/* _1.8v must first power down,else will get big problem in resume and esd event handler */
		rc = regulator_disable(data->vcc_i2c);
		if (rc) {
			VIVO_TS_LOG_ERR("Regulator vcc_i2c enable failed rc=%d\n", rc);
			return rc;
		}
		/* _3.3v */
		rc = regulator_disable(data->vcc_ana);
		if (rc) {
			VIVO_TS_LOG_ERR("Regulator vcc_ana enable failed rc=%d\n", rc);
			return rc;
		}
#endif
	}

	return 0;
}


static int fts_config_gpio(struct fts_i2c_platform_data *pdata ,int on)
{
	int error;

	struct pinctrl_state *set_state = NULL;
	struct pinctrl *phy_pinctrl=devm_pinctrl_get(&pdata->client->dev);

	VIVO_TS_LOG_INF("FTS pdata->attn_gpio = %d pdata->rst_gpio = %d\n",pdata->attn_gpio,pdata->rst_gpio);
	if(on){
		if (gpio_is_valid(pdata->attn_gpio)) {
			/* configure touchscreen irq gpio */
			error = gpio_request(pdata->attn_gpio, "fts_attn");
			if (error) {
				VIVO_TS_LOG_ERR("unable to request gpio [%d].\n",
							pdata->attn_gpio);
				return error;
			}

			error = gpio_direction_input(pdata->attn_gpio);
			if (error) {
				VIVO_TS_LOG_ERR("unable to set direction for gpio [%d]\n",
					pdata->attn_gpio);
				goto err_irq_gpio_req;
			}
		} else {
			VIVO_TS_LOG_ERR("irq gpio not provided\n");
			return -EINVAL;
		}
		if (gpio_is_valid(pdata->rst_gpio)) {
			/* configure touchscreen reset out gpio */
			set_state =pinctrl_lookup_state(phy_pinctrl, "default");
			error=pinctrl_select_state(phy_pinctrl, set_state);
			if (error) {
				VIVO_TS_LOG_ERR("unable to set direction for gpio [%d]\n",
					pdata->rst_gpio);
				return error;

			}
			msleep(5);
			error = gpio_request(pdata->rst_gpio, "fts_rst");
			if (error) {
				VIVO_TS_LOG_ERR("unable to request gpio [%d]\n",
							pdata->rst_gpio);
				return error;
			}

			error = gpio_direction_output(pdata->rst_gpio, 1);
			if (error) {
				VIVO_TS_LOG_ERR("unable to set direction out to 0 for gpio [%d]\n",
					pdata->rst_gpio);
				goto err_reset_gpio_req;
			}
			msleep(20);

			error = gpio_direction_output(pdata->rst_gpio, 0);
			if (error) {
				VIVO_TS_LOG_ERR("unable to set direction out to 0 for gpio [%d]\n",
					pdata->rst_gpio);
				goto err_reset_gpio_req;
			}

			msleep(20);

			error = gpio_direction_output(pdata->rst_gpio, 1);
			if (error) {
				VIVO_TS_LOG_ERR("unable to set direction out to 1 for gpio [%d]\n",
					pdata->rst_gpio);
				goto err_reset_gpio_req;
			}

			msleep(20);
		}
	}else{
		VIVO_TS_LOG_INF("FTS Free gpio :pdata->attn_gpio = %d pdata->rst_gpio = %d",pdata->attn_gpio,pdata->rst_gpio);
		gpio_free(pdata->attn_gpio);
		gpio_free(pdata->rst_gpio);
	}

	return 0;

err_reset_gpio_req:
	if (gpio_is_valid(pdata->rst_gpio))
		gpio_free(pdata->rst_gpio);
err_irq_gpio_req:
	if (gpio_is_valid(pdata->attn_gpio))
		gpio_free(pdata->attn_gpio);
	return error;

}
/* bbk add start*
 * just for 2D sensor
 */
static ssize_t fts_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
/*	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_MENU) ":117:1345:170:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BAR_SWIPE) ":238:1345:70:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE) ":360:1345:140:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BAR_SWIPE) ":481:1345:70:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":603:1345:170:100"
	"\n");
*/
	int rc = -EINVAL;

	if(pdata_gl.virtual_key_string != NULL) {
	VIVO_TS_LOG_INF("VKEY= %s \n", pdata_gl.virtual_key_string);
		rc = sprintf(buf,"%s\n", pdata_gl.virtual_key_string);
	}else{
		VIVO_TS_LOG_ERR("VKEY support failed or please check dts !!\n");
	}

	return rc;
}

static struct kobj_attribute fts_vkeys_attr = {
	.attr = {
		.name = "virtualkeys.fts",
		.mode = S_IRUGO,
	},
	.show = &fts_vkeys_show,
};


static struct attribute *fts_properties_attrs[] = {
	&fts_vkeys_attr.attr,
	NULL
};
static struct attribute_group fts_properties_attr_group = {
	.attrs = fts_properties_attrs,
};
static struct kobject *properties_kobj;
/*add key virtual property.*/

#if defined(BBK_DCLICK_WAKE)
void ts_scan_switch(int on_off)  // 0 -- off  1---on
{
	int error;
	u8 reg1[6] = {0xc3,0x01,0x00,0x00,0x00,0x00};	//enable
	u8 reg2[6] = {0xc3,0x02,0xFF,0xFF,0xFF,0xFF};	//disable

	VIVO_TS_LOG_INF("%s:enter!\n", __func__);
	
	if(info_gl == NULL)
	{
		VIVO_TS_LOG_ERR("[FTS] Fail to get info_gl.\n");
		return;
	}
	if(on_off == 0)//  exit from gesture mode
	{	
		VIVO_TS_LOG_INF("%s:into sleep mode!\n", __func__);
		if(fts_command(info_gl, SENSEOFF))
		{
			VIVO_TS_LOG_ERR("%s SENSEOFF failed \n", __func__);
		}
		mdelay(10);
		if(fts_command(info_gl, KEY_OFF))
		{
			VIVO_TS_LOG_ERR("%s KEY_OFF failed \n", __func__);
		}
#if 0
		mdelay(50);
		/*wunandi add */
             fts_write_reg(info_gl, reg2, sizeof(reg2));	//disable all gesture off
#endif
		atomic_set(&info_gl->ts_state, TOUCHSCREEN_SLEEP);		
	}else{
		VIVO_TS_LOG_INF("%s:into gesture mode!\n", __func__);

		fts_custom_gesture_template_rewrite_tochip(info_gl);
		
		if(info_gl->dclick_switch == 1){	//Dclick
			reg1[2]|= 0x02;			  
		}
		if(	(info_gl->gesture_switch & 0x02)){	//up   
				reg1[3]|= 0x04;	 
		}
		if(	(info_gl->gesture_switch & 0X01)){	//-->  
				reg1[2]|= 0x80;	
		}
		if( (info_gl->gesture_switch & 0X01)){	//<--  
				reg1[3]|= 0x01;		  
		}
		if( (info_gl->swipe_switch == 1)){	//down  
				reg1[3]|= 0x02;    
		}
		if(	(info_gl->gesture_switch & 0X40)){	//C  
				reg1[2]|= 0x08;	   
		}
		if(	(info_gl->gesture_switch & 0X10)){	//M  
				reg1[2]|= 0x10;			
		}
		if(	(info_gl->gesture_switch & 0X04)){	//O  
			   	reg1[2]|= 0x04;		      
		}
		if(	(info_gl->gesture_switch & 0X08)){	//W  
				reg1[2]|= 0x20;
		}
		if(	(info_gl->gesture_switch & 0X20)){	//e  
				reg1[2]|= 0x40;		
		}
		if(info_gl->user_define_gesture_switch==1) {	//enable custom gesture
			reg1[4] |=0x02;
		}
		VIVO_TS_LOG_DBG("%s enalbe gesture. reg1[2] = 0x%x,reg1[3] = 0x%x \n",__func__,reg1[2],reg1[3]);
	
		if(!fts_command(info_gl, SENSEON)) {
			VIVO_TS_LOG_INF("[FTS]:%s SENSEON success\n",__func__);
		} else {
			VIVO_TS_LOG_ERR("[FTS]:%s SENSEON fail\n",__func__);
		}
		mdelay(10);
		error = fts_write_reg(info_gl,reg2,sizeof(reg2));  		    //error = fts_write_reg(info_gl,reg,sizeof(reg));
		if (error) {
			VIVO_TS_LOG_ERR("FTS :%s Cannot write reg2 ,disable gesture.\n",__func__);
		}
		mdelay(10);
		//reg1[4] = 0xFF;                                             // add by terry 150312 for default enable user defind gesture
		error = fts_write_reg(info_gl,reg1,sizeof(reg1));  		    //error = fts_write_reg(info_gl,reg,sizeof(reg));
		if (error) {
			VIVO_TS_LOG_ERR("FTS :%s Cannot write reg1,enalbe gesture. reg1[2] = 0x%x,reg1[3] = 0x%x \n",__func__,reg1[2],reg1[3]);
		}
		mdelay(10);

		VIVO_TS_LOG_INF("FTS fts gesture enable bit : %02X %02X %02X %02X %02X %02X\n",
			reg1[0], reg1[1], reg1[2], reg1[3],
			reg1[4], reg1[5]);

		if(!fts_command(info_gl, ENTER_GESTURE_MODE)) {
			VIVO_TS_LOG_INF("[FTS]:%s ENTER_GESTURE_MODE success\n",__func__);
		} else {
			VIVO_TS_LOG_ERR("[FTS]:%s ENTER_GESTURE_MODE fail\n",__func__);
		}
		atomic_set(&info_gl->ts_state, TOUCHSCREEN_GESTURE);
	}
}
#endif


/*add toushcreen Dir. start*/
static ssize_t touchscreen_power_state_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF( "FTS parameter is %d\n", val);
	if (val == 0) {
		VIVO_TS_LOG_INF( "FTS set parameter is off\n");
		info->power_state = 0;
		fts_set_power_state(0);
	}else if (val == 1) {
		VIVO_TS_LOG_INF( "FTS set parameter is on\n");
		info->power_state = 1;
		fts_set_power_state(1);
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}


static ssize_t touchscreen_power_state_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_INF("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "power_state = %d\n", info->power_state);
}

#if defined (BBK_LARGE_SUPRESSION)
static ssize_t fts_is_calling_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", is_calling);
}

static ssize_t fts_is_calling_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	unsigned int val;
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);
	
	if (sscanf(buf, "%d", &val) != 1) {
		return -EINVAL;
	}

	VIVO_TS_LOG_INF("%s: val is %d\n", __func__, val);
	if(val!=0 && val!=1) {
		return -EINVAL;
	}
	
	VIVO_TS_LOG_INF( "[%s]is_large_press_mode=%d is_3_finger_touch=%d\n", __func__, info->is_large_press_mode, info->is_3_finger_touch);

	if (val == 0) {
		is_calling = 0;
		is_calling_save = 0;
		/* if large press, phone call should ring */
	}

	if (val == 1 && ((info->is_large_press_mode==0) && (info->is_3_finger_touch==0))) {
		is_calling = 1;
	} else { 
		is_calling_save = 1;
	}
		
		
	
	return count;
}
#endif


static ssize_t touchscreen_log_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
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

	VIVO_TS_LOG_INF( "FTS parameter is %d\n", val);
	if (val == 0) {
		info->log_switch = 0;
	}else if (val == 1) {
		info->log_switch = 1;
	}else{
		VIVO_TS_LOG_INF( "Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}


static ssize_t touchscreen_log_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "log_switch = %d\n", info->log_switch);
}

static ssize_t touchscreen_fw_update_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	char name[32];
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	/* reading out firmware upgrade mode */
	if (sscanf(buf, "%s", name) != 1) {
		VIVO_TS_LOG_ERR("Invalide parameter\n");
		return -EINVAL;
	}
	VIVO_TS_LOG_INF("[%s]firmware name:%s\n", __func__, name);
	fts_command(info, SENSEOFF);
	release_point_and_key(info);
	fts_fw_upgrade_and_init(info, 2,name);
	release_point_and_key(info);	
	bbk_rewrite_usb_charger_flag_tochip(info);
	fts_glove_mode_switch_rewrite_tochip(info);

	return count;
}

static ssize_t touchscreen_gloves_mode_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;
	unsigned char regAdd[4];
	int ret;

	if(!info) {
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	/* add for firmware updating,while updating,not change glove mode state. */
	if(info->firmware_updating_flag) {
		VIVO_TS_LOG_INF("[%s]firmware is updating.no glove mode switch.\n", __func__);	
		return -EIO;
	}	

	VIVO_TS_LOG_INF( "FTS parameter is %d\n", val);
	if (val == 0) {
		regAdd[0] = 0XC2;		//set disable data
		regAdd[1] = 0X01;
		ret = fts_write_reg(info, regAdd, sizeof(regAdd));
		if(ret<0)
		{
			VIVO_TS_LOG_ERR("FTS Fail to write reg(ret = %d)\n", ret);
		}
		info->glove_mode_switch = 0;
	}else if (val == 1) {		
		regAdd[0] = 0XC1;		//set enable data
		regAdd[1] = 0X01;
		ret = fts_write_reg(info, regAdd, sizeof(regAdd));
		if(ret !=0)
		{
			VIVO_TS_LOG_ERR("FTS Fail to write reg(ret = %d)\n", ret);
		}
		info->glove_mode_switch = 1;
	}else{
		VIVO_TS_LOG_ERR("FTS Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}

static ssize_t touchscreen_gloves_mode_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "glove_mode_switch = %d\n", info->glove_mode_switch);
}

static ssize_t touchscreen_interrupt_enable_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF( "FTS parameter is interrupt_enable %d\n", val);
	if (val == 0) {
		info->interrupt_enable = 0;
		fts_interrupt_disable(info);
	}else if (val == 1) {
		info->interrupt_enable = 1;
		fts_interrupt_enable(info);
	}else{
		VIVO_TS_LOG_INF( "Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}


static ssize_t touchscreen_interrupt_enable_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "interrupt_enable = %d\n", info->interrupt_enable);
}

static ssize_t touchscreen_sensor_rx_tx_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int rx,tx,temp;
	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	rx = NUM_RX;//for PD1227L 0xf
	tx = NUM_TX;//for PD1227L 0x1a
	temp = rx <<8 | tx;
	return sprintf(buf, "%d\n", temp);

}
static ssize_t touchscreen_firmware_module_id_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	//u8 version = 0;;
	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}

	if (info->ic_fw_base_version< 0) {
		VIVO_TS_LOG_ERR("FTS Failed to get version\n");
		return sprintf(buf, "%s\n", "Get version failed");
	}else{
		return sprintf(buf, "0x%x\n", 0x90);
	}
}

static ssize_t touchscreen_version_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);
	int ret;
	//int error = 0;

	VIVO_TS_LOG_INF("FTS ====fts_get_ic_fw_base_version\n");
	ret = fts_get_ic_fw_base_version(info);

	VIVO_TS_LOG_INF("FTS ====fts_get_ic_fw_config_version\n");
	ret = fts_get_ic_fw_config_version(info);

	if(info->ic_fw_base_version!=0xFFFF && info->ic_fw_config_version!=0xFFFF ){
		return sprintf(buf, "0x%x0x%x\n",info->ic_fw_base_version, info->ic_fw_config_version);
		//return sprintf(buf, "FW 0x%04x CF 0x%04x\n",info->fw_version,info->config_version);
	}
	else{
		return sprintf(buf, "FW:0x%04x\n",0xFFFF);
	}
}

//add by liukangfei start
static ssize_t touchscreen_rawdata_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);
	unsigned char *mutual_raw_data =NULL;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int j = 0, 
		i = 0,
		error =0 , 
		address_offset = 0, 
		retry = 0,
		size_hex =0;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEOFF);	
	fts_command(info, FLUSHBUFFER);
	msleep(50);

	release_point_and_key(info);
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			VIVO_TS_LOG_ERR( "fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		if(1 == info->log_switch)
			VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
						data[0], data[1], data[2], data[3],
						data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT - 1)
			{
				VIVO_TS_LOG_ERR("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
	tx_num = data[4];//M
	rx_num = data[5];//N
	size_hex = (tx_num * rx_num)* 2 ;
	
	/* Read Offset Address for Mutual Raw data*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x00;//Offset address for MS screen raw frame, Please verify this address from supporting fw engineer
	fts_read_reg(info,regAdd,3, &buff_read[0], 4);
	
	address_offset = ((buff_read[2]<<8) |buff_read[1]) + (rx_num *2);  
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ascii/test */
	mutual_raw_data = (unsigned char *)kmalloc((size_hex + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)

	// ============================================ Mutual Raw data Reading ======================================//	
	
	/* Read compensation data*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mutual_raw_data, (size_hex+1));
	
	/*Copying Mutual Raw data  */
	count += sprintf(&buf[count], "tp channel: %u * %u\n", tx_num, rx_num);
	for(i = 0; i < tx_num; i++ )
	{
		for(j = 0; j < rx_num*2; j=j+2 )
		{
			count += sprintf(&buf[count], "%4u ", 
					(u16)(mutual_raw_data[i * rx_num * 2 + j + 2]) << 8 | (u16)mutual_raw_data[i * rx_num * 2 + j + 1]);
		}
		count += sprintf(&buf[count], "\n");
	}
	
	fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);

	release_point_and_key(info);	
	kfree(mutual_raw_data);
	return count;
}
static ssize_t touchscreen_self_rawdata_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);
	unsigned char *self_raw_data_sense =NULL;

	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int j = 0, 
		error =0 , 
		address_offset_sense = 0,
		retry = 0,
		size_hex_sense = 0;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEOFF);	
	fts_command(info, FLUSHBUFFER);
	msleep(50);

	release_point_and_key(info);
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			VIVO_TS_LOG_ERR("fts_read_cx_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT - 1)
			{
				VIVO_TS_LOG_ERR("fts_read_cx_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	
	/* f_cnt and s_cnt ,considering first byte as dummy  byte*/
	tx_num = data[4];//M
	rx_num = data[5];//N
	
	size_hex_sense = rx_num * 2 ;
	
	/* Read Offset Address for Self Raw data */
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x1A;//Offset address for SS touch raw force line, Please verify this address from supporting fw engineer 
	fts_read_reg(info,regAdd, 3, &buff_read[0], 5);
	
	address_offset_sense = ((buff_read[4]<<8) |buff_read[3]);
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ASCII/text */
	self_raw_data_sense = (unsigned char *)kmalloc((size_hex_sense + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
	
	// ============================================ Self Raw data Reading(force) ======================================//		
	/* Read compensation data*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset_sense & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset_sense & 0xFF);
	fts_read_reg(info,regAdd, 3, self_raw_data_sense, (size_hex_sense+1));
	
	/*Copying self raw data Sense  */
		for(j = 0; j < rx_num*2; j=j+2 )
		{
			count += sprintf(&buf[count], "%u ", 
					(u16)(self_raw_data_sense[ j + 2]) << 8 | (u16)self_raw_data_sense[ j + 1]);
		}
		count += sprintf(&buf[count], "\n");
	
	fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);

	release_point_and_key(info);
	kfree(self_raw_data_sense);
	return count;
}
static ssize_t touchscreen_sensor_test_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	//unsigned char data[8];
	//unsigned char event_id = 0;
	//unsigned char tune_flag = 0;
	//unsigned char retry = 0;
	//unsigned char error ;
	//unsigned char regAdd = 0;
//	unsigned int fts_check_status = 0;
	int count = 0;
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR( "FTS Failed to get info\n");
		return  -EINVAL;
	}

	msleep(1000);
	release_point_and_key(info);
	count = fts_production_test(info, buf, count);
	release_point_and_key(info);
	VIVO_TS_LOG_INF("sensor_test result:%s\n", buf);
	
	return count;
}
static ssize_t touchscreen_delta_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);
	unsigned char *mutual_strength_data =NULL;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int count = 0;
	int j = 0, 
		i = 0,
		error =0 , 
		address_offset = 0,
		retry = 0,
		size_hex =0;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEON);	
	fts_command(info, FLUSHBUFFER);
	msleep(50); // TODO: add longer delay time after sense on ?
	release_point_and_key(info);
	
	/* Request Compensation Data,just get sense and force lines*/
	regAdd[0] = 0xB8;
	regAdd[1] = 0x20;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	
	/* Read completion event*/
	for (retry = 0; retry < READ_CNT; retry++) // poll with time-out 2200ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			VIVO_TS_LOG_ERR("fts_read_mutual_Strength_show : I2c READ ERROR : Cannot read device info\n");
			return -ENODEV;
		}
		if(1 == info->log_switch)
			VIVO_TS_LOG_INF("FTS fts status event: %02X %02X %02X %02X %02X %02X %02X %02X\n",
				data[0], data[1], data[2], data[3],
				data[4], data[5], data[6], data[7]);

		if ((data[0] == 0x13) && (data[1] == 0x20)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT - 1)
			{
				VIVO_TS_LOG_ERR("fts_read_mutual_raw_show : TIMEOUT ,Cannot read completion event\n");
			}
		}
	}	
	/* f_cnt and s_cnt */
	tx_num = data[4];//M
	rx_num = data[5];//N
	size_hex = (tx_num * rx_num)* 2 ;
	
	/* Read Offset Address for Mutual Strength data*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x04;//Offset address for MS screen Strength frame, Please verify this address from supporting fw engineer
	fts_read_reg(info,regAdd,3, &buff_read[0], 4);
	
	address_offset = ((buff_read[2]<<8) |buff_read[1]) + (rx_num *2);  
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ascii/test */
	mutual_strength_data = (unsigned char *)kmalloc((size_hex + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)

	// ============================================ Mutual strength data Reading ======================================//
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mutual_strength_data, (size_hex+1));
	
	/*Copying Mutual strength data  */
	count += sprintf(&buf[count], "tp channel: %u * %u\n", tx_num, rx_num);
	for(i = 0; i < tx_num; i++ )
	{
		for(j = 0; j < rx_num*2; j=j+2 )
		{
			count += sprintf(&buf[count], "%4d ", 
					(signed short)((u16)(mutual_strength_data[i * rx_num * 2 + j + 2]) << 8 | (u16)mutual_strength_data[i * rx_num * 2 + j + 1]));
		}
		count += sprintf(&buf[count], "\n");
	}
	
	fts_interrupt_set(info, INT_ENABLE);

	release_point_and_key(info);
	kfree(mutual_strength_data);
	return count;
}
static ssize_t touchscreen_touch_ic_name_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "%d", 12);
}

//add by liukangfei end
#if defined(BBK_DCLICK_WAKE)
static ssize_t touchscreen_dclick_simulate_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
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

	VIVO_TS_LOG_INF( "FTS parameter is %d\n", val);
	if (val == 0) {
		info->ts_dclick_simulate_switch= 0;
	}else if (val == 1) {
		info->ts_dclick_simulate_switch = 1;
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}


static ssize_t touchscreen_dclick_simulate_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "ts_dclick_simulate_switch = %d,0--off,1---on\n", info->ts_dclick_simulate_switch);
}

static ssize_t touchscreen_dclick_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF( "[%s] parameter is %d\n", __func__, val);
	if (val == 0) {
		info->dclick_switch= 0;
	}else if (val == 1) {
		info->dclick_switch = 1;
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}


static ssize_t touchscreen_dclick_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "dclick_switch = %d,0--off,1---on\n", info->dclick_switch);
}
static ssize_t touchscreen_dclick_lcd_state_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}
	
	VIVO_TS_LOG_INF( "[%s]parameter is %d\n", __func__, val);
	touchscreen_request_send(TOUCHSCREEN_REQ_ID_LCD_STATE, val);
	
	return count;
}


static ssize_t touchscreen_dclick_lcd_state_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "has_lcd_shutoff = %d,0--lcd on,1---lcd off\n", info->has_lcd_shutoff);
}
static ssize_t touchscreen_swipe_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF( "FTS parameter is %d\n", val);
	if (val == 0) {
		info->swipe_switch= 0;
	}else if (val == 1) {
		info->swipe_switch = 1;
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}


static ssize_t touchscreen_swipe_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "swipe_switch = %d,0--off,1---on\n", info->swipe_switch);
}
static ssize_t touchscreen_gesture_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF( "FTS parameter is %d\n", val);
	if (val <0x80) {
		info->gesture_switch= val;
	}else{
		VIVO_TS_LOG_ERR("Invalide parameter passed(%d)\n", val);
		return -EINVAL;
	}

	return count;
}
/*Change to dclick */
static ssize_t touchscreen_dclick_proximity_switch_store(struct kobject *kobj,
					struct kobj_attribute *attr,  const char *buf, size_t count)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	int val;

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	if (sscanf(buf, "%d", &val) != 1) {
		VIVO_TS_LOG_ERR("%s Invalide number of parameters passed\n",__func__);
		return -EINVAL;
	}

	VIVO_TS_LOG_INF( "FTS:  dclick_proxi_switch val is %d\n", val);

	if(val==0 || val==1)
		touchscreen_request_send(TOUCHSCREEN_REQ_ID_PROX_STATE,val);
	return count;
}


static ssize_t touchscreen_gesture_switch_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}
	return sprintf(buf, "0x01:LR 0x02:up 0x04:O 0x08:W 0x10:M 0x20:e 0x40:C  gesture_switch = 0x%x\n",info->gesture_switch);
}


static ssize_t touchscreen_gesture_point_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct fts_ts_info *info = container_of(kobj, struct fts_ts_info, kobject_debug);

	if(!info)
	{
		VIVO_TS_LOG_ERR("FTS Failed to get info\n");
		return  -EINVAL;
	}

	return sprintf(buf,"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d  ",
	 	gesture_data[0], gesture_data[1], gesture_data[2], gesture_data[3], gesture_data[4],
	 	gesture_data[5], gesture_data[6], gesture_data[7], gesture_data[8], gesture_data[9],
	 	gesture_data[10], gesture_data[11], gesture_data[12], gesture_data[13], gesture_data[14],
	 	gesture_data[15], gesture_data[16], gesture_data[17], gesture_data[18], gesture_data[19],
	 	gesture_data[20], gesture_data[21], gesture_data[22], gesture_data[23], gesture_data[24],
	 	gesture_data[25],	gesture_data[26], gesture_data[27], gesture_data[28], gesture_data[29],
	 	gesture_data[30], gesture_data[31], gesture_data[32], gesture_data[33], gesture_data[34],
	 	gesture_data[35], gesture_data[36], gesture_data[37], gesture_data[38], gesture_data[39],
	 	gesture_data[40], gesture_data[41], gesture_data[42], gesture_data[43], gesture_data[44],
	 	gesture_data[45],	gesture_data[46], gesture_data[47], gesture_data[48], gesture_data[49],
	 	gesture_data[50], gesture_data[51], gesture_data[52], gesture_data[53], gesture_data[54],
	 	gesture_data[55], gesture_data[56], gesture_data[57], gesture_data[58], gesture_data[59],
	 	gesture_data[60], gesture_data[61]);

}

static ssize_t touchscreen_gesture_index_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	unsigned char index;

    index = gesture_index;

	return snprintf(buf, PAGE_SIZE, "%u\n", index);

}


#endif


static ssize_t touchscreen_null_show(struct kobject *kobj,
							struct kobj_attribute *attr, char *buf)
{
		return -EINVAL;
}
static ssize_t touchscreen_null_store(struct kobject *kobj,
							struct kobj_attribute *attr,  const char *buf, size_t count)
{
		return -EINVAL;
}

//add by liukangfei start
static struct touchscreen_driver_sysfs_entry touchscreen_sensor_rawdata =
	__ATTR(sensor_rawdata, 0644,
			touchscreen_rawdata_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_sensor_self_rawdata =
	__ATTR(sensor_self_rawdata, 0644,
			touchscreen_self_rawdata_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_sensor_test =
	__ATTR(sensor_test, 0644,
			touchscreen_sensor_test_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_sensor_delta =
	__ATTR(sensor_delta, 0644,
			touchscreen_delta_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_touchpanel_device =
	__ATTR(touchpanel_devices, 0644,
			touchscreen_touch_ic_name_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_edge_suppress =
	__ATTR(edge_suppress_switch, 0644,
			touchscreen_edge_suppress_show, touchscreen_edge_suppress_store);
//add by liukangfei end


static struct touchscreen_driver_sysfs_entry touchscreen_user_defined_gesture_enable =
	__ATTR(user_defined_gesture_enable, 0644,
			touchscreen_user_defined_gesture_enable_show, touchscreen_user_defined_gesture_enable_store);

static struct touchscreen_driver_sysfs_entry touchscreen_ts_log_switch =
	__ATTR(ts_log_switch, 0644,
			touchscreen_log_switch_show, touchscreen_log_switch_store);

static struct touchscreen_driver_sysfs_entry touchscreen_firmware_version =
	__ATTR(firmware_version, 0644,
			touchscreen_version_show, touchscreen_null_store);

static struct touchscreen_driver_sysfs_entry touchscreen_firmware_update =
	__ATTR(firmware_update, 0644,
			touchscreen_null_show,touchscreen_fw_update_store);


static struct touchscreen_driver_sysfs_entry touchscreen_firmware_module_id =
	__ATTR(firmware_module_id, 0644,
			touchscreen_firmware_module_id_show, touchscreen_null_store);

static struct touchscreen_driver_sysfs_entry touchscreen_gloves_mode_switch =
	__ATTR(gloves_mode_switch, 0644,
			touchscreen_gloves_mode_switch_show, touchscreen_gloves_mode_switch_store);

static struct touchscreen_driver_sysfs_entry touchscreen_power_state =
	__ATTR(ts_power_state, 0644,
			touchscreen_power_state_show, touchscreen_power_state_store);

static struct touchscreen_driver_sysfs_entry touchscreen_interrupt_enable =
	__ATTR(chip_int_enable, 0644,
			touchscreen_interrupt_enable_show, touchscreen_interrupt_enable_store);

static struct touchscreen_driver_sysfs_entry touchscreen_sensor_rx_tx =
	__ATTR(sensor_rx_tx, 0644,
			touchscreen_sensor_rx_tx_show, touchscreen_null_store);
#if defined (BBK_LARGE_SUPRESSION)
static struct touchscreen_driver_sysfs_entry touchscreen_fts_is_calling =
	__ATTR(ts_is_calling, 0644,
			fts_is_calling_show, fts_is_calling_store);
#endif
#if defined(BBK_DCLICK_WAKE)
static struct touchscreen_driver_sysfs_entry touchscreen_dclick_switch =
	__ATTR(dclick_switch, 0644,
			touchscreen_dclick_switch_show, touchscreen_dclick_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_dclick_lcd_state =
	__ATTR(dclick_lcd_state, 0644,
			touchscreen_dclick_lcd_state_show, touchscreen_dclick_lcd_state_store);
static struct touchscreen_driver_sysfs_entry touchscreen_swipe_switch =
	__ATTR(swipe_switch, 0644,
			touchscreen_swipe_switch_show, touchscreen_swipe_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_gesture_point =
	__ATTR(gesture_point, 0644,
			touchscreen_gesture_point_show, touchscreen_null_store);
static struct touchscreen_driver_sysfs_entry touchscreen_gesture_index =
	__ATTR(gesture_index, 0644,
			touchscreen_gesture_index_show, touchscreen_null_store);         // add by terry 150313

static struct touchscreen_driver_sysfs_entry touchscreen_gesture_switch =
	__ATTR(gesture_switch, 0644,
			touchscreen_gesture_switch_show, touchscreen_gesture_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_dclick_proximity_switch =
	__ATTR(dclick_proximity_switch, 0644,
			touchscreen_null_show, touchscreen_dclick_proximity_switch_store);
static struct touchscreen_driver_sysfs_entry touchscreen_dclick_simulate_switch =
	__ATTR(dclick_simulate_switch, 0644,
			touchscreen_dclick_simulate_switch_show, touchscreen_dclick_simulate_switch_store);

#endif

//user define gesture
//add by chenpeng
static struct touchscreen_driver_sysfs_entry touchscreen_userdefine_gesture_template = 
	__ATTR(gesture_template, 0644, fts_custom_gesture_template_show, fts_custom_gesture_template_store);	
static struct touchscreen_driver_sysfs_entry touchscreen_userdefine_gesture_remove = 
	__ATTR(gesture_remove, 0644, fts_custom_gesture_remove_show, fts_custom_gesture_remove_store);	
static struct touchscreen_driver_sysfs_entry touchscreen_userdefine_gesture_template_valid = 
	__ATTR(template_valid, 0644, fts_custom_gesture_valid_show, fts_custom_gesture_valid_store);	


static struct attribute *our_own_sys_attrs[] = {
//add by liukangfei start
	&touchscreen_sensor_test.attr,
	&touchscreen_edge_suppress.attr,
	&touchscreen_sensor_rawdata.attr,
	&touchscreen_sensor_self_rawdata.attr,
	&touchscreen_sensor_delta.attr,
	&touchscreen_touchpanel_device.attr,
//add by liukangfei end

	&touchscreen_user_defined_gesture_enable.attr,
	&touchscreen_firmware_version.attr,
	&touchscreen_firmware_update.attr,
	&touchscreen_ts_log_switch.attr,
	&touchscreen_power_state.attr,
	&touchscreen_firmware_module_id.attr,
	&touchscreen_gloves_mode_switch.attr,
	&touchscreen_interrupt_enable.attr,
	&touchscreen_sensor_rx_tx.attr,
	#if defined (BBK_LARGE_SUPRESSION)
	&touchscreen_fts_is_calling.attr,
	#endif
	#if defined(BBK_DCLICK_WAKE)
	&touchscreen_dclick_switch.attr,
	&touchscreen_dclick_lcd_state.attr,
	&touchscreen_swipe_switch.attr,
	&touchscreen_gesture_point.attr,
	&touchscreen_gesture_index.attr,
	&touchscreen_gesture_switch.attr,
	&touchscreen_dclick_proximity_switch.attr,
	&touchscreen_dclick_simulate_switch.attr,
	#endif
	NULL
};
static ssize_t touchscreen_debug_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}
static ssize_t touchscreen_debug_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}

static void touchscreen_debug_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}
static const struct sysfs_ops touchscreen_debug_object_sysfs_ops = {
	.show = touchscreen_debug_object_show,
	.store = touchscreen_debug_object_store,
};
static struct kobj_type touchscreen_debug_object_type = {
	.sysfs_ops	= &touchscreen_debug_object_sysfs_ops,
	.release	= touchscreen_debug_object_release,
	.default_attrs = our_own_sys_attrs,
};

//guest
static struct attribute *touchscreen_guest_attrs[] = {
	//user define gesture
	&touchscreen_userdefine_gesture_template.attr,
	&touchscreen_userdefine_gesture_remove.attr,	
	&touchscreen_userdefine_gesture_template_valid.attr,
	
	NULL
};

static ssize_t touchscreen_guest_object_show(struct kobject *k, struct attribute *attr, char *buf)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->show)
		ret = kobj_attr->show(k, kobj_attr, buf);

	return ret;
}
static ssize_t touchscreen_guest_object_store(struct kobject *k, struct attribute *attr,
			      const char *buf, size_t count)
{
	struct kobj_attribute *kobj_attr;
	int ret = -EIO;

	kobj_attr = container_of(attr, struct kobj_attribute, attr);

	if (kobj_attr->store)
		ret = kobj_attr->store(k, kobj_attr, buf, count);

	return ret;
}
static void touchscreen_guest_object_release(struct kobject *kobj)
{
	/* nothing to do temply */
	return;
}
static const struct sysfs_ops touchscreen_guest_object_sysfs_ops = {
	.show = touchscreen_guest_object_show,
	.store = touchscreen_guest_object_store,
};
static struct kobj_type touchscreen_guest_object_type = {
	.sysfs_ops	= &touchscreen_guest_object_sysfs_ops,
	.release	= touchscreen_guest_object_release,
	.default_attrs = touchscreen_guest_attrs,
};

static int touchscreen_creat_our_own_sys_file(struct fts_ts_info *info)
{
	int ret;

	ret = kobject_init_and_add(&info->kobject_debug, &touchscreen_debug_object_type,
					NULL, "touchscreen");
	if (ret) {
		VIVO_TS_LOG_INF("%s: Create kobjetct error!\n", __func__);
		return -1;
	}

	//add guest gesture dir
	ret = kobject_init_and_add(&info->kobject_guest, &touchscreen_guest_object_type, &info->kobject_debug, "guest");
	if (ret) {
		VIVO_TS_LOG_INF("%s: Create kobjetct guest error!\n", __func__);
		return -1;
	}	
    return 0;
}
static void touchscreen_log_switch_set(bool on)
{
	if(info_gl== NULL)
	{
		VIVO_TS_LOG_INF("[CYTTSPQQ] : Fail to get info_gl\n");
		return;
	}
	info_gl->log_switch  = on;
}

static struct bbk_drivers_callback_handler touchscreen_log_switch_handler = {
	.name = "ts_driver",
	.callback = touchscreen_log_switch_set,
};

extern unsigned int is_atboot;
extern unsigned int power_off_charging_mode;

//add for charge judge and log_switch interface. fts_common_data will be register in probe
static int fts_get_log_switch(void)
{
	return info_gl->log_switch;
}
static void charger_work_handler(struct work_struct *work)
{
	VIVO_TS_LOG_INF("[%s]charger work run.\n", __func__);
	bbk_rewrite_usb_charger_flag_tochip(info_gl);
}

void fts_charger_connect_judge(char on_or_off) 
{
	VIVO_TS_LOG_INF("[%s]charger detelct.\n", __func__);
	info_gl->usb_charger_flag = (int)on_or_off;

	/* just usb change,we change the glove_switch's value */ 
	if(1 == info_gl->usb_charger_flag) {
		info_gl->glove_mode_state_save_in_charging = info_gl->glove_mode_switch;
		info_gl->glove_mode_switch = 0;
	} else if (0 == info_gl->usb_charger_flag) {
		info_gl->glove_mode_switch = info_gl->glove_mode_state_save_in_charging;	
	}
	

	/* add for firmware updating,while updating,not charger state switch. */
	if(info_gl->firmware_updating_flag) {
		VIVO_TS_LOG_INF("[%s]firmware is updating.no charger state switch.\n", __func__);	
		return;
	}	

	if(info_gl->is_probe_complete == 1) {
		schedule_work(&info_gl->charger_work);
	}
}
static vivo_touchscreen_common_data fts_common_data = {
	.driver_name = DRIVER_NAME,
	.charge_connect_judge = NULL,
    	.get_ts_log_switch = fts_get_log_switch,
    	.charge_connect_judge = fts_charger_connect_judge,
};

static unsigned char fts_cap_button_codes[] = {KEY_MENU, 172, KEY_BACK};
static struct fts_cap_button_map cap_button_map = {
	.nbuttons = ARRAY_SIZE(fts_cap_button_codes),
	.map = fts_cap_button_codes,
};
static int fts_standard_resume(struct fts_ts_info *info);
static int fts_standard_suspend(struct fts_ts_info *info);
int fts_kthread_resume_suspend(void *data, int state)
{
     struct fts_ts_info *info = (struct fts_ts_info *)data;
      int ret;
	
	if(state) {
		ret = fts_standard_resume(info);
		if(ret<0) {
			VIVO_TS_LOG_INF("[%s]:resume fail\n", __func__);
		}
	} else {
		ret = fts_standard_suspend(info);
		if(ret){
			VIVO_TS_LOG_INF("[%s]:suspend fail\n", __func__);
		}
	}

	return 0;	
}
int fts_kthread_dclick_proximity_switch(void *data, int state)
{
      int dclick_switch = state;
      struct fts_ts_info * info =  (struct fts_ts_info *)data;

	if (atomic_read(&info->ts_state) == TOUCHSCREEN_NORMAL) {
		info->need_change_to_dclick = dclick_switch;
		VIVO_TS_LOG_DBG("[%s] Not first switch set in normal mode\n", __func__);
		return 0;
	}

	/* add for firmware updating,while updating,not change proximity state. */
	if(info->firmware_updating_flag) {
		VIVO_TS_LOG_INF("[%s]firmware is updating.no proximity switch.\n", __func__);	
		return 0;
	}	
	
	/* normal change state */
	ts_scan_switch(dclick_switch);

	return 0;
}
int fts_kthread_dclick_lcd_state(void *data, int state)
{
	struct fts_ts_info * info =  (struct fts_ts_info *)data;

	/* add for super power mode,in this mode,we must no gesture wakeup 
	  * quick lcd on/off,system not call resume and suspend,must clear last value state.
	  **/
	//if(state == 1)
		//info->need_change_to_dclick = 0;
	VIVO_TS_LOG_INF("[%s]kthread lcd.lcd state=%d\n", __func__, state);
	
	if (state == 0 ||state == 1) {		
		info->has_lcd_shutoff = (!state);	//high layer write lcd's current state.
	}else{
		VIVO_TS_LOG_ERR( "[%s]Proximity node set invalide parameter passed:%d\n", __func__, state);
		return -EINVAL;
	}
	return 0;
}

/* register request functions */
static int fts_kthread_register_resp_funs(struct fts_ts_info *info)
{
	touchscreen_set_priv_data((void *)info);
	touchscreen_register_resp_func(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND, fts_kthread_resume_suspend);
	touchscreen_register_resp_func(TOUCHSCREEN_REQ_ID_LCD_STATE, fts_kthread_dclick_lcd_state);
	touchscreen_register_resp_func(TOUCHSCREEN_REQ_ID_PROX_STATE, fts_kthread_dclick_proximity_switch);
//	touchscreen_register_resp_func(TOUCHSCREEN_REQ_ID_USB_CHARGE,NULL);

	return 0;
}

/*bbk add end*/
static int fts_probe(struct i2c_client *client, const struct i2c_device_id *idp)
{
	struct fts_ts_info *info = NULL;
	struct fts_i2c_platform_data *pdata = &pdata_gl;
	char fts_ts_phys[64];
	int error = 0;

	VIVO_TS_LOG_INF("[%s]:step 0.i2c info:1.addr=0x%x\n", __func__, client->addr);
 
	VIVO_TS_LOG_INF("[%s]:step 1.get product name\n", __func__);
	vivo_touchscreen_get_product_name(&global_product_name);
	if(NULL == global_product_name) {
		VIVO_TS_LOG_ERR("[%s]no product name! please check device tree\n", __func__);
		return -1;
	}
	VIVO_TS_LOG_INF("[%s]product name:%s\n", __func__, global_product_name);

	pdata->client = client;
	/* different product do defferent process */
	if(!strcmp(global_product_name, "PD1516A")) {
		VIVO_TS_LOG_INF("[%s]get PD1516A regular.\n", __func__);
		/* PD1516A get regular */
		pdata->vcc_ana = regulator_get(&pdata->client->dev, "vdd_ana");
		if (IS_ERR(pdata->vcc_ana)) {
			error = PTR_ERR(pdata->vcc_ana);
			VIVO_TS_LOG_ERR("[%s]Regulator get failed vcc_ana rc=%d\n", __func__, error);
			return error;
		}

		//__1.8v
		pdata->vcc_i2c = regulator_get(&pdata->client->dev, "vcc_i2c");
		if (IS_ERR(pdata->vcc_i2c)) {
			error = PTR_ERR(pdata->vcc_i2c);
			VIVO_TS_LOG_ERR("Regulator get failed vcc_i2c rc=%d\n", error);
			return error;
		}
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		VIVO_TS_LOG_ERR("Unsupported I2C functionality\n");
		error = -EIO;
		goto ProbeErrorExit_0;
	}

	/* get info from dtsi */
	error = fts_parse_dt(&client->dev, pdata);
	if (error){
		VIVO_TS_LOG_ERR("fts:failed to parse dt info!\n");
		error = -ENODEV;
		goto ProbeErrorExit_0;
	}
	//client->dev.platform_data = pdata;
	pdata->cap_button_map = &cap_button_map;

	VIVO_TS_LOG_INF("fts:fts_probe1*******\n");

	info = kzalloc(sizeof(struct fts_ts_info), GFP_KERNEL);
	if (!info) {
		VIVO_TS_LOG_ERR("Out of memory\n");
		error = -ENOMEM;
		goto ProbeErrorExit_0;
	}
	info_gl = info;	//add by bbk
	
	wake_lock_init(&info->wakelock, WAKE_LOCK_SUSPEND, "smartwake");	

	/* large press touch id init -1 must */
	info->large_press_touch_id = -1;	
	info->log_switch = 0;
	info->power_state= 1;
	info->glove_mode_switch  = 0;	// default close.

	/* probe finsh flag */
	info->is_probe_complete = 0;
	
	#if defined(BBK_DCLICK_WAKE)
	info->dclick_switch = 1;
	info->gesture_switch = 0;
	info->ts_dclick_simulate_switch = 0;
	atomic_set(&info->ts_state, TOUCHSCREEN_NORMAL);
	info->has_lcd_shutoff = 0;
	info->swipe_switch = 0;
	info->need_change_to_dclick = 0;

	setup_timer(&info->dclick_timer, rmi_f11_dclick_timer_func, (unsigned long)&info);
	INIT_WORK(&info->dclick_timer_work, rmi_f11_dclick_timer_work_func);
	info->has_dclick_timer_start = false;
	info->is_dclick_valide = false;
	info->pressed_count = 0;
	info->release_count = 0;
	info->first_press_x = -1;
	info->first_press_y = -1;
	info->second_press_x = -1;
	info->second_press_y = -1;
	info->pre_state = 0;
	info->dclick_dimension_x_min = 140;
	info->dclick_dimension_x_max = X_AXIS_MAX - 140;
	info->dclick_dimension_y_min = 140;
	info->dclick_dimension_y_max = Y_AXIS_MAX - 140;
	#endif

	/* init 0d to 2d and 2d to 0d swip */
	setup_timer(&info->home_timer, home_timer_func, (unsigned long)&info);
	setup_timer(&info->menu_timer, menu_timer_func, (unsigned long)&info);
	setup_timer(&info->back_timer, back_timer_func, (unsigned long)&info);
	INIT_WORK(&info->home_work, home_work_handler);
	INIT_WORK(&info->back_work, back_work_handler);
	INIT_WORK(&info->menu_work, menu_work_handler);
	
	/* charger work */
	INIT_WORK(&info->charger_work, charger_work_handler);

	/* init input report mutex,must do. */
	mutex_init(&(info->input_report_mutex));	

	/* init i2c and chip reset mutex,must do. */
	mutex_init(&(info->i2c_reset_mutex));

	VIVO_TS_LOG_INF("fts:fts_probe2*******\n");
	info->event_wq = create_singlethread_workqueue("fts-event-queue");
	if (!info->event_wq) {
		VIVO_TS_LOG_ERR("Cannot create work thread\n");
		error = -ENOMEM;
		goto ProbeErrorExit_1;
	}
	VIVO_TS_LOG_INF("fts:fts_probe3*******\n");

	INIT_WORK(&info->work, fts_event_handler);
	pdata->client = client;
	info->client = client;
	//client->timing = 400;
	i2c_set_clientdata(client, info);


	INIT_WORK(&info->irq_err_work, fts_irq_err_work_func);

	info->irq_err_workqueue = create_singlethread_workqueue("fts_err_wq");
	if (!info->irq_err_workqueue) {
		VIVO_TS_LOG_ERR("%s: can't create irq err worqueue\n",__func__);
		goto ProbeErrorExit_2;
	}	

	VIVO_TS_LOG_INF("[%s]chip power on.\n", __func__);
	error = fts_power_on(1);
	if (error) {
		VIVO_TS_LOG_ERR("[%s]failed to power on.\n", __func__);
		error = -ENODEV;
		goto ProbeErrorExit_2;
	}

	error = fts_config_gpio(pdata,1);
	if (error) {
		VIVO_TS_LOG_ERR("[%s]failed to config gpio.\n", __func__);
		error = -ENODEV;
		goto ProbeErrorExit_2;
	}
	
	mdelay(200);//add for wait ic ready
	info->dev = &info->client->dev;
#if 0	
	if(fts_check_fw_version_and_is_ic_connected(info))
	{
		VIVO_TS_LOG_ERR("[%s]:failed read fw version.So no ic has connected.\n", __func__);
		goto ProbeErrorExit_freegpio;
	}
#endif

	VIVO_TS_LOG_INF("fts:*********fts_probe4 *******\n");
	info->client->irq = pdata->attn_gpio;//add by qiuguifu
//	info->dev = &info->client->dev;
	info->input_dev = input_allocate_device();
	if (!info->input_dev) {
		VIVO_TS_LOG_ERR("No such device\n");
		error = -ENODEV;
		goto ProbeErrorExit_freegpio;
	}
	info->input_dev->dev.parent = &client->dev;

	info->input_dev->name = FTS_TS_DRV_NAME;
	snprintf(fts_ts_phys, sizeof(fts_ts_phys), "%s/input0",
			 info->input_dev->name);
	info->input_dev->phys = fts_ts_phys;
	info->input_dev->id.bustype = BUS_I2C;
	info->input_dev->id.vendor = 0x0001;
	info->input_dev->id.product = 0x0002;
	info->input_dev->id.version = 0x0100;


	VIVO_TS_LOG_INF("fts:*********fts_probe5*******\n");

	__set_bit(EV_SYN, info->input_dev->evbit);
	__set_bit(EV_KEY, info->input_dev->evbit);
	__set_bit(EV_ABS, info->input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, info->input_dev->propbit);//qiuguifu add
	input_mt_init_slots(info->input_dev, TOUCH_ID_MAX,0);
	input_set_abs_params(info->input_dev, ABS_MT_TRACKING_ID,
					 0, FINGER_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_X,
					 X_AXIS_MIN, X_AXIS_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_POSITION_Y,
					 Y_AXIS_MIN, Y_AXIS_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_TOUCH_MAJOR,
					 AREA_MIN, AREA_MAX, 0, 0);
	input_set_abs_params(info->input_dev, ABS_MT_PRESSURE,
					 PRESSURE_MIN, 63, 0, 0);
	input_set_capability(info->input_dev, EV_KEY, KEY_WAKEUP);
	input_set_capability(info->input_dev, EV_KEY, KEY_M);
	input_set_capability(info->input_dev, EV_KEY, KEY_O);
	input_set_capability(info->input_dev, EV_KEY, KEY_E);
	input_set_capability(info->input_dev, EV_KEY, KEY_W);
	input_set_capability(info->input_dev, EV_KEY, KEY_C);

	input_set_capability(info->input_dev, EV_KEY, KEY_LEFT);
	input_set_capability(info->input_dev, EV_KEY, KEY_RIGHT);
	input_set_capability(info->input_dev, EV_KEY, KEY_UP);
	input_set_capability(info->input_dev, EV_KEY, KEY_WAKEUP_SWIPE);
	input_set_capability(info->input_dev, EV_KEY, KEY_TS_LARGE_SUPPRESSION);

	input_set_capability(info->input_dev, EV_KEY, KEY_MENU);
	input_set_capability(info->input_dev, EV_KEY, KEY_HOMEPAGE);
	input_set_capability(info->input_dev, EV_KEY, KEY_BACK);

	input_set_capability(info->input_dev, EV_KEY, KEY_CUSTOM_GESTURE);

	/* Active Stylus */
//	input_set_capability(info->input_dev, EV_KEY, BTN_STYLUS);
//	input_set_capability(info->input_dev, EV_KEY, BTN_STYLUS2);

//	input_set_abs_params(info->input_dev, ABS_MT_TOOL_TYPE,
	//					 0, MT_TOOL_MAX, 0, 0);

//	input_set_abs_params(info->input_dev, ABS_MT_TOOL_TYPE,
//						 0, MT_TOOL_MAX, 0, 0);
	VIVO_TS_LOG_INF("fts:*********fts_probe6 *******\n");

	/* register the multi-touch input device */
	error = input_register_device(info->input_dev);
	if (error) {
		VIVO_TS_LOG_ERR("No such device\n");
		error = -ENODEV;
		goto ProbeErrorExit_freegpio;
	}

	/* track slots */
	info->touch_id = 0;

	/* track buttons */
	info->buttons = 0;

	/* before system reset,must check crc,ensure firmware is right,else controler ready will timeout */
	if(cx_crc_check(info) != 0) {
		/* avoid bad update firmware in chip */
			VIVO_TS_LOG_INF("[%s]CRC Error.firmware need update.bad fw in chip\n", __func__);
			error = fts_fw_upgrade_and_init(info,2,NULL);//update release and config add by bbk.
			if(error != 0) {
				VIVO_TS_LOG_ERR("[%s]firmware update or init flash reload fail.\n", __func__);
				goto ProbeErrorExit_freegpio;
			}
	}
	/* init hardware device */
	VIVO_TS_LOG_INF("fts:*********fts_probe7 *******\n");
	error = fts_init(info);
	if (error) {
		VIVO_TS_LOG_ERR("Cannot initialize the device\n");
		error = -ENODEV;
		goto ProbeErrorExit_3;
	}
	fts_interrupt_disable(info);
	VIVO_TS_LOG_INF("fts:*********fts_probe8 *******\n");
	error = fts_init_hw(info);
	if (error) {
		VIVO_TS_LOG_ERR("Cannot initialize the hardware device\n");
		error = -ENODEV;
		goto ProbeErrorExit_3;
	}

	/* register kthread request function */
	fts_kthread_register_resp_funs(info);

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	info->early_suspend.suspend = fts_early_suspend;
	info->early_suspend.resume = fts_late_resume;
	register_early_suspend(&info->early_suspend);
#endif

#if defined(CONFIG_FB) //bbk add
	info->fb_notif.notifier_call = fts_fb_notifier_callback;

	error = fb_register_client(&info->fb_notif);

	if (error)
		VIVO_TS_LOG_ERR("Unable to register fb_notifier: %d\n",error);
#endif /* CONFIG_FB */

	VIVO_TS_LOG_INF("fts:*********Enter upgrade fw.*******\n");
	error=fts_get_ic_fw_base_version(info);
	if(error<0)
		VIVO_TS_LOG_ERR("fts_get_ic_fw_base_version error: %d\n",error);

	error=fts_get_ic_fw_config_version(info);
	if(error<0)
		VIVO_TS_LOG_ERR("fts_get_ic_fw_config_version error: %d\n",error);

	info->driver_fw_base_version = fts_get_driver_fw_base_version();
	info->driver_fw_config_version= fts_get_driver_fw_config_version();

	VIVO_TS_LOG_INF("ic's fw base version = 0x%x , ic's fw config version =%04x \n", info->ic_fw_base_version, info->ic_fw_config_version);
	VIVO_TS_LOG_INF("driver's fw base version = 0x%x , driver's fw config version =%04x \n", info->driver_fw_base_version, info->driver_fw_config_version);

	if(!is_atboot){
		if(info->ic_fw_base_version != info->driver_fw_base_version) {
			VIVO_TS_LOG_INF("fw base version in ic is not equal fw base version in driver,so upate firmware\n");
			error = fts_fw_upgrade_and_init(info,2,NULL);//update release and config add by bbk.
			if(error != 0) {
				VIVO_TS_LOG_ERR("[%s]firmware update or init flash reload fail.\n", __func__);
				goto ProbeErrorExit_3;
			}
		} else if (info->ic_fw_config_version!= info->driver_fw_config_version) {
			VIVO_TS_LOG_INF("fw config version in ic is not equal fw config version in driver,so upate firmware\n");
			error = fts_fw_upgrade_and_init(info,2,NULL);//update release and config add by bbk.
			if(error != 0) {
				VIVO_TS_LOG_ERR("[%s]firmware update or init flash reload fail.\n", __func__);
				goto ProbeErrorExit_3;
			}
		} else if(cx_crc_check(info) != 0) {
			/* avoid bad update firmware in chip */
			VIVO_TS_LOG_INF("[%s]CRC Error.firmware need update.bad fw in chip\n", __func__);
			error = fts_fw_upgrade_and_init(info,2,NULL);//update release and config add by bbk.
			if(error != 0) {
				VIVO_TS_LOG_ERR("[%s]firmware update or init flash reload fail.\n", __func__);
				goto ProbeErrorExit_3;
			}
		} else {
			VIVO_TS_LOG_INF("[%s]base fw version,fw config version,and crc is right.no need update fw.\n", __func__);
		}
		VIVO_TS_LOG_INF("fts:*********fts_probe9 *******\n");
	}else{
		VIVO_TS_LOG_INF("fts:boot mode is AT mode. So not up firmware.\n");
	}

	fts_command(info, KEY_ON);

	fts_interrupt_enable(info);
/*bbk qiuguif add for debug*/

	error = touchscreen_creat_our_own_sys_file(info);
	if (error < 0) {
		VIVO_TS_LOG_ERR("%s: Failed to create our own sysfs file\n",
				__func__);
		goto ProbeErrorExit_3;
	}
	error = bbk_drivers_log_switch_register_callback(&touchscreen_log_switch_handler);
	if(error < 0)
	{
		VIVO_TS_LOG_ERR("%s: Fail set log switch bbk  r=%d\n",
			__func__, error);
	}
/*bbk qiuguif add for debug end*/
/*bbk add */
	properties_kobj = kobject_create_and_add("board_properties",NULL);
	if (properties_kobj)
		error = sysfs_create_group(properties_kobj,&fts_properties_attr_group);
	if (!properties_kobj ||error)
	{
		VIVO_TS_LOG_ERR("%s: failed to create board_properties\n",__func__);
	}
/*bbk add end*/
	/* sysfs stuff */
	info->attrs.attrs = fts_attr_group;
	error = sysfs_create_group(&client->dev.kobj, &info->attrs);
	if (error) {
		VIVO_TS_LOG_ERR("Cannot create sysfs structure\n");
		error = -ENODEV;
		goto ProbeErrorExit_3;
	}
VIVO_TS_LOG_INF("fts:*********fts_probe10*******\n");
	i2c_cmd_class = class_create(THIS_MODULE,FTS_TS_DRV_NAME);
	info->i2c_cmd_dev = device_create(i2c_cmd_class,
						NULL, FTS_ID0, info, "fts_i2c");
	if (IS_ERR(info->i2c_cmd_dev))
	{
		VIVO_TS_LOG_ERR("FTS Failed to create device for the sysfs\n");
		goto ProbeErrorExit_4;
	}

	dev_set_drvdata(info->i2c_cmd_dev, info);
	error = sysfs_create_group(&info->i2c_cmd_dev->kobj,
							&i2c_cmd_attr_group);
	if (error)
	{
		VIVO_TS_LOG_ERR("FTS Failed to create sysfs group\n");
		goto ProbeErrorExit_5;
	}

	register_touchscreen_common_interface(&fts_common_data);

	info->is_probe_complete = 1;
	//while probe done we must check is or no charger
	bbk_rewrite_usb_charger_flag_tochip(info);
	return 0;

/* error exit path */
ProbeErrorExit_5:
	device_destroy(i2c_cmd_class, FTS_ID0);

ProbeErrorExit_4:
	sysfs_remove_group(&client->dev.kobj, &info->attrs);
ProbeErrorExit_3:
	input_unregister_device(info->input_dev);
	fts_command(info, KEY_OFF);
	fts_command(info, SENSEOFF);

ProbeErrorExit_freegpio:
	fts_config_gpio(pdata,0);// free gpio.
ProbeErrorExit_2:
	destroy_workqueue(info->event_wq);

ProbeErrorExit_1:
	wake_lock_destroy(&info->wakelock);
	kfree(info);

ProbeErrorExit_0:
	dev_err(&client->dev, "Probe failed.\n");

	return error;
}


static int fts_remove(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	/* sysfs stuff */
	sysfs_remove_group(&client->dev.kobj, &info->attrs);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&info->early_suspend);
#endif

	/* remove interrupt and event handlers */
	fts_interrupt_uninstall(info);

	/* Empty the FIFO buffer */
	fts_command(info, FLUSHBUFFER);

	/* Remove the work thread */
	destroy_workqueue(info->event_wq);

	/* unregister the device */
	input_unregister_device(info->input_dev);

	/* Power-off the device */
	if ((info->power) && (info->power(FTS_POWER_OFF)))
		dev_warn(info->dev, "Cannot power-off the device\n");

	/* free all */
	kfree(info);

	return 0;
}

#if 0
static int fts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);
	int i;
		if(!info)
		{
			VIVO_TS_LOG_ERR("[fts]:%s fail to get info.\n",__func__);
			return 1;
		}
	VIVO_TS_LOG_INF("[fts]:%s\n",__func__);
	fts_interrupt_enable(info);
	fts_interrupt_disable(info);

	/* Release all buttons */
	info->buttons = 0;

	/* Release all slots */
	for (i = 0; i < TOUCH_ID_MAX; i++)
		if (__test_and_clear_bit(i, &info->touch_id)) {
			input_mt_slot(info->input_dev, i);
			input_mt_report_slot_state(info->input_dev,
			(i < FINGER_MAX) ? MT_TOOL_FINGER : MT_TOOL_PEN, 0);
		}
	input_sync(info->input_dev);

	/* No need ot check for error code */
	cancel_work_sync(&info->work);

	/* Read out device mode, used when resuming device */
	fts_get_mode(info);

	/* suspend the device and flush the event FIFO */
	fts_command(info, SLEEPIN);
	fts_command(info, FLUSHBUFFER);

	fts_command(info, ENTER_GESTURE_MODE);// bbk add

	if ((info->power) && (info->power(FTS_POWER_OFF)))
		dev_warn(info->dev, "Cannot power-off device\n");

	/* ignore errors */
	return 0;
}


static int fts_resume(struct i2c_client *client)
{
	struct fts_ts_info *info = i2c_get_clientdata(client);

	/* Power-on the device */
	if ((info->power) && (info->power(FTS_POWER_ON))) {
		VIVO_TS_LOG_ERR("Cannot power-on device\n");
		return -ENODEV;
	}
	VIVO_TS_LOG_INF("[fts]:%s\n",__func__);
	/* enable interrupts */
	fts_interrupt_enable(info);

	/* wake-up the device */
	init_completion(&info->cmd_done);
	fts_command(info, SLEEPOUT);
	WAIT_WITH_TIMEOUT(info, HZ, SLEEPOUT);

	/* enable sense */
	fts_command(info, SENSEON);

	/* put back the device in the original mode (see fts_suspend()) */
	switch (info->mode) {
	case MODE_PROXIMITY:
		fts_command(info, PROXIMITY_ON);
		break;

	case MODE_HOVER:
		fts_command(info, HOVER_ON);
		break;

	case MODE_GESTURE:
		fts_command(info, GESTURE_ON);
		break;

	case MODE_HOVER_N_PROXIMITY:
		fts_command(info, HOVER_ON);
		fts_command(info, PROXIMITY_ON);
		break;

	case MODE_GESTURE_N_PROXIMITY:
		fts_command(info, GESTURE_ON);
		fts_command(info, PROXIMITY_ON);
		break;
	case MODE_GESTURE_N_PROXIMITY_N_HOVER:
		fts_command(info, HOVER_ON);
		fts_command(info, GESTURE_ON);
		fts_command(info, PROXIMITY_ON);
		break;

	default:
		dev_warn(info->dev, "Invalid device mode - 0x%02x\n",
				info->mode);
		break;
	}

	return 0;
}
#else
static int fts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int fts_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static int fts_check_into_gesture_mode(struct fts_ts_info *info) 
{
	int ret = 0;
	
	mdelay(10);
	if(!fts_command(info, ENTER_GESTURE_MODE)) {// bbk add
		VIVO_TS_LOG_INF("[fts]:%s ENTER_GESTURE_MODE success\n",__func__);
	} else {
		VIVO_TS_LOG_INF("[fts]:%s ENTER_GESTURE_MODE fail\n",__func__);
		return -1;
	}

	/* send esd protect cmd to change internal circuit after check into gesture mode*/
	ret = fts_esd_protect(info);
	if(ret == 0) {
		VIVO_TS_LOG_INF("[%s]fts_esd_protect success\n",__func__);
	} else {
		VIVO_TS_LOG_INF("[%s]fts_esd_protect fail\n",__func__);
		return -1;
	}
	return 0;
}

static unsigned char parameter_data[8] = {0};
static int fts_get_noise_parameter(struct fts_ts_info *info) {
	unsigned char reg_write[2] = {0xc8, 0x01};
	//unsigned char reg_read[2] = {0x85, 0x08};
	unsigned char addr = 0;
	unsigned char data[8] = {0};
	int i = 0;
	int j = 0;
	int ret = 0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info_gl, FLUSHBUFFER);
	//flush buffer
	ret = fts_write_reg(info, reg_write, sizeof(reg_write));
	if (ret) {
		VIVO_TS_LOG_ERR("%s Cannot write reg\n",__func__);
		return -EINVAL;		
	}
	msleep(10);

	addr = READ_ONE_EVENT;
	for(j=0; j<10; j++) {
		ret = fts_read_reg(info, &addr, 1, data, FTS_EVENT_SIZE);
		if (ret) {
			VIVO_TS_LOG_ERR("read : Cannot read device info\n");
			return -ENODEV;
		}

		VIVO_TS_LOG_INF( "[%s]get data:%x %x %x %x %x %x %x %x\n", __func__, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
	
		if(data[0]==0x17 && data[1]==0x01) {	
			for(i=0; i<8; i++) {
				parameter_data[i] = data[i];	
			}
			break;
		}
		
	}
	fts_interrupt_set(info, INT_ENABLE);

	return 0;
	
}
static int fts_set_noise_parameter(struct fts_ts_info *info) {
	int ret = 0;
	unsigned char addr = 0;
	unsigned char data[8] = {0};
	int i = 0;

	fts_command(info, FLUSHBUFFER);
	
	parameter_data[0] = 0xc7;
	parameter_data[1] = 0x01;
	parameter_data[7] = 0x00;
	ret = fts_write_reg(info, parameter_data, 6);
	if (ret) {
		VIVO_TS_LOG_ERR("%s Cannot write reg\n",__func__);
		return -EINVAL;		
	}

	addr = READ_ONE_EVENT;
	for(i=0; i<10; i++) {
		ret = fts_read_reg(info, &addr, 1, data, FTS_EVENT_SIZE);
		if (ret) {
			VIVO_TS_LOG_ERR("read : Cannot read device info\n");
			return -ENODEV;
		}
		
		VIVO_TS_LOG_INF( "[%s]get data:%x %x %x %x %x %x %x %x\n", __func__, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
		if(data[0]==0x18 && data[1]==1) {
			break;
		}
	}
	
	return 0;
}
static int fts_standard_suspend(struct fts_ts_info *info)
{
	int glove_mode_state_save = 0;
	
	VIVO_TS_LOG_INF("[fts]:%s suspend enter.\n",__func__);

	release_point_and_key(info);//add by qiuguifu
	/* must release large press mode,else tp maybe no function */
	info->large_press_touch_id = -1;
	info->is_large_press_mode = 0;

	/* while firmeware updating,do not execute suspend process */
	if(info->firmware_updating_flag) {
		VIVO_TS_LOG_INF("[%s]firmware is updating.no suspend\n", __func__);	
		return 0;
	}
	
	if(info->dclick_switch == 0 && info->gesture_switch==0 && 
   	   info->swipe_switch==0 &&	info->user_define_gesture_switch == 0) {
		info->need_change_to_dclick = 0;
	}
	
	if(info->need_change_to_dclick) {
		VIVO_TS_LOG_INF("fts_standard_suspend:change to gesture mode!\n");
		atomic_set(&info->ts_state, TOUCHSCREEN_GESTURE);

		if(fts_command(info_gl, KEY_OFF)) {
			VIVO_TS_LOG_ERR("%s KEY_OFF failed \n", __func__);
		}
		
		//must be first write template
		fts_custom_gesture_template_rewrite_tochip(info);
		fts_gesture_switch_rewrite_tochip(info);
		
		/* gesture mode, must close glove mode */
		glove_mode_state_save = info->glove_mode_switch;
		info->glove_mode_switch = 0;
		fts_glove_mode_switch_rewrite_tochip(info);
		info->glove_mode_switch = glove_mode_state_save;
		
		fts_check_into_gesture_mode(info);
	}else{
		VIVO_TS_LOG_INF("fts_standard_suspend:change to sleep mode!\n");
		atomic_set(&info->ts_state, TOUCHSCREEN_SLEEP);
		if(fts_command(info_gl, SENSEOFF)) {
			VIVO_TS_LOG_ERR("%s SENSEOFF failed \n", __func__);
		}
		mdelay(10);
		if(fts_command(info_gl, KEY_OFF)) {
			VIVO_TS_LOG_ERR("%s KEY_OFF failed \n", __func__);
		}
		//mdelay(10);
	}
	if(info->ts_dclick_simulate_switch == 1) {
		info->ts_dclick_simulate_switch = 0;
	}

	info->has_tp_suspend = 1;
	release_point_and_key(info);//add by qiuguifu
	return  0;
}

static int fts_standard_resume(struct fts_ts_info *info)
{
	int ret = 0;
	int power_reset_count = 0;
	int error = 0;

	/* while firmeware updating,do not execute resume process */
	if(info->firmware_updating_flag) {
		VIVO_TS_LOG_INF("[%s]firmware is updating.no resume\n", __func__);	
		return 0;
	}	
	
	VIVO_TS_LOG_INF("[%s]resume begin.\n",__func__);
	atomic_set(&info->ts_state, TOUCHSCREEN_NORMAL);

	/* clear need_change_to_dclick state,avoid last state affect next suspend*/
	if(info->has_lcd_shutoff == 0)
		info->need_change_to_dclick = 0;

	fts_get_noise_parameter(info);

power_reset_tag:
	if(power_reset_count > 0) {
			VIVO_TS_LOG_INF("[%s]power reset error!power_reset_count=%d.error=%d\n", __func__, power_reset_count, error);
	}
	error = 0;
	VIVO_TS_LOG_INF("[%s]step 1.power down reset.\n",__func__);
	fts_power_reset(info);
	error += fts_systemreset(info);

	fts_set_noise_parameter(info);
	//fts_get_noise_parameter(info);
	
	VIVO_TS_LOG_INF("[%s]step 2.sense on and key on.\n",__func__);
	error += fts_command(info, SENSEON);
	mdelay(10);
	error += fts_command(info, KEY_ON);
	mdelay(10);

	error += fts_interrupt_set(info, INT_ENABLE);
	if(error < 0) {
		power_reset_count++;
		if(power_reset_count > 2) {
			VIVO_TS_LOG_ERR("[%s]3 times power down reset fail.\n", __func__);
			return -1;
		} else {
			msleep(50);
			goto power_reset_tag;
		}
	}
	
	//while resume,avoid exception in sleep,we must rewrite glove mode switch to chip.
	VIVO_TS_LOG_INF("[%s]step 3.rewrite glove mode setting to chip.\n",__func__);
	ret = fts_glove_mode_switch_rewrite_tochip(info);
	if(ret) {
		VIVO_TS_LOG_INF("[%s]step 3 fail!rewrite glove mode setting to chip.\n",__func__);
	}
	//rewrite charger flag
	bbk_rewrite_usb_charger_flag_tochip(info);	

	VIVO_TS_LOG_INF("[%s]step 4.release point.\n",__func__);
	release_point_and_key(info);//add by qiuguifu
	/* must release large press mode,else tp maybe no function */
	info->large_press_touch_id = -1;	
	info->is_large_press_mode = 0;

	VIVO_TS_LOG_INF("[%s]step 5.set suspend state.has_tp_suspend=%d.\n", __func__, info->has_tp_suspend);
	info->has_tp_suspend = 0;

	VIVO_TS_LOG_INF("[%s]resume end.\n",__func__);
	
	return 0;
}

static int fts_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank = NULL;
	static int fb_suspend_flag = 0;
	//struct fts_ts_info  *info = container_of(self, struct fts_ts_info , fb_notif);
		
#if 0
	if (evdata && evdata->data && event==FB_EARLY_EVENT_BLANK && info) {		//FB_EVENT_BLANK
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK && fb_suspend_flag == 1)
		{
			fb_suspend_flag = 0;
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND,1);
		} else if (*blank == FB_BLANK_POWERDOWN) {
			fb_suspend_flag = 1;
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND,0);
		}
	}
#endif
	VIVO_TS_LOG_INF("[%s]enter.\n", __func__);
	if(evdata == NULL) {
		VIVO_TS_LOG_ERR("[%s]evdata is NULL.\n", __func__);
		return 0;
	}
	if(evdata->data == NULL) {
		VIVO_TS_LOG_ERR("[%s]evdata->data is NULL.\n", __func__);
		return 0;
	}
			
	VIVO_TS_LOG_INF("[%s]event=%ld\n", __func__, event);

	/* resume */
	if(event==FB_EVENT_BLANK) {	//wake_p
		blank = evdata->data;		//0:wakeup 4:suspend
		if(*blank==FB_BLANK_UNBLANK && fb_suspend_flag == 1) {
			VIVO_TS_LOG_INF("[%s]blank=%d\n", __func__, *blank);
			fb_suspend_flag = 0;
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND,1);
		}
		
	}
	/* suspend */
	if(event==FB_EARLY_EVENT_BLANK) {	//wake_p
		blank = evdata->data;		//0:wakeup 4:suspend
		if(*blank==FB_BLANK_POWERDOWN && fb_suspend_flag == 0) {
			VIVO_TS_LOG_INF("[%s]blank=%d\n", __func__, *blank);
			fb_suspend_flag = 1;
			touchscreen_request_send(TOUCHSCREEN_REQ_ID_RESUME_SUSPEND,0);
		}
	}
	
	VIVO_TS_LOG_INF("[%s]end.\n", __func__);
	return 0;
}

static const struct i2c_device_id fts_device_id[] = {
	{FTS_TS_DRV_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static struct of_device_id fts_match_table[] = {
	{ .compatible = "fts-ts,fts",},
	{ },
};
#else
#define fts_match_table NULL
#endif

static struct i2c_driver fts_i2c_driver = {
	.driver = {
	.owner	= THIS_MODULE,
       .name = FTS_TS_DRV_NAME,
	.of_match_table = fts_match_table,
    },
	.probe    = fts_probe,
	.remove   = fts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend  = fts_suspend,
	.resume   = fts_resume,
#endif
	.id_table = fts_device_id,
};

static int __init fts_driver_init(void)
{
	VIVO_TS_LOG_INF("fts driver init!\n");
	if(!vivo_touchscreen_test_ic_in_use(FTS_TS_DRV_NAME)){
	    return 0;
	}

	/* both atboot mode and phone off charge,we must not load driver */
	if(is_atboot==1 || power_off_charging_mode==1){
		//power off all voltage

		VIVO_TS_LOG_INF("[%s]TS is in at mood of power off charging mode\n!", __func__);
		
		return 0;		 
	}

	VIVO_TS_LOG_INF("ic and driver is %s\n", FTS_TS_DRV_NAME);

	return i2c_add_driver(&fts_i2c_driver);
}


static void __exit fts_driver_exit(void)
{
	i2c_del_driver(&fts_i2c_driver);
}


MODULE_DESCRIPTION("STMicroelectronics MultiTouch IC Driver");
MODULE_AUTHOR("JHJANG");
MODULE_AUTHOR("Giuseppe Di Giore <giuseppe.di-giore@st.com");
MODULE_LICENSE("GPL");

late_initcall(fts_driver_init);
module_exit(fts_driver_exit);
