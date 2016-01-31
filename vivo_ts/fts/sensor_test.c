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
#include "fts.h"
#include <linux/completion.h>

#include <linux/kernel.h>
#include <linux/lockdep.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>

#include <linux/list.h>
#include <linux/fb.h>
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/time.h>
#include <linux/kthread.h>

#include <linux/vivo_touchscreen_common.h>
#include <linux/vivo_touchscreen_config.h>
#include "bbk_ts_node.h"
#include <linux/wakelock.h> 


unsigned char *cx2_h_thres   = NULL;	
unsigned char *cx2_v_thres   = NULL;
unsigned char *cx2_min_thres = NULL;//CX2_MIN_THRESHOLD
unsigned char *cx2_max_thres = NULL;//CX2_MAX_THRESHOLD
unsigned int *self_ix2_min  = NULL;//SELF_IX2_MIN
unsigned int *self_ix2_max  = NULL;//SELF_IX2_MAX
unsigned int *self_cx2_min  = NULL;//SELF_CX2_MIN
unsigned int *self_cx2_max  = NULL;//SELF_CX2_MAX
unsigned int mutual_raw_min     ;
unsigned int mutual_raw_max     ;
unsigned int mutual_raw_gap     ;
unsigned int self_raw_force_min ;
unsigned int self_raw_force_max ;
unsigned int self_raw_sense_min ;
unsigned int self_raw_sense_max ;

unsigned int *mskey_cx2_min = NULL;
unsigned int *mskey_cx2_max = NULL;
unsigned int mskey_raw_min_thres ;
unsigned int mskey_raw_max_thres ;
unsigned int sp_cx2_gap_node_count;
unsigned int *sp_cx2_gap_node_index = NULL;
unsigned int sp_cx2_gap_node;

//static char Out_buff[512];

static char sensor_test_result[8];
static char ito_test_result[8];

static char fts_threshold_data[] = {
	/*0000:*/ 0xaa, 0x55, 0x00, 0x10, 0x1c, 0xaa, 0x55, 0x01, 0x10, 0x1c, 0x08, 0xaa, 0x55, 0x02, 0x10, 0x1c,
	/*0010:*/ 0x08, 0xaa, 0x55, 0x03, 0x0d, 0xaa, 0x55, 0x04, 0x2a, 0xaa, 0x55, 0x05, 0x00, 0x30, 0x00, 0x10,
	/*0020:*/ 0xaa, 0x55, 0x06, 0x00, 0x96, 0x00, 0x69, 0xaa, 0x55, 0x07, 0x00, 0x24, 0x00, 0x2a, 0xaa, 0x55,
	/*0030:*/ 0x08, 0x00, 0x2c, 0x00, 0x34, 0xaa, 0x56, 0x01, 0x10, 0x68, 0xaa, 0x56, 0x02, 0x16, 0xa8, 0xaa,
	/*0040:*/ 0x56, 0x03, 0x03, 0x20, 0xaa, 0x56, 0x04, 0x23, 0x28, 0xaa, 0x56, 0x05, 0x3a, 0x98, 0xaa, 0x56,
	/*0050:*/ 0x06, 0x23, 0x28, 0xaa, 0x56, 0x07, 0x3a, 0x98, 0xaa, 0x56, 0x08, 0x10, 0x2c, 0xaa, 0x56, 0x09, 
	/*0060:*/ 0x16, 0xe4, 0xaa, 0x55, 0x09, 0x03, 0x10, 0xaa, 0x55, 0x0a, 0x03, 0x32, 0xaa, 0x57, 0x10, 0x1c, 
	/*0070:*/ 0x06, 0x0a, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x0c, 0x0e, 0x0c, 0x0f, 0x0b, 0x0f, 0x0c	
 };

static int fts_abs_math(int node1, int node2)
{
	int ret =0;
	if(node1 >= node2)
		ret = node1- node2;
	else
		ret = node2 - node1;
		
	return ret;
}

/* Function to check special node , return 1 if node is special node otherwise return 0*/
int check_sp_cx2_gap_node(int cx2_index)
{
	int index = 0;

	for(index = 0 ; index < sp_cx2_gap_node_count;index++)
	{
		if(sp_cx2_gap_node_index[index] == cx2_index)
			return 1;
	}
	return 0;	
}
int fts_wait_controller_ready(struct fts_ts_info *info)
{
	unsigned int retry =0,
				error = 0;

	unsigned char regAdd[8];
	unsigned char data[FTS_EVENT_SIZE];			
	/* Read controller ready event*/
	for (retry = 0; retry <= CNRL_RDY_CNT; retry++) // poll with time-out 5000ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) {
			VIVO_TS_LOG_ERR("%s : I2c READ ERROR : Cannot read device info\n", __func__);
			return -ENODEV;
		}

		if (data[0] == EVENTID_CONTROLLER_READY) {
			VIVO_TS_LOG_INF("[%s]get controller ready event.data[0]=%d\n",__func__, data[0]);
			break;
		} else {
			msleep(10);
			if(retry == CNRL_RDY_CNT)
			{
				VIVO_TS_LOG_INF("[%s]TIMEOUT ,Cannot read controller ready event after system reset\n",__func__);
				return -ENODEV;
			}
		}
	}			
	return 0;
}

int fts_chip_initialization(struct fts_ts_info *info)
{
	int ret2 = 0;
	int retry;
	int error;
	int initretrycnt = 0;
		
	//initialization error, retry initialization
	for(retry = 0; retry <= INIT_FLAG_CNT; retry++)
	{
		//fts_chip_powercycle(info); 
		fts_power_reset(info);
		fts_systemreset(info);
		release_point_and_key(info);
		
		ret2 = fts_init_flash_reload(info);
		if(ret2 == 0)break;					    	
		initretrycnt++;				
		VIVO_TS_LOG_DBG("initialization cycle count = %04d\n", initretrycnt);					    	  	    	
	}
	if(ret2 != 0)     //initialization error
	{ 
		VIVO_TS_LOG_DBG("fts initialization 3 times error\n");		
		error = fts_systemreset(info);
		//fts_wait_controller_ready(info);
		error += fts_command(info, SENSEON);
				error += fts_command(info, FORCECALIBRATION);
#ifdef PHONE_KEY
			fts_command(info, KEY_ON);
#endif					
		error += fts_command(info, FLUSHBUFFER);
		fts_interrupt_set(info, INT_ENABLE);		
		if (error) {
			VIVO_TS_LOG_ERR("%s: Cannot reset the device----------\n", __func__);
		}					    							    		
	}								

	return ret2;
}

/* initialization test*/
static int fts_initialization_test(struct fts_ts_info *info)
{
	int ret  = 0 ; 
	ret =  fts_chip_initialization(info);
	return ret;
}

int read_compensation_event(struct fts_ts_info *info,unsigned int compensation_type, unsigned char *data)
{
	unsigned char regAdd[8];
	int 	error =0 , 
		retry = 0;
	int ret = 0;
	
	/* Request Compensation Data*/
	regAdd[0] = 0xB8;
	regAdd[1] = compensation_type;
	regAdd[2] = 0x00;
	fts_write_reg(info,regAdd, 3);
	msleep(10);
	
	/* Read completion event*/
	for (retry = 0; retry <= READ_CNT; retry++) // poll with time-out 3000ms
	{
		regAdd[0] = READ_ONE_EVENT;
		error = fts_read_reg(info, regAdd,1, data, FTS_EVENT_SIZE);
		if (error) 
		{
			VIVO_TS_LOG_INF("FTS %s: I2c READ ERROR : Cannot read device info\n",__func__);
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("[%s]%02X %02X %02X %02X %02X %02X %02X %02X\n",__func__,
				data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);

		if ((data[0] == EVENTID_COMP_DATA_READ) && (data[1] == compensation_type)) 
		{
			break;
		}
		else
		{
			msleep(10);
			if(retry == READ_CNT)
			{
				VIVO_TS_LOG_INF("FTS %s :  TIMEOUT ,Cannot read completion event\n",__func__);
				ret = -1;
			}
		}
	}	
	VIVO_TS_LOG_INF("%s exited \n",__func__);
	return ret;
}

/* 2.2.1 Read Mutual data test*/
static int fts_mutual_tune_data_test(struct fts_ts_info *info)		
{
	unsigned char *mutual_cx_data =NULL;
	unsigned char *mutual_cx2_err = NULL;
	unsigned char cx1_num = 0;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int i,	j = 0, error =0 , 
		address_offset = 0,
		start_tx_offset = 0, 
		cx2_zero_error_flag = 0,
		cx2_adjacent_error_flag =0 ,
		cx2_min_max_error_flag = 0,
		mutual_tune_error = 0;
	VIVO_TS_LOG_INF("%s entered \n",__func__);
	
	fts_interrupt_set(info, INT_DISABLE);	
	//msleep(10);
	fts_command(info, SENSEOFF);
#ifdef PHONE_KEY
	fts_command(info, KEY_OFF);
#endif	
	//msleep(10);
	fts_command(info, FLUSHBUFFER);
	//msleep(50);
	
	error = read_compensation_event(info,0x02,data);
	if(error != 0) {
		VIVO_TS_LOG_INF("[%s]read_compensation_event fail.\n",__func__);
		return -1;
	}
	
	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);	

	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);
	address_offset  = start_tx_offset + 0x10;

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	/* f_cnt and s_cnt */
	tx_num = data[4];//M
	rx_num = data[5];//N
	cx1_num = buff_read[10];
	
	mutual_cx_data = (unsigned char *)kmalloc(((tx_num * rx_num)+1), GFP_KERNEL);// added one to accommodate  dummy byte
	mutual_cx2_err = (unsigned char *)kmalloc(((tx_num * rx_num)), GFP_KERNEL);

/*
	o CX2_HORIZONTAL_THRESHOLD : cx2_h_thres [f_cnt] [s_cnt]
	o CX2_VERTICAL_THRESHOLD : cx2_v_thres [f_cnt] [s_cnt]
	o CX2_MIN_THRESHOLD : cx2_min_thres [f_cnt] [s_cnt]
	o CX2_MAX_THRESHOLD : cx2_max_thres [f_cnt] [s_cnt]
*/		
	
	
	//======= Cx data Reading =========//	
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mutual_cx_data, ((tx_num * rx_num)+1));

	/*ALL ZERO Test*/
	{
		int cx2_sum = 0 ;
		for(j = 0 ; j <= tx_num ; j++)
		{
			for(i = 0 ; i <= rx_num ; i++)
			{
				cx2_sum += mutual_cx_data[(j*tx_num)+i];
			}			
		
		}
		if (cx2_sum == 0) 
		{
			cx2_zero_error_flag = 1; // all zero error 
			VIVO_TS_LOG_INF("FTS %s: zero_error\n",__func__);
		}
	}
	
	/*--- Check adjacent CX2 nodes against threshold maps ---*/
	{	
		int hrzErrCnt = 0; // horizontal error counter 
		int vrtErrCnt = 0; // vertical error counter
		int node1 , node2 ,threshold,delta;
		// sweep for horizontal delta
		for(j = 0 ; j < tx_num ; j++)
		{
			for(i = 0 ; i < rx_num-1 ; i++)
			{
				if(check_sp_cx2_gap_node((j*rx_num)+i))
				{
					threshold = sp_cx2_gap_node;
					VIVO_TS_LOG_INF("FTS %s:Horizontal check_sp_cx2_gap_node  tx = %d Rx = %d  threshold = %02X \n",__func__,j,i, threshold);
				}
				else
				{
					threshold = cx2_h_thres[(j*rx_num)+i];
				}
				node1 = mutual_cx_data[(j*rx_num)+i];
				node2 = mutual_cx_data[(j*rx_num)+i+1];
				delta = fts_abs_math(node1,node2);
				
				if(delta > threshold)
				{
					mutual_cx2_err[(j*tx_num)+i+1] += 0x01; // bit 0 to indicate horizontal 
					hrzErrCnt++;		
					VIVO_TS_LOG_INF("FTS %s: Error cx2_adjacent_error Horizontal threshold tx = %d Rx = %d delta = %02X threshold = %02X \n",__func__,j,i,delta, threshold);		
				}				
			}			
		
		}		
		// sweep for vertical delta
		for(j = 0 ; j < tx_num-1 ; j++)
		{
			for(i = 0 ; i < rx_num ; i++)
			{
				if(check_sp_cx2_gap_node((j*rx_num)+i))
				{
					threshold = sp_cx2_gap_node;
					VIVO_TS_LOG_INF("FTS %s:Vertical check_sp_cx2_gap_node  tx = %d Rx = %d threshold = %02X \n",__func__,j,i, threshold);
				}
				else
				{
					threshold = cx2_v_thres[(j*rx_num)+i];
				}				
				
				node1 = mutual_cx_data[(j*rx_num)+i];
				node2 = mutual_cx_data[((j+1)*rx_num)+i];
				delta = fts_abs_math(node1,node2);
				
				if(delta > threshold)
				{
					mutual_cx2_err[(j*tx_num)+i+1] += 0x02; // bit 0 to indicate horizontal 
					vrtErrCnt++;		
					VIVO_TS_LOG_INF("FTS %s: Error cx2_adjacent_error Vertical threshold tx = %d Rx = %d delta = %02X threshold = %02X \n",__func__,j,i,delta, threshold);		
				}
			}			
		
		}
		if ((hrzErrCnt + vrtErrCnt) != 0) 
		{
			cx2_adjacent_error_flag = 1; // adjacent node error 
			VIVO_TS_LOG_INF("FTS %s: cx2_adjacent_error : \n",__func__);
		}
	}
	
	/*--- Check CX2 against min and max theshold ---*/
	{
		//cx2_min_err = new byte[f_cnt, s_cnt]; // for recording min error nodes 
		//cx2_max_err = new byte[f_cnt, s_cnt]; // for recording max error nodes 
		int minErrCnt = 0; // min error counter 
		int maxErrCnt = 0; // max error counter

		for(j = 0 ; j < tx_num ; j++)
		{
			for(i = 0 ; i < rx_num ; i++)
			{
				if(mutual_cx_data[(j*rx_num)+i] < cx2_min_thres[(j*rx_num)+i])
				{
					minErrCnt++;
					VIVO_TS_LOG_INF("FTS %s:Error Min error tx = %d Rx = %d mutual_cx_data= %04X cx2_min_thres = %04X \n",__func__,j,i,mutual_cx_data[(j*rx_num)+i] ,cx2_min_thres[(j*rx_num)+i]);	
				}
					
				if(mutual_cx_data[(j*rx_num)+i] > cx2_max_thres[(j*rx_num)+i])
				{
					maxErrCnt++;
					VIVO_TS_LOG_INF("FTS %s:Error Max error tx = %d Rx = %d mutual_cx_data= %04X cx2_max_thres = %04X \n",__func__,j,i,mutual_cx_data[(j*rx_num)+i] ,cx2_min_thres[(j*rx_num)+i]);	
				}
			}			
		}
		if((minErrCnt+maxErrCnt) != 0)
		{
			cx2_min_max_error_flag = 1;
			VIVO_TS_LOG_INF("FTS %s: cx2_min_max_error\n",__func__);
		}
	}
	fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);
	//msleep(10);
#ifdef PHONE_KEY
	fts_command(info, KEY_ON);
	//msleep(10);
#endif	

	kfree(mutual_cx_data);
	kfree(mutual_cx2_err);
	
	if(cx2_min_max_error_flag || cx2_adjacent_error_flag || cx2_zero_error_flag)
	{
		mutual_tune_error = 1;
		VIVO_TS_LOG_INF("FTS %s: Mutual tune value Test:Failed\n",__func__);
	}
	else
		VIVO_TS_LOG_INF("FTS %s: Mutual tune value test Passed\n",__func__);
	
	return mutual_tune_error;
}

/*  2.3.1	Read Self Tune Data  */
static int fts_self_tune_data_test(struct fts_ts_info *info)		
{
	unsigned char *self_tune_data =NULL;
	unsigned int f_ix1_num = 0;
	unsigned int s_ix1_num = 0;
	unsigned int f_cx1_num = 0;
	unsigned int s_cx1_num = 0;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int i = 0, error = 0,
		address_offset = 0,
		start_tx_offset = 0, 
		self_tune_error = 0;
	VIVO_TS_LOG_INF("%s entered \n",__func__);
	
	fts_interrupt_set(info, INT_DISABLE);	
	//msleep(10);
	fts_command(info, SENSEOFF);
#ifdef PHONE_KEY
	fts_command(info, KEY_OFF);
#endif	
	//msleep(10);
	fts_command(info, FLUSHBUFFER);
	//msleep(50);	
	
	error = read_compensation_event(info,0x20,data);
	if(error != 0) {
		VIVO_TS_LOG_INF("[%s]read_compensation_event fail.\n",__func__);
		return -1;
	}
	
	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);	

	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);
	address_offset  = start_tx_offset + 0x10;

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	/* f_cnt and s_cnt */
	tx_num = data[4];//M
	rx_num = data[5];//N
	f_ix1_num = buff_read[10];
	s_ix1_num = buff_read[11];
	f_cx1_num = buff_read[12];
	s_cx1_num = buff_read[13];
	
	//all_strbuff = (unsigned char *)kmalloc(((tx_num * rx_num)+5)*2, GFP_KERNEL);
	self_tune_data = (unsigned char *)kmalloc(((tx_num + rx_num)* 2 + 1), GFP_KERNEL);// 

/*
1. Call read_self_tune API, to read the Self tune data, and store in buffers.
	o s_ix1_force
	o s_ix1_sense
	o s_ix2_force[f_cnt]
	o s_ix2_sense[s_cnt]
2. Calculate the total Self IX values for each channel by the following formula:
	IX_force = K * IX1_force + L * IX2_force.
	IX_sense = M * IX1_sense + N * IX2_sense.
	o s_ix_force[f_cnt]
	o s_ix_sense[s_cnt]
3.Read the following threshold values from the Threshold file, and store in buffers.
	o SELF_IX_MIN :
		 s_ix_force_min [f_cnt]
		 s_ix_sense_min [s_cnt]
	o SELF_IX_MAX :
		 s_ix_force_max [f_cnt]
		 s_ix_sense_max [s_cnt]	
*/		
	
	// ============================================ Self tune data Reading ======================================//	
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, self_tune_data, ((tx_num + rx_num)*2+1));
	{
		/*--- Calculate Self IX ---*/ 
		int s_ix_force[tx_num]; 
		int s_ix_sense[rx_num];
		int minErrCnt = 0; // min error counter 
		int maxErrCnt = 0; // max error counter
		
		for (i = 0; i < tx_num ;i++ ) 
		{ 
			s_ix_force[i] = K_COFF * f_ix1_num +  L_COFF * self_tune_data[1+i]; 
		}
		for (i = 0; i < rx_num;i++ ) 
		{ 
			s_ix_sense[i] = M_COFF * s_ix1_num + N_COFF * self_tune_data[1+tx_num+i]; 		
		}
		VIVO_TS_LOG_INF("fts_read_self_tune_show : probe6\n");
		
		/*--- check Self IX against min & max thresholds ---*/
		for(i = 0; i < tx_num; i++) 
		{ 
			if(s_ix_force[i] < self_ix2_min[i])//force
			{ 
				minErrCnt++; 
				VIVO_TS_LOG_INF("FTS %s:Error  Self min error force = %d s_ix_force[i]=%04X self_ix2_min[i]=%04X \n",__func__,i,s_ix_force[i],self_ix2_min[i]);
			}
			if(s_ix_force[i] > self_ix2_max[i])//force
			{ 
				maxErrCnt++; 
				VIVO_TS_LOG_INF("FTS %s:Error  Self max error force = %d s_ix_force[i]=%04X self_ix2_min[i]=%04X \n",__func__,i,s_ix_force[i],self_ix2_min[i]);
			} 
		}	
		for(i = tx_num; i <(tx_num +rx_num); i++)
		{
			if(s_ix_sense[i-tx_num] < self_ix2_min[i]) 
			{ 
				minErrCnt++; 
				VIVO_TS_LOG_INF("FTS %s:Error  Self min error sense = %d  s_ix_sense[i]=%04X self_ix2_min[i]=%04X \n",__func__,i,s_ix_sense[i-tx_num],self_ix2_min[i]);
			}
			if(s_ix_sense[i-tx_num] > self_ix2_max[i])
			{ 
				maxErrCnt++; 
				VIVO_TS_LOG_INF("FTS %s:Error  Self max error sense = %d s_ix_sense[i]=%04X self_ix2_min[i]=%04X \n",__func__,i,s_ix_sense[i-tx_num],self_ix2_min[i]);
			}
		}
		
		if((minErrCnt + maxErrCnt) != 0) 
		{ 
			self_tune_error = 1; // self IX2 min/max error 
			VIVO_TS_LOG_INF("FTS %s: self tune data test Failed\n",__func__);
		}else
			VIVO_TS_LOG_INF("FTS %s: self tune data test Passes\n",__func__);	
	}
    fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);
	//msleep(10);
#ifdef PHONE_KEY
	fts_command(info, KEY_ON);
	//msleep(10);
#endif	

	kfree(self_tune_data);
	return self_tune_error;
}

/* 2.4.1 Read Mutual raw data test*/
static int fts_mutual_raw_data_test(struct fts_ts_info *info)		
{
	unsigned char *mutual_raw_data =NULL;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	unsigned int row,col = 0, address_offset = 0;
	int minErrCnt = 0; // min error counter
	int maxErrCnt = 0; // max error counter
	int m_raw_min = 0; 
	int m_raw_max = 0; 
	int error = 0;
	VIVO_TS_LOG_INF("%s entered \n",__func__);

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, FLUSHBUFFER);
	fts_command(info, SENSEON);	
	msleep(100); 
	fts_command(info, SENSEOFF);
	msleep(50);
	
	error = read_compensation_event(info,0x20,data);
	if(error != 0) {
		VIVO_TS_LOG_INF("[%s]read_compensation_event fail.\n",__func__);
		return -1;
	}

	/* f_cnt and s_cnt */
	tx_num = data[4];//M
	rx_num = data[5];//N

	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x00;//Offset address for MS screen raw frame
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);	

	address_offset = ((buff_read[2]<<8) |buff_read[1]) + (rx_num *2);
		
	mutual_raw_data = (unsigned char *)kmalloc(((tx_num * rx_num)*2+1), GFP_KERNEL);// added one to accommodate  dummy byte
	
	//======= Mutual Raw data Reading =========//	
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mutual_raw_data, ((tx_num * rx_num)*2+1));// added one extra byte to accommodate  dummy byte

	/*--- check Screen mutual raw data ---*/
	m_raw_min = (mutual_raw_data[2]<<8) | mutual_raw_data[1]; // initialize with first raw data
	m_raw_max = (mutual_raw_data[2]<<8) | mutual_raw_data[1]; ; // initialize with first raw data

	VIVO_TS_LOG_INF("[%s]mutual_raw_min=%d mutual_raw_max=%d mutual_raw_gap=%d\n", __func__, mutual_raw_min, mutual_raw_max, mutual_raw_gap);
	VIVO_TS_LOG_INF("[%s]mutual_raw:", __func__);
	for (row = 0; row < tx_num; row++)
	{
		int temp_mutual_raw = 0;
		for (col = 0; col < rx_num*2; )
		{
			temp_mutual_raw = (mutual_raw_data[2*row*rx_num+col+2]<<8)|mutual_raw_data[2*row*rx_num+col+1];
			printk("%d ", temp_mutual_raw);
			// min check
			if(temp_mutual_raw < mutual_raw_min)
			{
				minErrCnt++;
				VIVO_TS_LOG_INF("FTS %s:Error Min error tx = %d Rx = %d: \n",__func__,row,col);	

			}
			// max check
			if(temp_mutual_raw> mutual_raw_max)
			{
				maxErrCnt++;
				VIVO_TS_LOG_INF("temp_mutual_raw:%d  mutual_raw_max:%d\n", temp_mutual_raw, mutual_raw_max);				
				VIVO_TS_LOG_INF("FTS %s:Error  Max error tx = %d Rx = %d: \n",__func__,row,col);	
			}
			// find min 
			if(temp_mutual_raw < m_raw_min)
			{
				m_raw_min = temp_mutual_raw;
			}
			// find max
			if(temp_mutual_raw > m_raw_max)
			{
				m_raw_max = temp_mutual_raw;
			}
			col = col+2 ;
		}//for (col = 0; col < rx_num; col++)
		printk("\n");
	}//for (row = 0; row < tx_num; row++)

	kfree(mutual_raw_data);
	
	if ((minErrCnt + maxErrCnt) != 0)
	{
		VIVO_TS_LOG_INF("FTS %s: Test Failed :mutual raw min/max error\n",__func__);
		return -1; // mutual raw min/max error
	}

	if ((m_raw_max - m_raw_min) > mutual_raw_gap)
	{
		
		VIVO_TS_LOG_INF("[%s]:Error mutual_raw_gap = %d: \n",__func__,mutual_raw_gap);	
		VIVO_TS_LOG_INF("[%s]: Test Failed :mutual raw gap error\n",__func__);
		return -2; // mutual raw gap error
	}
	return 0;
}

/* 2.5.1 Read Self data test*/
static int fts_self_raw_data_test(struct fts_ts_info *info)		
{
	unsigned char *self_raw_data_force =NULL;
	unsigned char *self_raw_data_sense =NULL;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num;
	int i = 0, error = 0,
		address_offset_force = 0,
		address_offset_sense = 0,
		size_hex_force= 0,
		size_hex_sense = 0;
	int minErrCnt = 0; // min error counter
	int maxErrCnt = 0; // max error counter
	VIVO_TS_LOG_INF("%s entered \n",__func__);
	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, FLUSHBUFFER);
	msleep(100); 
	fts_command(info, SENSEOFF);
	msleep(50);
		
	error = read_compensation_event(info,0x20,data);
	if(error != 0) {
		VIVO_TS_LOG_INF("[%s]read_compensation_event fail.\n",__func__);
		return -1;
	}
		
	/* f_cnt and s_cnt */
	tx_num = data[4];//M
	rx_num = data[5];//N
	size_hex_force = tx_num * 2 ;
	size_hex_sense = rx_num * 2 ;
	
	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x1A;//Offset address for SS touch raw force line
	fts_read_reg(info,regAdd, 3, &buff_read[0], 5);	

	address_offset_force = ((buff_read[2]<<8) |buff_read[1]);
	address_offset_sense = ((buff_read[4]<<8) |buff_read[3]);

	
	self_raw_data_force = (unsigned char *)kmalloc((size_hex_force + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
	self_raw_data_sense = (unsigned char *)kmalloc((size_hex_sense + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
	
	// ============================================ Self Raw data Reading(force) ======================================//	

	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset_force & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset_force & 0xFF);
	fts_read_reg(info,regAdd, 3, self_raw_data_force, (size_hex_force+1));

	VIVO_TS_LOG_INF("[%s]self_raw_force_min=%d self_raw_force_max=%d\n",__func__, self_raw_force_min, self_raw_force_max);	
	VIVO_TS_LOG_INF("[%s]self_raw_force:", __func__);
	/*--- check Self Raw data  ---*/
	for (i = 0; i < tx_num*2; )
	{
		int temp_raw = 0;
		temp_raw = ((self_raw_data_force[i+2]<<8)|self_raw_data_force[i+1]); 
		printk("%d ", temp_raw);
		if (temp_raw < self_raw_force_min)
		{
			minErrCnt++;
			VIVO_TS_LOG_INF("FTS %s:Error Self raw min error force = %d raw_data = %d: \n",__func__,i,temp_raw);	
		}

		if (temp_raw > self_raw_force_max)
		{
			maxErrCnt++;
			VIVO_TS_LOG_INF("FTS %s:Error Self raw max error force = %d raw_data = %d: \n",__func__,i,temp_raw);
		}
		i= i + 2;
	}	
	printk("\n");
	// ============================================ Self Raw data Reading(sense) ======================================//	
	
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset_sense & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset_sense & 0xFF);
	fts_read_reg(info,regAdd, 3, self_raw_data_sense, (size_hex_sense+1));	

	VIVO_TS_LOG_INF("[%s]self_raw_sense_min=%d self_raw_sense_max=%d\n", __func__, self_raw_sense_min, self_raw_sense_max);

	VIVO_TS_LOG_INF("[%s]self_raw_sense:", __func__);
	for (i = 0; i < rx_num*2; )
	{
		int temp_raw = 0;
		temp_raw = ((self_raw_data_sense[i+2]<<8)| self_raw_data_sense[i+1]);
		printk("%d ", temp_raw);
		
		if( temp_raw < self_raw_sense_min)
		{
			minErrCnt++;
			VIVO_TS_LOG_INF("FTS %s:Error Self raw min error sense = %d sense_raw_data = %d: \n",__func__,i,temp_raw);
		}

		if(temp_raw > self_raw_sense_max)
		{
			maxErrCnt++;
			VIVO_TS_LOG_INF("FTS %s:Error Self raw max error sense = %d sense_raw_data = %d: \n",__func__,i,temp_raw);
		}
		i = i +2;
	}
	printk("\n");

	kfree(self_raw_data_force);
	kfree(self_raw_data_sense);
	
	if ((minErrCnt + maxErrCnt) != 0)
	{
		VIVO_TS_LOG_INF("FTS %s: Test Failed :self raw min/max error\n",__func__);
		return -1; // self raw min/max error
	}
	return 0;
}


/*  2.8.1	Read Mutual Key Tune Data  */
static int fts_mskey_tune_data_test(struct fts_ts_info *info)
{
	unsigned char *mskey_cx2 =NULL;
	unsigned char cx1_num = 0;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int rx_num, tx_num, mskeyNum = 0;
	int 	i = 0, error = 0,
		address_offset = 0,
		start_tx_offset = 0, 
		key_cnt = 0,
		mskey_tune_error = 0;
	VIVO_TS_LOG_INF("%s entered \n",__func__);
	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, SENSEOFF);
	//msleep(10);
#ifdef PHONE_KEY
	fts_command(info, KEY_OFF);
	//msleep(10);
#endif	
	fts_command(info, FLUSHBUFFER);
	//msleep(50);

	error = read_compensation_event(info,0x10,data);
	if(error != 0) {
		VIVO_TS_LOG_INF("[%s]read_compensation_event fail.\n",__func__);
		return -1;
	}
	/* Read Offset Address for Compensation*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x50;
	fts_read_reg(info,regAdd, 3, &buff_read[0], 4);
	
	start_tx_offset = ((buff_read[2]<<8) |buff_read[1]);
	address_offset  = start_tx_offset + 0x10;

	/* Read Offset Address for f_cnt and s_cnt*/
	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((start_tx_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(start_tx_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, &buff_read[0], 17);
	
	/* f_cnt and s_cnt */
	tx_num = data[4];//M
	rx_num = data[5];//N
	cx1_num = buff_read[10];

	if (tx_num > rx_num)
	{
		mskeyNum = tx_num;
	}
	else
	{
		mskeyNum = rx_num;
	}

	mskey_cx2 = (unsigned char *)kmalloc((mskeyNum+1), GFP_KERNEL);// added one to accommodate  dummy byte

	// ============================================ MS Key Cx data Reading ======================================//	

	regAdd[0] = 0xD0;
	regAdd[1] = (unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = (unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mskey_cx2, (mskeyNum+1));

	{
		
		/*--- check MSkey CX2 against min & max thresholds ---*/
		int minErrCnt = 0; // min error counter 
		int maxErrCnt = 0; // max error counter
		for(i = 0; i < key_cnt; i++) 
		{ 
			if(mskey_cx2[i] < mskey_cx2_min[i]) 
			{
				minErrCnt++; 
				VIVO_TS_LOG_INF("FTS %s:Error mskey_cx2 min error i = %d mskey_cx2[i] = %04X  mskey_cx2_min[i] = %04X \n",__func__,i,mskey_cx2[i],mskey_cx2_min[i]);
			}
			if(mskey_cx2[i] > mskey_cx2_max[i])
			{
				maxErrCnt++;
				VIVO_TS_LOG_INF("FTS %s:Error mskey_cx2 min error i = %d mskey_cx2[i] = %04X  mskey_cx2_max[i] = %04X \n",__func__,i,mskey_cx2[i],mskey_cx2_max[i]);
			} 
		}
		if((minErrCnt + maxErrCnt) != 0)
		{ 
			mskey_tune_error = 1; // MSkey tune min/max error 
		}
	}
	fts_interrupt_set(info, INT_ENABLE);
	fts_command(info, SENSEON);
	//msleep(10);
#ifdef PHONE_KEY
	fts_command(info, KEY_ON);
	//msleep(10);
#endif	
	kfree(mskey_cx2);
	return mskey_tune_error;
}//Read Mutual Key Tune Data

/*  2.9.1	Read Mutual Key Raw Data  */
static int fts_mskey_raw_data_test(struct fts_ts_info *info)
{
	unsigned char *mskey_raw_data =NULL;
	unsigned char regAdd[8];
	unsigned char buff_read[17];
	unsigned char data[FTS_EVENT_SIZE];
	unsigned int mskeyNum = 3;
	//int count = 0;
	int i =0 , error = 0,
		//address_offset = 0,
		size_hex = mskeyNum *2,
		mskey_raw_error = 0;

	fts_interrupt_set(info, INT_DISABLE);
	fts_command(info, FLUSHBUFFER);
#ifdef PHONE_KEY
	fts_command(info, KEY_ON);
#endif	
	msleep(100); 
	fts_command(info, KEY_OFF);
	msleep(50);

	error = read_compensation_event(info,0x10,data);
	if(error != 0) {
		VIVO_TS_LOG_INF("[%s]read_compensation_event fail.\n",__func__);
		return -1;
	}
#if 0	
	/* f_cnt and s_cnt */
	tx_num = data[4];//M
	rx_num = data[5];//N

	if (tx_num > rx_num)
	{
		mskeyNum = tx_num;
	}
	else
	{
		mskeyNum = rx_num;
	}

	size_hex = (mskeyNum)* 2 ;	
#endif

	/* Read Offset Address for Mutual Raw data*/
	regAdd[0] = 0xD0;
	regAdd[1] = 0x00;
	regAdd[2] = 0x32;//Offset address for MS screen raw frame, Please verify this address from supporting fw engineer
	fts_read_reg(info,regAdd,3, &buff_read[0], 4);
	
	//address_offset = ((buff_read[2]<<8) |buff_read[1]) + (rx_num *2);  
	
	/* size in device file will be 2*number of bytes ..2 bytes will be used to represent one hex byte in ascii/test */
	mskey_raw_data = (unsigned char *)kmalloc((size_hex + 1), GFP_KERNEL);// added one to accommodate  dummy byte (2*(M*N) + 1)
	
	// ============================================ Mutual Raw data Reading ======================================//	
	
	regAdd[0] = 0xD0;
	regAdd[1] = buff_read[2];	//(unsigned char)((address_offset & 0xFF00) >> 8);		
	regAdd[2] = buff_read[1];	//(unsigned char)(address_offset & 0xFF);
	fts_read_reg(info,regAdd, 3, mskey_raw_data, (size_hex+1));

	VIVO_TS_LOG_INF("[%s]mskey_raw_min_thres=%d mskey_raw_max_thres=%d\n",__func__, mskey_raw_min_thres, mskey_raw_max_thres);	
	VIVO_TS_LOG_INF("[%s]mskey_raw:", __func__);
	/*--- check mskey Raw data ---*/ 
	{	
		int minErrCnt = 0; // min error counter 
		int maxErrCnt = 0; // max error counter
		int mskey_raw;
		
		for(i = 0; i < mskeyNum; i++) 
		{ 
			mskey_raw = mskey_raw_data[2*i+2] << 8 | mskey_raw_data[i*2+1];
	
			printk("%d ", mskey_raw);
			if(mskey_raw < mskey_raw_min_thres)
			{ 
				minErrCnt++; 
				VIVO_TS_LOG_INF("FTS %s:Error mskey_raw min error i = %d mskey_raw[i] = %04X  mskey_raw_min_thres[i] = %04X \n",__func__,i,mskey_raw,mskey_raw_min_thres);
			}
			if(mskey_raw > mskey_raw_max_thres)
			{ 
				maxErrCnt++; 
				VIVO_TS_LOG_INF("FTS %s:Error mskey_raw max error i = %d mskey_raw[i] = %04X  mskey_raw_max_thres[i] = %04X \n",__func__,i,mskey_raw,mskey_raw_max_thres);
			} 
		}
		printk("\n");
		
		if((minErrCnt + maxErrCnt) != 0) 
		{ 
			mskey_raw_error = 1; // mskey raw min/max error 
		}
	}

	kfree(mskey_raw_data);
	return mskey_raw_error;
}//2.9	Read Mutual Key Raw Data 

static int fts_ito_test(struct fts_ts_info *info)
{
	unsigned char data[FTS_EVENT_SIZE];
	unsigned char event_id = 0;
	unsigned char tune_flag = 0;
	unsigned int retry = 0;
	unsigned char error ;
	unsigned char regAdd = 0;
	unsigned int ito_check_status = 0;

	VIVO_TS_LOG_INF("%s entered \n",__func__);
	fts_systemreset(info);
	msleep(200);
	fts_command(info, SENSEOFF);
	msleep(10);
#ifdef PHONE_KEY	
	fts_command(info, KEY_OFF);
	msleep(10);
#endif	
	fts_interrupt_set(info, INT_DISABLE);
	msleep(10);
	fts_command(info, FLUSHBUFFER);
	msleep(5);
	fts_command(info, ITO_CHECK);
	msleep(200);
	VIVO_TS_LOG_INF("fts ITO Check Start \n");

	for (retry = 0; retry < READ_CNT_ITO; retry++) 
	{
		regAdd = READ_ONE_EVENT;
		error = fts_read_reg(info, &regAdd,sizeof(regAdd), data, FTS_EVENT_SIZE);
		if (error) {
			VIVO_TS_LOG_ERR("fts_ito_test_show : i2C READ ERR , Cannot read device info\n");
			return -ENODEV;
		}
		VIVO_TS_LOG_INF("FTS ITO event : %02X %02X %02X %02X %02X %02X %02X %02X\n",
			data[0], data[1], data[2], data[3],data[4], data[5], data[6], data[7]);

		event_id = data[0];
		tune_flag = data[1];

		if ((event_id == 0x0F) && (tune_flag == 0x05)) 
		{
			if((data[2] == 0x00) && (data[3] == 0x00)){
				ito_check_status = 0;
				VIVO_TS_LOG_INF("fts ITO check ok \n");
				break;
			}
			else
			{
				ito_check_status = 1;
				VIVO_TS_LOG_INF("fts ITO check fail \n");

				switch (data[2]) 
				{
					case ERR_ITO_PANEL_OPEN_FORCE :
						ito_test_result[0] = 0;
						VIVO_TS_LOG_INF("ITO Test result : Force channel [%d] open.\n",
							data[3]);
						break;
					case ERR_ITO_PANEL_OPEN_SENSE :
						ito_test_result[1] = 0;
						VIVO_TS_LOG_INF("ITO Test result : Sense channel [%d] open.\n",
							data[3]);
						break;
					case ERR_ITO_F2G :
						ito_test_result[2] = 0;
						VIVO_TS_LOG_INF("ITO Test result : Force channel [%d] short to GND.\n",
							data[3]);
						break;
					case ERR_ITO_S2G :
						ito_test_result[3] = 0;
						VIVO_TS_LOG_INF("ITO Test result : Sense channel [%d] short to GND.\n",
							data[3]);
						break;
					case ERR_ITO_F2VDD :
						ito_test_result[4] = 0;
						VIVO_TS_LOG_INF("ITO Test result : Force channel [%d] short to VDD.\n",
							data[3]);
						break;
					case ERR_ITO_S2VDD :
						ito_test_result[5] = 0;
						VIVO_TS_LOG_INF("ITO Test result : Sense channel [%d] short to VDD.\n",
							data[3]);
						break;
					case ERR_ITO_P2P_FORCE :
						ito_test_result[6] = 0;
						VIVO_TS_LOG_INF("ITO Test result : Force channel [%d] ,Pin to Pin short.\n",
							data[3]);
						break;
					case ERR_ITO_P2P_SENSE :
						ito_test_result[7] = 0;
						VIVO_TS_LOG_INF("ITO Test result : Sense channel [%d] Pin to Pin short.\n",
							data[3]);
						break;					
			
					default:
 						break;
				}//switch case
				break;//for loop
			}
		}
		else
		{
			msleep(5);
			if (retry == (READ_CNT_ITO - 1)) 
			{
				VIVO_TS_LOG_ERR("Time over - wait for result of ITO test\n");
			}
		}
	}

	fts_systemreset(info);
	//msleep(200);
	fts_interrupt_set(info, INT_ENABLE);
	//msleep(10);
	fts_command(info, SENSEON);
#ifdef PHONE_KEY	
	fts_command(info, KEY_ON);
#endif	
	fts_command(info, FORCECALIBRATION);
	
	return ito_check_status;
	
}//ITO Test	


/*
 * add for sensor test
 *
 **/
ssize_t fts_production_test(struct fts_ts_info *info, char *buf, size_t count)
{
	//char buff[CMD_STR_LEN] = { 0 };
	//const struct firmware *fw_config = NULL;
	unsigned char *configData = NULL;
	unsigned int config_size = 0;
	static unsigned int tx_num, rx_num,i;

	VIVO_TS_LOG_INF("[%s]enter.\n", __func__);
	
	/* initialize the test values */
	mutual_raw_min      = 0xFFFF;
	mutual_raw_max      = 0xFFFF;
	mutual_raw_gap      = 0xFFFF;
	self_raw_force_min  = 0xFFFF;
	self_raw_force_max  = 0xFFFF;
	self_raw_sense_min  = 0xFFFF;
	self_raw_sense_max  = 0xFFFF;
	mskey_raw_min_thres = 0xFFFF;
	mskey_raw_max_thres = 0xFFFF;
	sp_cx2_gap_node_count = 0xFFFF;
	sp_cx2_gap_node		  = 0xFFFF;

	for(i=0; i<8; i++) {
		sensor_test_result[i] = 0;
		ito_test_result[i] = 1;
	}
	
	configData = fts_threshold_data;
	config_size = sizeof(fts_threshold_data);

	if(config_size > 0)	{
		VIVO_TS_LOG_INF("[%s]config size = %d\n", __func__, config_size);
	} else {
		VIVO_TS_LOG_ERR("[%s]config size is invalid", __func__);
		return count;
	}
	/* copying  the threshold value from fts_thresold.bin file*/	
	for(i=0; i < config_size; ) {
		/* checking the identifier 0xAA 0x55, copying the value based on identifier and id.  
		*  This driver release is using simplified reference threshold file.  
		*  For extensive reference threshold file ,optional code is also provided.
		*/
		if((configData[i] == 0xAA)&&(configData[i+1] == 0x55))	{
			i = i+2;
			VIVO_TS_LOG_INF("%s,0xaa0x55 configData[i] = %d\n",__func__,configData[i]);
			switch(configData[i++]) {
				case 0x00:
					//check f_cnt and s_cnt
					tx_num = configData[i++] ;
					rx_num = configData[i++] ;
					VIVO_TS_LOG_INF("%s,0xaa0x55 00 configData[i] = %d\n",__func__,configData[i]);
					break;
				
				case 0x01://CX2_HORIZONTAL_THRESHOLD
				{
					unsigned int cx2_h_thres_size=0,cx2_h_thres_index = 0;
					int tx,rx;
					tx                    = configData[i++];
					rx                    = configData[i++]; 
					
					cx2_h_thres_size = tx * rx;
					cx2_h_thres = (unsigned char *)kmalloc(cx2_h_thres_size, GFP_KERNEL);
				#if 0	
					while(cx2_h_thres_index < cx2_h_thres_size)
					{
						cx2_h_thres[cx2_h_thres_index++] = configData[i++];
					}
				#else //using simplified reference threshold file
					while(cx2_h_thres_index < cx2_h_thres_size)
					{
						cx2_h_thres[cx2_h_thres_index++] = configData[i];
					}
					i++;
					VIVO_TS_LOG_INF("%s,0xaa0x55 01 configData[i] = %d\n",__func__,configData[i]);
				#endif
				}break;
				case 0x02://CX2_VERTICAL_THRESHOLD
				{
					unsigned int cx2_v_thres_size=0,cx2_v_thres_index = 0;
					int tx,rx;
					tx                    = configData[i++];
					rx                    = configData[i++]; 
					
					cx2_v_thres_size = tx * rx;					
					cx2_v_thres = (unsigned char *)kmalloc(cx2_v_thres_size, GFP_KERNEL);
				#if 0	
					while(cx2_v_thres_index < cx2_v_thres_size)
					{
						cx2_v_thres[cx2_v_thres_index++] = configData[i++];
					}
				#else //using simplified reference threshold file
				while(cx2_v_thres_index < cx2_v_thres_size)
					{
						cx2_v_thres[cx2_v_thres_index++] = configData[i];
					}
					i++;		
					VIVO_TS_LOG_INF("%s,0xaa0x55 02 configData[i] = %d\n",__func__,configData[i]);		
				#endif	
				}break;	
				//cx2_min_thres
				case 0x03://CX2_MIN_THRESHOLD
				{
					unsigned int cx2_min_thres_size=0,cx2_min_thres_index = 0;
					cx2_min_thres_size = tx_num * rx_num;
					cx2_min_thres = (unsigned char *)kmalloc(cx2_min_thres_size, GFP_KERNEL);
				#if 0					
					while(cx2_min_thres_index < cx2_min_thres_size)
					{
						cx2_min_thres[cx2_min_thres_index++] = configData[i++];
					}
				#else //using simplified reference threshold file
					while(cx2_min_thres_index < cx2_min_thres_size)
					{
						cx2_min_thres[cx2_min_thres_index++] = configData[i];
					}
					i++;
					VIVO_TS_LOG_INF("%s,0xaa0x55 03 configData[i] = %d\n",__func__,configData[i]);
				#endif					
				}break;	
				//cx2_max_thres
				case 0x04://CX2_MAX_THRESHOLD
				{
					unsigned int cx2_max_thres_size=0,cx2_max_thres_index = 0;
					cx2_max_thres_size = tx_num * rx_num;
					cx2_max_thres = (unsigned char *)kmalloc(cx2_max_thres_size, GFP_KERNEL);
				#if 0					
					while(cx2_max_thres_index < cx2_max_thres_size)
					{
						cx2_max_thres[cx2_max_thres_index++] = configData[i++];
					}
				#else //simplified reference threshold file
					while(cx2_max_thres_index < cx2_max_thres_size)
					{
						cx2_max_thres[cx2_max_thres_index++] = configData[i];
					}
					i++;		
					VIVO_TS_LOG_INF("%s,0xaa0x55 04 configData[i] = %d\n",__func__,configData[i]);		
				#endif
				}break;
				//self_ix2_min
				case 0x05://SELF_IX2_MIN
				{
					unsigned int self_ix2_min_size=0,self_ix2_min_index = 0;
					self_ix2_min_size = (tx_num + rx_num);
					self_ix2_min = (unsigned int *)kmalloc(self_ix2_min_size*sizeof(unsigned int ), GFP_KERNEL);
				#if 0					
					while(self_ix2_min_index < self_ix2_min_size)
					{
						self_ix2_min[self_ix2_min_index++] = configData[i]<<8|configData[i+1];
						i = i +2;
					}
				#else //using simplified reference threshold file
					while(self_ix2_min_index < tx_num)
					{
						self_ix2_min[self_ix2_min_index++] = configData[i]<<8|configData[i+1];
					}
					i = i +2;
					while(self_ix2_min_index < self_ix2_min_size)
					{
						self_ix2_min[self_ix2_min_index++] = configData[i]<<8|configData[i+1];
					}
					i = i +2;
					VIVO_TS_LOG_INF("%s,0xaa0x55 05 configData[i] = %d\n",__func__,configData[i]);
				#endif					
				}break;	
				//self_ix2_max
				case 0x06://SELF_IX2_MAX
				{
					unsigned int self_ix2_max_size=0,self_ix2_max_index = 0;
					self_ix2_max_size = (tx_num + rx_num);
					self_ix2_max = (unsigned int *)kmalloc(self_ix2_max_size*sizeof(unsigned int ), GFP_KERNEL);
				#if 0					
					while(self_ix2_max_index < self_ix2_max_size)
					{
						self_ix2_max[self_ix2_max_index++] = configData[i]<<8|configData[i+1];
						i = i +2;
					}
				#else //simplified threshold value reference
					while(self_ix2_max_index < tx_num)
					{
						self_ix2_max[self_ix2_max_index++] = configData[i]<<8|configData[i+1];
					}
					i = i +2;
					while(self_ix2_max_index < self_ix2_max_size)
					{
						self_ix2_max[self_ix2_max_index++] = configData[i]<<8|configData[i+1];
					}
					i = i +2;
					VIVO_TS_LOG_INF("%s,0xaa0x55 06 configData[i] = %d\n",__func__,configData[i]);
				#endif					
				}break;			
				//self_cx2_min
				case 0x07://SELF_CX2_MIN
				{
					unsigned int self_cx2_min_size=0,self_cx2_min_index = 0;
					self_cx2_min_size = (tx_num + rx_num);
					self_cx2_min = (unsigned int *)kmalloc(self_cx2_min_size*sizeof(unsigned int ), GFP_KERNEL);
				#if 0					
					while(self_cx2_min_index < self_cx2_min_size)
					{
						self_cx2_min[self_cx2_min_index++] = configData[i]<<8|configData[i+1];
					}
				#else//simplified threshold value reference
					while(self_cx2_min_index < tx_num)
					{
						self_cx2_min[self_cx2_min_index++] =configData[i]<<8|configData[i+1];
					}
					i = i +2;
					while(self_cx2_min_index < self_cx2_min_size)
					{
						self_cx2_min[self_cx2_min_index++] =configData[i]<<8|configData[i+1];
					}
					i = i +2;
					VIVO_TS_LOG_INF("%s,0xaa0x55 07 configData[i] = %d\n",__func__,configData[i]);
				#endif					
				}break;
				//self_cx2_max
				case 0x08://SELF_CX2_MAX
				{
					unsigned int self_cx2_max_size=0,self_cx2_max_index = 0;
					self_cx2_max_size = (tx_num + rx_num);
					self_cx2_max = (unsigned int *)kmalloc(self_cx2_max_size*sizeof(unsigned int ), GFP_KERNEL);
				#if 0	
					while(self_cx2_max_index < self_cx2_max_size)
					{
						self_cx2_max[self_cx2_max_index++] = configData[i]<<8|configData[i+1];
					}
				#else
					while(self_cx2_max_index < tx_num)
					{
						self_cx2_max[self_cx2_max_index++] = configData[i]<<8|configData[i+1];
					}	
					i = i +2;
					while(self_cx2_max_index < self_cx2_max_size)
					{
						self_cx2_max[self_cx2_max_index++] = configData[i]<<8|configData[i+1];
					}	
					i = i +2;				
					VIVO_TS_LOG_INF("%s,0xaa0x55 08 configData[i] = %d\n",__func__,configData[i]);
				#endif					
				}break;
				case 0x09://MSKEY_CX2_MIN  //0xAA 0x55 <key_cnt> <mskey_cx2_min[0]> ...<mskey_cx2_min[key_cnt-1]>
				{
					unsigned int mskey_cx2_min_size=0,mskey_cx2_min_index = 0;
					mskey_cx2_min_size = (unsigned int)configData[i++]; //key_cnt
					mskey_cx2_min = (unsigned int *)kmalloc(mskey_cx2_min_size*sizeof(unsigned int ), GFP_KERNEL);
				#if 0	
					while(mskey_cx2_min_index < mskey_cx2_min_size)
					{
						mskey_cx2_min[mskey_cx2_min_index++] = configData[i++];
					}
				#else
					while(mskey_cx2_min_index < mskey_cx2_min_size)
					{
						mskey_cx2_min[mskey_cx2_min_index++] = configData[i];
					}	
					i++;
					VIVO_TS_LOG_INF("%s,0xaa0x55 09 configData[i] = %d\n",__func__,configData[i]);
				#endif					
				}break;	
				case 0x0A://MSKEY_CX2_MAX //0xAA 0x55 <id ><key_cnt> <mskey_cx2_max[0]> ...<mskey_cx2_max[key_cnt-1]>
				{
					unsigned int mskey_cx2_max_size=0,mskey_cx2_max_index = 0;
					mskey_cx2_max_size = (unsigned int)configData[i++]; //key_cnt
					mskey_cx2_max = (unsigned int *)kmalloc(mskey_cx2_max_size*sizeof(unsigned int ), GFP_KERNEL);
				#if 0	
					while(mskey_cx2_max_index < mskey_cx2_max_size)
					{
						mskey_cx2_max[mskey_cx2_max_index++] = configData[i++];
					}
				#else
					while(mskey_cx2_max_index < mskey_cx2_max_size)
					{
						mskey_cx2_max[mskey_cx2_max_index++] = configData[i];
					}	
					i++;
					VIVO_TS_LOG_INF("%s,0xaa0x55 0A configData[i] = %d\n",__func__,configData[i]);
				#endif					
				}break;

				
			}//switch case
					VIVO_TS_LOG_INF("%s,0xaa0x55 switch exit configData[i] = %d\n",__func__,configData[i]);			
		}//if((configData[i] == 0xAA)&&(configData[i+1] == 0x55))
		else if((configData[i] == 0xAA)&&(configData[i+1] == 0x56))
		{
			i=i+2;
			VIVO_TS_LOG_INF("%s, 0xaa0x56 configData[i] = %d\n",__func__,configData[i]);
			switch(configData[i]) // 0xaa 0x56 <id> <infobyte1> <infobyte2>
			{
				case 0x01:
					mutual_raw_min = (configData[i+1]<<8)|configData[i+2];
					break;
				case 0x02:			
					mutual_raw_max = (configData[i+1]<<8)|configData[i+2];
					break;
				case 0x03:
					mutual_raw_gap =(configData[i+1]<<8)|configData[i+2];
					break;
				case 0x04:
					self_raw_force_min = (configData[i+1]<<8)|configData[i+2];
					break;
				case 0x05:
					self_raw_force_max = (configData[i+1]<<8)|configData[i+2];
					break;
				case 0x06:
					self_raw_sense_min = (configData[i+1]<<8)|configData[i+2];
					break;
				case 0x07:
					self_raw_sense_max = (configData[i+1]<<8)|configData[i+2];
					break;
				case 0x08:
					mskey_raw_min_thres = (configData[i+1]<<8)|configData[i+2];
					break;
				case 0x09:
					mskey_raw_max_thres = (configData[i+1]<<8)|configData[i+2];
					break;					
				
				//MSKEY
			}//switch case		
			i = i + 2;
		}//else if((configData[i] == 0xAA)&&(configData[i+1] == 0x66))
		else if((configData[i] == 0xAA)&&(configData[i+1] == 0x57))
		{ /* 0xaa 0x57 <tx_cnt> <rx_cnt> <node_cnt(N)> <Tx1><Rx1> ..<TxN><RxN>*/
			int node_index=0;
			int tx , rx;
			i = i + 2 ;
			VIVO_TS_LOG_INF("%s,0xaa0x57 configData[i] \n",__func__);
			tx                    = (int)configData[i++];
			rx                    = (int)configData[i++]; 
			sp_cx2_gap_node_count = (unsigned int)configData[i++];
			sp_cx2_gap_node       = (unsigned int)configData[i++];
			sp_cx2_gap_node_index = (unsigned int *)kmalloc(sp_cx2_gap_node_count, GFP_KERNEL);
			for(node_index=0;node_index < sp_cx2_gap_node_count;node_index++)
			{
				sp_cx2_gap_node_index[node_index] = ((unsigned int)configData[i]*rx) + (unsigned int)configData[i+1];
				i=i+2;
			}
				
		}
		else
		{
			i++;
			VIVO_TS_LOG_INF("%s,Data not matching %d\n",__func__,configData[i]);
		}
	}//for loop
	
	/*Test Results are transferred to the calling application through 
	* this Production test API in simple text format, 
	* in case of any specific format , please update the string value 
	* in snprintf() statement under each test  
	*/
	
	/* INITIALIZATION Test */
	if(fts_initialization_test(info) >= 0) {
		sensor_test_result[0] = 1;
		VIVO_TS_LOG_INF("fts_initialization_test pass.\n");
	} else {
		sensor_test_result[0] = 0;
		VIVO_TS_LOG_INF("fts_initialization_test fail.\n");
	}
		
	/* ITO test */
	if(fts_ito_test(info) == 0) {	
		sensor_test_result[7] = 1;
		VIVO_TS_LOG_INF("fts_ito_test pass.\n");
	} else {
		sensor_test_result[7] = 0;
		VIVO_TS_LOG_INF("fts_ito_test fail.\n");
	}
	
	/* fts_mutual_tune_data_test */
	if(	(cx2_h_thres != NULL)&&(cx2_v_thres != NULL)&&
		(cx2_min_thres != NULL)&&(cx2_max_thres != NULL))
	{
		if(fts_mutual_tune_data_test(info) == 0) {	
			sensor_test_result[1] = 1;
			VIVO_TS_LOG_INF("mutual tune data test pass.\n");
		} else {
			sensor_test_result[1] = 0;
			VIVO_TS_LOG_INF("mutual tune data test fail.\n");
		}	
	} else {
		sensor_test_result[1] = 2;	/* 2 means test data not enter */
		VIVO_TS_LOG_INF("[%s]fts_mutual_tune_data_test test data not entered .\n", __func__);
	}
	
	/* fts_self_tune_data_test */
	if((self_ix2_min != NULL)&&(self_ix2_max != NULL))
	{
		if(fts_self_tune_data_test(info) == 0) {
			sensor_test_result[2] = 1;
			VIVO_TS_LOG_INF("self tune data test pass.\n");
		} else {
			sensor_test_result[2] = 0;
			VIVO_TS_LOG_INF("self tune data test fail.\n");
		}
	} else {
		sensor_test_result[2] = 2;
		VIVO_TS_LOG_INF("[%s]fts_self_tune_data_test test data not entered .\n", __func__);
	}
	
	/* fts_mutual_raw_data_test */
	if((mutual_raw_min != 0xFFFF)&&(mutual_raw_max !=0xFFFF)&&(mutual_raw_gap != 0xFFFF))
	{
		if(fts_mutual_raw_data_test(info) == 0) {
			sensor_test_result[3] = 1;
			VIVO_TS_LOG_INF("mutual raw data test pass.\n");
		} else {	
			sensor_test_result[3] = 0;
			VIVO_TS_LOG_INF("mutual raw data test fail.\n");
		}
	} else {
		sensor_test_result[3] = 2;
		VIVO_TS_LOG_INF("[%s]fts_mutual_raw_data_test test data not entered .\n", __func__);
	}

	/* fts_self_raw_data_test */
	if(	(self_raw_force_min != 0xFFFF)&&(self_raw_force_max != 0xFFFF)&&
		(self_raw_sense_min != 0xFFFF)&&(self_raw_sense_max != 0xFFFF))
	{
		if(fts_self_raw_data_test(info) == 0) {
			sensor_test_result[4] = 1;
			VIVO_TS_LOG_INF("self raw data test pass.\n");
		} else {
			sensor_test_result[4] = 0;
			VIVO_TS_LOG_INF("self raw data test fail.\n");
		}	
	} else {
		sensor_test_result[4] = 2;
		VIVO_TS_LOG_INF("[%s]fts_self_raw_data_test test data not entered .\n", __func__);
	}
		
	/* fts_mskey_tune_data_test */
	if((mskey_cx2_min != NULL)&&(mskey_cx2_max != NULL))
	{
		if(fts_mskey_tune_data_test(info) == 0) {
			sensor_test_result[5] = 1;
			VIVO_TS_LOG_INF("fts_mskey_tune_data_test pass.\n");
		} else {	
			sensor_test_result[5] = 0;
			VIVO_TS_LOG_INF("fts_mskey_tune_data_test fail.\n");
		}	
	} else {
		sensor_test_result[5] = 2;
		VIVO_TS_LOG_INF("[%s]fts_mskey_tune_data_test test data not entered .\n", __func__);
	}
	
	/* fts_mskey_raw_data_test */
	if((mskey_raw_min_thres != 0xFFFF)&&(mskey_raw_max_thres != 0xFFFF))
	{
		if(fts_mskey_raw_data_test(info) == 0)	{
			sensor_test_result[6] = 1;
			VIVO_TS_LOG_INF("fts_mskey_raw_data_test pass.\n");
		} else {	
			sensor_test_result[6] = 0;
			VIVO_TS_LOG_INF("fts_mskey_raw_data_test fail.\n");
		}		
	} else {
		sensor_test_result[6] = 2;
		VIVO_TS_LOG_INF("[%s]fts_mskey_raw_data_test test data not entered .\n", __func__);
	}
	/* release_firmware(fw_config); */
	kfree(cx2_h_thres);	
	kfree(cx2_v_thres);
	kfree(cx2_min_thres);//CX2_MIN_THRESHOLD
	kfree(cx2_max_thres);//CX2_MAX_THRESHOLD
	kfree(self_ix2_min);//SELF_IX2_MIN
	kfree(self_ix2_max);//SELF_IX2_MAX
	kfree(self_cx2_min);//SELF_CX2_MIN
	kfree(self_cx2_max);//SELF_CX2_MAX
	kfree(mskey_cx2_min);
	kfree(mskey_cx2_max);
	kfree(sp_cx2_gap_node_index);

	fts_command(info, FLUSHBUFFER);
	fts_interrupt_set(info, INT_ENABLE);	
	fts_command(info, SENSEON);
#ifdef PHONE_KEY
	fts_command(info, KEY_ON);
#endif	

	VIVO_TS_LOG_INF("All test result:\ninitialization_test[%d] \nmutual_tune_data_test[%d] \
				\nself_tune_data_test[%d] \nmutual_raw_data_test[%d] \nself_raw_data_test[%d] \
				\nmskey_tune_data_test[%d] \nmskey_raw_data_test[%d] \
				\nito_test[%d].    \n\nIto test result:\npanel_open_force[%d] \npanel_open_sense[%d] \
				\nforce_short_to_ground[%d] \nsense_short_to_ground[%d] \
				\nforce_short_to_VDD[%d] \nsense_short_to_VDD[%d]  \
				\npin_to_pin_short_force[%d] \npin_to_pin_short_sense[%d]\n\n", 
				sensor_test_result[0], sensor_test_result[1], sensor_test_result[2], 
				sensor_test_result[3], sensor_test_result[4], sensor_test_result[5], 
				sensor_test_result[6],	sensor_test_result[7],
				ito_test_result[0],ito_test_result[1],ito_test_result[2],ito_test_result[3],
				ito_test_result[4],ito_test_result[5],ito_test_result[6],ito_test_result[7]
		);

	for(i=0; i<8; i++) {
		if(sensor_test_result[i]==0) {
			return sprintf(buf, "Fail!All test result:\ninitialization_test[%d] \nmutual_tune_data_test[%d] \
				\nself_tune_data_test[%d] \nmutual_raw_data_test[%d] \nself_raw_data_test[%d] \
				\nmskey_tune_data_test[%d] \nmskey_raw_data_test[%d] \
				\nito_test[%d].    \n\nIto test result:\npanel_open_force[%d] \npanel_open_sense[%d] \
				\nforce_short_to_ground[%d] \nsense_short_to_ground[%d] \
				\nforce_short_to_VDD[%d] \nsense_short_to_VDD[%d] \
				\npin_to_pin_short_force[%d] \npin_to_pin_short_sense[%d]\n\n", 
				sensor_test_result[0], sensor_test_result[1], sensor_test_result[2], 
				sensor_test_result[3], sensor_test_result[4], sensor_test_result[5], 
				sensor_test_result[6],	sensor_test_result[7],
				ito_test_result[0],ito_test_result[1],ito_test_result[2],ito_test_result[3],
				ito_test_result[4],ito_test_result[5],ito_test_result[6],ito_test_result[7]
			);
		}
	}
			
	return sprintf(buf, "Pass\n");	
}
