/******************** (C) COPYRIGHT 2012 STMicroelectronics ********************
*
* File Name	: fts.c
* Authors		: AMS(Analog MEMS Sensor)
*			        : JH JANG(jh.jang@st.com)
* Description	: I2C Shell for FingerTipS
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
********************************************************************************
* REVISON HISTORY
*
*VERSION | DATE 	     |AUTHORS				| DESCRIPTION
* 0.10    |  11/Oct/2012| JH JANG				| First Release. Add FW Download function
* 0.20    |  22/Oct/2012| JH JANG				| Read Command, B0, B3/B1, B6, D0
* 0.30    |  22/Oct/2012| JH JANG				| Write Command, B0, B3/B1, B6
* 0.40    |  31/Oct/2012| JH JANG				| Add : Read frame, write multi bytes
* 0.41    |  01/Nov/2012| JH JANG				| Bug fix of writing cmd
* 0.42    |  01/Nov/2012| JH JANG				| add delay in Auto Tune
* 0.43    |  27/Nov/2012| JH JANG				| add write of B2
* 0.44    |  14/Dec/2012| JH JANG				| Support Nexus 7 Tablet, Support Alex GUI tool
* 0.45    |  07/Feb/2013| JH JANG				| Support New FW Memory organization(62+2 KB)
*******************************************************************************/

#define VERSION "v0.45"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/stat.h>
#include <linux/types.h>
#include "i2c-dev.h"
#include "fts.h"

// #define DEBUG_MSG

typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned char u8;

#define BUFFER_MAX								(256 * 1024) - 16
#define TEMP_MAX									0xFF
#define FTS_ADDRESS								0x49
#define I2C_ADDRESS								FTS_ADDRESS

#define CHIP_ID_0										0x39
#define CHIP_ID_1										0x80

#define WRITE_CHUNK_SIZE						(2 * 1024) - 16
#define READ_CHUNK_SIZE						(2 * 1024) - 16

#define FW_MAIN_BLOCK							0x01
#define FW_INFO_BLOCK							0x02

#define FW_CMD_BurnMainBlock					0xF2
#define FW_CMD_BurnInfoBlock					0xFB

#define FW_CMD_EraseMainBlock				0xF3

#define MAIN_BLOCK_ADDR						0x00000000
#define MAIN_BLOCK_SIZE							(62 * 1024)
#define INFO_BLOCK_ADDR						0x0000F800
#define INFO_BLOCK_SIZE							(2 * 1024)

enum {
	TYPE_RAW_TOUCH								= 0,
	TYPE_FILTER_TOUCH							= 2,
	TYPE_NORM_TOUCH							= 4,
	TYPE_CALIB_TOUCH								= 6,
	TYPE_RAW_SELF									= 8,
	TYPE_RAW_HOVER_FORCE					= 10,
	TYPE_RAW_HOVER_SENSE					= 12,
	TYPE_FILTER_HOVER_FORCE				= 14,
	TYPE_FILTER_HOVER_SENSE					= 16,
	TYPE_NORM_HOVER_FORCE					= 18,
	TYPE_NORM_HOVER_SENSE					= 20,
	TYPE_CALIB_HOVER_FORCE					= 22,
	TYPE_CALIB_HOVER_SENSE					= 24
};

#define EVENTID_NO_EVENT									0x00
#define EVENTID_SLEEPOUT_CONTROLLER_READY	0x02
#define EVENTID_ENTER_POINTER							0x03
#define EVENTID_LEAVE_POINTER							0x04
#define EVENTID_MOTION_POINTER							0x05
#define EVENTID_STATIONARY_TOUCH						0x06
#define EVENTID_HOVER_ENTER_POINTER				0x07
#define EVENTID_HOVER_LEAVE_POINTER				0x08
#define EVENTID_HOVER_MOTION_POINTER				0x09
#define EVENTID_ERROR										0x0F
#define EVENTID_CONTROLLER_READY					0x10
#define EVENTID_BUTTON_STATUS							0x11
#define EVENTID_SW_REGISTER_READ						0x12

#define SLEEPIN							0x90
#define SLEEPOUT						0x91
#define SENSEOFF						0x92
#define SENSEON							0x93

#define FLUSHBUFFER					0xA1
#define FORCECALIBRATION			0xA2
#define CX_TUNNING						0xA3
#define SELF_AUTO_TUNE				0xA4

#define CX_FLASH_BACKUP			0xFC

int SenseChannelLength = 0;
int ForceChannelLength = 0;
short pFrame[36*60*2];
unsigned char i2c_channels[] = {1, 3, 5, 0, 2, 4}; // 5 for ICON, 0 for Odroid

static int i2c_fd = 0;

void print_usage(void)
{
	printf("\n");
	printf("I2C Shell for FTS/FTT %s [%s] \n\n", VERSION, __DATE__);
	printf("==[ Read ]====================================================================== \n");
	printf("  fts r 85 8 \n");
	printf("  fts r b00000 2 \n");
	printf("  fts r b30000 b10020 10 \n");
	printf("  fts r b60066 100 \n");
	printf("  fts r d00000 16 \n");
	printf("==[ Write ]===================================================================== \n");
	printf("  fts w a1 \n");
	printf("  fts w b01000 1 2 f  ; Can write multi bytes \n");
	printf("  fts w b30000 b10020 1 2 3 d e f ; Can write multi bytes \n");
	printf("  fts w b60066 1 2 3 a b c ; Can write multi bytes \n");
	printf("==[ Read Touch Frame Data ]===================================================== \n");
	printf(" [Raw Touch]\n");
	printf("  fts raw\n");
	printf(" [Filtered Touch]\n");
	printf("  fts filter\n");
	printf(" [Normalized Touch\n");
	printf("  fts str | fts norm\n");
	printf(" [Calibrated Touch]\n");
	printf("  fts base | fts calib\n");
	printf(" [Raw Self]\n");
	printf("  fts rawself | fts rs\n");
	printf("==[ Read Hover Frame Data ]===================================================== \n");
	printf(" [Raw Hover Force]\n");
	printf("  fts rawhoverforce | fts rhf\n");
	printf(" [Raw Hover Sense]\n");
	printf("  fts rawhoversense | fts rhs\n");
	printf(" [Filtered Hover Force]\n");
	printf("  fts filterhoverforce | fts fhf\n");
	printf(" [Filtered Hover Sense]\n");
	printf("  fts filterhoversense | fts fhs\n");
	printf(" [Normalized Hover Force]\n");
	printf("  fts normhoverforce | fts nhf\n");
	printf(" [Normalized Hover Sense]\n");
	printf("  fts normhoversense | fts nhs\n");
	printf(" [Calibrated Hover Force]\n");
	printf("  fts calibhoverforce | fts chf\n");
	printf(" [Calibrated Hover Sense]\n");
	printf("  fts calibhoversense | fts chs\n");
	printf("==[ Flash ]===================================================================== \n");
	printf(" [Flash 62KB of Main Block] \n");
	printf("  fts main FILENAME.memh \n");
	printf(" [Flash 2KB of Info Block] \n");
	printf("  fts info FILENAME.memh \n");
	printf("==[ Utils ]===================================================================== \n");
	printf(" [System Reset (Write 0x1 to B60023)]\n");
	printf("  fts reset \n");
	printf(" [Auto Tune]\n");
	printf("  fts atune \n");
}

void delay_sec(unsigned long sec )
{
	struct timespec timeToSleep;

	timeToSleep.tv_sec = sec;
	timeToSleep.tv_nsec = 0;
	nanosleep(&timeToSleep, NULL);
}

void delay(unsigned long ms )
{
	struct timespec timeToSleep;

	timeToSleep.tv_sec = 0;
	timeToSleep.tv_nsec = 1000000 * ms; //1ms
	nanosleep(&timeToSleep, NULL);
}

void str_tolower(char *pStr)
{
	while(*pStr != 0)
	{
		if( *pStr >= 'A' && *pStr <= 'Z' )
	         *pStr += 32;
		pStr++;
	}
}

int strnum_to_digtial(char *pInBuf, int InCnt)
{
	int	num[16] = {0}; 	  
	int	count = 1;		 
	int	result = 0;
	int  i;

	for	(i = InCnt - 1; i >= 0; i--)		 
	{
		if ((pInBuf[i] >= '0') && (pInBuf[i] <= '9'))
			num[i] = pInBuf[i] - 48;
		else if ((pInBuf[i] >= 'a') && (pInBuf[i] <= 'f'))
			num[i] = pInBuf[i] - 'a' + 10;
		else if ((pInBuf[i] >= 'A') && (pInBuf[i] <= 'F'))
			num[i] = pInBuf[i] - 'A' + 10;
		else
			num[i] = 0;

		result = result+num[i] * count;
		count = count * 16;
	}

	return result;
}

static int i2c_write_reg(unsigned char *pInBuf, __u16 InCnt)
{
	int ret = 0;
	struct i2c_msg                I2CMsg[2];
	struct i2c_rdwr_ioctl_data I2CIOCTLData;
	
	I2CMsg[0].addr = (__u16)I2C_ADDRESS;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)InCnt;
	I2CMsg[0].buf = (__u8*)pInBuf;

	I2CIOCTLData.msgs = &I2CMsg[0];
	I2CIOCTLData.nmsgs = 1;

	ret =  ioctl(i2c_fd, I2C_RDWR, &I2CIOCTLData);
	if(ret <= 0) printf("i2c_write_reg error \n");
	return ret;
}

static int i2c_read_reg(unsigned char *pInBuf, int InCnt, unsigned char *pOutBuf, int OutCnt)
{
	int ret = 0;
	struct i2c_msg                I2CMsg[2];
	struct i2c_rdwr_ioctl_data I2CIOCTLData;
	
	//write msg
	I2CMsg[0].addr = (__u16)I2C_ADDRESS;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)InCnt;
	I2CMsg[0].buf = (__u8*)pInBuf;

	//read msg
	I2CMsg[1].addr = (__u16)I2C_ADDRESS;
	I2CMsg[1].flags = I2C_M_RD;
	I2CMsg[1].len = OutCnt;
	I2CMsg[1].buf = (__u8*)pOutBuf;

	I2CIOCTLData.msgs = &I2CMsg[0];
	I2CIOCTLData.nmsgs = 2;

	ret = ioctl(i2c_fd, I2C_RDWR, &I2CIOCTLData);
	if(ret <= 0) printf("i2c_read_reg error \n");
	return ret;
}

static int i2c_read_reg2(unsigned char *pInBuf, int InCnt, unsigned char *pOutBuf, int OutCnt)
{
	struct i2c_msg                I2CMsg[2];
	struct i2c_rdwr_ioctl_data I2CIOCTLData;
	
	//write msg
	I2CMsg[0].addr = (__u16)I2C_ADDRESS;
	I2CMsg[0].flags = (__u16)0;
	I2CMsg[0].len = (__u16)InCnt;
	I2CMsg[0].buf = (__u8*)pInBuf;

	//read msg
	I2CMsg[1].addr = (__u16)I2C_ADDRESS;
	I2CMsg[1].flags = I2C_M_RD;
	I2CMsg[1].len = OutCnt;
	I2CMsg[1].buf = (__u8*)pOutBuf;

	I2CIOCTLData.msgs = &I2CMsg[0];
	I2CIOCTLData.nmsgs = 2;

	return ioctl(i2c_fd, I2C_RDWR, &I2CIOCTLData);
}

int read_chip_id(void)
{
	unsigned char regAdd[3] = {0xB6, 0X00, 0X07};
	unsigned char val[8] = {0};	

	if(i2c_read_reg2(&regAdd[0], 3, &val[0], 5) <= 0)
		return 1;

	#ifdef DEBUG_MSG
	printf("read_chip_id B60007 : %02X %02X %02X %02X \n", val[1], val[2], val[3], val[4]);
	#endif

	if(val[1] != CHIP_ID_0 || val[2] != CHIP_ID_1)
	{
		return 1;
	}

	return 0;
}

void fts_system_reset(void)
{
	unsigned char regAdd[4] = {0xB6, 0x00, 0x23, 0x01};
	
	i2c_write_reg(&regAdd[0],4);
	printf("\n System Reset \n");
	delay(10);
}

void fts_command(unsigned char cmd)
{
	unsigned char regAdd = 0;

	regAdd = cmd;
	i2c_write_reg(&regAdd, 1);
}

int i2c_open(void)
{
	int fd = 0;
	char filename[20] = {0};
	int i = 0;
	int cnt = sizeof(i2c_channels);

	for(i = 0; i < cnt; i++)
	{
		memset(filename, 0x0, 20);
		snprintf(filename, 19, "/dev/i2c-%d", i2c_channels[i]);
		
		fd = open(filename, O_RDWR);
		if(fd > 0)
		{
			i2c_fd = fd;
			if(read_chip_id() == 0)
				return i2c_fd;
			else
			{
				i2c_fd = 0;
				close(fd);
			}
		}
	}

	printf("Wrong i2c channel or cannot read Chip ID \n");
	return 0;
}

void i2c_close(int fd)
{
	if(fd)
		close(fd);
}

int i2c_get_values(int cnt, unsigned char **pIn, unsigned char *pOut)
{
	int i = 0;
	int len = 0;
	int ret = 0;

	for(i = 0; i < cnt; i++)
	{
		len = strlen(pIn[i]);
		
		if(len > 0 &&  len < 3)
		{
			pOut[i] =  strnum_to_digtial(pIn[i], len);
			ret++;
		}
		else
		{
			printf("Wrong value = %s \n", pIn[i]);
			return ret;
		}
	}

	return ret;
}

int i2c_read_cmd(unsigned int address, int argc, char **argv)
{
	int requestlen = 0;
	int requestbytes = 0;
	int ret = 0;
	int i = 0;
	unsigned char pAddress[8] = {0};
	unsigned char pRead[BUFFER_MAX] = {0x0};

	if(argc == 3) // no input
	{
		requestlen = 1;
		requestbytes = 1;
	}
	else
	{
		requestlen = strlen(argv[3]);
		requestbytes = atoi(argv[3]);

		if(requestbytes > BUFFER_MAX)
			requestbytes = BUFFER_MAX;
	}
	
	pAddress[0] = address & 0xFF;

	#ifdef DEBUG_MSG
	printf("read cmd = %02X \n", pAddress[0]);
	#endif

	ret = i2c_read_reg(pAddress, 1, pRead, requestbytes);
	if(ret >= 0)
	{
		printf("{\n");
		for(i = 0; i < requestbytes; i++)
		{
			printf("%02X ", pRead[i]);
		}
		printf("\n}\n");
	}
	else
	{
		printf("read failed rc = %d \n", ret);
		goto ErrorExit;
	}

ErrorExit:

	return ret;
}

int i2c_read_B3(unsigned int address, int argc, char **argv)
{
	int addresslen = 0;
	unsigned int address_lower = 0;
	int requestlen = 0;
	int requestbytes = 0;
	int ret = 0;
	int i = 0;
	unsigned char pAddress[8] = {0};
	unsigned char pRead[BUFFER_MAX] = {0x0};

	if(argc == 4) // no input
	{
		requestlen = 2;
		requestbytes = 2;
	}
	else
	{
		requestlen = strlen(argv[4]) + 1;
		requestbytes = atoi(argv[4]) + 1;

		if(requestbytes > BUFFER_MAX)
			requestbytes = BUFFER_MAX;
	}
	
	// Read Address (Hex)
	addresslen = strlen(argv[3]);
	if(addresslen > 0 &&  addresslen <= 8)
	{
		address_lower = strnum_to_digtial(argv[3], addresslen);
	}
	else
	{
		printf("Wrong address_lower = 0x%X \n", address_lower);
		goto ErrorExit;
	}

	if(address_lower >= 0xB10000 && address_lower < 0xB20000)
	{
		pAddress[0] = (address >> 16) & 0xFF;
		pAddress[1] = (address >> 8) & 0xFF;
		pAddress[2] = address & 0xFF;
		#ifdef DEBUG_MSG
		printf("%02X%02X%02X ", pAddress[0], pAddress[1], pAddress[2]);
		#endif
		
		 i2c_write_reg(&pAddress[0], 3);

		pAddress[3] = (address_lower >> 16) & 0xFF;
		pAddress[4] = (address_lower >> 8) & 0xFF;
		pAddress[5] = address_lower & 0xFF;
		#ifdef DEBUG_MSG
		printf("%02X%02X%02X \n", pAddress[3], pAddress[4], pAddress[5]);
		#endif

		ret = i2c_read_reg(&pAddress[3], 3, pRead, requestbytes);
		if(ret >= 0)
		{
			printf("{\n");
			for(i = 1; i < requestbytes; i++)
			{
				printf("%02X ", pRead[i]);
			}
			printf("\n}\n");
		}
		else
		{
			printf("read failed rc = %d \n", ret);
			goto ErrorExit;
		}
	}
	else
	{
		printf("Wrong address_lower = 0x%X \n", address_lower);
		goto ErrorExit;
	}

ErrorExit:

	return ret;
}

int i2c_read_B0_B6(unsigned int address, int argc, char **argv)
{
	int requestlen = 0;
	int requestbytes = 0;
	int ret = 0;
	int i = 0;
	unsigned char pAddress[8] = {0};
	unsigned char pRead[BUFFER_MAX] = {0x0};

	if(argc == 3) // no input
	{
		requestlen = 2;
		requestbytes = 2;
	}
	else
	{
		requestlen = strlen(argv[3]) + 1;
		requestbytes = atoi(argv[3]) + 1;

		if(requestbytes > BUFFER_MAX)
			requestbytes = BUFFER_MAX;
	}

	pAddress[0] = (address >> 16) & 0xFF;
	pAddress[1] = (address >> 8) & 0xFF;
	pAddress[2] = address & 0xFF;

	#ifdef DEBUG_MSG
	printf("read B6 = %02X%02X%02X \n", pAddress[0], pAddress[1], pAddress[2]);
	#endif

	ret = i2c_read_reg(pAddress, 3, pRead, requestbytes);
	if(ret >= 0)
	{
		printf("{\n");
		for(i = 1; i < requestbytes; i++)
		{
			printf("%02X ", pRead[i]);
		}
		printf("\n}\n");
	}
	else
	{
		printf("read failed rc = %d \n", ret);
		goto ErrorExit;
	}

ErrorExit:

	return ret;
}

int i2c_read_D0(unsigned int address, int argc, char **argv)
{
	int requestlen = 0;
	int requestbytes = 0;
	int ret = 0;
	int i = 0;
	unsigned char pAddress[8] = {0};
	unsigned char pRead[BUFFER_MAX] = {0x0};

	if(argc == 3) // no input
	{
		requestlen = 2;
		requestbytes = 2;
	}
	else
	{
		requestlen = strlen(argv[3]);
		requestbytes = atoi(argv[3]);

		if(requestbytes > BUFFER_MAX)
			requestbytes = BUFFER_MAX;
	}

	pAddress[0] = (address >> 16) & 0xFF;
	pAddress[1] = (address >> 8) & 0xFF;
	pAddress[2] = address & 0xFF;

	#ifdef DEBUG_MSG
	printf("read D0 = %02X%02X%02X \n", pAddress[0], pAddress[1], pAddress[2]);
	#endif

	ret = i2c_read_reg(pAddress, 3, pRead, requestbytes);
	if(ret >= 0)
	{
		printf("{\n");
		for(i = 0; i < requestbytes; i++)
		{
			printf("%02X ", pRead[i]);
		}
		printf("\n}\n");
	}
	else
	{
		printf("read failed rc = %d \n", ret);
		goto ErrorExit;
	}

ErrorExit:

	return ret;
}

int i2c_read(int argc, char **argv)
{
	int addresslen = 0;
	unsigned int address = 0;
	int rc = 0;

	// Read Address (Hex)
	addresslen = strlen(argv[2]);
	if(addresslen > 0 &&  addresslen <= 8)
	{
		address = strnum_to_digtial(argv[2], addresslen);
	}
	else
	{
		printf("Wrong address = 0x%X \n", address);
		goto ErrorExit;
	}

	#ifdef DEBUG_MSG
	printf("parsing address = %X \n", address);
	#endif
		
	// Processing
	if(address >= 0x00 && address < 0x100) // Command
	{		
		return i2c_read_cmd(address, argc, argv);
	}
	else if(address >= 0xB00000 && address < 0xB10000) // B0
	{
		return i2c_read_B0_B6(address, argc, argv);
	}
	else if(address >= 0xB30000 && address < 0xB40000) // B3 and B1
	{
		return i2c_read_B3(address, argc, argv);
	}
	else if(address >= 0xB60000 && address < 0xB70000) // B6
	{
		return i2c_read_B0_B6(address, argc, argv);
	}
	else if(address >= 0xD00000 && address < 0xE00000) // D0
	{
		return i2c_read_D0(address, argc, argv);
	}

ErrorExit:

	return rc;
}

int i2c_write_cmd(unsigned int address, int argc, char **argv)
{
	unsigned char pAddress[BUFFER_MAX] = {0};
	int ret = 0;
	
	pAddress[0] = address & 0xFF;

	#ifdef DEBUG_MSG
	printf("write cmd = %02X \n", pAddress[0]);
	#endif
	
	ret = i2c_write_reg(pAddress, 1);

	return ret;
}

int i2c_write_B3(unsigned int address, int argc, char **argv)
{
	int addresslen = 0;
	unsigned int address_lower = 0;
	unsigned char pAddress[BUFFER_MAX] = {0};
	unsigned char pValue[BUFFER_MAX] = {0};
	unsigned char readback = 0;
	int ret = 0;
	
	// Read Address (Hex)
	addresslen = strlen(argv[3]);
	if(addresslen > 0 &&  addresslen <= 8)
	{
		address_lower = strnum_to_digtial(argv[3], addresslen);
	}
	else
	{
		printf("Wrong address_lower = 0x%X \n", address_lower);
		goto ErrorExit;
	}

	ret = i2c_get_values(argc - 4, (unsigned char**)&argv[4], &pAddress[6]);

	// Target Value (Hex)
	if(ret != argc - 4 )
	{
		printf("Wrong value \n");
		goto ErrorExit;
	}

	#ifdef DEBUG_MSG
	{
		int i = 0;

		for(i = 0; i < ret; i++)
		{
			printf("%02X ", pAddress[i + 6]);
		}
		printf("\n");
	}
	#endif

	if(address_lower >= 0xB10000 && address_lower < 0xB20000)
	{
		pAddress[0] = (address >> 16) & 0xFF;
		pAddress[1] = (address >> 8) & 0xFF;
		pAddress[2] = address & 0xFF;
		#ifdef DEBUG_MSG
		printf("%02X%02X%02X ", pAddress[0], pAddress[1], pAddress[2]);
		#endif
		
		i2c_write_reg(&pAddress[0], 3);

		pAddress[3] = (address_lower >> 16) & 0xFF;
		pAddress[4] = (address_lower >> 8) & 0xFF;
		pAddress[5] = address_lower & 0xFF;
		#ifdef DEBUG_MSG
		printf("%02X%02X%02X%02X \n", pAddress[3], pAddress[4], pAddress[5], pAddress[6]);
		#endif
		i2c_write_reg(&pAddress[3], 3 + ret);

		//Readback for verify
		ret = i2c_read_reg(&pAddress[3], 3, pValue, 2);
		if( pAddress[6] != pValue[1])
		{
			printf("Failed - Write(0x%02X) but Readback(0x%02X) \n", pAddress[6], pValue[1]);
			goto ErrorExit;
		}
	}
	else
	{
		printf("Wrong address_lower = 0x%X \n", address_lower);
		goto ErrorExit;
	}

ErrorExit:

	return ret;

}

int i2c_write_B0_B2_B6(unsigned int address, int argc, char **argv)
{
	unsigned char pAddress[BUFFER_MAX] = {0};
	unsigned char pValue[BUFFER_MAX] = {0};
	int valuelen = 0;
	unsigned int value = 0;
	unsigned char readback = 0;
	int ret = 0;

	ret = i2c_get_values(argc - 3, (unsigned char**)&argv[3], &pAddress[3]);

	// Target Value (Hex)
	if(ret != argc - 3 )
	{
		printf("Wrong value \n");
		goto ErrorExit;
	}
	
	pAddress[0] = (address >> 16) & 0xFF;
	pAddress[1] = (address >> 8) & 0xFF;
	pAddress[2] = address & 0xFF;

	#ifdef DEBUG_MSG
	printf("write %02X%02X%02X \n", pAddress[0], pAddress[1], pAddress[2]);
	{
		int i = 0;

		for(i = 0; i < ret; i++)
		{
			printf("%02X ", pAddress[i + 3]);
		}
		printf("\n");
	}
	#endif

	ret = i2c_write_reg(pAddress, 3 + ret);

	//Readback for verify
	ret = i2c_read_reg(pAddress, 3, pValue, 2);
	if( pAddress[3] != pValue[1])
	{
		printf("Failed - Write(0x%02X) but Readback(0x%02X) \n", pAddress[6], pValue[1]);
		goto ErrorExit;
	}

ErrorExit:

	return ret;	
}

int i2c_write(int argc, char **argv)
{
	int addresslen = 0;
	unsigned int address = 0;
	unsigned int address_lower = 0;
	int valuelen = 0;
	unsigned int value = 0;
	int ret = 0;
	int rc = 0;
	unsigned char pAddress[BUFFER_MAX] = {0};
	unsigned char pValue[BUFFER_MAX] = {0};
	unsigned char readback = 0;
	
	// Target Address (Hex)
	addresslen = strlen(argv[2]);
	if(addresslen > 0 &&  addresslen <= 8)
	{
		address = strnum_to_digtial(argv[2], addresslen);
	}
	else
	{
		printf("Wrong address = 0x%X \n", address);
		rc = 1;
		goto ErrorExit;
	}

	#ifdef DEBUG_MSG
	printf("write address = %X \n", address);
	#endif

	// Processing
	if(address > 0x00 && address < 0x100) // Command
	{
		return i2c_write_cmd(address, argc, argv);
	}
	else if(address >= 0xB00000 && address < 0xB10000) // B0
	{
		if(argc < 4)
		{
			printf("Parameter missing \n");
			rc = 3;
			goto ErrorExit;
		}

		return i2c_write_B0_B2_B6(address, argc, argv);
	}
	else if(address >= 0xB20000 && address < 0xB30000) // B2
	{
		if(argc < 4)
		{
			printf("Parameter missing \n");
			rc = 3;
			goto ErrorExit;
		}

		return i2c_write_B0_B2_B6(address, argc, argv);
	}
	else if(address >= 0xB30000 && address < 0xB40000) // B3 and B1
	{
		if(argc < 5)
		{
			printf("Parameter missing \n");
			rc = 3;
			goto ErrorExit;
		}
		
		return i2c_write_B3(address, argc, argv);
	}
	else if(address >= 0xB60000 && address < 0xB70000) // B6
	{
		if(argc < 4)
		{
			printf("Parameter missing \n");
			print_usage();
			rc = 3;
			goto ErrorExit;
		}

		return i2c_write_B0_B2_B6(address, argc, argv);
	}

ErrorExit:

	return rc;
}

int fts_auto_tune(void)
{
	fts_system_reset();

	fts_command(SLEEPOUT);
	printf(" Sleep Out \n");
	delay(300);
	
	fts_command(SENSEON);
	printf(" Sense On \n");
	delay(100);
	
	fts_command(CX_TUNNING);
	printf(" CX Tunning \n");
	delay(500);

	fts_command(CX_FLASH_BACKUP);
	printf(" CX Flash Backup (0xFC)\n");
	delay(500);

	fts_command(FORCECALIBRATION);
	printf(" Force Calibration \n");
	
	printf(" Auto Tune done \n");
}

unsigned char fw_load_ram(unsigned int addr, unsigned int maxsize, unsigned char *pData, int size)
{
	unsigned int writeAddr = 0;
	int i = 0;
	int j = 0;
	unsigned char byteWork0[2], byteWork1[WRITE_CHUNK_SIZE + 3], regAdd[3] = { 0xB3, 0xB1, 0 };
	unsigned char pReadback[67] = {0};
	char run = 1;

	fflush(stdout);
	
	while ((j < maxsize) && (j < size)) {
		writeAddr = addr + j;

		byteWork0[0] = (writeAddr >> 24) & 0xFF;
		byteWork0[1] = (writeAddr >> 16) & 0xFF;
		regAdd[0] = 0xB3;
		regAdd[1] = byteWork0[0];
		regAdd[2] = byteWork0[1];
		i2c_write_reg(&regAdd[0], 3);

		pReadback[0] = byteWork1[0] = 0xB1;
		pReadback[1] = byteWork1[1] = (writeAddr >> 8) & 0xFF;
		pReadback[2] = byteWork1[2] = writeAddr & 0xFF;

		i = 0;
		while ((j < maxsize) && (i < WRITE_CHUNK_SIZE)) {
			byteWork1[i + 3] = pData[j];
			i++;
			j++;
		}
		i2c_write_reg(&byteWork1[0], WRITE_CHUNK_SIZE + 3);
		#if 0 // STKorea_JHJANG 121011
		if(run == 1)
		{
			i2c_read_reg(&pReadback[0], 3, &pReadback[3], READ_CHUNK_SIZE);
			if( memcmp(pData, &pReadback[4], READ_CHUNK_SIZE-4) != 0)
			{
				printf("Diff \n");
				
				for(i = 0; i < 120; i += 4)
				{
					printf("%05d : %02X%02X%02X%02X \n", (i/4)+1, pData[i+3], pData[i+2], pData[i+1], pData[i]);
				}

				printf("\n");

				for(i = 0; i < 120; i += 4)
				{
					printf("%05d : %02X%02X%02X%02X \n", (i/4)+1, pReadback[i+3+4], pReadback[i+2+4], pReadback[i+1+4], pReadback[i+4]);
				}
				
			}
			run = 0;
		}
		#endif

	}
	
	printf( "Done %d Bytes\n", j);

	return 0;
}

void fts_parsing_memh(unsigned char *pFilename, unsigned char **pOut, int *pOutsize)
{
	FILE *pFile = NULL;
	unsigned char pBuf[64] = {0};
	unsigned char *pFileBuf = NULL;
	unsigned char *pData = NULL;
	int length = 0;
	int datapos = 0;
	int i = 0;
	
	pFile = fopen(pFilename, "rb");
	if (pFile == NULL)
	{
		printf("Open %s file error\n", pFilename);
		return;
	}

	fseek(pFile, 0, SEEK_END);
	length = ftell(pFile);
	fseek(pFile, 0, SEEK_SET);

	pData = (unsigned char*)malloc(MAIN_BLOCK_SIZE * 2);
	if(pData == NULL)
	{
		goto ErrorExit;
	}
	
	while(1)
	{
		pBuf[0]= '\0';
		fgets(pBuf, 64, pFile);
				
		length  = strlen(pBuf);
		if(length==0)	break;

		pData[datapos+3] = strnum_to_digtial(&pBuf[0], 2);
		pData[datapos+2] = strnum_to_digtial(&pBuf[2], 2);
		pData[datapos+1] = strnum_to_digtial(&pBuf[4], 2);
		pData[datapos+0] = strnum_to_digtial(&pBuf[6], 2);
		datapos += 4;
	}

	#if 0 // STKorea_JHJANG 121011
	for(i = 0; i < datapos; i += 4)
	{
		printf("%05d : %02X%02X%02X%02X \n", (i/4)+1, pData[i+3], pData[i+2], pData[i+1], pData[i]);
	}
	#endif

ErrorExit:
	fclose(pFile);

	*pOut = pData;
	*pOutsize = datapos;
}

int fw_unlock(void)
{
	int ret = 0;
	unsigned char regAdd[4] = {0xF7, 0x74, 0x45, 0x0}; //Unlock Flash Command ==> F7 74 45

	ret = i2c_write_reg(&regAdd[0],4);

	if(ret <= 0)
		printf("fw_erase error \n");
	else
	{
		printf("\n Flash Unlocked\n");
		delay_sec(3);
	}

	return ret;	
}

int fw_erase(char block_type)
{
	int ret = 0;
	unsigned char regAdd = 0;
	
	if(block_type == FW_MAIN_BLOCK || block_type == FW_INFO_BLOCK)
	{
		regAdd = FW_CMD_EraseMainBlock;
		ret = i2c_write_reg(&regAdd,1);
	}
	else
	{
		printf("Block Type error \n");
	}
	
	if(ret <= 0)
		printf("fw_erase error \n");
	else
	{
		printf("\n Flash Erase\n");
		delay_sec(3);
	}

	return ret;
}

int fw_flash_cmd(char block_type)
{
	int ret = 0;
	unsigned char regAdd = 0;
	
	if(block_type == FW_MAIN_BLOCK || block_type == FW_INFO_BLOCK)
	{
		regAdd = FW_CMD_BurnMainBlock;
		ret = i2c_write_reg(&regAdd,1);
	}
	else
	{
		printf("Block Type error \n");
	}

	if(block_type == FW_INFO_BLOCK)
	{
		regAdd = FW_CMD_BurnInfoBlock;
		ret = i2c_write_reg(&regAdd,1);
	}

	if(ret <= 0)
		printf("fw_write_cmd error \n");
	else
	{
		printf("\n Flash burning starts...\n");
		delay_sec(3);
	}
	
	return ret;
}

void fw_download(unsigned char *pFilename, char block_type, char system_reset)
{
	unsigned char *pData = NULL;
	int datasize = 0;
	int i =0;

	if(block_type == FW_MAIN_BLOCK || block_type == FW_INFO_BLOCK)
	{
		printf("\n Reading the MEMH file... ");
		fts_parsing_memh(pFilename, &pData, &datasize);
		printf("%d KB \n", datasize/1024);

		#if 0 // STKorea_JHJANG 121011
		for(i = 0; i < 64; i += 4)
		{
			printf("%05d : %02X%02X%02X%02X \n", (i/4)+1, pData[i+3], pData[i+2], pData[i+1], pData[i]);
		}
		#endif
	
		if(pData == NULL || datasize == 0)
		{
			printf("%s file read error\n", pFilename);
			goto ErrorExit;
		}
		
		if(fw_unlock() <= 0)
			goto ErrorExit;
		
		printf("\n PRAM loading starts... ");

		if(block_type == FW_MAIN_BLOCK)
			fw_load_ram(MAIN_BLOCK_ADDR, MAIN_BLOCK_SIZE, pData, datasize);
		else if(block_type == FW_INFO_BLOCK)
			fw_load_ram(INFO_BLOCK_ADDR, INFO_BLOCK_SIZE, pData, datasize);
		
		if(fw_erase(block_type) <= 0)
			goto ErrorExit;

		if(fw_flash_cmd(block_type) <= 0)
			goto ErrorExit;

		if(system_reset)
			fts_system_reset();

		printf("\n Flash completed \n\n");
	}
	else
		printf("Unknown block type \n");

ErrorExit :
	if(pData)
		free(pData);

	return;
}

void fts_print_frame(short *pData)
{
	int i = 0;
	int j = 0;
	short value = 0;
	short min = 0x7FFF;
	short max = 0;
	short min_x = 0x7FFF;
	short min_y = 0x7FFF;
	short max_x = 0;
	short max_y = 0;
	short col_min = 0x7FFF;
	short col_max = 0;

	printf("        ");
	for(i = 0; i < SenseChannelLength; i++)
	{
		printf("Rx%02d  ", i);
	}
	printf("  Min  ");
	printf(" Max  ");
	printf("\n");

	printf("     +");
	for(i = 0; i < SenseChannelLength; i++)
	{
		printf("------");
	}
	printf("-+-------------");
	printf("\n");
	
	for(i = 0; i < ForceChannelLength; i++)
	{
		col_min = 0x7FFF;
		col_max = 0;
	
		printf("Tx%02d | ", i);

		for(j = 0; j < SenseChannelLength; j++)
		{
			value = pData[(i*SenseChannelLength) + j];
			printf("%5d ", value);

			if (value < min)
			{
				if( i > 0)
				{
					min = value;
					min_x = j;
					min_y = i;
				}
			}
			if (value > max)
			{
				if( i > 0)
				{
					max = value;
					max_x = j;
					max_y = i;
				}
			}

			if (value < col_min)
				col_min = value;
			if (value > col_max)
				col_max = value;
		}
		printf("| %5d %5d", col_min, col_max);
		printf("\n");
	}
	printf(" Min : %5d [ %02d , %02d ] \n", min, min_x , min_y );
	printf(" Max : %5d [ %02d , %02d ] \n", max, max_x , max_y );
	printf(" Dif : %5d \n", max - min);
}

int fts_read_frame(unsigned char type)
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
	unsigned char pRead[BUFFER_MAX] = {0};
	int rc = 0;
	int ret = 0;
	int i = 0;
	int j = 0;
	int requestbytes = 0;

	i2c_write_reg(&pChannelLength[0], 3);
	ret = i2c_read_reg(&pChannelLength[3], 3, pRead, pChannelLength[6]);
	if(ret >= 0)
	{
		SenseChannelLength = pRead[1];
		ForceChannelLength = pRead[2];
		totalbytes = SenseChannelLength * ForceChannelLength * 2;
	}
	else
	{
		printf("read failed rc = %d \n", ret);
		rc = 1;
		goto ErrorExit;
	}

	#ifdef DEBUG_MSG
	printf("SenseChannelLength = %X, ForceChannelLength = %X \n", SenseChannelLength, ForceChannelLength);
	#endif

	pFrameAddress[2] = type;

	ret = i2c_read_reg(&pFrameAddress[0], 3, pRead, pFrameAddress[3]);
	if(ret >= 0)
	{
		FrameAddress = pRead[0] + (pRead[1] << 8);
		start_addr = 0xD0000000 + FrameAddress;
		end_addr = 0xD0000000 + FrameAddress + totalbytes;
	}
	else
	{
		printf("read failed rc = %d \n", ret);
		rc = 2;
		goto ErrorExit;
	}

	#ifdef DEBUG_MSG
	printf("FrameAddress = %X \n", start_addr);
	printf("start_addr = %X, end_addr = %X \n", start_addr, end_addr);
	#endif

	remained = totalbytes;
	
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
		#ifdef DEBUG_MSG
		printf("%02X%02X%02X readbytes=%d\n", pFrameAddress[0], pFrameAddress[1], pFrameAddress[2], readbytes);
		#endif
		i2c_read_reg(&pFrameAddress[0], 3, pRead, readbytes);

		remained -= readbytes;

		for(i = 0; i < readbytes; i += 2)
		{
			pFrame[dataposition++] = pRead[i] + (pRead[i + 1] << 8);
		}
	}

	#ifdef DEBUG_MSG
	printf("writeAddr = %X, start_addr = %X, end_addr = %X \n", writeAddr, start_addr, end_addr);
	#endif

	switch(type)
	{
		case 	TYPE_RAW_TOUCH :
			printf("[Raw Touch : 0x%X] \n", start_addr);
			break;
		case TYPE_FILTER_TOUCH :
			printf("[Filtered Touch : 0x%X] \n", start_addr);
			break;
		case TYPE_NORM_TOUCH:
			printf("[Normalized Touch : 0x%X] \n", start_addr);
			break;
		case TYPE_CALIB_TOUCH :
			printf("[Calibrated Touch : 0x%X] \n", start_addr);
			break;
		case TYPE_RAW_SELF :
			printf("[Raw Self : 0x%X] \n", start_addr);
			break;
		case TYPE_RAW_HOVER_FORCE :
			printf("[Raw Hover Force : 0x%X] \n", start_addr);
			break;
		case TYPE_RAW_HOVER_SENSE :
			printf("[Raw Hover Sense : 0x%X] \n", start_addr);
			break;
		case TYPE_FILTER_HOVER_FORCE :
			printf("[Filtered Hover Force : 0x%X] \n", start_addr);
			break;
		case TYPE_FILTER_HOVER_SENSE :
			printf("[Filtered Hover Sense : 0x%X] \n", start_addr);
			break;
		case TYPE_NORM_HOVER_FORCE :
			printf("[Normalized Hover Force : 0x%X] \n", start_addr);
			break;
		case TYPE_NORM_HOVER_SENSE :
			printf("[Normalized Hover Sense : 0x%X] \n", start_addr);
			break;
		case TYPE_CALIB_HOVER_FORCE :
			printf("[Calibrated Hover Force : 0x%X] \n", start_addr);
			break;
		case TYPE_CALIB_HOVER_SENSE :
			printf("[Calibrated Hover Sense : 0x%X] \n", start_addr);
			break;
	}

	fts_print_frame(pFrame);

ErrorExit :
	return rc;
}

int main (int argc, char **argv)
{
	unsigned char pArg[256] = {0};
	int rc = 0;

	if(argc == 1)
	{
		print_usage();
		return 0;
	}

	i2c_fd = i2c_open();
	if(i2c_fd == 0)
		goto ErrorExit;

	if(argc == 2)
	{
		strcpy(pArg, argv[1]);
		
		str_tolower(pArg);

		if(strcmp(pArg, "raw") == 0)
			fts_read_frame(TYPE_RAW_TOUCH);
		else if(strcmp(pArg, "filter") == 0)
			fts_read_frame(TYPE_FILTER_TOUCH);
		else if(strcmp(pArg, "str") == 0 || strcmp(pArg, "strength") == 0 || strcmp(pArg, "delta") == 0 || strcmp(pArg, "norm") == 0 )
			fts_read_frame(TYPE_NORM_TOUCH);
		else if(strcmp(pArg, "base") == 0 || strcmp(pArg, "baseline") == 0 || strcmp(pArg, "calib") == 0)
			fts_read_frame(TYPE_CALIB_TOUCH);
		else if(strcmp(pArg, "rawself") == 0 || strcmp(pArg, "rs") == 0)
			fts_read_frame(TYPE_RAW_SELF);
		else if(strcmp(pArg, "rawhoverforce") == 0 || strcmp(pArg, "rhf") == 0)
			fts_read_frame(TYPE_RAW_HOVER_FORCE);
		else if(strcmp(pArg, "rawhoversense") == 0 || strcmp(pArg, "rhs") == 0)
			fts_read_frame(TYPE_RAW_HOVER_SENSE);
		else if(strcmp(pArg, "filterhoverforce") == 0 || strcmp(pArg, "fhf") == 0)
			fts_read_frame(TYPE_FILTER_HOVER_FORCE);
		else if(strcmp(pArg, "filterhoversense") == 0 || strcmp(pArg, "fhs") == 0)
			fts_read_frame(TYPE_FILTER_HOVER_SENSE);
		else if(strcmp(pArg, "normhoverforce") == 0 || strcmp(pArg, "nhf") == 0)
			fts_read_frame(TYPE_NORM_HOVER_FORCE);
		else if(strcmp(pArg, "normhoversense") == 0 || strcmp(pArg, "nhs") == 0)
			fts_read_frame(TYPE_NORM_HOVER_SENSE);
		else if(strcmp(pArg, "calibhoverforce") == 0 || strcmp(pArg, "chf") == 0)
			fts_read_frame(TYPE_CALIB_HOVER_FORCE);
		else if(strcmp(pArg, "calibhoversense") == 0 || strcmp(pArg, "chs") == 0)
			fts_read_frame(TYPE_CALIB_HOVER_SENSE);
		else if(strcmp(pArg, "tune") == 0 || strcmp(pArg, "atune") == 0 || strcmp(pArg, "autotune") == 0)
			fts_auto_tune();
		else if(strcmp(pArg, "reset") == 0)
			fts_system_reset();

		goto ErrorExit;
	}

	if(argc == 3)
	{
		strcpy(pArg, argv[1]);
		
		str_tolower(pArg);
		
		if(strcmp(pArg, "main") == 0 || strcmp(pArg, "m") == 0)
		{
			fw_download(argv[2], FW_MAIN_BLOCK, 1);
			goto ErrorExit;
		}
		else if(strcmp(pArg, "info") == 0 || strcmp(pArg, "i") == 0)
		{
			fw_download(argv[2], FW_INFO_BLOCK, 1);
			goto ErrorExit;
		}
	}

	if(argc >= 2)
	{
		if(argv[1][0] == 'r' || argv[1][0] == 'R')
		{
			rc = i2c_read(argc, argv);
		}
		else if(argv[1][0] == 'w' || argv[1][0] == 'W')
		{
			rc = i2c_write(argc, argv);
		}
		else
		{
			print_usage();
		}
	}

ErrorExit :
	i2c_close(i2c_fd);
	
	return rc;
}

