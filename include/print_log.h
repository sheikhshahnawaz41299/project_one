/*
 * =====================================================================================
 *
 *       Filename:  print_log.h
 *
 *    Description:  this file defined some debug function.
 *
 *        Version:  1.0
 *        Created:  01/19/2016 10:02:17 PM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author: 	chenpeng 
 *        Company:  vivo
 *
 * =====================================================================================
 */

#ifndef __PRINT_LOG_H__
#define __PRINT_LOG_H__

#define PRINTF_INF(fmt,param...) printf("INF[%s][%s][%d]"fmt, __FILE__, __func__, __LINE__, ##param);
#define PRINTF_ERR(fmt,param...) printf("ERR[%s][%s][%d]"fmt, __FILE__, __func__, __LINE__, ##param);

/* if 1,PRINTF_DBG will print debug infomation */
#if 1
#define PRINTF_DBG(fmt,param...) printf("DBG[%s][%s][%d]"fmt, __FILE__, __func__, __LINE__, ##param);
#else
#define PRINTF_DBG(fmt,param...) 
#endif

#endif
