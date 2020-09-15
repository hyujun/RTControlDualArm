/* NRMKFoundation, Copyright 2016- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
 
#ifndef _NRMKSERCAN_TP_H_
#define _NRMKSERCAN_TP_H_

#include <stdint.h>
#include <string.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include <native/timer.h>
#include <rtdk.h>
#include <rtdm/rtserial.h>

#include "NRMKhw_tp.h"

typedef struct can_frame			CAN_FRAME;
#define	MAX_SEND_BUFFER_SIZE		512		///< Maximum Send Packet Size
#define	MAX_RECV_BUFFER_SIZE		512		///< Maximum Receive Packet Size

//Error definition
#define SERCAN_ERR_FREE		0x00
#define SERCAN_ERR_BUFFER	0x01
#define SERCAN_ERR_FRAME	0x02
#define SERCAN_ERR_CRC		0x03
#define SERCAN_ERR_READ		0x10

#define	sB1000000	"1M"
#define sB500000	"500K"
#define sB250000	"250K"
#define sB125000	"125K"
#define sB50000		"50K"
#define sB10000		"10K"
#define sB5000		"5K"


#ifdef __cplusplus
extern "C"{
#endif

	//bit rate
	int SERCAN_SetBitRate(int fd, char *strBitrate);
	int32_t SERCAN_GetBitRate(int fd, uint32_t timeOut /* in ms */);

	//normal driver
	int  SERCAN_open(void);
	int  SERCAN_write(int fd, CAN_FRAME TxFrame);
	int  SERCAN_read(int fd, CAN_FRAME *lpRxFrame);
	void print_CANFrame(CAN_FRAME Rx_Frame);
	//----------------------------------------

	//rt driver
	int  RTSERCAN_open(void);
	int  RTSERCAN_write(int fd, CAN_FRAME TxFrame);
	int  RTSERCAN_read(int fd, CAN_FRAME *lpRxFrame);
	void rt_print_CANFrame(CAN_FRAME Rx_Frame);

#ifdef __cplusplus
}
#endif




#endif //_NRMKSERCAN_TP_H_

