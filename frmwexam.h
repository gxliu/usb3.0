#ifndef _INCLUDED_FRMWEXAM_H_
#define _INCLUDED_FRMWEXAM_H_

#include "cyu3types.h"
#include "cyu3usbconst.h"
#include "cyu3externcstart.h"

#define BUF_COUNT			(2)                       /* channel buffer count */
#define DMA_TX_SIZE			(0)                       /* DMA transfer size is set to infinite */
#define THREAD_STACK		(0x8000)                  /* application thread stack size */
#define THREAD_PRIORITY		(8)                       /* application thread priority */
#define BULK_BURST			(10)					/* Burst size for bulk EPs (SS only) */
#define SIZE_MULT			(2)						/* dma buffer size multiplier */

#define EP_PROD_1               0x01    /* EP 1 OUT */
#define EP_PROD_2               0x02    /* EP 2 OUT */
#define EP_PROD_3               0x03    /* EP 3 OUT */
#define EP_PROD_4               0x04    /* EP 4 OUT */
#define EP_PROD_5               0x05    /* EP 5 OUT */
#define EP_CONS_1               0x81    /* EP 1 IN */
#define EP_CONS_2               0x82    /* EP 2 IN */
#define EP_CONS_3               0x83    /* EP 3 IN */
#define EP_CONS_4               0x84    /* EP 4 IN */
#define EP_CONS_5               0x85    /* EP 5 IN */

#define PROD_SOCKET_1        CY_U3P_UIB_SOCKET_PROD_1    /* Socket 1 is producer */
#define PROD_SOCKET_2        CY_U3P_UIB_SOCKET_PROD_2    /* Socket 2 is producer */
#define PROD_SOCKET_3        CY_U3P_UIB_SOCKET_PROD_3    /* Socket 3 is producer */
#define PROD_SOCKET_4        CY_U3P_UIB_SOCKET_PROD_4    /* Socket 4 is producer */
#define PROD_SOCKET_5        CY_U3P_UIB_SOCKET_PROD_5    /* Socket 5 is producer */
#define CONS_SOCKET_1        CY_U3P_UIB_SOCKET_CONS_1    /* Socket 1 is consumer */
#define CONS_SOCKET_2        CY_U3P_UIB_SOCKET_CONS_2    /* Socket 2 is consumer */
#define CONS_SOCKET_3        CY_U3P_UIB_SOCKET_CONS_3    /* Socket 3 is consumer */
#define CONS_SOCKET_4        CY_U3P_UIB_SOCKET_CONS_4    /* Socket 4 is consumer */
#define CONS_SOCKET_5        CY_U3P_UIB_SOCKET_CONS_5    /* Socket 5 is consumer */

#define BIT_RES				0	//bit RES in FPGA command
#define BIT_ON				1
#define BIT_EN				2
#define BIT_OUT				3
#define BIT_INT				7
#define BIT_DIR				7
#define BIT_STREAM			2
#define BIT_TYPE			5
#define BIT_WR				7
#define BIT_CTYPE			3	//bit shift Type in host command
#define BIT_CPORT			0	//Port in HARDWARE commands
#define BIT_CCODE			0	//Code in SYSTEM commands
#define BIT_CTCOND			0	//Condition in TETRAD commands
#define BIT_CNAME			0	//Name in STREAM commands
#define BIT_CSCOND			5	//Condition in STREAM commands
#define BIT_CWR				2	//Write in HARDWARE commands
#define BYTE_HVAL			4
#define BYTE_PARAM			8
#define BYTE_COMM			9
#define BYTE_TRANSACTION	10
#define BYTE_DEBUG			12
#define BYTE_VERSION		14
#define BYTE_SIGN			15

#define FLAG_APPSTART	0
#define FLAG_APPSTOP	1
#define FLAG_GETI2C		2
#define FLAG_REQI2C		3
#define FLAG_CLEAREP	4
#define FLAG_WAITCOMM	5
#define FLAG_GETCOMM	6
#define FLAG_COMMREADY	7
#define FLAG_WAITREPLY	8
#define FLAG_GETREPLY	9
#define FLAG_REPLYREADY	10
#define FLAG_FPGARESET	11
#define FLAG_DATAREADY	12
#define FLAG_MANIN		13
#define FLAG_MANOUT		14

#define SIGN_IN				0xA5
#define SIGN_OUT			0xB5
#define COMM_VER			0

/* Extern definitions for the USB Descriptors */
extern const uint8_t USB20DeviceDscr[];
extern const uint8_t USB30DeviceDscr[];
extern const uint8_t USBDeviceQualDscr[];
extern const uint8_t USBFSConfigDscr[];
extern const uint8_t USBHSConfigDscr[];
extern const uint8_t USBBOSDscr[];
extern const uint8_t USBSSConfigDscr[];
extern const uint8_t USBStringLangIDDscr[];
extern const uint8_t USBManufactureDscr[];
extern const uint8_t USBProductDscr[];

#include "cyu3externcend.h"

#endif /* _INCLUDED_FRMWEXAM_H_ */

/*[]*/
