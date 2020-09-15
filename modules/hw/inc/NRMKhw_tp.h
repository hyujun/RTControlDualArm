/* NRMKFoundation, Copyright 2016- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */

#ifndef _NRMKHW_TP_H_
#define _NRMKHW_TP_H_

#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/io.h>			//IO access
#include <linux/serial.h>
#include <asm/ioctls.h>

#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <errno.h>

//******GPIO*************************
//pin mask
#define pin00	0x01
#define pin01	0x02
#define pin02	0x04
#define pin03	0x08
#define pin04	0x10
#define pin05	0x20
#define pin06	0x40
#define pin07	0x80
#define pin(i)	((uint8_t)(1<<i))
#define gpio_allpin	0xFF

typedef enum _TP_UART_CLK_SRC{
	MODE01_8462MHz=0x00,		//01.8462 MHz (24 MHz / 13) 	: Max baud=115.2K (default)
	MODE02_0000MHz=0x01,		//02.0000 MHz (24 MHz / 12) 	: Max baud=125.0K
	MODE24_0000MHz=0x02,		//24.0000 MHz (24 MHz / 1) 		: Max baud=1.5M
	MODE14_1690MHz=0x03			//14.7690 MHz (24 MHz / 1.625) 	: Max baud=921.6k
} TP_UART_CLK_SRC;

typedef enum {
	GPIO_GDIR_INPUT  = 0,
	GPIO_GDIR_OUTPUT = 1,
} TP_GPIO_GDIR;

typedef enum {
	GPIO_LOW_LEVEL  = 0,
	GPIO_HIGH_LEVEL = 1,
} TP_GPIO_LEVEL;

typedef enum _TP_PORT{
	port0=0,
	port1=1
} TP_PORT;


//******SERIAL PORT******************
//! @brief Available serial ports on board
#define SERIALPORT1		"/dev/ttyS0"
#define SERIALPORT4		"/dev/ttyS3"
#define SERIALPORT5		"/dev/ttyS4"
#define SERIALPORT6		"/dev/ttyS5"

//! @brief Available serial ports on STEP-PC2
#define RS485PORT		"/dev/ttyS0"	//RS485 port
#define COM1			"/dev/ttyS3"	//COM1 port
#define COM2			"/dev/ttyS4"	//COM2 port

#define RS485_BAUD_LIMIT	1500000		//maximum RS485 baudrate
#define RS232_BAUD_LIMIT	230400		//maximum baudrate

static int rate_to_constant(int baudrate) {
#define B(x) case x: return B##x
        switch(baudrate) {
        B(50);     B(75);     B(110);    B(134);    B(150);
        B(200);    B(300);    B(600);    B(1200);   B(1800);
        B(2400);   B(4800);   B(9600);   B(19200);  B(38400);
        B(57600);  B(115200); B(230400); B(460800); B(500000);
        B(576000); B(921600); B(1000000);B(1152000);B(1500000);
    default: return 0;
    }
#undef B
}

							
#ifdef __cplusplus
extern "C"{
#endif

	//GPIO
	int tp_gpio_init(void);
	void tp_gpio_setdirout(TP_PORT port, uint8_t regcode);
	void tp_gpio_setdirin(TP_PORT port, uint8_t regcode);
	void tp_gpio_set_dir(TP_PORT port, uint8_t regcode);

	uint8_t tp_readport(TP_PORT port);
	void tp_writeport(TP_PORT port, TP_PORT portval);

	void tp_onpin(TP_PORT port, uint8_t  pincode);
	void tp_offpin(TP_PORT port, uint8_t  pincode);
	void tp_togglepin(TP_PORT port, uint8_t  pincode);
	
	//STeP-TP Serial
	int NRMKkbhit(void);
	int tp_open_serial_port(const char* portname, unsigned int baud);
	int tp_configure_serial_port(int fd, int baud);
	
#ifdef __cplusplus
}
#endif


#endif
