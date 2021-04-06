/**
 * @file RTClient.h
 * @brief Main Client for Dual-Arm System
 * @date 2018-11-28.
 * @author Junho Park
 * @version 1.0.0
 */

#ifndef RTCLIENT_H_
#define RTCLIENT_H_

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <csignal>
#include <cstring>		// string function definitions
#include <cerrno>		// Error number definitions
#include <ctime>		// time calls
#include <cmath>

#include <unistd.h>
#include <fcntl.h>		// File control definitions
#include <termios.h>	// POSIX terminal control definitions
#include <sys/mman.h>
#include <sys/ioctl.h>

//-xenomai-///////////////////////////////////////////////////////////////
#include <native/task.h>
#include <native/timer.h>
#include <native/mutex.h>
#include <native/queue.h>
#include <rtdk.h>		//The rdtk real-time printing library
#include <alchemy/task.h>
#include <rtdm/can.h>
/****************************************************************************/

#include "EcatSystem/Ecat_Master.h"
#include "EcatSystem/Ecat_Elmo.h"
#include "EcatSystem/Ecat_KistFinger.h"
#include "EcatSystem/Ecat_KistSensor.h"
#include "Control/Controller.h"
#include "Control/Motion.h"
#include "Control/KistHand.h"
#include "KDL/SerialManipulator.h"
#include "Network/SocketHandler.h"

#define WAKEUP_TIME				(5)				/**<Initial waiting time*/
#define NSEC_PER_SEC            (1000000000L)	/**<Expression of second in nano second*/

#define _DEBUG_ 			/**<Debug Print Parameter*/
#define _ECAT_ON_ 			/**<EtherCAT device enable Parameter*/
#define _USE_DC_MODE_		/**<EtherCAT Distributed Clock mode enable Parameter*/

typedef unsigned int UINT32;	/**<typedef uint32_t*/
typedef int64_t		INT64;		/**<typedef uint64_t*/
typedef int32_t 	INT32;		/**<typedef int32_t*/
typedef int16_t 	INT16;		/**<typedef int16_t*/
typedef uint16_t 	UINT16;		/**<typedef uint16_t*/
typedef uint8_t 	UINT8;		/**<typedef uint8_t*/
typedef int8_t 		INT8;   	/**<typedef int8_t*/


// Cycle time in nanosecond
unsigned long cycle_ns = 1000000;  	/**< 1 ms, Initial Value */


#endif
