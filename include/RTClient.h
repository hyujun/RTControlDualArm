/**
 * @file RTClient.h
 * @brief Main Client for Dual-Arm System
 * @date 2018-11-28.
 * @author Junho Park
 * @version 1.0.0
 */

#pragma once

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <csignal>
#include <cstring>		// string function definitions
#include <cerrno>		// Error number definitions
#include <ctime>		// time calls
#include <cmath>

#include <termios.h>
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

#include "ecat/Ecat_Master.h"
#include "ecat/Ecat_Elmo.h"
#include "ecat/Ecat_KistFinger.h"
#include "ecat/Ecat_KistSensor.h"
#include "control/Controller.h"
#include "control/Motion.h"
#include "control/KistHand.h"
#include "kdl/SerialManipulator.h"
#include "net/SocketHandler.h"

inline constexpr int  WAKEUP_TIME  = 5;               // Initial waiting time (s)
inline constexpr long NSEC_PER_SEC = 1'000'000'000L;  // 1 second in nanoseconds

//#define _DEBUG_ 			/**<Debug Print Parameter*/
#define _ECAT_ON_ 			/**<EtherCAT device enable Parameter*/
#define _TCPIP_ON_
#define _PRINT_ON_
//#define _PLOT_ON_
//#define _KEYBOARD_ON_

// Use <cstdint> standard types directly: uint32_t, int64_t, int32_t, int16_t, uint16_t, uint8_t, int8_t
// Legacy aliases kept for backward compatibility
using UINT32 = uint32_t;
using INT64  = int64_t;
using INT32  = int32_t;
using INT16  = int16_t;
using UINT16 = uint16_t;
using UINT8  = uint8_t;
using INT8   = int8_t;

// Cycle time in nanosecond (C++17 inline variable)
inline unsigned long cycle_ns = 1000e3;  	/**< 1 ms, Initial Value */
