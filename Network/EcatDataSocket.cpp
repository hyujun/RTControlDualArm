/*
 * EcatDataSocket.cpp
 *
 *  Created on: Mar 3, 2015
 *      Author: Thach Do
 */

#include "EcatDataSocket.h"

/// TO DO: Fill in the number of data and the data channel index array
const unsigned int EcatDataSocket::_dataChannelArray[]
// The following code generates two (or three) channels charts (all of time chart style),
// where each chart plots the signals (as many as the joint dof).
//
// This is the example to plot the joint angles in the first channel, the joint velocities in the second channel,
// the joint torque (in controlled simulation) in the third channel for a seven dof manipulator.
	= {
		NRMK_SCOKET_PACKET_DATA_CH(0), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_WAIST, 0, 1, 6, 7,

		NRMK_SCOKET_PACKET_DATA_CH(1), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_RIGHTARM, 12, 13, 18, 19, 24, 25, 30, 31, 36, 37, 42, 43,

		NRMK_SCOKET_PACKET_DATA_CH(2), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_LEFTARM, 48, 49, 54, 55, 60, 61, 66, 67, 72, 73, 78, 79,

		NRMK_SCOKET_PACKET_DATA_CH(3), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_WAIST, 2, 3, 8, 9,

		NRMK_SCOKET_PACKET_DATA_CH(4), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_RIGHTARM, 14, 15, 20, 21, 26, 27, 32, 33, 38, 39, 44, 45,

		NRMK_SCOKET_PACKET_DATA_CH(5), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_LEFTARM, 50, 51, 56, 57, 62, 63, 68, 69, 74, 75, 80, 81,

		NRMK_SCOKET_PACKET_DATA_CH(6), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_WAIST, 4, 5, 10, 11,

		NRMK_SCOKET_PACKET_DATA_CH(7), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_RIGHTARM, 16, 17, 22, 23, 28, 29, 34, 35, 40, 41, 46, 47,

		NRMK_SCOKET_PACKET_DATA_CH(8), NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART, 2*NUM_LEFTARM, 52, 53, 58, 59, 64, 65, 70, 71, 76, 77, 82, 83,
	};

// TO DO: Set the actual size of the data channel format array defined above
const unsigned int EcatDataSocket::_sizeDataChannelArray = sizeof(_dataChannelArray)/sizeof(unsigned int);



