/**
 * @file EcatDataSocket.h
 * @date 2015-03-03
 * @author Thach Do
 */
 
#pragma once
#include "NRMKDataSocket.h"

enum
{
	NUM_JOINT = 14,
	NUM_WAIST = 2,
	NUM_LEFTARM = 6,
	NUM_RIGHTARM = 6,
	NUM_DATASCOPE_SIGNALS = 6 * NUM_JOINT,

};

typedef uint64_t 		UINT64;
typedef int64_t 		INT64;
typedef unsigned int 	UINT32;
typedef int32_t 		INT32;
typedef int16_t 		INT16;
typedef uint16_t 		UINT16;
typedef uint8_t 		UINT8;
typedef int8_t 			INT8;
typedef NRMKHelper::NRMKDataSocket<NUM_DATASCOPE_SIGNALS> _ECAT_DATA_SOCKET;

/**
 * @brief TCP/IP data socket for NRMK data scope
 * @version 1.2.0
 */
class EcatDataSocket : public _ECAT_DATA_SOCKET
{
public:
	enum
	{
		NUM_DATA = _ECAT_DATA_SOCKET::NUM_DATA,
	};

	/**
	 * @brief EcatDataSocket constructor
	 */
	inline EcatDataSocket() : _ECAT_DATA_SOCKET(_dataChannelArray, _sizeDataChannelArray)
	{
		_base_freq = 3.0;
		_tick = 0;
		_delT = 0.001;
	}

	/**
	 * @brief set up the data period
	 * @param[in] delT time in second
	 */
	void setPeriod(float delT)
	{
		_delT = delT;
	}

	void updateControlData( double *_ActualPos, double *_TargetPos )
	{
		for(int i=0; i < NUM_JOINT; i++ )
		{
			ActualPos[i] = (float)_ActualPos[i];
			TargetPos[i] = (float)_TargetPos[i];
		}
	}

	void updateControlData( double *_ActualPos, double *_TargetPos, double *_ActualVel, double *_TargetVel, short *_ActualToq, short *_TargetToq )
	{
		for(int i=0; i < NUM_JOINT; i++ )
		{
			ActualPos[i] = (float)_ActualPos[i];
			TargetPos[i] = (float)_TargetPos[i];
			ActualVel[i] = (float)_ActualVel[i];
			TargetVel[i] = (float)_TargetVel[i];
			ActualToq[i] = (float)_ActualToq[i];
			TargetToq[i] = (float)_TargetToq[i];
		}
	}

private:

	virtual int _updatePacketData(float * const data)
	{
        // Prepare datas
		for (int k = 0; k < NUM_JOINT; k++)
		{
			data[6*k] = TargetPos[k];
			data[6*k+1] = ActualPos[k];
			data[6*k+2] = TargetVel[k];
			data[6*k+3] = ActualVel[k];
			data[6*k+4] = TargetToq[k];
			data[6*k+5] = ActualToq[k];
		}

		_tick++;

		// Always return the number of bytes of the updated data packet
		return NUM_DATA;
	}



private:

	static const unsigned int _dataChannelArray[];
	static const unsigned int _sizeDataChannelArray;

	float _base_freq;
	float _delT;
	long _tick;

	float ActualPos[NUM_JOINT];
	float TargetPos[NUM_JOINT];

	float ActualVel[NUM_JOINT];
	float TargetVel[NUM_JOINT];

	float ActualToq[NUM_JOINT];
	float TargetToq[NUM_JOINT];
};
