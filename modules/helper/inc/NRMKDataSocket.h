/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "Poco/Mutex.h"

#include "NRMKSocketBase.h"

namespace NRMKHelper
{

template <int _NUM_DATA>
class NRMKDataSocket : public NRMKSocketBase
{
public:
	enum
	{
		NUM_DATA = _NUM_DATA,
		// FIXME @ 20131117
		//NUM_CHANNELS = _NUM_CHANNELS,
		//MAX_SIZE_CHANNEL_FORMAT = (3 + NUM_DATA) * NUM_CHANNELS,
	};

public:
	inline NRMKDataSocket(unsigned int const * const dataChannel, unsigned int size_data_channel, unsigned int ch_row = -1, unsigned int ch_col = -1)
		: NRMKSocketBase()
		, _hasConnection(false), _requestKey(0), _event(true), _speedEnhancing(true)
	{
		_setDataChannel(dataChannel, size_data_channel, ch_row, ch_col);
		_initBuf();
	}

	//NRMKDataSocket(LPCTSTR sIP, LPCTSTR sPort)

	//~NRMKDataSocket();
	
	inline virtual bool hasConnection() const { return _hasConnection; }

	inline virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
	{
		_requestKey = lpBuffer[0];
		//printf("RequestKey: %i\n", RequestKey);
		switch (_requestKey)
		{
		case 'O':
			if (!_hasConnection)
			{
				sendDataChannelArray(); //send index array
				Sleep(100); //should wait for finishing sendBodyIndexArray

				// temporary
				//_hasConnection = true;

				// FIXME @ 20131117
				//delete[] _dataChannel;
				_hasConnection = true;
				_event.set();
			}

			break;

		case 'R':
			if (_hasConnection)
			{
				_event.set();
				_requestKey = 0;
				// now ready to run...
			}

			break;

		/// TO DO: Define custom keyboard handler
		}
	}

	inline virtual void OnEvent(UINT uEvent, LPVOID lpvData)
	{
		switch( uEvent )
		{
		case EVT_CONSUCCESS:
			std::cout << "One client has connected to the Data Server..." << std::endl;
			break;

		case EVT_CONDROP:
			std::cout << "Data Server Connection abandoned " << std::endl;

			// Added@20140822
			_hasConnection = false;
			_requestKey = 0;

			StopComm();

			if (IsServer())
				waitForConnection(0);

			break;

		default:
			NRMKSocketBase::OnEvent(uEvent, lpvData);
			break;
		}
	}

	//-----Simulation data communication-----------------
	int sendDataChannelArray()
	{
		if (!IsOpen())
			return -1;

		//unsigned char buf[MAX_SIZE_CHANNEL_FORMAT*sizeof(unsigned int) + 100];
		// FIXME @ 20131117
		//unsigned char buf[MAX_SIZE_CHANNEL_FORMAT*sizeof(unsigned int) + 23];
		unsigned char * buf = new unsigned char[_size_channel_format*sizeof(unsigned int) + 23];
		int numData = NUM_DATA;

		memcpy(buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	// 2 bytes, NRMK_SOCKET_START_TOKEN
		buf[2] = (unsigned char) NRMK_SOCKET_PACKET_INDEX;						// 1 byte, Packet type
		memcpy(buf + 3, &numData, sizeof(unsigned int));					// 4 bytes, Number of integer data (bytes = NumOfData*4)
		memcpy(buf + 7, &_channel[0], sizeof(int));					// 4 bytes, Number of integer data (bytes = NumOfData*4)
		memcpy(buf + 11, &_channel[1], sizeof(int));					// 4 bytes, Number of integer data (bytes = NumOfData*4)
		// FIXME @20131015 Data Scope should be modified to packet format change
		memcpy(buf + 15, &_size_channel_format, sizeof(unsigned int));
		memcpy(buf + 19, _dataChannel, _size_channel_format*sizeof(unsigned int));		// nLen bytes, data
		memcpy(buf + 19 + _size_channel_format*sizeof(unsigned int), NRMK_SOCKET_END_TOKEN, NRMK_SOCKET_TOKEN_SIZE);		// 2 bytes, NRMK_SOCKET_END_TOKEN
		
		memcpy(buf + 21 + _size_channel_format*sizeof(unsigned int), "\r\n", 2);

		WriteComm(buf, 23 + _size_channel_format*sizeof(unsigned int), INFINITE);

		// FIXME @ 20131117
		delete[] buf;

		return 0;
	}

	inline void update(float time, int size = 0, float const * const data = NULL)
	{
		memcpy(_buf + PACKET_DATA_OFFSET, &time, sizeof(float));
		
		int offset = sizeof(float);
		offset += _updatePacketData((float * const) (_buf + PACKET_DATA_OFFSET + offset));

		if (size > 0)
		{
			//unsigned int num_bytes = size*sizeof(float);
			// FIXME @20131015 address comparison works ?
			if (data && (unsigned char *) data != _buf + PACKET_DATA_OFFSET + offset)
				memcpy(_buf + PACKET_DATA_OFFSET + offset, data, size*sizeof(float));
			
			//offset += num_bytes;
		}

		if (hasConnection())
		{
			

			if (!_speedEnhancing)	
			{
				
				try { _event.wait(1000); }
				catch (Poco::TimeoutException ex){}
			}
			_sendData(); 
		}
	}

	void enhanceSpeed(bool enable) { _speedEnhancing = enable; }

	unsigned char * const databuf() { return _buf + PACKET_DATA_OFFSET + sizeof(float); } // sizeof(float) for time

private:
	virtual int _updatePacketData(float * const data) 
	{
		return 0;
	}

	// FIXED @20131009: the first argument curTime is of type float instead of double. 
	//  DataScope should be modified accordingly. 
	inline int _sendData(/*float curTime, float const * const data*/) // in float
	{
		if (!IsOpen() || !_hasConnection)
			return -1;
		
// 		memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, START_token
// 		_buf[2] = (unsigned int) NRMK_SOCKET_PACKET_DATA;				//1 byte, Packet type
// 
// 		// FIXME @20131015 
// 		//memcpy(_buf + 3, &curTime, sizeof(float));				//4 bytes (float), current time (absolute)
// 		// FIXME @20131015 Data Scope should be modified to packet format change
// 		//memcpy(_buf + 7, &_numData, sizeof(int));					//4 bytes (uint) , Number of integer data (bytes = NumOfData*4)
// 		//memcpy(_buf + 7, data, DATA_PACKET_DATA_LENGTH);					//nLen bytes, data
// 		memcpy(_buf + PACKET_DATA_OFFSET + sizeof(float) + DATA_PACKET_DATA_LENGTH, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token
// 
// 		memcpy(_buf + PACKET_DATA_OFFSET + sizeof(float) + DATA_PACKET_DATA_LENGTH + 2, "\r\n", 2);

		WriteComm(_buf, NUM_BYTES_DATA_PACKET_FILLIN + NUM_BYTES_DATA_PACKET, INFINITE);

		return NUM_BYTES_DATA_PACKET + NUM_BYTES_DATA_PACKET_FILLIN;
	}

	inline void _initBuf() 
	{
		memcpy(_buf, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, START_token
		_buf[2] = (unsigned char) NRMK_SOCKET_PACKET_DATA;				//1 byte, Packet type

		// FIXME @20131015 
		//memcpy(_buf + 3, &curTime, sizeof(float));				//4 bytes (float), current time (absolute)
		// FIXME @20131015 Data Scope should be modified to packet format change
		//memcpy(_buf + 7, &_numData, sizeof(int));					//4 bytes (uint) , Number of integer data (bytes = NumOfData*4)
		//memcpy(_buf + 7, data, DATA_PACKET_DATA_LENGTH);					//nLen bytes, data
		
		memcpy(_buf + PACKET_DATA_OFFSET + sizeof(float) + NUM_BYTES_DATA_PACKET, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token
		memcpy(_buf + PACKET_DATA_OFFSET + sizeof(float) + NUM_BYTES_DATA_PACKET + 2, "\r\n", 2);
	}

	inline void _setDataChannel(unsigned int const * const channel_format, unsigned int size_channel_format, unsigned int ch_row, unsigned int ch_col)
	{  
		// FIXME @ 20131117
		_dataChannel = new unsigned int[size_channel_format];

		memcpy(_dataChannel, channel_format, sizeof(unsigned int)*size_channel_format);
		_size_channel_format = size_channel_format;
		
		_channel[0] = ch_row;
		_channel[1] = ch_col;
	}

private:
	enum
	{
		PACKET_DATA_OFFSET = 3,
		NUM_BYTES_DATA_PACKET = sizeof(float) * NUM_DATA,
		NUM_BYTES_DATA_PACKET_FILLIN = PACKET_DATA_OFFSET + sizeof(float) + 4,
		//NUM_CHANNELS = CHANNEL_ROW * CHANNEL_COLUMN,
	};

private:
	//unsigned char _buf[DATA_PACKET_DATA_LENGTH + 100];
	unsigned char _buf[NUM_BYTES_DATA_PACKET + NUM_BYTES_DATA_PACKET_FILLIN];


	// FIXME @ 20131117
	//unsigned int _dataChannel[MAX_SIZE_CHANNEL_FORMAT];
	unsigned int * _dataChannel;
	unsigned int _size_channel_format;

	unsigned int _channel[2];

	// FIXED by THACHDO 20150717
	volatile bool _hasConnection; // moved from NRMKSocketBase
	unsigned char _requestKey;
	Poco::Event _event;
	bool _speedEnhancing;
	
	// Do not try to move this to the base class
	// Nor to delete volatile attribute. (Then release mode does not work properly).
};

} // namespace NRMKHelper