/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "NRMKSocketBase.h"
#include "NRMKSyncObject.h"

#ifdef __XENO__
#ifdef __ARM__
#ifdef _BBB_
#include "NRMKgpio.h"
#define sigport	1
#define sigpin1	pin28
#define sigpin2	pin18
#endif
#define gpio_init()
#define gpio_setdirout(x, y)
#define onpin(x, y)
#define offpin(x, y)
#else
#define gpio_init()
#define gpio_setdirout(x, y)
#define onpin(x, y)
#define offpin(x, y)
#endif
#endif

namespace NRMKHelper
{

template<int _NUM_BYTES_READ_BUF, int _NUM_BYTES_WRITE_BUF, int _NUM_BYTES_MONITOR_BUF = 0, 
		 int _NUM_FLOATS_CONTROL_DATA = 0, int _NUM_BYTES_CONTROL_GRAPHICS = 0>
class NRMKControlSocket : public NRMKSocketBase
{
public:
	enum
	{
		NUM_BYTES_READ_BUF = _NUM_BYTES_READ_BUF, 
		NUM_BYTES_WRITE_BUF = _NUM_BYTES_WRITE_BUF, 
		NUM_BYTES_MONITOR_BUF = _NUM_BYTES_MONITOR_BUF,
		NUM_FLOATS_CONTROL_DATA = _NUM_FLOATS_CONTROL_DATA, 
		NUM_BYTES_CONTROL_DATA = NUM_FLOATS_CONTROL_DATA*sizeof(float), 
		NUM_BYTES_CONTROL_GRAPHICS = _NUM_BYTES_CONTROL_GRAPHICS, 
	};

public:
	inline NRMKControlSocket()
		: NRMKSocketBase()
		, _databuf(_wrbuf + 100), _graphicsbuf(_databuf + NUM_BYTES_CONTROL_DATA), _num_bytes_graphics_data(0)
		//, _wrbufReadyEvent("WriteBufReadyEvent"), _rdbufReadyEvent("ReadBufReadyEvent")
		, _hasConnection(false)
		, _requestKey(0)
		, _state(0x00)
	{
	}

#ifdef __XENO__
	inline virtual void signal_on(BYTE ch)
	{
		switch (ch)
		{
		case 0:
			onpin(sigport, sigpin1);
			break;

		case 1:
			onpin(sigport, sigpin2);
			break;

		default:
			break;
		}
	}

	inline virtual void signal_off(BYTE ch)
	{
		switch (ch)
		{
		case 0:
			offpin(sigport, sigpin1);
			break;

		case 1:
			offpin(sigport, sigpin2);
			break;

		default:
			break;
		}
	}
#endif

	inline BYTE getState() const
	{
		return _state;
	}

	inline void setState(BYTE newState)
	{
		_state = newState;
	}

	//NRMKControlSocket(LPCTSTR sIP, LPCTSTR sPort)

	//~NRMKControlSocket();
	
	inline virtual bool hasConnection() const { return _hasConnection; }
	inline void setConnected() 
	{ 
		_hasConnection = true; 
		// to initiate the connection
		//sendKey('0');
		_requestKey = 'O';
		WriteComm((LPBYTE) &_requestKey, 1, INFINITE);
	}

	inline virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount)
	{
		//LockList();

		if (!_hasConnection)
		{
			_requestKey = lpBuffer[0];
			//printf("RequestKey: %i\n", RequestKey);
			switch (_requestKey)
			{
			case 'O':
				if (!_hasConnection)
				{
					_hasConnection = true;
					_requestKey = 0;
				}

				break;
			}

			return;
		}
		
		//unsigned int PacketNum = 0;	// FIXED by THACHDO 20170804
		unsigned int curPos = 0;
		BYTE Token[2], PacketType;		

		// process
		_state = lpBuffer[curPos++]; //second byte is state flag

		memcpy(Token, lpBuffer + curPos, 2); 
		curPos += 2;

		if (memcmp(Token, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE) != 0) 
			return;

		memcpy(&PacketType, lpBuffer + curPos, 1); 
		curPos++;

		assert (dwCount > curPos);
			
		// FIXME @20131014
		dwCount -= curPos;

		if (IsServer())
		{ 
			//this is server, receive data from client (controller)
			// FIXME @20131014
			//assert(PacketType == NRMK_SOCKET_PACKET_ACTUATOR);
			if (_state & NRMK_SOCKET_PACKET_MASK_ACTUATOR)
			{
				memcpy(_wrbuf, lpBuffer + curPos, NUM_BYTES_WRITE_BUF);
				curPos += NUM_BYTES_WRITE_BUF;

				// FIXME @20131012
// 					if (!SetEvent(_wrbufReadyEvent)) 
// 						printf("SetEvent failed (%d)\n", GetLastError());

				_wrbufReadyEvent.set();

				dwCount -= NUM_BYTES_WRITE_BUF;
			}

			if (_state & NRMK_SOCKET_PACKET_MASK_CONTROL_DATA)
			{
				memcpy(_databuf, lpBuffer + curPos, NUM_BYTES_CONTROL_DATA);
				curPos += NUM_BYTES_CONTROL_DATA;

				dwCount -= NUM_BYTES_CONTROL_DATA;
			}

			if (_state & NRMK_SOCKET_PACKET_MASK_CONTROL_GRAPHICS)
			{
				// Graphics data can have varying length , so ...
				memcpy(&_num_bytes_graphics_data, lpBuffer + curPos, sizeof(unsigned int));
				curPos += sizeof(unsigned int);
				memcpy(&_num_graphics_objects, lpBuffer + curPos, sizeof(unsigned int));
				curPos += sizeof(unsigned int);

				//memcpy(_graphicsbuf, lpBuffer + curPos, NUM_BYTES_CONTROL_GRAPHICS);
				// FIXME @20131025
				_num_bytes_graphics_data -= 2*sizeof(unsigned int);
				memcpy(_graphicsbuf, lpBuffer + curPos, _num_bytes_graphics_data);
				//curPos += NUM_BYTES_CONTROL_GRAPHICS;
			}
		}
		else
		{ 
			//this is client, data come from server (sim,...)
			if (_state & NRMK_SOCKET_PACKET_MASK_SENSOR)
			{
				memcpy(_rdbuf, lpBuffer + curPos, NUM_BYTES_READ_BUF);
				curPos += NUM_BYTES_READ_BUF;

				// FIXME @20131012
// 					if (!SetEvent(_rdbufReadyEvent)) 
// 						printf("SetEvent failed (%d)\n", GetLastError());
				_rdbufReadyEvent.set();
			}

			if (_state & NRMK_SOCKET_PACKET_MASK_MONITOR)
			{
				memcpy(_monbuf, lpBuffer + curPos, NUM_BYTES_MONITOR_BUF);
			}
		}

		//_state=State;

		
		//UnlockList();
	}	

	inline virtual void OnEvent(UINT uEvent, LPVOID lpvData)
	{
		switch( uEvent )
		{
		case EVT_CONSUCCESS:
			std::cout << "One client has connected to the Control Server..." << std::endl;
			break;

		case EVT_CONDROP:
			std::cout << "Control Server Connection abandoned " << std::endl;

			// Added@20140822
			_hasConnection = false;
			_requestKey = 0;
			_state = 0x00;

			StopComm();

			if (IsServer())
				waitForConnection(0);
			else
				CloseComm();

			break;

		default:
			NRMKSocketBase::OnEvent(uEvent, lpvData);
			break;
		}
	}

	inline int sendReadData(unsigned char const * const data)
	{
		if (!IsOpen())
			return -1;

		_state |= NRMK_SOCKET_PACKET_MASK_SENSOR;

		return _sendData(NUM_BYTES_READ_BUF/*, NRMK_SOCKET_PACKET_SENSOR*/, data, _rdbuf); 
	}

	inline int sendWriteData(unsigned char const * const data) 
	{
		if (!IsOpen())
			return -1;

		_state |= NRMK_SOCKET_PACKET_MASK_ACTUATOR;
		return _sendData(NUM_BYTES_WRITE_BUF/*, NRMK_SOCKET_PACKET_ACTUATOR*/, data, _wrbuf); 
	}

	inline int sendMonitorData(unsigned char const * const data)
	{
		if (!IsOpen() || !_hasConnection)
			return -1;

		_state |= NRMK_SOCKET_PACKET_MASK_MONITOR;
		return _sendData(NUM_BYTES_MONITOR_BUF/*, NRMK_SOCKET_PACKET_MONITOR*/, data, _monbuf); 
	}

	// FIXME @20131014
	// These two functions should be called inside control client before calling sendWriteData()
	inline int setControlData(unsigned char const * const data)
	{
		if (!IsOpen())
			return -1;

		_state |= NRMK_SOCKET_PACKET_MASK_CONTROL_DATA;
		memcpy(_databuf, data, NUM_BYTES_CONTROL_DATA);

		return NUM_BYTES_CONTROL_DATA; 
	}

	// FIXME @20131014
	// note that control graphics data is of varying length
	inline int setGraphicsData(unsigned char const * const data, unsigned int num_bytes = NUM_BYTES_CONTROL_GRAPHICS)
	{
		if (!IsOpen())
			return -1;

		if (num_bytes == 0)
			return 0;

		_state |= NRMK_SOCKET_PACKET_MASK_CONTROL_GRAPHICS;
		_num_bytes_graphics_data = num_bytes;

		memcpy(_graphicsbuf, data, _num_bytes_graphics_data);

		return _num_bytes_graphics_data; 
	}

	inline int receiveReadData(unsigned char * const data)
	{
		if (!IsOpen())
			return -1;

		if (_state & NRMK_SOCKET_PACKET_MASK_SENSOR)
		{ 
			//read buffer is updated
			memcpy(data, _rdbuf, NUM_BYTES_READ_BUF);
			_state &= ~NRMK_SOCKET_PACKET_MASK_SENSOR; //reset rdbuf_ready bit, rdbuf_ready is expired

// 			if (!ResetEvent(_rdbufReadyEvent)) 
// 				printf("ResetEvent failed (%d)\n", GetLastError());
			_rdbufReadyEvent.reset();

			return NUM_BYTES_READ_BUF;
		}

		return 0;
	}

	inline int receiveWriteData(unsigned char * const data) 
	{
		if (!IsOpen())
			return -1;

		if (_state & NRMK_SOCKET_PACKET_MASK_ACTUATOR)
		{ 
			//write buffer is updated
			memcpy(data, _wrbuf, NUM_BYTES_WRITE_BUF);
			_state &= ~NRMK_SOCKET_PACKET_MASK_ACTUATOR; //reset rdbuf_ready bit, rdbuf_ready is expired

// 			if (!ResetEvent(_wrbufReadyEvent)) 
// 				printf("ResetEvent failed (%d)\n", GetLastError());
			_wrbufReadyEvent.reset();

			return NUM_BYTES_WRITE_BUF;
		}

		return 0;
	}

	inline int receiveMonitorData(unsigned char * const data)
	{
		if (!IsOpen())
			return -1;

		if (_state & NRMK_SOCKET_PACKET_MASK_MONITOR)
		{ 
			//monitor buffer is updated
			memcpy(data, _monbuf, NUM_BYTES_MONITOR_BUF);
			_state &= ~NRMK_SOCKET_PACKET_MASK_MONITOR; //reset monbuf_ready bit, rdbuf_ready is expired

			return NUM_BYTES_MONITOR_BUF;
		}

		return 0;
	}

	inline int retrieveControlData(unsigned char * const data)
	{
		if (!IsOpen())
			return -1;

		if (_state & NRMK_SOCKET_PACKET_MASK_CONTROL_DATA)
		{ 
			memcpy(data, _databuf, NUM_BYTES_CONTROL_DATA);
			_state &= ~NRMK_SOCKET_PACKET_MASK_CONTROL_DATA; //reset monbuf_ready bit, rdbuf_ready is expired

			return NUM_BYTES_CONTROL_DATA;
		}

		return 0;
	}

	inline int retrieveControlGraphics(unsigned char * const data)
	{
		if (!IsOpen())
			return -1;

		if (_state & NRMK_SOCKET_PACKET_MASK_CONTROL_GRAPHICS)
		{ 
			memcpy(data, _graphicsbuf, _num_bytes_graphics_data);
			_state &= ~NRMK_SOCKET_PACKET_MASK_CONTROL_GRAPHICS; //reset monbuf_ready bit, rdbuf_ready is expired

			return _num_bytes_graphics_data;
		}

		return 0;
	}

	inline void clearControlGraphics()
	{
		_num_bytes_graphics_data = 0;
		_num_graphics_objects = 0;
	}

	inline int waitForWriteBufReady()
	{
		return _wrbufReadyEvent.wait();

	}

	inline int waitForReadBufReady()
	{
		return _rdbufReadyEvent.wait();

	}

//  	inline int waitForReadBufReady_bruteforce()
//  	{
//  		return _state & NRMK_SOCKET_PACKET_MASK_SENSOR;
//  	}
// 
// 	inline int waitForWriteBufReady_bruteforce()
// 	{
// 		return _state & NRMK_SOCKET_PACKET_MASK_ACTUATOR;
// 	}

	unsigned int numBytesGraphicsData() const { return _num_bytes_graphics_data; }
	unsigned int numGraphicsObjects() const { return _num_graphics_objects; }

	unsigned char const * const databuf() const { return _databuf; }
	unsigned char const * const graphicsbuf() const { return _graphicsbuf; }

private:
	inline int _sendData(int num_bytes/*, unsigned char mode*/, unsigned char const * const data, unsigned char * const buf)
	{
		//LockList();

		int cur = 0;
		// FIXME @ 20131123
		//buf[cur++] = _requestKey;
		buf[cur++] = _state; // now cur = 2
		
		memcpy(buf + cur, NRMK_SOCKET_START_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, START_token
		cur += 2; // now cur = 4
		
		buf[cur++] = (unsigned char) NRMK_SOCKET_PACKET_CONTROL;			//1 unsigned char, Packet type
		
		memcpy(buf + cur, data, num_bytes);					//nLen bytes, data
		cur += num_bytes;

		if (!IsServer() && (_state & NRMK_SOCKET_PACKET_MASK_CONTROL_DATA))
		{
			memcpy(buf + cur, _databuf, NUM_BYTES_CONTROL_DATA);
			cur += NUM_BYTES_CONTROL_DATA;
		}

		if (!IsServer() && (_state & NRMK_SOCKET_PACKET_MASK_CONTROL_GRAPHICS))
		{
			memcpy(buf + cur, _graphicsbuf, _num_bytes_graphics_data);
			cur += _num_bytes_graphics_data;
		}

		memcpy(buf + cur, NRMK_SOCKET_UPDATE_TOKEN, NRMK_SOCKET_TOKEN_SIZE);	//2 bytes, END_token

		cur += 2;
		memcpy(buf + cur, "\r\n", 2);

		cur += 2;
		WriteComm(buf, cur, INFINITE);

		//UnlockList();

		return cur;	
	}

private:
	NRMKEvent		_rdbufReadyEvent;
	NRMKEvent		_wrbufReadyEvent;

// 	HANDLE		_rdbufReadyEvent;
// 	HANDLE		_wrbufReadyEvent;

	unsigned char _rdbuf[NUM_BYTES_READ_BUF + 100];
	unsigned char _wrbuf[NUM_BYTES_WRITE_BUF + 100 + NUM_BYTES_CONTROL_DATA + NUM_BYTES_CONTROL_GRAPHICS];
	unsigned char _monbuf[NUM_BYTES_MONITOR_BUF + 100];

	unsigned char * _databuf;
	unsigned char * _graphicsbuf;

// 	unsigned char * _databuf[NUM_FLOATS_CONTROL_DATA];
//  	unsigned char * _graphicsbuf[NUM_BYTES_CONTROL_GRAPHICS];
	unsigned int _num_graphics_objects;
	unsigned int _num_bytes_graphics_data;

	// FIXED by THACHDO 20150717
	volatile bool _hasConnection; // moved from NRMKSocketBase
	volatile char _requestKey;
	BYTE _state;	
	
	// Do not try to move this to the base class
	// Nor to delete volatile attribute. (Then release mode does not work properly).
};

} // namespace NRMKHelper
