/* NRMKFoundation, Copyright 2013- Neuromeka. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Neuromeka
 */
#pragma once

#include "NRMKHelper.h"
#include "SocketComm.h"

// for license mac----
#define MASK		0x85
#define hlen		0x19
#define packetsize	6
#define flen		0x82
#define packetlen	(hlen+packetsize+flen)

#define WM_UPDATE_CONNECTION	(WM_APP + 0x1234)

#define WSA_VERSION  MAKEWORD(2,0)
#define MAX_HOSTNAME 256
#define MAX_HOSTADDR 40

#define MAX_CONNECTION		10
#define MaxLineLen	100000	// maximum length of one LINE in file
#define MaxWordLen	1024	// maximum length of one WORD in file (WORD: meaningful set between ' ', '\n', '\t' ) 

#define  SOCK_TCP		0
#define  SOCK_UDP		1

#define	NORMAL_RUN		0
#define	REALTIME_RUN	1

// Port allocation
/*
#define NRMK_PORT_SIM		"2001"
#define NRMK_PORT_DATA		"3001"
#define NRMK_PORT_GRAPHICS	"4001"
#define NRMK_PORT_CONTROL	"5001"
#define NRMK_PORT_COMMAND	"5999"
*/
#define NRMK_PORT_SIM		2001
#define NRMK_PORT_DATA		3001
#define NRMK_PORT_GRAPHICS	4001
#define NRMK_PORT_CONTROL	5001
#define NRMK_PORT_COMMAND	5999
#define NRMK_PORT_SMARTTP	6001
#define NRMK_PORT_ANIMATION 7001
#define NRMK_PORT_PENDANT	8001

//Communication---
// Simulation data
#define NRMK_SOCKET_PACKET_READYCHECK		-1
#define NRMK_SOCKET_PACKET_TRANSFORM		1
#define NRMK_SOCKET_PACKET_ZERO_CONFIG		2
#define NRMK_SOCKET_PACKET_INDEX			3

// Custom graphics object data
#define NRMK_SOCKET_PACKET_GRAPHICS_DATA	4

// Data scope data
#define NRMK_SOCKET_PACKET_DATA				10

#define NRMK_SOCKET_PACKET_DATA_STYLE_TIME_CHART	((unsigned int) 0)
#define NRMK_SOCKET_PACKET_DATA_STYLE_AREA_CHART	((unsigned int) 1)
#define NRMK_SOCKET_PACKET_DATA_STYLE_XY_CHART		((unsigned int) 2)

#define NRMK_SCOKET_PACKET_DATA_CH(ch)				((unsigned int) ch)

// Control client data
#define NRMK_SOCKET_PACKET_CONTROL					20
#define NRMK_SOCKET_PACKET_MASK_SENSOR				0x1 
#define NRMK_SOCKET_PACKET_MASK_ACTUATOR			0x2 
#define NRMK_SOCKET_PACKET_MASK_MONITOR				0x4 
#define NRMK_SOCKET_PACKET_MASK_CONTROL_DATA		0x10 // for data transmission from control client to application manager (then toward Data Scope)
#define NRMK_SOCKET_PACKET_MASK_CONTROL_GRAPHICS	0x20 // for data transmission from control client to application manager (then toward CADKit Viewer)

// Pendant data
#define NRMK_SOCKET_PACKET_PENDANT_CMD		30
#define NRMK_SOCKET_PACKET_PENDANT_MSG		31


#define NRMK_SOCKET_TOKEN_SIZE		2	
#define NRMK_SOCKET_START_TOKEN		"NS"
#define NRMK_SOCKET_END_TOKEN		"NE"
#define NRMK_SOCKET_UPDATE_TOKEN	"NU"
#define NRMK_SOCKET_DATA_TOKEN		"ND"

namespace NRMKHelper 
{

class NRMKHelper_API NRMKSocketBase : public CSocketComm
{
public:
	NRMKSocketBase();
	//NRMKSocketBase(NRMKCHAR sIP, NRMKCHAR sPort);

	~NRMKSocketBase();
	
	int connect(NRMK_PCHAR sIP, int port, char RunMode = NORMAL_RUN);
	bool startServer(int m_nSockType, int port, char RunMode = NORMAL_RUN);

	int connect(NRMK_PCHAR sIP, NRMK_PCHAR sPort, char RunMode = NORMAL_RUN);
	bool startServer(int m_nSockType, NRMK_PCHAR sPort, char RunMode = NORMAL_RUN);

	//virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount);
	virtual void OnEvent(UINT uEvent, LPVOID lpvData);

	// you have to override this virtual function
	virtual bool hasConnection() const { return false; }

	NRMK_CHAR const * const getAddress() const { return _strAddr; }
	bool waitForConnection(unsigned int msec = -1) const; 

	//int getKey();

// private:
// 	void _winSockInit();

protected:
	//char _requestKey;
	//volatile bool _hasConnection;

private:
	NRMK_CHAR  _strAddr[MaxWordLen];

#ifdef __GNUC__
	int  _getMyMac(BYTE myMac[], char aname[]);
	int _readLicenseFile(BYTE allMac[]);
	char _checkLicense();
#endif

	char _licensed;
	char _runMode;	// normal or realtime
};

} // namespace NRMKHelper
