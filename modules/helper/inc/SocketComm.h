///////////////////////////////////////////////////////////////////////////////
// FILE : SocketComm.h
// Header file for CSocketComm class
// CSocketComm
//     Generic class for Socket Communication
///////////////////////////////////////////////////////////////////////////////

#ifndef _SOCKETCOMM_H_
#define _SOCKETCOMM_H_
#include <list>

#include <stdlib.h>
#include <stdio.h>
#include "Poco/Net/TCPServer.h"
#include "Poco/Net/TCPServerParams.h"
#include "Poco/Net/TCPServerConnectionFactory.h"
#include "Poco/Net/TCPServerConnection.h"
#include "Poco/Net/Socket.h"
#include "Poco/Net/StreamSocket.h"
#include "Poco/Net/DNS.h"
#include "Poco/Net/HostEntry.h"
#include "Poco/Net/NetException.h"
#include "Poco/Timespan.h"
#include "Poco/ThreadPool.h"
#include "Poco/Runnable.h"
#include "Poco/BasicEvent.h"
#include "Poco/Delegate.h"

#include <string.h>
#include <iostream>

// Event value
#define EVT_CONSUCCESS      0x0000  // Connection established
#define EVT_CONFAILURE      0x0001  // General failure - Wait Connection failed
#define EVT_CONDROP         0x0002  // Connection dropped
#define EVT_ZEROLENGTH      0x0003  // Zero length message
#define INFINITE			0xFFFFFFFF	// Infinite timeout
#ifndef BUFFER_SIZE
#define BUFFER_SIZE			100000
#endif
#define HOSTNAME_SIZE		1000
#define STRING_LENGTH		1000

#ifndef __GNUC__
#include <tchar.h>
#include <process.h>
#include <crtdbg.h>
#include <windows.h>

#else
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netdb.h>
#include <ifaddrs.h>

#define Sleep(x)	usleep(1000*x)

typedef	uint8_t 		BYTE;
typedef BYTE            *LPBYTE;
typedef unsigned long   DWORD;
typedef unsigned int	UINT;
typedef	void			*HANDLE;
typedef	void 			*LPVOID;
typedef int  			SOCKET;
typedef unsigned short	USHORT;

#ifdef __XENO__
#include <native/task.h>
#include <native/timer.h>
#endif
#endif

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "NRMKHelper.h"

typedef char	*NRMK_PCHAR;
typedef char	NRMK_CHAR;

class NRMKHelper_API CSocketComm : Poco::Runnable
{
public:
    CSocketComm();
    virtual ~CSocketComm();

    bool IsOpen() const;    // Is Socket valid?
    bool IsStart() const;   // Is Thread started?
    bool IsServer() const;  // Is running in server mode
    bool IsBroadcast() const; // Is UDP Broadcast active
    bool IsSmartAddressing() const; // Is Smart Addressing mode support
    Poco::Net::ServerSocket GetSocket() const;
    void SetServerState(bool bServer);  // Run as server mode if true
    void SetSmartAddressing(bool bSmartAddressing); // Set Smart addressing mode
    void CloseComm();       // Close Socket
    bool WatchComm();       // Start Socket thread
    void StopComm();        // Stop Socket thread

    // Create a socket - Server side (support for multiple adapters)
    bool CreateSocketEx(NRMK_PCHAR strHost, NRMK_PCHAR strServiceName, int nFamily, int nType, UINT uOptions /* = 0 */);
	bool CreateSocketEx(NRMK_PCHAR strHost, int port, int nFamily, int nType, UINT uOptions /* = 0 */);
	// Create a Socket - Server side
    bool CreateSocket(NRMK_PCHAR strServiceName, int nProtocol, int nType, UINT uOptions = 0);
	bool CreateSocket(int port, int nProtocol, int nType, UINT uOptions = 0);
    // Create a socket, connect to (Client side)
    bool ConnectTo(NRMK_PCHAR strDestination, NRMK_PCHAR strServiceName, int nProtocol, int nType);
	bool ConnectTo(NRMK_PCHAR strDestination, int port, int nProtocol, int nType);

	// Event function - override to get data
    virtual void OnDataReceived(const LPBYTE lpBuffer, DWORD dwCount);
    virtual void OnEvent(UINT uEvent, LPVOID lpvData);
	// Run function - override to implement a new behavior
    virtual void run();

	// Data function
    DWORD ReadComm(LPBYTE lpBuffer, DWORD dwSize, DWORD dwTimeout);
	DWORD WriteComm(const LPBYTE lpBuffer, DWORD dwCount, DWORD dwTimeout);

	void rtServerInit();
	int ReadCommRT();       //receive data, used for realtime case

    // Utility functions
    bool ShutdownConnection(SOCKET sock);  // Shutdown a connection
    NRMK_PCHAR GetIPAddress( NRMK_PCHAR strHostName );   // Get IP address of a host
    bool GetLocalName(NRMK_PCHAR strName, UINT nSize);   // GetLocalName
    bool GetLocalAddress(NRMK_PCHAR strAddress, UINT nSize); // GetLocalAddress

// SocketComm - data
protected:
    Poco::Net::ServerSocket socketServer;	// Server Socket: server handle
	Poco::Net::StreamSocket	socketConn;		// Socket Connection : client handle
	Poco::Thread			hThread;		// Thread
	bool					_IsOpen;
	bool					_IsStart;
	bool					_IsServer;
    bool        m_bSmartAddressing; // Smart Addressing mode (true) - many listeners
	bool        m_bBroadcast;   // Broadcast mode

#ifdef __XENO__
	DWORD   dwBytes;
	DWORD   dwTimeout;
	DWORD   dwSize;
	BYTE	buffer[BUFFER_SIZE];
	LPBYTE  lpData;
#endif
    
// SocketComm - function
protected:
    // Synchronization function
	Poco::Mutex				_mutex;
	void LockList();            // Lock the object
	void UnlockList();          // Unlock the object	

	virtual void signal_on(BYTE ch);
	virtual void signal_off(BYTE ch);
};

#endif // _SOCKETCOMM_H_
