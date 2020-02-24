/*
 * RTControlSocket.h
 *
 *  Created on: 2019. 12. 12.
 *      Author: Administrator
 */

#ifndef RTCONTROLSOCKET_H_
#define RTCONTROLSOCKET_H_

#include <iostream>
#include <cstring>

#include "Poco/Net/TCPServer.h"
#include "Poco/Net/TCPServerConnection.h"
#include "Poco/Net/TCPServerConnectionFactory.h"
#include "Poco/Thread.h"

using Poco::Net::ServerSocket;
using Poco::Net::StreamSocket;
using Poco::Net::TCPServerConnection;
using Poco::Net::TCPServerConnectionFactory;
using Poco::Net::TCPServer;
using Poco::Timestamp;
using Poco::Thread;

const Poco::UInt16 SERVER_PORT = 9911;

class Session : public TCPServerConnection
{
public:
	enum {
		SIZE_HEADER = 52,
		SIZE_COMMAND = 4,
		SIZE_HEADER_COMMAND = 56,
		SIZE_DATA_MAX = 200,
		SIZE_DATA_ASCII_MAX = 32
	};
	Session(const StreamSocket &socket) :TCPServerConnection(socket)
	{
		j=0;
		//cout << "Session Object Construct" << endl;
	}
	virtual ~Session() {
		//cout << "Session Object Destruct" << endl;
	}
	union Data
	{
		unsigned char byte[SIZE_DATA_MAX];
		double double6dArr[6];
	};
	Data data_rev;
	Data data;

	virtual void run()
	{
		unsigned char readBuff[1024];
		unsigned char writeBuff[1024];

		try
		{
			int recvSize = 0;
			do
			{
				memcpy(writeBuff, data.byte, SIZE_HEADER_COMMAND);

				Timestamp now;
				recvSize = socket().receiveBytes(readBuff, 1024);

				memcpy(data_rev.byte, readBuff, SIZE_HEADER_COMMAND);

				received_Data[0] = data_rev.double6dArr[0];   //x
				received_Data[1] = data_rev.double6dArr[1];   //y
				received_Data[2] = data_rev.double6dArr[2];   //z
				received_Data[3] = data_rev.double6dArr[4];   //Q
				received_Data[4] = data_rev.double6dArr[3];   //angle

				data_rev.double6dArr[5] = 1;
				memcpy(writeBuff, data_rev.byte, SIZE_HEADER_COMMAND);

				socket().sendBytes(writeBuff, 1024);
			} while (recvSize > 0);
			//cout << "Client Disconnected." << endl;
		}
		catch (Poco::Exception& exc)
		{
			std::cout << "Session: " << exc.displayText() << std::endl;
		}

	}
private:
	double received_Data[6];
	int j;
};

class SessionFactory :public TCPServerConnectionFactory
{
public:
	SessionFactory() {}
	virtual ~SessionFactory() {}

	virtual TCPServerConnection* createConnection(const StreamSocket &socket)
	{
		return new Session(socket);
	}
};


#endif /* RTCONTROLSOCKET_H_ */
