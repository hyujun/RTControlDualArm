/*
 * RTControlSocket.h
 *
 *  Created on: 2019. 12. 12.
 *      Author: Administrator
 */

#ifndef SOCKETHANDLER_H_
#define SOCKETHANDLER_H_

#include <iostream>
#include <cstring>
#include <mutex>
#include <Poco/Net/TCPServer.h>
#include <Poco/Net/TCPServerConnection.h>
#include <Poco/Net/TCPServerConnectionFactory.h>

const Poco::UInt16 SERVER_PORT = 32452;

class Session : public Poco::Net::TCPServerConnection
{
public:
	Session(const Poco::Net::StreamSocket &socket) : TCPServerConnection(socket)
	{

	}
	virtual ~Session() { }

	virtual void run()
	{
		try
		{
			int recvSize = 0;

			do
			{
				recvSize = socket().receiveBytes(buffer, sizeof(buffer));
				std::cout << "Recieved Massage from client " << buffer << std::endl;

				snprintf(szSendMessage, 256 - 1, "Re:%s", buffer);
				int nMsgLen = (int)strnlen(szSendMessage, 256 - 1);

				socket().sendBytes(szSendMessage, nMsgLen);
			} while (recvSize > 0);

			std::cout << "Disconeected with the client" << std::endl;
		}
		catch (Poco::Exception& exc)
		{
			std::cout << "Session: " << exc.displayText() << std::endl;
		}
	}
private:
	char buffer[256] = { 0, };
	char szSendMessage[256] = { 0, };
};

class SessionFactory : public Poco::Net::TCPServerConnectionFactory
{
public:
	SessionFactory() { }
	virtual ~SessionFactory() { }

	virtual Poco::Net::TCPServerConnection* createConnection(const Poco::Net::StreamSocket &socket)
	{
		m_mtx.lock();
		++m_ConnectedCount;

		std::cout << m_ConnectedCount << "th Client Connected!" << std::endl;
		m_mtx.unlock();

		return new Session(socket);
	}

private:
	int m_ConnectedCount = 0;
	std::mutex m_mtx;
};

void PrintServerStatus(Poco::Net::TCPServer& server)
{

	//printf("maxThreads:%d, maxConcurrentConnections:%d\n",
	//	server.maxThreads(), server.maxConcurrentConnections());

	printf("currentThreads:%d, currentConnections:%d\n",
		server.currentThreads(), server.currentConnections());

	printf("queuedConnections:%d, totalConnections:%d\n\n",
		server.queuedConnections(), server.totalConnections());
}

#endif /* SOCKETHANDLER_H_ */
