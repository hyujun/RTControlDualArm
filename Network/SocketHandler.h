/*
 * RTControlSocket.h
 *
 *  Created on: 2019. 12. 12.
 *      Author: Administrator
 */

#ifndef SOCKETHANDLER_H_
#define SOCKETHANDLER_H_

#include <iostream>
#include <string.h>
#include <mutex>
#include <stdio.h>
#include <Poco/Net/TCPServer.h>
#include <Poco/Net/TCPServerConnection.h>
#include <Poco/Net/TCPServerConnectionFactory.h>

#include "PacketHandler.h"
#include "KRISSSensorHandler.h"

const Poco::UInt16 SERVER_PORT = 32452;
int TargetHandMotion=0x00;
int TargetHandState=0x00;

class Session : public Poco::Net::TCPServerConnection
{
public:
	Session(const Poco::Net::StreamSocket &socket) : TCPServerConnection(socket)
	{

	}
	virtual ~Session() {}

	virtual void run()
	{
		try
		{
			int recvSize = 0;

			do
			{
                buff_size = sizeof(buffer);

				recvSize = socket().receiveBytes(buffer, sizeof(buffer));
				std::cout << "Recieved Massage from client " << buffer << std::endl;

                memcpy(TxFrame.data, buffer, sizeof(buffer));
                mPackethandler.packetlibrary(TxFrame);

                memset(RxFrame.data, 0, sizeof(RxFrame.data));
                RxFrame = TxFrame;
                if(RxFrame.info.index == Index_HandCommand_request)
                {
                    TargetHandState = RxFrame.info.subindex;
                    RxFrame.info.subindex = TargetHandMotion;
                }
                else
                {
                    RxFrame.info.subindex += 0x01;
                }


				socket().sendBytes(RxFrame.data, sizeof(RxFrame.data));
			} while (recvSize > 0);

			std::cout << "Disconeected with the client" << std::endl;
		}
		catch (Poco::Exception& exc)
		{
			std::cout << "Session: " << exc.displayText() << std::endl;
		}
	}

private:
	char buffer[32] = { 0, };
	char szSendMessage[32] = { 0, };
    TCP_Packet TxFrame, RxFrame;
    int buff_size=0;
    PacketHandler mPackethandler;



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

void TCP_SetTargetTaskData(int &txdata, int &rxdata)
{
    TargetHandMotion = txdata;
    rxdata = TargetHandState;
}

#endif /* SOCKETHANDLER_H_ */
