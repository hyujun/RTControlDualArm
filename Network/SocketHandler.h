/*
 * RTControlSocket.h
 *
 *  Created on: 2019. 12. 12.
 *      Author: Junho Park
 */

#ifndef SOCKETHANDLER_H_
#define SOCKETHANDLER_H_

#include <iostream>
#include <algorithm>
#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketAddress.h>

#include "PacketHandler.h"
#include "KRISSSensorHandler.h"

const Poco::UInt16 SERVER_PORT = 32452;
int TargetHandMotion=0x00;
int TargetHandState=0x00;

class SocketHandler
{
public:
    SocketHandler(const Poco::Net::ServerSocket &socket)
	{
        server_sock = socket;
        connectedSockList.push_back(socket);
	}
	virtual ~SocketHandler()
	{

	}

    void TXRXUpdate()
    {
        Poco::Net::Socket::SocketList readList(connectedSockList.begin(), connectedSockList.end());
        Poco::Net::Socket::SocketList writeList(connectedSockList.begin(), connectedSockList.end());
        Poco::Net::Socket::SocketList exceptList(connectedSockList.begin(), connectedSockList.end());

        Poco::Timespan timeout;
        if(Poco::Net::Socket::select(readList, writeList, exceptList, timeout) == 0)
            return;

        Poco::Net::Socket::SocketList delSockList;

        for (auto& readSock : readList)
        {
            if (server_sock == readSock)
            {
                auto newSock = server_sock.acceptConnection();
                connectedSockList.push_back(newSock);
                std::cout << "New Client connected" << std::endl;
            }
            else
            {
                auto n = ((Poco::Net::StreamSocket*)&readSock)->receiveBytes(buffer, sizeof(buffer));
                if (n > 0)
                {
                    std::cout << "Received Message from Client: " << buffer << std::endl;
                    //snprintf(szSendMessage, sizeof(szSendMessage), "Re:%s", buffer);
                    auto nMsgLen = strnlen(szSendMessage, sizeof(szSendMessage)-1);
                    ((Poco::Net::StreamSocket*)&readSock)->sendBytes(szSendMessage, nMsgLen);
                }
                else
                {
                    std::cout << "Client Disconnected" << std::endl;
                    delSockList.push_back(readSock);
                }
            }
        }

        for (auto& delSock : delSockList)
        {
            auto delIter = std::find_if(connectedSockList.begin(),connectedSockList.end(),[&delSock](auto& sock){return delSock == sock ? true : false;});
            if (delIter != connectedSockList.end())
            {
                connectedSockList.erase(delIter);
                std::cout << "Remove the Client from connectedSockList" << std::endl;
            }
        }
    }

private:

    Poco::Net::ServerSocket server_sock;
	char buffer[128] = { 0, };
	char szSendMessage[128] = { 0, };
    int buff_size=0;
    PacketHandler mPackethandler;
    Poco::Net::Socket::SocketList connectedSockList;

};
#endif /* SOCKETHANDLER_H_ */
