/*
 * RTControlSocket.h
 *
 *  Created on: 2019. 12. 12.
 *      Author: Administrator
 */

#ifndef RTCONTROLSOCKET_H_
#define RTCONTROLSOCKET_H_

#include <iostream>

#include <Poco/Net/SocketReactor.h>
#include <Poco/Net/SocketAcceptor.h>


const Poco::UInt16 PORT = 32452;


class Session {
public:
	Session(Poco::Net::StreamSocket& socket, Poco::Net::SocketReactor& reactor) :
		m_Socket(socket),
		m_Reactor(reactor)
	{
		m_PeerAddress = socket.peerAddress().toString();

		std::cout << "connection from " << m_PeerAddress << " ..." << std::endl;

		m_Reactor.addEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::ReadableNotification>(*this, &Session::onReadable));

		m_Reactor.addEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::ShutdownNotification>(*this, &Session::onShutdown));

		m_Reactor.addEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::ErrorNotification>(*this, &Session::onError));

		m_Reactor.addEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::IdleNotification>(*this, &Session::onIdle));

		m_Reactor.addEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::TimeoutNotification>(*this, &Session::onTimeout));

	}

	~Session()
	{
		std::cout << m_PeerAddress << " disconnected ..." << std::endl;

		m_Reactor.removeEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::ReadableNotification>(*this, &Session::onReadable)
			);

		m_Reactor.removeEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::ShutdownNotification>(*this, &Session::onShutdown)
			);

		m_Reactor.removeEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::ErrorNotification>(*this, &Session::onError)
			);

		m_Reactor.removeEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::IdleNotification>(*this, &Session::onIdle)
			);

		m_Reactor.removeEventHandler(m_Socket,
			Poco::Observer<Session, Poco::Net::TimeoutNotification>(*this, &Session::onTimeout)
			);
	}

	void onReadable(Poco::Net::ReadableNotification* pNf)
	{
		pNf->release();

		char buffer[256] = { 0, };
		int n = m_Socket.receiveBytes(buffer, sizeof(buffer));
		if (n > 0)
		{
			std::cout << "Recieved Msg: " << buffer << std::endl;

			char szSendMessage[256] = { 0, };
			//sprintf_s(szSendMessage, 256 - 1, "Re:%s", buffer);
			//int nMsgLen = (int)strnlen_s(szSendMessage, 256 - 1);

			//m_Socket.sendBytes(szSendMessage, nMsgLen);
		}
		else
		{
			m_Socket.shutdownSend();
			delete this;
		}
	}

	void onShutdown(Poco::Net::ShutdownNotification* pNf)
	{
		pNf->release();

		std::cout << "onShutdown ERROR" << std::endl;
	}

	void onError(Poco::Net::ErrorNotification* pNf)
	{
		pNf->release();

		std::cout << "onError ERROR" << std::endl;
	}

	void onTimeout(Poco::Net::TimeoutNotification* pNf)
	{
		pNf->release();

		std::cout << "onTimeout ERROR" << std::endl;
	}

	void onIdle(Poco::Net::IdleNotification* pNf)
	{
		pNf->release();

		std::cout << "onIdle ERROR" << std::endl;
	}

private:
	Poco::Net::StreamSocket   m_Socket;
	std::string m_PeerAddress;
	Poco::Net::SocketReactor& m_Reactor;
};


#endif /* RTCONTROLSOCKET_H_ */
