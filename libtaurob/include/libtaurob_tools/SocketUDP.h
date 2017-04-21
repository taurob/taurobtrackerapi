/**************************************************************************//**
 *
 * @file SocketUDP.h
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Library containing misc. tools for other taurob libraries
 *
 *
 *  Copyright (c) 2016 taurob GmbH. All rights reserved.
 *  Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 ******************************************************************************/
 

#ifndef SOCKETUDP_H_
#define SOCKETUDP_H_

#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "IUdpReceiver.h"
#include "Debug.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include "string.h"
#include <arpa/inet.h>

#include <errno.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <pthread.h>  /* pthread functions and data structures */
#include <signal.h>

#define BUFFER_LEN 512
#define null 0
 
class SocketUDP 
{
	public:
		std::string send_to_host;
		int send_to_port;
		int bind_to_local_port;
		int detect_host;

		SocketUDP(int port);
		SocketUDP(std::string host, int port);
		virtual ~SocketUDP();

		bool send(const std::string s);

		bool is_listening;
		bool start_listen(IUdpReceiver * receiver_callback);
		bool start_listen(void (*callback)(std::string data, char* from_remote_ip, int from_remote_port, int from_local_port));
		void stop_listen();

		static bool isDebugEnable;
		
		std::string receive(); 	// public so async_receive_loop can see it
		bool is_closing;
		
	private:
		int socket_id;
		sockaddr_in* send_to;
		IUdpReceiver* receiver_callback;
		void (*receiver_callback_fn)(std::string data, char* from_remote_ip, int from_remote_port, int from_local_port);

		pthread_t p_thread;

		int createSocket();
		bool prepareToSend();
		bool bind();
};

#endif /* SOCKETUDP_H_ */
