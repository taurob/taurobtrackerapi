/**************************************************************************//**
 *
 * @file SocketUDPBinary.h
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
 

#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "IUdpReceiverBinary.h"

#ifndef SOCKETUDPBINARY_H_
#define SOCKETUDPBINARY_H_

class SocketUDPBinary {
	public:
		std::string send_to_host;
		int send_to_port;
		int bind_to_local_port;
		int detect_host;

		SocketUDPBinary(int port, unsigned int receive_buffer_len);
		SocketUDPBinary(std::string host, int port, unsigned int receive_buffer_len);
		SocketUDPBinary(std::string host, int port, int bind_to_port, unsigned int receive_buffer_len);
		virtual ~SocketUDPBinary();

		bool send(const unsigned char* data, const unsigned int len);

		bool is_listening;
		bool start_listen(IUdpReceiverBinary* receiver_callback);
		bool start_listen(void (*callback)(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port));
		void stop_listen();

		static bool isDebugEnable;

		void receive(); 	// must be public so the async receive thread, which is not part of this class, can access it
		bool is_closing;
		
	private:
		int socket_id;
		sockaddr_in * send_to;
		IUdpReceiverBinary* receiver_callback;
		void (*receiver_callback_fn)(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port);
		pthread_t p_thread;
		
		unsigned int receive_buffer_len;

		int createSocket();
		bool prepareToSend();
		bool bind();
};
	
#endif
