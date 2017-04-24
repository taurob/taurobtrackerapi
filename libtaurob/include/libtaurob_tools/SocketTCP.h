/*
 * Copyright (c) 2017 taurob GmbH.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SOCKETTCP_H_
#define SOCKETTCP_H_

#include <string>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

class SocketTCP {
public:
	std::string send_to_host;
	int send_to_port;
	int receive_from_port;
	int detect_host;

	SocketTCP(int port);
	SocketTCP(std::string host, int port);
	virtual ~SocketTCP();
	
	bool send(const std::string s);

	bool is_listening;
	bool start_listen(void (*callback)(std::string));
	void stop_listen();
	void dispose();

	static bool isDebugEnable;
	
	std::string receive(); 	// public so the async receive loop, which is not a member, can see it
	bool is_closing;
	
private:
	int socket_id;
	int client_id;
	bool sending_ready;
	bool is_server;
	sockaddr_in * send_to = NULL;
	void (*receiver_callback)(std::string);
	pthread_t p_thread;

	int createSocket();
	bool prepareToSend();
	bool bind();
	
};

#endif /* SOCKETTCP_H_ */
