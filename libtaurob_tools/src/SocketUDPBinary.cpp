/**************************************************************************//**
 *
 * @file SocketUDPBinary.cpp
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
 

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <string>
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
#define DEBUG if (SocketUDPBinary::isDebugEnable) printf

#include <libtaurob_tools/SocketUDPBinary.h>

bool SocketUDPBinary::isDebugEnable = false;

SocketUDPBinary::SocketUDPBinary(int bind_to_local_port, unsigned int receive_buffer_len) 
{
	this->receive_buffer_len = receive_buffer_len;
	
	this->detect_host = true;
	this->bind_to_local_port = bind_to_local_port;
	send_to = 0;
	socket_id = createSocket();
}

SocketUDPBinary::SocketUDPBinary(std::string send_to_host, int send_to_port, unsigned int receive_buffer_len) 
{
	this->receive_buffer_len = receive_buffer_len;
	send_to = 0;
	this->detect_host = false;
	this->send_to_host = send_to_host.data();
	this->send_to_port = send_to_port;
	this->bind_to_local_port = -1;
	
	socket_id = createSocket();
	prepareToSend();
}

SocketUDPBinary::SocketUDPBinary(std::string send_to_host, int send_to_port, int bind_to_local_port, unsigned int receive_buffer_len) 
{
	send_to = 0;
	this->receive_buffer_len = receive_buffer_len;
	
	this->detect_host = false;
	this->send_to_host = send_to_host.data();
	this->send_to_port = send_to_port;
	this->bind_to_local_port = bind_to_local_port;
	
	socket_id = createSocket();
	prepareToSend();
}
 

/* function to be executed by the new thread */
void* async_receive_loop_udp_binary(void* context) {
	SocketUDPBinary* socket = (SocketUDPBinary*) context;

	while (!socket->is_closing) {
		socket->receive();
	}
	socket->is_closing = false;
	
	DEBUG("Closing Socket\n");
	pthread_exit(NULL);
	return NULL;
}

int SocketUDPBinary::createSocket() {
	DEBUG("Create Socket\n");
	int socket_id = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket_id == -1) {
		perror("socket");
	}
	return socket_id;
}

SocketUDPBinary::~SocketUDPBinary() {
	if (send_to != 0) free(send_to);
	::close(socket_id);
}

bool SocketUDPBinary::prepareToSend() {
	DEBUG("Prepare to send \n");
	DEBUG("Host %s \n", this->send_to_host.data());

	if (send_to != 0) free(send_to);
	int struct_size = sizeof(sockaddr);
	send_to = (sockaddr_in *) malloc(struct_size);
	send_to->sin_family = AF_INET;
	send_to->sin_port = htons(this->send_to_port);

	bool convertionSuccess = inet_aton(send_to_host.data(), &send_to->sin_addr) != 0;
	if (!convertionSuccess) {
		fprintf(stderr, "inet_aton() failed\n");
		return false;
	}
	return true;
}

bool SocketUDPBinary::start_listen(void (*callback)(unsigned char* data, int len, char* from_remote_ip, int from_remote_port, int from_local_port)) {
	this->receiver_callback_fn = callback;
	this->receiver_callback = null;
	is_closing = false;
	if (socket_id == -1) {
		return false;
	}
	DEBUG("Socket Id: %d\n", socket_id);

	if (bind_to_local_port != -1) {
		bool bindSuccess = bind();
		if (!bindSuccess) {
			return false;
		}
	}

	// create a new thread that will execute 'asycn_receive_loop()'
	int status = pthread_create(&p_thread, NULL, &async_receive_loop_udp_binary, this);
	if (status < 0) {
		perror("pthread_create failed");
		return false;
	}
	is_listening = true;
	DEBUG("Socket %d is listening\n", socket_id);
	return true;
}

bool SocketUDPBinary::start_listen(IUdpReceiverBinary* receiver_callback) {
	this->receiver_callback = receiver_callback;
	this->receiver_callback_fn = 0;
	is_closing = false;
	if (socket_id == -1) {
		return false;
	}
	DEBUG("Socket Id: %d\n", socket_id);

	if (bind_to_local_port != -1) {
		bool bindSuccess = bind();
		if (!bindSuccess) {
			return false;
		}
	}

	// create a new thread that will execute 'asycn_receive_loop()'
	int status = pthread_create(&p_thread, NULL, &async_receive_loop_udp_binary, this);
	if (status < 0) {
		perror("pthread_create failed");
		return false;
	}
	is_listening = true;
	DEBUG("Socket %d is listening\n", socket_id);
	return true;
}

void SocketUDPBinary::stop_listen() {
	is_closing = true;
	//pthread_kill(p_thread, SIGTERM);
	usleep(250*1000); 	// 250ms to stop the async receive thread
	
	is_listening = false;
	DEBUG("Socket %d is close\n", socket_id);
}

bool SocketUDPBinary::send(const unsigned char* data, const unsigned int len) {
	if (send_to == 0) {
		DEBUG("Socket is NOT ready to send\n");
		return false;
	}

	const unsigned char* send_buf = data;
	int send_buf_len = len;

	//DEBUG("Sending %d bytes\n", len);
	int struct_size = sizeof(sockaddr);
	int send_result = sendto(socket_id, send_buf, send_buf_len, 0, (struct sockaddr *) send_to, struct_size);
	bool success = send_result != -1;
	if (!success) {
		perror("sendto()");
		return false;
	}
	return true;
}

void SocketUDPBinary::receive() {
	DEBUG("Receive from Socket Id: %d\n", socket_id);
	int struct_size = sizeof(sockaddr);
	sockaddr_in *receive_from = (sockaddr_in*) malloc(struct_size);

	unsigned char* receive_buffer = (unsigned char*)malloc(receive_buffer_len);
	
	// clear buffer beforehand
	bzero((void*)receive_buffer, receive_buffer_len);
	int receive_len = recvfrom(socket_id, receive_buffer, receive_buffer_len, 0, (sockaddr *)receive_from, (socklen_t *) &struct_size);

	if (receive_len == -1) {
		DEBUG("Error in received packet");
		DEBUG("Error data: %s\n", receive_buffer);
		perror("recvfrom()");
		return;
	}
	else if (receive_len > 0)
	{
		char* received_from_addr = inet_ntoa(receive_from->sin_addr);
		int receive_from_port = ntohs(receive_from->sin_port);

		DEBUG("Received packet from: %s:%d\n", received_from_addr, receive_from_port);
	
		if ((unsigned int)receive_len <= receive_buffer_len)
		{
			if (detect_host &&
				(send_to_port != receive_from_port || send_to_host != received_from_addr)) {

				this->send_to_host = received_from_addr;
				this->send_to_port = receive_from_port;
				prepareToSend();
			}
		}
		
		if (is_closing == false)
		{
			if (receiver_callback != 0)
				receiver_callback->On_data_received(receive_buffer, receive_len, received_from_addr, receive_from_port, this->bind_to_local_port);
			if (receiver_callback_fn != 0)
				receiver_callback_fn(receive_buffer, receive_len, received_from_addr, receive_from_port, this->bind_to_local_port);
		}
	}

	free(receive_buffer);
	free(receive_from);
}

bool SocketUDPBinary::bind() {
	DEBUG("Bind to socket %d\n", socket_id);
	int struct_size = sizeof(sockaddr);
	sockaddr_in *bind_to = (sockaddr_in*) malloc(struct_size);
	bind_to->sin_family = AF_INET;
	bind_to->sin_port = htons(this->bind_to_local_port);
	bind_to->sin_addr.s_addr = htonl(INADDR_ANY);

	int bind_res = ::bind(socket_id, (sockaddr *) bind_to, sizeof(sockaddr_in));
	if (bind_res == -1) {
		perror("bind()");
		return false;
	}
	DEBUG("Bind success\n");

	free(bind_to);
	return true;
}


