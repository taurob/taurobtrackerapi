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
#include <assert.h>

#include <libtaurob_tools/SocketTCP.h>


#define BUFFER_LEN 512
#define DEBUG if (SocketTCP::isDebugEnable) printf


bool SocketTCP::isDebugEnable = false;


// continuous receive -- for servers, in its own thread -- uses the blocking receive!
void* async_receive_loop_tcp(void* context) {
	SocketTCP* socket = (SocketTCP*) context;
	
	while (!socket->is_closing) {
		socket->receive();
	}
	
	DEBUG("Closing Socket\n");
	pthread_exit(NULL);
	return NULL;
}


// use this constructor for a server, with no specific client in mind
// (i.e., sending will only happen as a response to a request)
SocketTCP::SocketTCP(int receive_from_port) {
	detect_host = true;
	sending_ready = false;
	this->receive_from_port = receive_from_port;
	socket_id = createSocket();
	is_server = true;
}

// use this constructor for a client, with the intention to send to a 
// specific host and maybe expect a reply
SocketTCP::SocketTCP(std::string send_to_host, int send_to_port) {
	detect_host = false;
	sending_ready = false;
	this->send_to_host = send_to_host.data();
	this->send_to_port = send_to_port;
	receive_from_port = -1;
	is_server = false;
	
	socket_id = createSocket();
	prepareToSend();
	if (connect(socket_id, (struct sockaddr*)send_to, sizeof(*send_to)) < 0)
	{
		perror("connect()");
	}
	else
	{
		sending_ready = true;
	}
}

int SocketTCP::createSocket() {
	DEBUG("Create Socket\n");
	int socket_id = socket(AF_INET, SOCK_STREAM, 0);
	if (socket_id == -1) {
		perror("socket");
	}
	return socket_id;
}

void SocketTCP::dispose()
{
	if (send_to != NULL) free(send_to);
	::close(socket_id);
}

SocketTCP::~SocketTCP() {
	dispose();
}

bool SocketTCP::prepareToSend() {
	DEBUG("Prepare to send \n");
	DEBUG("Host %s \n", this->send_to_host.data());
	
	int struct_size = sizeof(struct sockaddr);
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

bool SocketTCP::start_listen(void (*callback)(std::string)) {
	this->receiver_callback = callback;
	is_closing = false;
	if (socket_id == -1) {
		return false;
	}
	DEBUG("Socket Id: %d\n", socket_id);

	if (receive_from_port != -1) {
		bool bindSuccess = bind();
		if (!bindSuccess) {
			return false;
		}
	}
	
	// we accept at most one connection at once
	listen(socket_id, 1);

	// create a new thread that will execute 'asycn_receive_loop()'
	int status = pthread_create(&p_thread, NULL, &async_receive_loop_tcp, this);
	if (status < 0) {
		perror("pthread_create failed");
		return false;
	}
	is_listening = true;
	DEBUG("Socket %d is listening\n", socket_id);
	return true;
}

void SocketTCP::stop_listen() {
	is_closing = true;
	pthread_kill(p_thread, SIGTERM);

	is_closing = false;
	is_listening = false;
	close(socket_id);
	DEBUG("Socket %d is close\n", socket_id);
}

bool SocketTCP::send(std::string data) {
	if (sending_ready == false) {
		DEBUG("Socket is NOT ready to send\n");
		return false;
	}

	char* send_buf = (char *) data.data();
	int send_buf_len = data.length();
	
	DEBUG("Send: %s\n", send_buf);
	int send_sock = (is_server ? client_id : socket_id);
	int send_result = ::send(send_sock, send_buf, send_buf_len, 0);
	bool success = send_result != -1;
	if (!success) {
		perror("send()");
		return false;
	}
	return true;
}

// private function -- actual receive logic, called by async_receive_loop
std::string SocketTCP::receive() 
{
	int struct_size = sizeof(struct sockaddr);
	DEBUG("Receive from Socket Id: %d\n", socket_id);
	sockaddr_in *receive_from = (sockaddr_in*) malloc(struct_size);
	
	char receive_buf[BUFFER_LEN];
	client_id = accept(socket_id, (sockaddr *) receive_from, (socklen_t *) &struct_size);

	if (client_id == -1)
	{
		DEBUG("Failed to accept connection");
		perror("accept()");
		return NULL;
	}
	
	int receive_len = read(client_id, receive_buf, BUFFER_LEN);
	if (receive_len <= 0)
	{
		DEBUG("Failed to read from socket");
		perror("read()");
		return NULL;
	}
	DEBUG("Count of received bytes: %d\n", receive_len);

	char* received_from_addr = inet_ntoa(receive_from->sin_addr);
	int receive_from_port = ntohs(receive_from->sin_port);
	DEBUG("Received packet from: %s:%d\n", received_from_addr, receive_from_port);
	DEBUG("Received content: %s\n", receive_buf);
	
	if (detect_host &&
			(send_to_port != receive_from_port || send_to_host != received_from_addr)) {

		this->send_to_host = received_from_addr;
		this->send_to_port = receive_from_port;
		prepareToSend();
	}

	std::string result = std::string(receive_buf);
	result.resize(receive_len);
	
	sending_ready = true;
	if (receiver_callback != 0)
	{
		receiver_callback(result);
	}
	sending_ready = false;
		
	free(receive_from);
	
	usleep(200 * 1000); 	// sleep 200ms so the client can close the connection which avoids TIME_WAIT sockets stacking up on the server side	
	close(client_id);

	return result;
}

bool SocketTCP::bind() {
	DEBUG("Bind to socket %d\n", socket_id);
	
	int struct_size = sizeof(sockaddr);
	sockaddr_in *bind_to = (sockaddr_in*) malloc(struct_size);
	
	bzero(bind_to, sizeof(bind_to));
	bind_to->sin_family = AF_INET;
	bind_to->sin_port = htons(this->receive_from_port);
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



