/**************************************************************************//**
 *
 * @file SocketUDP.cpp
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
 

#include <libtaurob_tools/SocketUDP.h>

#undef DEBUG
#define DEBUG if (false) printf

bool SocketUDP::isDebugEnable = false;


/* function to be executed by the new thread */
void* async_receive_loop_udp(void* context) {
	SocketUDP* socket = (SocketUDP*) context;

	while (!socket->is_closing) {
		socket->receive();
	}
	socket->is_closing = false;

	DEBUG("Closing Socket\n");
	pthread_exit(NULL);
	return NULL;
}

SocketUDP::SocketUDP(int bind_to_local_port) : receiver_callback(null) {
	this->detect_host = true; 	// detect_host means that if something was received, 
								// the next call to send will send the data to the host we just received from.
								// since this is the constructor intended for server sockets, it is enabled here.
	this->bind_to_local_port = bind_to_local_port;
	socket_id = createSocket();
	send_to = 0;
}

SocketUDP::SocketUDP(std::string send_to_host, int send_to_port) : receiver_callback(null) {
	this->detect_host = false;
	this->send_to_host = send_to_host.data();
	this->send_to_port = send_to_port;
	this->bind_to_local_port = -1;
	send_to = 0;

	socket_id = createSocket();
	prepareToSend();
}

int SocketUDP::createSocket() {
	DEBUG("Create Socket\n");
	int socket_id = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket_id == -1) {
		perror("socket");
	}
	return socket_id;
}

SocketUDP::~SocketUDP() {
	if (send_to != 0) free(send_to);
	::close(socket_id);
}

bool SocketUDP::prepareToSend() {
	DEBUG("Prepare to send \n");
	DEBUG("Host %s \n", this->send_to_host.data());
	int struct_size = sizeof(struct sockaddr);
	
	if (send_to != 0) free(send_to);
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

bool SocketUDP::start_listen(IUdpReceiver * receiver_callback) {
	this->receiver_callback = receiver_callback;
	this->receiver_callback_fn = null;
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
	int status = pthread_create(&p_thread, NULL, &async_receive_loop_udp, this);
	if (status < 0) {
		perror("pthread_create failed");
		return false;
	}
	is_listening = true;
	DEBUG("Socket %d is listening\n", socket_id);
	return true;
}

bool SocketUDP::start_listen(void (*callback)(std::string data, char* from_remote_ip, int from_remote_port, int from_local_port)) {
	this->receiver_callback = null;
	this->receiver_callback_fn = callback;
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
	int status = pthread_create(&p_thread, NULL, &async_receive_loop_udp, this);
	if (status < 0) {
		perror("pthread_create failed");
		return false;
	}
	is_listening = true;
	DEBUG("Socket %d is listening\n", socket_id);
	return true;
}

void SocketUDP::stop_listen() {
	is_closing = true;
	//pthread_kill(p_thread, SIGTERM);
	usleep(250*1000); 	// 250ms to stop the async receive thread

	is_listening = false;
	DEBUG("Socket %d is close\n", socket_id);
}

bool SocketUDP::send(std::string data) {
	if (send_to == NULL) {
		DEBUG("Socket is NOT ready to send\n");
		return false;
	}

	char* send_buf = (char *) data.data();
	int send_buf_len = data.length();
	int struct_size = sizeof(struct sockaddr);

	DEBUG("Send: %s\n", send_buf);
	int send_result = sendto(socket_id, send_buf, send_buf_len, 0, (struct sockaddr *) send_to, struct_size);
	bool success = send_result != -1;
	if (!success) {
		perror("sendto()");
		return false;
	}
	return true;
}

std::string SocketUDP::receive() 
{
	int struct_size = sizeof(struct sockaddr);
	DEBUG("Receive from Socket Id: %d; struct_size is %d\n", socket_id, struct_size);
	sockaddr_in *receive_from = (sockaddr_in*) malloc(struct_size);

	char receive_buf[BUFFER_LEN];
	int receive_len = recvfrom(socket_id, receive_buf, BUFFER_LEN, 0, (sockaddr *) receive_from, (socklen_t *) &struct_size);

	DEBUG("Count of received bytes: %d\n", receive_len);
	DEBUG("Received content:\n%s\n\n", receive_buf);

	char* received_from_addr = inet_ntoa(receive_from->sin_addr);
	int receive_from_port = ntohs(receive_from->sin_port);
	DEBUG("Received packet from: %s:%d\n", received_from_addr, receive_from_port);

	if (receive_len == -1) {
		DEBUG("Error in received packet");
		DEBUG("Error data: %s\n", receive_buf);
		perror("recvfrom()");
		return NULL;
	}

	if (detect_host &&
			(send_to_port != receive_from_port || send_to_host != received_from_addr)) {

		this->send_to_host = received_from_addr;
		this->send_to_port = receive_from_port;
		prepareToSend();
	}

	std::string result = std::string(receive_buf);
	result.resize(receive_len);

	if (is_closing == false) 
	{
		if (receiver_callback != null)
			receiver_callback->On_string_received(result, received_from_addr, receive_from_port, this->bind_to_local_port);
		if (receiver_callback_fn != null)
			receiver_callback_fn(result, received_from_addr, receive_from_port, this->bind_to_local_port);
	}
	
	free(receive_from);
	
	return result;
}

bool SocketUDP::bind() {
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

