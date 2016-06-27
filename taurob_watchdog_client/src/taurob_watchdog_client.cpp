/**************************************************************************//**
 *
 * @file taurob_watchdog_client.cpp
 * @author Martin Schenk, taurob GmbH
 * @date 30 May 2016
 * @brief Watchdog client for taurob ROS driver nodes
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

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>
#include <strings.h>
#include <termios.h>

#define WATCHDOG_TOKEN "TWF\n"
#define WATCHDOG_FEED_INTERVAL 50

pthread_t p_thread;
int feed_watchdog_thread_running = 0;

char** args;

void *feed_watchdog_thread(void* data_ptr)
{
	int sockfd,n;
	struct sockaddr_in servaddr, cliaddr;
	
	sockfd = socket(AF_INET,SOCK_DGRAM,0);

	bzero(&servaddr,sizeof(servaddr));
	servaddr.sin_family = AF_INET;
	servaddr.sin_addr.s_addr = inet_addr(args[1]);
	servaddr.sin_port = htons(19090);
	
	while (feed_watchdog_thread_running != 0)
	{
		sendto(sockfd, WATCHDOG_TOKEN, 4, 0,
			 (struct sockaddr *)&servaddr, sizeof(servaddr));
		
		usleep(WATCHDOG_FEED_INTERVAL * 1000);
	}
}

int main(int argc, char**argv)
{
	static struct termios oldt, newt;

	if (argc != 2)
	{
	  printf("usage:  %s <IP address>\n", argv[0]);
	  return 1;
	}
	args = argv;

	int status = pthread_create(&p_thread, NULL, &feed_watchdog_thread, NULL);
	if (status < 0) {
		printf("pthread_create failed\n");
		return status;
	}
	feed_watchdog_thread_running = 1;
	
	// modify terminal flags to get character without buffering
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;

    newt.c_lflag &= ~(ICANON);          // disable ICANON, i.e., buffering until '\n'
    tcsetattr( STDIN_FILENO, TCSANOW, &newt); // TCSANOW: change attributes immediately.
	
	printf("Waiting for emergency stop (press any key to trigger)...\n");
	getchar();
	printf("Emergency stop pressed, terminating watchdog feeder.\n");
	
	feed_watchdog_thread_running = 0;	
	
    /*restore the old settings*/
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
	
	return 0;
}
