#pragma once

#include<stdio.h>
#include<winsock2.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <ctime>

#pragma comment(lib,"ws2_32.lib")
#define BUFLEN 512  
#define PORT 5555   
class UDPserver
{
public:
	UDPserver();
	~UDPserver();
	std::string getBuffer();
	


private:
	SOCKET s;
	struct sockaddr_in server, si_other;
	int slen, recv_len;
	char buf[BUFLEN];
	WSADATA wsa;
	void recvData();
};

