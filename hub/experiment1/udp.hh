#ifndef UDP_HH
#define UDP_HH

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>
#include <string>
#include <unistd.h>


class UDP{

    
    public:
    int socktalker, sockListener;
    struct addrinfo *servinfo,*clientinfo;
    struct addrinfo *talker,*listener;
    socklen_t addr_len;
    struct sockaddr_storage their_addr;
    struct addrinfo hints;
    
    
    int initListenSocket(std::string ip, std::string port);
    void RecvListenSocket(char *buf,size_t len,int &numbytes);
    int initTalkerSocket(std::string ipClient, std::string portClient);
    void SendTalkerSocket(char *buf,size_t len);
    void RecvTalkerSocket(char *buf,size_t len,int &numbytes);
    ~UDP();


};




#endif