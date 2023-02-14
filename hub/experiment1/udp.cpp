#include "udp.hh"
#include <errno.h>



int UDP::initListenSocket(std::string ipServ, std::string portServ){

    struct addrinfo hints;
    
    
    int rv;
    
    
    memset (&hints,0,sizeof(hints));
    hints.ai_family=AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;
   
    const char *ip=ipServ.c_str();
    const char *port=portServ.c_str();

    if ((rv = getaddrinfo(ip, port, &hints, &servinfo)) != 0) {
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return -1;
    }
    for(listener=servinfo;listener != NULL; listener=listener->ai_next){
        if((sockListener=socket(listener->ai_family, listener->ai_socktype,listener->ai_protocol)) == -1){
            //perror("listener: socket");
            continue;
        }
        if(bind(sockListener, listener->ai_addr, listener->ai_addrlen) == -1){
            close(sockListener);
            //perror("listener : bind");
            continue;
        }
        
        break;
    }
    
    if(listener == NULL){
        std::cout<<"listener: failed to bind socket"<<std::endl;
        return -1;
    }
    addr_len = sizeof their_addr;
    return 0;
}

void UDP::RecvListenSocket(char *buf,size_t len,int &numbytes){
    
    if ((numbytes = recvfrom(sockListener,buf, len , 0, (struct sockaddr *)&their_addr, &addr_len)) == -1) {
        perror("recvfrom");
    }
    cout<<len<<endl;


}


int UDP::initTalkerSocket(std::string ipClient, std::string portClient){
    struct addrinfo hints;
    
    int rv;
    memset(&hints, 0, sizeof hints);
    hints.ai_family=AF_UNSPEC;
    hints.ai_socktype = SOCK_DGRAM;

    const char *ip=ipClient.c_str();
    const char *port=portClient.c_str();
    
    if ((rv = getaddrinfo( ip, port, &hints, &clientinfo)) != 0) {
        //fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return 1;
    }
    // loop through all the results and make a socket
    for(talker = clientinfo; talker != NULL; talker = talker->ai_next) {
        if ((socktalker = socket(talker->ai_family, talker->ai_socktype,
        talker->ai_protocol)) == -1) {
        //perror("talker: socket");
        continue;
        }
        break;
        }
        if (talker == NULL) {
        //fprintf(stderr, "talker: failed to create socket\n");
        return 2;
    }
    
    return 0;
}

void UDP::SendTalkerSocket(char *buf,size_t len){
    
    int numbytes;
    
    sendto(socktalker,buf,len, 0,talker->ai_addr, talker->ai_addrlen);
   
}
void UDP::RecvTalkerSocket(char *buf,size_t len,int &numbytes){
    
    if ((numbytes = recvfrom(socktalker,buf, len-1 , 0, (struct sockaddr *)&their_addr, &addr_len)) == -1) {
        perror("recvfrom");
    }


}
UDP::~UDP(){

    close(socktalker);
    close(sockListener);
    
}
