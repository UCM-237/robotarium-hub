//#include <string>
#pragma once
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <vector>
#include <sys/wait.h>
#include <atomic>
#include <filesystem>
#include <sys/time.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <pthread.h>
#include <errno.h>

#include <pthread.h> 
#include <time.h> 
#include <math.h> 
#include <fstream>
#include <bits/stdc++.h>
#include <termios.h>    // POSIX terminal control definitions




const int MAXDATASIZE =256; //numero de bytes que se pueden recibir
const int HEADER_LEN = sizeof(unsigned short)*3;
const int MAXROBOTS = 4;
const std::string IP_SERVER = "192.168.78.2";
const std::string SERVERPORT = "4240";

struct appdata{

        unsigned short id; //identificador
        unsigned short op; //codigo de operacion
        unsigned short len;                       /* longitud de datos */
        unsigned char data [MAXDATASIZE-HEADER_LEN];//datos
        //nota¡ actualizar char data a string o puntero para que sea mas versatil.


};

struct appdata *operation_recv;//message received of operation 
struct appdata operation_send;//struct for message send
