//#include <string>
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

#include "misc.cc"

#include <pthread.h> 
#include <time.h> 
#include <math.h> 
#include <fstream>
#include <bits/stdc++.h>
#include <termios.h>    // POSIX terminal control definitions
#include "tinyxml2.cpp"
#include "robot.cpp"
#include "udp.cpp"



const int MAXDATASIZE =256; //numero de bytes que se pueden recibir
const int HEADER_LEN = sizeof(unsigned short)*3;
const int MAXROBOTS = 4;
const string IP_SERVER = "192.168.78.2";
const string SERVERPORT = "4240";

struct appdata{

        unsigned short id; //identificador
        unsigned short op; //codigo de operacion
        unsigned short len;                       /* longitud de datos */
        unsigned char data [MAXDATASIZE-HEADER_LEN];//datos
        //nota¡ actualizar char data a string o puntero para que sea mas versatil.


};
//operacion error
#define OP_ERROR            0xFFFF
//operaciones requeridas por central
#define OP_SALUDO           0x0001
#define OP_MOVE_WHEEL       0x0002
#define OP_STOP_WHEEL       0x0003
#define OP_VEL_ROBOT        0X0005//devuelve la velocidad de las ruedas en rad/s
#define OP_IMU              0x0006
#define OP_POSITION         0x0008
//operaciones cliente
#define OP_MESSAGE_RECIVE   0x0004
//broadcast
#define OP_BROADCAST        0x9999
#define OP_STOP_SERIAL      0X0007

struct appdata *operation_recv;//message received of operation 
struct appdata operation_send;//struct for message send
