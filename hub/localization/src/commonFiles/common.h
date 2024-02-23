//#include <string>
#pragma once
#include <string>

#include <zmqpp/zmqpp.hpp>
    // POSIX terminal control definitions




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

struct dataToSend{
        std::string topic;
        zmqpp::message_t zmqMessage;
};