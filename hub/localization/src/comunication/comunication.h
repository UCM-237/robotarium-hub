#pragma once
#include "ringBuffer.cpp"
#include <iostream>
#include <zmqpp/zmqpp.hpp>
#include <nlohmann/json.hpp>
#include <string>
#include <pthread.h> 
#include "common.h"
using json = nlohmann::json;
static const std::string PUBLISH_ENDPOINT = "tcp://*:5557";
static const std::string SUBSCRIBE_ENDPOINT = "tcp://*:5556";
static const std::string HUB_IP = "127.0.0.1";
class AgentCommunication
{
    public:
    /// @brief 
    AgentCommunication();
    ~AgentCommunication();
    void InitCommunication();
    void setRobotariumData(ArenaLimits RobotariumData);
    void setRingBuffer(std::shared_ptr<ringBuffer> buff);
    void registerAgent();
    
    static void *sendData(void* arg);

    std::shared_ptr<ringBuffer> buffer;
    private:
        ArenaLimits RobotariumData;
        bool requestRobotariumData=false;
        // static ringBuffer *buffer;
        
        pthread_t listenThread;
        pthread_t sendThread;

        static void *sendArucoPosition(void *This);
        static void *listenSocket(void *This);
        zmqpp::socket *publisher;
};