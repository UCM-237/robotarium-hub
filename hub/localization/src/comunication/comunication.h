#pragma once
#include "ringBuffer.cpp"
#include <iostream>
#include <zmqpp/zmqpp.hpp>
#include <nlohmann/json.hpp>
#include <string>
using json = nlohmann::json;
static const std::string PUBLISH_ENDPOINT = "tcp://*:5557";
static const std::string HUB_IP = "127.0.0.1";
class AgentCommunication
{
    public:
    /// @brief 
    AgentCommunication();
    ~AgentCommunication();
    void setRobotariumData(record_data RobotariumData);
    void setRingBuffer(ringBuffer *buff);
    void registerAgent();
    static void *sendArucoPosition(void *arg);
    private:
        record_data RobotariumData;
        static ringBuffer *buffer;
};