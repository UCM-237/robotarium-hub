#pragma once    
#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>

class MQTTClient
{
    public:
        MQTTClient();
        ~MQTTClient();
        void connect();
    private:
        WiFiClient wifiClient;
        MqttClient mqttClient;
};