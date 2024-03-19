#include "comunication.h"
//ringBuffer AgentCommunication::buffer;  // Definition of the static member

AgentCommunication::AgentCommunication()
{
    
    
}
AgentCommunication::~AgentCommunication()
{
    // Esperar a que los hilos terminen
    pthread_join(listenThread, NULL);
    pthread_join(sendThread, NULL);
}

void AgentCommunication::InitCommunication()
{


     // Crear los hilos
    pthread_create(&listenThread, NULL, &AgentCommunication::listenSocket, this);
    pthread_create(&sendThread, NULL, &AgentCommunication::sendArucoPosition, this);

    
}

void AgentCommunication::setRobotariumData(ArenaLimits RobotariumData)
{
    this->RobotariumData = RobotariumData;
}

void AgentCommunication::setRingBuffer(std::shared_ptr<ringBuffer> buff)
{
    this->buffer =buff;
}

void AgentCommunication::registerAgent()
{
    //Register to the control hub
    std::cout<<"Registering Localization"<<std::endl;
    zmqpp::context context;

    // generate a push socket
    zmqpp::socket_type type = zmqpp::socket_type::request;
    zmqpp::socket control (context, type);
    std::cout << "connecting" << std::endl;
    control.connect("tcp://"+HUB_IP+":5555");

    json message;
    message["operation"] = "hello";
    message["source_id"] = "Camara_0";
    message["payload"]["url"] = PUBLISH_ENDPOINT;
    message["timestamp"] = 1000 * time(nullptr);
    std::string jsonStr = message.dump();

    zmqpp::message_t zmqMessage;
    // memcpy(zmqMessage.data(), jsonStr.c_str(), jsonStr.size());
    zmqMessage<<jsonStr;
    control.send(zmqMessage);
    std::string response;
    control.receive(response);
    //pass string to json and find ok
    json responseJson = json::parse(response);  
    if(responseJson["result"] == "ok")
    {
        std::cout<<"Registered"<<std::endl;
        control.close();
    }
    else
    {
        std::cout<<"Error registering"<<std::endl;
    }
    
    
}

void *AgentCommunication::sendData(void *arg)
{
    AgentCommunication *agent = (AgentCommunication*)arg;
    //const string endpoint = "tcp://


}
void *AgentCommunication::sendArucoPosition(void *This)
{
    AgentCommunication *agent = (AgentCommunication*)This;
    const std::string endpoint = "tcp://"+HUB_IP+":5555"; //5555
    // initialize the 0MQ context
    // zmqpp::context context;

    // // generate a push socket
    // zmqpp::socket_type type = zmqpp::socket_type::publish;
    // zmqpp::socket publisher (context, type);
   
    // // open the connection
    // std::cout << "Binding to " << PUBLISH_ENDPOINT << "..." << std::endl;
    // publisher.bind(PUBLISH_ENDPOINT);
    
     //initialize the 0MQ context
    zmqpp::context context;

    // generate a push socket
    zmqpp::socket_type type = zmqpp::socket_type::publish;
    zmqpp::socket newPublisher (context, type);
    
    // open the connection
    std::cout << "Binding to " << PUBLISH_ENDPOINT << "..." << std::endl;
    newPublisher.bind(PUBLISH_ENDPOINT);
    
    //while(arucoInfo.size()<=0);//the thread stop it until an aruco is detected
    std::string data2;
    //main code for read variables
    std::string topic ="data";

    record_data data;
    json message;
    json position;
    zmqpp::message_t ztopic;
    std::string jsonStr;
    zmqpp::message_t zmqMessage;
     while (true) {
    
        if(agent->RobotariumData.x.size()>0 && agent->requestRobotariumData)
        {
            message["topic"]="ArenaSize";
            message["source_id"] = "Camara_0";
            
            position["arenaSize"]["x1"] =agent->RobotariumData.x.at(0);
            position["arenaSize"]["y1"] =agent->RobotariumData.y.at(0);
            position["arenaSize"]["x2"] =agent->RobotariumData.x.at(1);
            position["arenaSize"]["y2"] =agent->RobotariumData.y.at(1);
            position["arenaSize"]["x3"] =agent->RobotariumData.x.at(2);
            position["arenaSize"]["y3"] =agent->RobotariumData.y.at(2);
            position["arenaSize"]["x4"] =agent->RobotariumData.x.at(3);
            position["arenaSize"]["y4"] =agent->RobotariumData.y.at(3);
            message["payload"] = position;
            jsonStr = message.dump();
            zmqMessage<<jsonStr;

            newPublisher.send(topic,0);
            newPublisher.send(zmqMessage);
            agent->requestRobotariumData = false;
        }
        else
        {
            data = agent->buffer->pop();
            std::string id = std::to_string(data.id);
            std::string x = std::to_string(data.x);
            std::string y = std::to_string(data.y);
            std::string yaw = std::to_string(data.yaw);
            std::string sep = "/";
            data2=  x+ sep +y+sep+yaw;
            //std::cout<<id+sep+data2<<std::endl;
            
            position[id]["x"] =x;
            position[id]["y"] =y;
            position[id]["yaw"] =yaw;
            
            ztopic<<topic;
            
            message["topic"]="position";
            message["source_id"] = "Camara_0";
            message["payload"] = position;
            message["timestamp"] = 1000 * time(nullptr);
            jsonStr = message.dump();
            zmqMessage;
            
            
            zmqMessage<<jsonStr;
            
            newPublisher.send(topic);
            newPublisher.send(zmqMessage);
        
        }
        usleep(50*1000);
        message.clear();
        position.clear();
    }
    
    
    agent->publisher->close();
    pthread_exit(NULL);
}


void *AgentCommunication::listenSocket(void *This)
{
    AgentCommunication *agent = (AgentCommunication*)This;
    //generate subscriber socket
    //initialize the 0MQ context
    zmqpp::context context;
    zmqpp::socket_type type2 = zmqpp::socket_type::subscribe;
    zmqpp::socket newSubscriber (context, type2);
    newSubscriber.subscribe("");
    //subscriber connect
    std::cout << "Connecting to " << SUBSCRIBE_ENDPOINT << "..." << std::endl;
    newSubscriber.connect(SUBSCRIBE_ENDPOINT);
    while(true)
    {
            // Receive (blocking call)
        zmqpp::message message;
        newSubscriber.receive(message);
        std::string text;
        message >> text;
        //split to find localization and RobotariumData

        std::string delimiter = "/";
        size_t pos = 0;
        std::string token;
        while ((pos = text.find(delimiter)) != std::string::npos) {
            token = text.substr(0, pos);
            text.erase(0, pos + delimiter.length());
            if (text == "RobotariumData")
            {
                std::cout<<"Requested RObotarium Data"<<std::endl;
                agent->requestRobotariumData = true;
            }
        }
    }
}
