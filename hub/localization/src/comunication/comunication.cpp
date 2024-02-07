#include "comunication.h"
//ringBuffer AgentCommunication::buffer;  // Definition of the static member

AgentCommunication::AgentCommunication()
{
    
    
}
AgentCommunication::~AgentCommunication()
{
    // Esperar a que los hilos terminen
    //pthread_join(listenThread, NULL);
    pthread_join(sendThread, NULL);
}

void AgentCommunication::InitCommunication()
{
    //initialize the 0MQ context
    zmqpp::context context;

    // generate a push socket
    zmqpp::socket_type type = zmqpp::socket_type::publish;
    zmqpp::socket newPublisher (context, type);
    
    // open the connection
    std::cout << "Binding to " << PUBLISH_ENDPOINT << "..." << std::endl;
    newPublisher.bind(PUBLISH_ENDPOINT);
    this->publisher = &newPublisher;

     // Crear los hilos
    //pthread_create(&listenThread, NULL, &AgentCommunication::listenSocket, this);
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
    control.connect("tcp://127.0.0.1:5555");

    json message;
    json size;
    size["x1"] =this->RobotariumData.x.at(0);
    size["y1"] =this->RobotariumData.y.at(0);
    size["x2"] =this->RobotariumData.x.at(1);
    size["y2"] =this->RobotariumData.y.at(1);
    size["x3"] =this->RobotariumData.x.at(2);
    size["y3"] =this->RobotariumData.y.at(2);
    size["x4"] =this->RobotariumData.x.at(3);
    size["y4"] =this->RobotariumData.y.at(3);
    message["operation"] = "hello";
    message["source_id"] = "Camara_0";
    message["payload"]["url"] = "tcp://127.0.0.1:5557";
    message["payload"]["ArenaSize"] = size;
    message["timestamp"] = 1000 * time(nullptr);
    std::string jsonStr = message.dump();

    zmqpp::message_t zmqMessage;
    // memcpy(zmqMessage.data(), jsonStr.c_str(), jsonStr.size());
    zmqMessage<<jsonStr;
    control.send(zmqMessage);


    zmqpp::message_t response;
    
    control.close();
}

void *AgentCommunication::sendData(zmqpp::message_t zmqMessage, std::string topic)
{

}

void *AgentCommunication::sendArucoPosition(void *This)
{
    AgentCommunication *agent = (AgentCommunication*)This;
    //const string endpoint = "tcp://127.0.0.1:4242"; //5555
    // // initialize the 0MQ context
    // zmqpp::context context;

    // // generate a push socket
    // zmqpp::socket_type type = zmqpp::socket_type::publish;
    // zmqpp::socket publisher (context, type);
   
    // // open the connection
    // std::cout << "Binding to " << PUBLISH_ENDPOINT << "..." << std::endl;
    // publisher.bind(PUBLISH_ENDPOINT);
    

    
    //while(arucoInfo.size()<=0);//the thread stop it until an aruco is detected
    std::string data2;
    //main code for read variables
    std::cout<<"hola"<<std::endl;
     while (true) {
    
        record_data data = agent->buffer->pop();
        std::string topic ="data";
        
        std::string id = std::to_string(data.id);
        std::string x = std::to_string(data.x);
        std::string y = std::to_string(data.y);
        std::string yaw = std::to_string(data.yaw);
        
        std::string sep = "/";
        data2=  x+ sep +y+sep+yaw;
        std::cout<<id+sep+data2<<std::endl;
        json message;
        json position;
    
        position[id]["x"] =x;
        position[id]["y"] =y;
        position[id]["yaw"] =yaw;
        zmqpp::message_t ztopic;
        ztopic<<topic;
        
        message["topic"]="position";
        //message["operation"] = "position";
        message["source_id"] = "Camara_0";
        message["payload"] = position;
        message["timestamp"] = 1000 * time(nullptr);
        std::string jsonStr = message.dump();
        zmqpp::message_t zmqMessage;
        
        
        zmqMessage<<jsonStr;
        
        agent->publisher->send(topic);
        agent->publisher->send(zmqMessage);
        
        
        usleep(100*1000);
        message.clear();
        position.clear();
        message["topic"]="ArenaSize";
        message["source_id"] = "Camara_0";
        position["arenaSize"]["x"]=agent->RobotariumData.x;
        position["arenaSize"]["y"] =agent->RobotariumData.y;
        message["payload"] = position;
        jsonStr = message.dump();
        zmqMessage<<jsonStr;
        
        agent->publisher->send(topic);
        agent->publisher->send(zmqMessage);
    }
    
    
    agent->publisher->close();
    pthread_exit(NULL);
}


void *AgentCommunication::listenSocket(void *This)
{
    AgentCommunication *agent = reinterpret_cast<AgentCommunication*>(This);
    //const string endpoint = "tcp://
}
