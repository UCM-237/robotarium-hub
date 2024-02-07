#include "localization.cpp"
#include <iostream>
#include <memory>
int main(int argc,char **argv)
{
    Localization *localization = new Localization();
    std::shared_ptr<ringBuffer> buffer = std::make_shared<ringBuffer>();
    std::shared_ptr<AgentCommunication> agentCommunication = std::make_shared<AgentCommunication>();
    agentCommunication->setRingBuffer(buffer);
    agentCommunication->InitCommunication();
    localization->setRingBuffer(buffer);
    if(localization->init(argc,argv))
    {
        if(localization->FindArena())
        {
            agentCommunication->setRobotariumData(localization->getRobotariumData());
            agentCommunication->registerAgent();

        }
        localization->FindRobot();
    }

}