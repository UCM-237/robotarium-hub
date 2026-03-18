// Programa principal para la localización y comunicación en Robotarium
#include "localization/localization.cpp"
#include <iostream>
#include <memory>

// Función principal
int main(int argc,char **argv)
{
    // Crear instancia de Localization
    Localization *localization = new Localization();
    // Crear buffer de anillo compartido
    std::shared_ptr<ringBuffer> buffer = std::make_shared<ringBuffer>();
    // Crear instancia de comunicación de agente
    std::shared_ptr<AgentCommunication> agentCommunication = std::make_shared<AgentCommunication>();
    // Configurar el buffer en la comunicación
    agentCommunication->setRingBuffer(buffer);
    // Inicializar la comunicación
    agentCommunication->InitCommunication();
    // Configurar el buffer en la localización
    localization->setRingBuffer(buffer);
    // Si la localización se inicializa correctamente
    if(localization->init(argc,argv))
    {
        // Si encuentra el arena
        if(localization->FindArena())
        {
            // Establecer datos del robotarium en la comunicación
            agentCommunication->setRobotariumData(localization->getRobotariumData());
            // Registrar el agente
            agentCommunication->registerAgent();

        }
        // Encontrar el robot
        localization->FindRobot();
    }
    // Imprimir mensaje de finalización
    std::cout<<"Finishing"<<std::endl;  
    return 0;
}
