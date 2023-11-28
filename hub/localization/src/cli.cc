#include "localization.cpp"
int main(int argc,char **argv)
{
    Localization *localization = new Localization();
    if(localization->init(argc,argv))
    {
        localization->FindArena();
        localization->registerAgent();
        localization->initRingBuffer();
        localization->FindRobot();
    }

}