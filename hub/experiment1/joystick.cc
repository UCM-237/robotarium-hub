#include "common.hh"
#include <SDL2/SDL.h>

#include <cinttypes>
//g++ joystick.cpp -lsdl2
void SetupRobots();
const int W=5;
const int V=0;
using namespace std;
using namespace tinyxml2;
Robot robot1,robot2,robot3,robot4;//se define la clase para los distintos robots.
UDP comRobot1,comRobot2,comRobot3;//comunication udp
const Uint8 *keyboard_state_array = SDL_GetKeyboardState(NULL);


int main(int argc, char *argv[]) {

    SetupRobots();//prepare the network data of every robot
    SDL_Init(SDL_INIT_JOYSTICK);
    string ip,port;
    int id;
    robot2.SetupConection(id,ip,port);
    cout<<ip<<endl;
    cout<<port<<endl;
    comRobot2.initTalkerSocket(ip,port);

    SDL_Joystick *joystick;
    joystick = SDL_JoystickOpen(0);

    printf("Name: %s\n", SDL_JoystickNameForIndex(0));
    int axis;
    int value;
    double v, w,auxV=0,auxW=0;
    double velocity_robot[2];
    double angularWheel[2];
    SDL_Event event;
    while (1) {
        memset (operation_send.data, '\0', MAXDATASIZE-HEADER_LEN);
       
        SDL_WaitEvent(&event);
        if (event.type == SDL_QUIT) {
            break;
        } else if (event.type == SDL_KEYDOWN) {
            if (event.key.keysym.sym == SDLK_ESCAPE) {
                break;
            }
        } else if (event.type == SDL_JOYAXISMOTION) {
            axis=event.jaxis.axis;
            value=event.jaxis.value;
            //printf("axis: %i %i\n", axis,value);
            switch (axis)
            {
                case V:
                    v=0.002161*value;
                    if(value<2 && value >-2){
                        v=0;
                    }
                break;

                case W:
                    w=0.001565*value/1.5;
                    if(value<1 && value >-1){
                        w=0;
                    }
                break;
            }

        } else if (event.type == SDL_JOYBUTTONDOWN) {
            //printf("button: %i\n", event.jbutton.button);
        }
            if(w == 0 && v==0){
                operation_send.op=OP_STOP_WHEEL;
               
            }
            else{
            printf("w: %f, v: %f \n",w,v);
                
                velocity_robot[0]=w;
                velocity_robot[1]=v;
                robot2.angularWheelSpeed(angularWheel,velocity_robot);
                operation_send.op=OP_MOVE_WHEEL;
                doubleToBytes(angularWheel[0], &operation_send.data[0]);
                doubleToBytes(angularWheel[1], &operation_send.data[8]);
            }
            auxV=v;
            auxW=w;
            //send angular Wheel
            operation_send.len = strlen ((char*)operation_send.data);
            comRobot2.SendTalkerSocket((char*)&operation_send,operation_send.len+HEADER_LEN);
            cout<<angularWheel[0]<<","<<angularWheel[1]<<endl;
        
    }

    SDL_JoystickClose(joystick);
    SDL_Quit();
    return 0;
}

void SetupRobots()
{
    // Read the sample.xml file
    XMLDocument Robotdoc;
    Robotdoc.LoadFile( "robots_info.xml" );

    XMLNode* Robotarium =Robotdoc.FirstChild();
    XMLElement *robot=Robotarium->FirstChildElement("robot");
    int i=0;
    while(robot !=NULL)
    {
        
        XMLElement *robotChild=robot->FirstChildElement("ID");
        int ID;
        robotChild->QueryIntText(&ID);
        cout<<"ID:"<<ID<<endl;
    
         robotChild=robot->FirstChildElement("IP");
         const char* ip=robotChild->GetText();
         string ss=ip;
         cout<<"ip:"<<ip<<endl;

        robotChild=robot->FirstChildElement("PORT");
        const char* port=robotChild->GetText();
        string p=port;
        cout<<"puerto:"<<p<<endl;
      
        robot=robot->NextSiblingElement("robot"); 
        switch (i)
        {
            case 0:
                robot1.SetupRobotData(ID,ss,p);
                break;
            case 1:
                robot2.SetupRobotData(ID,ss,p);
                break;
            case 2:
                robot3.SetupRobotData(ID,ss,p);
                break;
             case 3:
                robot4.SetupRobotData(ID,ss,p);
                break;

        }       
        i++;   
    }
}