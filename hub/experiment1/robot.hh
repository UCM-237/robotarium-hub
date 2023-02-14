#ifndef ROBOT_HH
#define ROBOT_HH

#include <string>
/* se crea un clase robot para poder manejar los diferentes parametros
de los robots y las instrucciones que se desean hacer*/
using namespace std;



class Robot
{
    private:
        int ID ;
        string ip;
        string port; 

    public:

        const double R=3.35;//cm

        const double L=12.4;//cm distance between wheels

        static const int  WHEEL_RESOLUTION=20;
        static const int NUMWHEELS=2;
        double A[2][2]={{L/(2*R),1/R}, {-L/(2*R),1/R}};//inverse matrix for compute the angular velocities of every wheel
        void angularWheelSpeed(double (&w_wheel)[NUMWHEELS],double velocity_robot[2]);
        void SetupRobotData(int,string,string);
        void SetupConection(int& ,string& ,string&);
        void linearVelocity();
        void angularVelocity();
        
        void IMU();

};

/*class Wheel: public Robot
{
	public:
    
		
        static const float R;//cm
		static const float L;//cm distance between wheels
        static const int  WHEEL_RESOLUTION=20;
        static const int NUMWHEELS=2;
        float A[2][2]={{L/(2*R),1/R}, {-L/(2*R),1/R}};//inverse matrix for compute the angular velocities of every wheel
        void angularSpeed(float (&w_wheel)[NUMWHEELS],float velocity_robot[2]);
};*/



#endif
