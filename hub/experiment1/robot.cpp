#include "robot.hh"

void Robot::SetupRobotData(int a,string b, string c){
    ID=a;
    ip=b;
    port=c;

}

void Robot::SetupConection(int &id,string &IP,string &P){
    id=ID;
    IP=ip;
    P=port;

}
void Robot::angularWheelSpeed(double (&w_wheel)[NUMWHEELS],double velocity_robot[2])
{
	int fila=2;
	int columna =2;
	double aux=0;
	for (int i =0;i<NUMWHEELS;i++){
		w_wheel[i]=0;
	}
		
  	for(int i=0; i <fila;i++){
		for(int j=0;j<columna;j++){
			aux+=(A[i][j] * velocity_robot[j]);
		}
		w_wheel[i]=aux;
		if(w_wheel[i]<0 && w_wheel[i]>-6){
			w_wheel[i]=0.0;
		}
		else if(w_wheel[i]>0 && w_wheel[i]<6)
		{
			w_wheel[i]=0.0;
		}
		aux=0;
	}

}

/*void Wheel::angularSpeed(float (&w_wheel)[NUMWHEELS],float velocity_robot[2])
{
	int fila=2;
	int columna =2;
	float aux=0;
  	for(int i=0; i <fila;i++){
		for(int j=0;j<columna;j++){
			aux=+A[i][j] *velocity_robot[j];
		}
		w_wheel[i]=aux;
		aux=0;
	}

}*/
