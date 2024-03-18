
//robot2
#include "common.h"
#include "controler.h"
#include "robot.h"
#include "EncoderHandlerInterruptions.h"
#include <math.h>
#include <SimpleKalmanFilter.h>


using namespace std;
uint8_t packetBuffer[256]; //buffer to hold incoming packet

//--------------------------------------------filtro de media movil simple para estabilizar la lectura de rad/s---------------------------------------

MeanFilter<long> meanFilterRight(5);
MeanFilter<long> meanFilterLeft(5);
const double MAX_OPTIMAL_VEL=15;//rad/S
//Time variables
unsigned long previous_timer;
unsigned long timer=10000;


//operation variables
struct appdata operation_send;
struct appdata *server_operation;

//prototypes//
//TODO: DO a reuqest manager for the opeartions and handle serial and mqtt communication
void do_operation(int operation);
void op_saludo();
void op_message();
void op_moveWheel();
void op_StopRobot();
void op_vel_robot();
void isrRight();
void isrLeft();

int option;
int led = 13;

/*define controler*/
controler wheelControlerRight;
controler wheelControlerLeft;
/*define robot*/
robot robot;

/*define mqttClient*/
void setup() {
  robot.pinSetup();
  robot.motorSetup();
  // Interrupciones para contar pulsos de encoder
  pinMode(encoderRight, INPUT_PULLUP);
  pinMode(encoderLeft, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderLeft), isrLeft, RISING);//prepara la entrada del encoder como interrupcion
  attachInterrupt(digitalPinToInterrupt(encoderRight), isrRight, RISING);
  // Se prepara la IMU para poder ser leida
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  // Se empieza con los motores parados
  robot.fullStop();
 
  wheelControlerRight.setControlerParam(0.2, 0.05, 0.009);
  wheelControlerLeft.setControlerParam(0.2, 0.05, 0.009);

  // Comunicacion por puerto serie
  Serial1.begin(9600);//for debuggin
  Serial.begin(9600);
  pinMode(led, OUTPUT); 
}
//define common variables
//TODO: Refactor
int serialOperation = 0;
bool sendDataSerial = true;
bool serialCom = false;
unsigned char *read_ptr; 
unsigned long currentTime, timeAfter = 0;
const unsigned long SAMPLINGTIME= 100;//ms
double wLeft,wRight; // measured angular velocity

const int INIT_FLAG = 112;

void loop() {
  currentTime = millis();
  serialEvent();
  if (serialCom) 
  {
    Serial.println(server_operation->InitFlag);
    if (server_operation->InitFlag == INIT_FLAG)
    {
        Serial.print("operation: \t");
        Serial.println(server_operation->op);
        Serial.println(server_operation->InitFlag);
        do_operation(server_operation->op);
        serialCom = false;
    }
  }

  // TO DO: Refactor
  if(currentTime - timeAfter >= SAMPLINGTIME) {
  
    int auxPWMD = 0, auxPWMI = 0;
    double fD, fI;
    timeStopD = timeStopI = millis();
    //condition for know when the wheel is stoped
    //se define un contador de tiempo para comprobar que las reudas estan paradas
    deltaTimeStopD = timeStopD - timeAfterDebounceD;
    deltaTimeStopI = timeStopI - timeAfterDebounceI;
    meanFilterRight.AddValue(deltaTimeRight);
    meanFilterLeft.AddValue(deltaTimeLeft);
    fI = deltaTimeStopI >= 200 ? 0 : (double)1 / (meanFilterLeft.GetFiltered() * N) * 1000000;
    fD = deltaTimeStopD >= 200 ? 0 : (double)1 / (meanFilterRight.GetFiltered() * N) * 1000000;
    //condicion para que no supere linealidad y se sature.
    //es un filtro para que no de valores ridiculos
    if(fD < double(MAX_OPTIMAL_VEL/2.0/M_PI)) 
    { 
      wRight = 2*M_PI*fD; 
    }
    if(fI < double(MAX_OPTIMAL_VEL/2.0/M_PI)) 
    { 
      wLeft = 2*M_PI*fI; 
    }
    //fin filtro

    if(wRight !=0.0 && wheelControlerRight.getSetPoint() !=0.0) {   
      PWM_Right = constrain(PWM_Right + wheelControlerRight.pid(wRight), MINPWM, MAXPWM);
    }
    if(wLeft !=0.0 && wheelControlerLeft.getSetPoint() !=0.0) {
      PWM_Left = constrain(PWM_Left + wheelControlerLeft.pid(wLeft), MINPWM, MAXPWM);
    }
    //avoid send the same instruction
    if(auxPWMI != PWM_Left){
      // moveWheel(PWM_Left, setpointWLeft, pinMotorI, backI);
      robot.moveLeftWheel(PWM_Left, wheelControlerLeft.getSetPoint(), backI);
      auxPWMI = PWM_Left;
    }
    if(auxPWMD != PWM_Right){
      //moveWheel(PWM_Right, setpointWRight, pinMotorD, backD);
      robot.moveRightWheel(PWM_Right, wheelControlerRight.getSetPoint(), backD);
      auxPWMD = PWM_Right;
    }

    if (1) {
      op_vel_robot();
    }
  }
  timeAfter = currentTime;
}

void do_operation(int operation) {
  switch (operation) {
    case OP_SALUDO:
      op_saludo();
      break;
    case OP_MOVE_WHEEL:
      op_moveWheel();
      //digitalWrite(led, HIGH);
      break;
    case OP_STOP_WHEEL:
      op_StopRobot();
      break;
    case OP_VEL_ROBOT:
      sendDataSerial = true;
      break;
    case OP_CONF_PID:
      op_conf_pid();
    default:
      break;
  }
}

void send(unsigned int operation, byte *data) {
  operation_send.op = operation;
  operation_send.len = sizeof(data);
  Serial1.write((char*)&operation_send.op, 2);
  Serial1.write((char*)&operation_send.len, 2);
  Serial1.write((char*)&data, operation_send.len);
  Serial1.flush();
}

void op_saludo() {
  operation_send.op = OP_SALUDO;

  operation_send.len = sizeof (operation_send.data);  /* len */
  Serial1.write((char*)operation_send.data, operation_send.len + HEADER_LEN);
  Serial1.flush();
//  send(ID, OP_SALUDO, )
}

void op_message() { }

void op_moveWheel() {
  Serial.println("move");
  digitalWrite(led, LOW);
  double setpointWRight = bytesToDouble(&server_operation->data[0]);
  double setpointWLeft = bytesToDouble(&server_operation->data[8]);
  Serial.println(setpointWRight);
  Serial.println(setpointWLeft);
  if(setpointWRight < 1 && setpointWRight > -1) {
    setpointWRight = 0;
  }
  if(setpointWLeft < 1 && setpointWLeft > -1) {
    setpointWLeft = 0;
  }
  if(setpointWRight < 0) {
    setpointWRight = setpointWRight*(-1);
    backD = true;
  } else if(setpointWRight > 0) {
    backD = false;
  }
  if(setpointWLeft < 0) {
    setpointWLeft = setpointWLeft*(-1);
    backI = true;
  }
  else if(setpointWLeft>0) {
    backI = false;
  }
  wheelControlerLeft.setSetPoint(setpointWLeft);
  wheelControlerRight.setSetPoint(setpointWRight);

  PWM_Left=wheelControlerLeft.feedForward();
  PWM_Right=wheelControlerRight.feedForward();
  // feedForwardD();
  // feedForwardI();  
  // moveWheel(PWM_Left, setpointWLeft, pinMotorI, backI);
  // moveWheel(PWM_Right, setpointWRight, pinMotorD, backD);
  robot.moveLeftWheel(PWM_Left, setpointWLeft, backI);
  robot.moveRightWheel(PWM_Right, setpointWRight, backD);
  //take the mean of the last 5 values for measure the angular velocity 
  //of every wheel
  for(int i=0; i<5; i++)
  {
    meanFilterRight.AddValue(deltaTimeRight);
    meanFilterLeft.AddValue(deltaTimeLeft);
  }
}

void op_StopRobot() {
  
  wheelControlerLeft.setSetPoint(0.0);
  wheelControlerRight.setSetPoint(0.0);
  // fullStop(pinMotorI);
  // fullStop(pinMotorD);
  robot.fullStop();
}

void op_vel_robot() {
  //Serial.println(OP_VEL_ROBOT);
  operation_send.InitFlag=INIT_FLAG;
  operation_send.id=1;
  operation_send.op = 5;
  short int a=1;
  doubleToBytes(wRight, &operation_send.data[0]);
  doubleToBytes(wLeft, &operation_send.data[8]);
  //Serial.println(wRight);
  //Serial.println(wI);
  /*if(backD) {
    shortToBytes(a, &operation_send.data[16]);
  }
  if(backI) {
    shortToBytes(a, &operation_send.data[18]);
  }*/
  operation_send.len = sizeof(double)*2;
  /*Serial.println(operation_send.op);
  Serial.println(wRight);
  Serial.print("len \t");
  Serial.println(operation_send.len);*/
  //Serial.println(operation_send.InitFlag);
  Serial1.write((char*)&operation_send.InitFlag,4);
  Serial1.write((char*)&operation_send.id,2);
  Serial1.write((char*)&operation_send.op, 2);
  Serial1.write((char*)&operation_send.len, 2);
  Serial1.write((char*)&operation_send.data, operation_send.len);
  Serial1.flush();
  //send(ID, OP_VEL_ROBOT, &operation_send.data);
}
void op_conf_pid(){
  double kp_right= bytesToDouble(&server_operation->data[0]);
  double ki_right = bytesToDouble(&server_operation->data[8]);
  double kd_right = bytesToDouble(&server_operation->data[16]);

  double kp_left = bytesToDouble(&server_operation->data[24]);
  double ki_left = bytesToDouble(&server_operation->data[32]);
  double kd_left = bytesToDouble(&server_operation->data[40]);

  wheelControlerRight.setControlerParam(kp_right, ki_right, kd_right);
  wheelControlerLeft.setControlerParam(kp_left, ki_left, kd_left);

  Serial.print(kp_right);
  Serial.print(",");
  
  Serial.print(ki_right);
  Serial.print(",");
  
  Serial.print(kd_right);
  Serial.print(",");

  Serial.print(kp_left);
  Serial.print(",");
   Serial.print(ki_left);
  Serial.print(",");
   Serial.println(kd_left);

}




void serialEvent() {
  read_ptr = (unsigned char*)&packetBuffer;
  while (Serial1.available()) {
    *(read_ptr++) = Serial1.read();
    //Serial.readBytes(packetBuffer, MAXDATASIZE);
    server_operation = (struct appdata *)&packetBuffer;
    serialCom = true;
  }
}
