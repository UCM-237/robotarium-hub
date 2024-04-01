
//robot2
#include "common.h"
#include <SimpleKalmanFilter.h>

using namespace std;
uint8_t packetBuffer[256]; //buffer to hold incoming packet

//--------------------------------------------filtro de media movil simple para estabilizar la lectura de rad/s---------------------------------------
MeanFilter<long> meanFilterD(10);
MeanFilter<long> meanFilterI(10);
//Time variables
unsigned long previous_timer;
unsigned long timer=10000;


//operation variables
struct appdata operation_send;
struct appdata *server_operation;

//prototypes//
void do_operation(int operation);
void op_saludo();
void op_message();
void op_moveWheel();
void op_StopWheel();
void op_vel_robot();
void motorSetup();
void moveForward(const int pinMotor[3], int speed);
void moveBackward(const int pinMotor[3], int speed);
void fullStop(const int pinMotor[3]);
void moveWheel(int pwm,double w, const int pinMotor[3],bool back);
void isrRight();
void isrLeft();
int pidD(double wD);
int pidI(double wI);
void feedForwardD();
void feedForwardI();
void tokenize(const string s, char c,vector<string>& v);
int option;
int led = 13;

void setup() {
  motorSetup();
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
  fullStop(pinMotorD);
  fullStop(pinMotorI);
  // Comunicacion por puerto serie
  Serial1.begin(9600);//for debuggin
  Serial.begin(9600);
  pinMode(led, OUTPUT); 
}
  
int serialOperation = 0;
bool sendDataSerial = true;
bool serialCom = false;
unsigned char *read_ptr; 

void loop() {
  currentTime = micros();
  delay(100);
  serialEvent();
  if (serialCom) {
    Serial.println(server_operation->InitFlag);
    if (server_operation->InitFlag == INIT_FLAG){
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
    meanFilterD.AddValue(deltaTimeD);
    meanFilterI.AddValue(deltaTimeI);
    fI = deltaTimeStopI >= 200 ? 0 : (double)1 / (meanFilterI.GetFiltered() * N) * 1000000;
    fD = deltaTimeStopD >= 200 ? 0 : (double)1 / (meanFilterD.GetFiltered() * N) * 1000000;
    //condicion para que no supere linealidad y se sature.
    //es un filtro para que no de valores ridiculos
    if(fD < 15/2/3.14) { wD = 2*3.14*fD; }
    if(fI < 15/2/3.14) { wI = 2*3.14*fI; }
    //fin filtro
    if(wD !=0.0 && setpointWD !=0.0) {   
      PWM_D = constrain(PWM_D + pidD(wD), MINPWM, MAXPWM);
    }
    if(wI !=0.0 && setpointWI !=0.0) {
      PWM_I = constrain(PWM_I + pidI(wI), MINPWM, MAXPWM);
    }
    if(auxPWMI != PWM_I){
      moveWheel(PWM_I, setpointWI, pinMotorI, backI);
      auxPWMI = PWM_I;
    }
    if(auxPWMD != PWM_D){
      moveWheel(PWM_D, setpointWD, pinMotorD, backD);
      auxPWMD = PWM_D;
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
      digitalWrite(led, HIGH);
      break;
    case OP_STOP_WHEEL:
      op_StopWheel();
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
  Serial.println("moveWheel");
  digitalWrite(led, LOW);
  setpointWD = bytesToDouble(&server_operation->data[0]);
  setpointWI = bytesToDouble(&server_operation->data[8]);
  Serial.println(setpointWD);
  Serial.println(setpointWI);
  if(setpointWD < 1 && setpointWD > -1) {
    setpointWD = 0;
  }
  if(setpointWI < 1 && setpointWI > -1) {
    setpointWI = 0;
  }
  if(setpointWD < 0) {
    setPointGWD = setpointWD;
    setpointWD = setpointWD*(-1);
    backD = true;
  } else if(setpointWD > 0) {
    backD = false;
  }
  if(setpointWI < 0) {
    setPointGWI = setpointWI;
    setpointWI = setpointWI*(-1);
    backI = true;
  }
  else if(setpointWI>0) {
    backI = false;
  }
  feedForwardD();
  feedForwardI();  
  moveWheel(PWM_I, setpointWI, pinMotorI, backI);
  moveWheel(PWM_D, setpointWD, pinMotorD, backD);
  for(int i=0; i<20; i++){
    meanFilterD.AddValue(deltaTimeD);
    meanFilterI.AddValue(deltaTimeI);
  }
}

void op_StopWheel() {
  setpointWD=0;
  setpointWI=0;
  fullStop(pinMotorI);
  fullStop(pinMotorD);
}

void op_vel_robot() {
  //Serial.println(OP_VEL_ROBOT);
  operation_send.InitFlag=INIT_FLAG;
  operation_send.id=1;
  operation_send.op = 5;
  short int a=1;
  doubleToBytes(wD, &operation_send.data[0]);
  doubleToBytes(wI, &operation_send.data[8]);
  //Serial.println(wD);
  //Serial.println(wI);
  /*if(backD) {
    shortToBytes(a, &operation_send.data[16]);
  }
  if(backI) {
    shortToBytes(a, &operation_send.data[18]);
  }*/
  operation_send.len = sizeof(double)*2;
  /*Serial.println(operation_send.op);
  Serial.println(wD);
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
  kp_right= bytesToDouble(&server_operation->data[0]);
  ki_right = bytesToDouble(&server_operation->data[8]);
  kd_right = bytesToDouble(&server_operation->data[16]);

  kp_left = bytesToDouble(&server_operation->data[24]);
  ki_left = bytesToDouble(&server_operation->data[32]);
  kd_left = bytesToDouble(&server_operation->data[40]);

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

void motorSetup() {
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinENB, OUTPUT);
}

void moveForward(const int pinMotor[3], int speed) {
  digitalWrite(pinMotor[1], HIGH);
  digitalWrite(pinMotor[2], LOW);
  analogWrite(pinMotor[0], speed);
}

void moveBackward(const int pinMotor[3], int speed) {
  digitalWrite(pinMotor[1], LOW);
  digitalWrite(pinMotor[2], HIGH);
  analogWrite(pinMotor[0], speed);
}

void fullStop(const int pinMotor[3]) {
  digitalWrite(pinMotor[1], HIGH);
  digitalWrite(pinMotor[2], HIGH);
  analogWrite(pinMotor[0], 0);
}

void moveWheel(int pwm, double w, const int pinMotor[3], bool back) {
  if(pwm == 0 || ((int)w) == 0){
    fullStop(pinMotor);
  } else {
    if(back) {
      moveBackward(pinMotor, pwm);
    } else if(!back) {
      moveForward(pinMotor, pwm);
    }
  }
  //se espera un tiempo antes de cambiar  PWM
  //no se usa delay opara evitar interferir con las interruociones.
}

void isrRight() {
  timeBeforeDebounceD = millis();//tiempo para evitar rebotes
  deltaDebounceD = timeBeforeDebounceD - timeAfterDebounceD;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceD > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeD=micros();
    encoder_countRight++;
      deltaTimeD = startTimeD - timeAfterD;
      encoder_countRight = 0;
      timeAfterD = micros();
  }
  timeAfterDebounceD = millis();  
}

void isrLeft() {
  timeBeforeDebounceI = millis();//tiempo para evitar rebotes
  deltaDebounceI = timeBeforeDebounceI - timeAfterDebounceI;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceI > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeI = micros();
    encoder_countI++;//se cuenta los pasos de encoder
    deltaTimeI = startTimeI - timeAfterI;
    encoder_countI = 0;
    timeAfterI = micros();
  }
  timeAfterDebounceI = millis();
}

int pidD(double wD) {
  currentTimeD = millis();
  elapsedTimeD = currentTimeD - previousTimeD;
  int outputD = 0;
  errorD = setpointWD - wD;
  double aux = abs(errorD);
  if(aux >= 0.30) {
    cumErrorD += errorD * elapsedTimeD;                     // calcular la integral del error
    if(lastErrorD > 0 && errorD < 0) { 
       cumErrorD = errorD * elapsedTimeD;
    }
    if(lastErrorD < 0 && errorD > 0) {
       cumErrorD = errorD * elapsedTimeD/1000;
    }
    constrain(cumErrorD, -MAXCUMERROR, MAXCUMERROR);
    rateErrorD = (errorD - lastErrorD) / elapsedTimeD; // calcular la derivada del error
    outputD = static_cast<int> (round(kp_right*errorD + ki_right*cumErrorD + kd_right*rateErrorD ));     // calcular la salida del PID    0.1*errorD +0.0065*cumErrorD + 0.0*rateErrorD
    lastErrorD = errorD;                               // almacenar error anterior
  }
  previousTimeD=currentTimeD;
  //Serial.println(errorD);//0.7*errorD + 0.06*cumErrorD + 0.0*rateErrorD 
  //Serial.println(cumErrorD);
  return outputD;
}

int pidI(double wI) {
  currentTimeI = millis();
  elapsedTimeI = currentTimeI - previousTimeI;
  int outputI = 0;
  errorI = setpointWI - wI;   
  double aux = abs(errorI);
  if(aux >= 0.30) {
    cumErrorI += errorI * elapsedTimeI; 
    //se resetea el error acumulativo cuando se cambia de signo
    if(lastErrorI>0 && errorI<0) {
      cumErrorI = errorI * elapsedTimeI;
    }
    if(lastErrorI<0 && errorI>0) {
      cumErrorI = errorI* elapsedTimeI/1000;
    }
    constrain(cumErrorI, -MAXCUMERROR, MAXCUMERROR);
    rateErrorI = (errorI - lastErrorI) / elapsedTimeI; // calcular la derivada del error    
    outputI = static_cast<int> (round(kp_left*errorI + ki_left*cumErrorI + kd_left*rateErrorI));     // calcular la salida del PID 0.42*errorI  + 0.006*cumErrorI + 0.00*rateErrorI
    lastErrorI = errorI;
  }
  previousTimeI = currentTimeI;
  return outputI;
}

void feedForwardD() {
  PWM_D = (setpointWD != 0.0) ?
    constrain(round((setpointWD - 0.0825) / 0.0707), MINPWM, MAXPWM) : 0;
}

void feedForwardI(){
  PWM_I = (setpointWI != 0.0) ? 
    constrain(round((setpointWI + 1.656) / 0.0720), MINPWM, MAXPWM) : 0;
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
