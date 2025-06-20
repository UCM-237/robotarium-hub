
//robot2
#include "common.h"
#include "controler.h"
#include "robot.h"
#include <math.h>
#include <SimpleKalmanFilter.h>
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "arduino_secrets.h"


// Define a macro for debug printing
#define DEBUG_ENABLED  // Comment out this line to disable debug prints

#ifdef DEBUG_ENABLED
#define DEBUG_PRINT(...)   Serial1.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial1.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)   // Debug print is empty when DEBUG_ENABLED is not defined
#define DEBUG_PRINTLN(...) // Debug println is empty when DEBUG_ENABLED is not defined
#endif
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status



using namespace std;
unsigned char packetBuffer[256]; //buffer to hold incoming packet

//--------------------------------------------filtro de media movil simple para estabilizar la lectura de rad/s---------------------------------------

MeanFilter<long> meanFilterRight(10);
MeanFilter<long> meanFilterLeft(10);
const double MAX_OPTIMAL_VEL=20;//rad/S



//operation variables
struct appdata operation_send;
struct appdata *server_operation;

//prototypes//
//TODO: DO a reuqest manager for the opeartions and handle serial and mqtt communication
void do_operation(int operation);
void op_saludo();
void op_message();
void op_moveRobot();
void op_StopRobot();
void op_telemtry();
void op_turn_robot();
void op_error();//unrecognized operation
void isrRight();
void isrLeft();
void send(int operation, byte *data);
void connect();



int option;
int led = 13;
//Time variables
unsigned long previous_timer;
unsigned long timer=10000;
/*define controler*/
controler wheelControlerRight;
controler wheelControlerLeft;
/*define robot*/
robot robot;
WiFiClient wifi;
MqttClient mqttClient(wifi);
const char broker[] ="192.168.1.109";
int mqttPort = 1883;
const char device[] = "arduinoClient";

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
      Serial1.println("Failed to initialize IMU!");
      while (1);
    }
    // Se empieza con los motores parados
    robot.fullStop();
  
    wheelControlerRight.setControlerParam(0.15, 0.01, 0.00);
    wheelControlerRight.setFeedForwardParam(0.0895,-5.424);
    wheelControlerLeft.setControlerParam(0.15, 0.01, 0.00);
    wheelControlerLeft.setFeedForwardParam(0.0772,-3.173);
    // Comunicacion por puerto serie
    Serial1.begin(9600);//for debuggin
    Serial.begin(9600);

    //connect();
   DEBUG_PRINTLN("setup ok");
}
//define common variables
//TODO: Refactor
int serialOperation = 0;
static bool sendDataSerial = false;
bool serialCom = false;
bool control = true;
unsigned char *read_ptr; 
unsigned long currentTime, timeAfter = 0;
const unsigned long SAMPLINGTIME= 10;//ms
double wLeft,wRight; // measured angular velocity

const int INIT_FLAG = 112;

void loop() {
  currentTime = millis();
  serialEvent();
  if (serialCom) 
  {
    // Serial.print("operationFlag: \t");
    // Serial.println(server_operation->InitFlag);
    if (server_operation->InitFlag == INIT_FLAG)
    {
     DEBUG_PRINT("operationFlag: \t");
     DEBUG_PRINTLN(server_operation->InitFlag);
     DEBUG_PRINT("operation: \t");
     DEBUG_PRINTLN(server_operation->op);

      do_operation((operation_t)server_operation->op);
      serialCom = false;
    }
  }
  
  // call poll() regularly to allow the library to send MQTT keep alive which

  // avoids being disconnected by the broker

  //mqttClient.poll();
  int auxPWMD = 0, auxPWMI = 0;
  double fD, fI;
  timeStopD = timeStopI = millis();
  //condition for know when the wheel is stoped
  //se define un contador de tiempo para comprobar que las reudas estan paradas
  deltaTimeStopD = timeStopD - timeAfterDebounceRight;
  deltaTimeStopI = timeStopI - timeAfterDebounceLeft;
  meanFilterRight.AddValue(deltaTimeRight);
  meanFilterLeft.AddValue(deltaTimeLeft);
  do_operation(OP_HELLO);  

  if(currentTime - timeAfter >= SAMPLINGTIME) 
  {
    fI = deltaTimeStopI >= 100 ? 0 : (double)1 / (meanFilterLeft.GetFiltered() * MAX_ENCODER_STEPS) * 1000;
    fD = deltaTimeStopD >= 100 ? 0 : (double)1 / (meanFilterRight.GetFiltered() * MAX_ENCODER_STEPS) *1000;
    // DEBUG_PRINT("fI:");
    // DEBUG_PRINT(fI);
    // DEBUG_PRINT(" fD:");
    // DEBUG_PRINTLN(fD);
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
    if(control)
    { 
      
      if(wRight !=0.0 && wheelControlerRight.getSetPoint() !=0.0) {   
        int controllerValue =  wheelControlerRight.pid(wRight);
        PWM_Right = constrain(PWM_Right +controllerValue, MINPWM, MAXPWM);
      }
      if(wLeft !=0.0 && wheelControlerLeft.getSetPoint() !=0.0) {
        int controllerValue = wheelControlerLeft.pid(wLeft);
        PWM_Left = constrain(PWM_Left +controllerValue , MINPWM, MAXPWM);
      }
      // DEBUG_PRINT("PWM_Left:");
      // DEBUG_PRINT(PWM_Left);
      // DEBUG_PRINT(" PWM_Right:");
      // DEBUG_PRINTLN(PWM_Right);
      if(wRight >0){
      DEBUG_PRINT("wRight:");
      DEBUG_PRINT(wRight);
      DEBUG_PRINT(" wLeft:");
      DEBUG_PRINTLN(wLeft);
      
      Serial.print("wRight:");
      Serial.print(wRight);
      Serial.print(" wLeft:");
      Serial.println(wLeft);
      }
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

    
    if(sendDataSerial)
    {
      op_telemtry();
    }
    timeAfter = currentTime; 
  }

  
}

void do_operation(operation_t operation) {
  switch (operation) {
    case OP_HELLO:
      op_saludo();
      break;
    case OP_MOVE_ROBOT:
      op_moveRobot();
      break;
    case OP_STOP_ROBOT:
      op_StopRobot();
      break;
    case OP_TELEMETRY:
      sendDataSerial = true;
      break;
    case OP_TURN_ROBOT:
      op_turn_robot();
      break;
    case OP_SILENCE:
      op_silence();
      break;
    case OP_CONF_PID:
      op_conf_pid();
      break;
    case OP_CONF_FF:
      op_conf_ff();
      break;
    case OP_MOVE_WHEELS:
      op_moveWheels();
      break;
    default:
      op_error();
      break;
  }
}

void send(int operation, byte *data) {
  operation_send.op = operation;
  operation_send.len = sizeof(data);
  Serial1.write((char*)&operation_send.op, 1);
  Serial1.write((char*)&operation_send.len, 2);
  Serial1.write((char*)&data, operation_send.len);
  Serial1.flush();
}

void op_saludo() {
  operation_send.op = OP_HELLO;

  operation_send.len = sizeof (operation_send.data);  /* len */
  Serial1.write((char*)operation_send.data, operation_send.len + HEADER_LEN);
  Serial1.flush();
//  send(ID, OP_HELLO, )
}

void op_message() { }
void op_moveWheels()
{
  DEBUG_PRINTLN("move Wheels");
  int PWMRight = bytesToLong(&server_operation->data[0]);
  int PWMLeft = bytesToLong(&server_operation->data[4]);
  robot.moveLeftWheel(PWMLeft, 1, false);
  robot.moveRightWheel(PWMRight, 1, false);
  for(int i=0; i<10; i++)
  {
    meanFilterRight.AddValue(deltaTimeRight);
    meanFilterLeft.AddValue(deltaTimeLeft);
  }
}
void op_moveRobot() {
 DEBUG_PRINTLN("move");
  double setpointWRight = bytesToDouble(&server_operation->data[0]);
  double setpointWLeft = bytesToDouble(&server_operation->data[8]);
  DEBUG_PRINT("setpointWRight:");
  DEBUG_PRINT(setpointWRight);

  DEBUG_PRINT(" setpointWLeft:");
  DEBUG_PRINTLN(setpointWLeft);
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
  DEBUG_PRINT("PWM_Left:");
  DEBUG_PRINT(PWM_Left);
  DEBUG_PRINT(" PWM_Right:");
  DEBUG_PRINTLN(PWM_Right);
  // feedForwardD();
  // feedForwardI();  
  // moveWheel(PWM_Left, setpointWLeft, pinMotorI, backI);
  // moveWheel(PWM_Right, setpointWRight, pinMotorD, backD);
  robot.moveLeftWheel(PWM_Left, setpointWLeft, backI);
  robot.moveRightWheel(PWM_Right, setpointWRight, backD);
  //take the mean of the last 5 values for measure the angular velocity 
  //of every wheel
  for(int i=0; i<10; i++)
  {
    meanFilterRight.AddValue(deltaTimeRight);
    meanFilterLeft.AddValue(deltaTimeLeft);
  }
}

void op_StopRobot() {
  DEBUG_PRINT("op stop:");
  DEBUG_PRINTLN(OP_STOP_ROBOT);
  wheelControlerLeft.setSetPoint(0.0);
  wheelControlerRight.setSetPoint(0.0);
  // fullStop(pinMotorI);
  // fullStop(pinMotorD);
  robot.fullStop();
}

void op_telemtry() {
  DEBUG_PRINT("op telemetry:");
  DEBUG_PRINTLN(OP_TELEMETRY);
  operation_send.InitFlag=(int)INIT_FLAG;
  operation_send.id=robot.getRobotID();
  operation_send.op =(int)OP_TELEMETRY;
  short int a=1;
  doubleToBytes(wLeft, &operation_send.data[0]);
  doubleToBytes(wRight, &operation_send.data[8]);
  longToBytes(PWM_Left, &operation_send.data[16]);
  longToBytes(PWM_Right, &operation_send.data[20]);
  DEBUG_PRINT("vel robot--->");
  DEBUG_PRINT(" wRight:");
  DEBUG_PRINT(wRight);
  DEBUG_PRINT(" wLeft:");
  DEBUG_PRINTLN(wLeft);
  operation_send.len = (int)sizeof(double)*2+sizeof(int)*2;  /* len */
  Serial1.write((char*)&operation_send.InitFlag,4);
  Serial1.write((char*)&operation_send.id,4);
  Serial1.write((char*)&operation_send.op, 4);
  Serial1.write((char*)&operation_send.len, 4);
  Serial1.write((char*)&operation_send.data, operation_send.len);
  Serial1.flush();
  //send(ID, OP_TELEMETRY, &operation_send.data);
}
void op_turn_robot()
{
  DEBUG_PRINT("op turn:");
  DEBUG_PRINTLN(OP_TURN_ROBOT);
  bool turnRight;
  int angle = bytesToLong(&server_operation->data[0]);
  DEBUG_PRINT("angle:");
  DEBUG_PRINTLN(angle);
  if(angle<0)
  {
    turnRight = true;
    angle = angle*(-1);
  }
  else
  {
    turnRight = false;
  }
  //If angle is negative, turn right else turn left
  double angleInRad = ((double)angle)*M_PI/180;
  double angleToTurn = angleInRad*(robot.getRobotDiameter())/2;
  //Reset the encoder count
  encoder_countRight=0;
  encoder_countLeft=0;
  //Set the target encoder count
  int targetEncoderCount = int(angleToTurn/(2*M_PI*robot.getRobotWheelRadius())*MAX_ENCODER_STEPS);
  while (encoder_countRight < targetEncoderCount && encoder_countLeft < targetEncoderCount)
  {
    if(turnRight<0)
    {
      robot.moveLeftWheel(MINPWM, 1, false);
      robot.moveRightWheel(MINPWM, 1, true);
    }
    else
    {
      robot.moveLeftWheel(MINPWM, 1, true);
      robot.moveRightWheel(MINPWM, 1, false);
    }
  }
  robot.fullStop();
  op_done();
}
inline void op_silence()
{
  DEBUG_PRINT("op silense:");
  DEBUG_PRINTLN(OP_SILENCE);
  sendDataSerial = false;
  op_done();
}

inline void op_error()
{
  DEBUG_PRINT("op error:");
  DEBUG_PRINTLN(OP_ERROR);
  operation_send.op = OP_ERROR;
  operation_send.InitFlag=INIT_FLAG;
  operation_send.id=(int)robot.getRobotID();
  operation_send.len = sizeof (operation_send.data);  /* len */

  Serial1.write((char*)&operation_send.InitFlag,4);
  Serial1.write((char*)&operation_send.id,4);
  Serial1.write((char*)&operation_send.op, 4);
  Serial1.write((char*)&operation_send.len, 4);
  Serial1.write((char*)&operation_send.data, operation_send.len);
  Serial1.flush();
}
void op_conf_pid()
{
    DEBUG_PRINT("conf pid:");
    DEBUG_PRINTLN(OP_CONF_PID);
    double kp_right = bytesToDouble(&server_operation->data[0]);
    double ki_right = bytesToDouble(&server_operation->data[8]);
    double kd_right = bytesToDouble(&server_operation->data[16]);

    double kp_left = bytesToDouble(&server_operation->data[24]);
    double ki_left = bytesToDouble(&server_operation->data[32]);
    double kd_left = bytesToDouble(&server_operation->data[40]);

    wheelControlerRight.setControlerParam(kp_right, ki_right, kd_right);
    wheelControlerLeft.setControlerParam(kp_left, ki_left, kd_left);

    op_done();
    DEBUG_PRINT(kp_right);
    DEBUG_PRINT(",");

    DEBUG_PRINT(ki_right);
    DEBUG_PRINT(",");

    DEBUG_PRINT(kd_right);
    DEBUG_PRINT(",");

    DEBUG_PRINT(kp_left);
    DEBUG_PRINT(",");
    DEBUG_PRINT(ki_left);
    DEBUG_PRINT(",");
    DEBUG_PRINT(kd_left);

}

void op_conf_ff()
{
  DEBUG_PRINT("conf ff:");
  DEBUG_PRINTLN(OP_CONF_FF);
  double A_right = bytesToDouble(&server_operation->data[0]);
  double B_right = bytesToDouble(&server_operation->data[8]);

  double A_left = bytesToDouble(&server_operation->data[16]);
  double B_left = bytesToDouble(&server_operation->data[24]);

  wheelControlerRight.setFeedForwardParam(A_right, B_right);
  wheelControlerLeft.setFeedForwardParam(A_left, B_left);

  DEBUG_PRINT(A_right);
 DEBUG_PRINT(",");
  
  DEBUG_PRINT(B_right);
  DEBUG_PRINT(",");
  
  DEBUG_PRINT(A_left);
  DEBUG_PRINT(",");
  
  DEBUG_PRINTLN(B_left);
  op_done();
}
void op_done()
{
  DEBUG_PRINT("op done:");
  DEBUG_PRINTLN(OP_DONE);
  operation_send.op = OP_DONE;
  operation_send.InitFlag=INIT_FLAG;
  operation_send.id=(int)robot.getRobotID();
  operation_send.len = sizeof (operation_send.data);  /* len */

  Serial1.write((char*)&operation_send.InitFlag,4);
  Serial1.write((char*)&operation_send.id,4);
  Serial1.write((char*)&operation_send.op, 4);
  Serial1.write((char*)&operation_send.len, 4);
  Serial1.write((char*)&operation_send.data, operation_send.len);
  Serial1.flush();

}
void isrRight() {
  timeBeforeDebounceRight = millis();//tiempo para evitar rebotes
  deltaDebounceRight = timeBeforeDebounceRight - timeAfterDebounceRight;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceRight > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeRight=millis();
    encoder_countRight++;
    if(encoder_countRight == MAX_ENCODER_STEPS)
    {
      wheelTurnCounterRight++;
    }
    deltaTimeRight = startTimeRight - timeAfterRight;
      
    timeAfterRight = startTimeRight;
  }
  timeAfterDebounceRight = timeBeforeDebounceRight;  
}

void isrLeft() {
  timeBeforeDebounceLeft = millis();//tiempo para evitar rebotes
  deltaDebounceLeft = timeBeforeDebounceLeft - timeAfterDebounceLeft;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceLeft > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeLeft = millis();
    encoder_countLeft++;//se cuenta los pasos de encoder
    if(encoder_countLeft == MAX_ENCODER_STEPS)
    {
      wheelTurnCounterLeft++;
    
    }
    deltaTimeLeft = startTimeLeft - timeAfterLeft;
    timeAfterLeft = startTimeLeft;
  }
  timeAfterDebounceLeft = timeBeforeDebounceLeft;
}


void serialEvent() {
  // read_ptr = (unsigned char*)&packetBuffer;
  // while (Serial1.available()) {
  //   *(read_ptr++) = Serial1.read();
  //   server_operation = (struct appdata *)&packetBuffer;
  //   serialCom = true;
  // }
  if(Serial1.available())
  {
    int rlen = Serial1.readBytesUntil('\n', (char*)&packetBuffer, sizeof(packetBuffer));
    server_operation = (struct appdata *)&packetBuffer;
    serialCom = true;
    DEBUG_PRINTLN(rlen);
  }
}

// void serialEvent() {
//   static unsigned char* read_ptr = (unsigned char*)&packetBuffer;
//   static bool start_of_data = false;

//   while (Serial1.available()) {
//     char c = Serial1.read();
    
//    if (c == 'K') {  // Si se encuentra un carácter de nueva línea
//       // Agrega un terminador nulo para indicar el final de los datos
//       // Reinicia el puntero de datos para leer los datos desde el principio
//       read_ptr = (unsigned char*)&packetBuffer;
//       // Marca el final de los datos
//       server_operation = (struct appdata *)&packetBuffer;
//       serialCom = true;
//       start_of_data = false;  // Reinicia el marcador del inicio de los datos
//       DEBUG_PRINTLN(server_operation->len);
//     } 
//     else 
//     {
//       if (!start_of_data) 
//       {  // Si no se ha empezado a recibir datos, marca el inicio
//         start_of_data = true;
//         read_ptr = (unsigned char*)&packetBuffer;
//       }
      
//       *read_ptr++ = c;  // Almacena el byte en el buffer y avanza el puntero
//     }
//   }
  
// }

void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);


  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}

void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}


/*
Connects to Wifi and MQTT Broker
*/
void connect() {
  Serial.print("checking wifi...");
  while ( status != WL_CONNECTED) {
    status = WiFi.begin(ssid, pass);
    Serial.print(".");
    delay(1000);
  }
  Serial.println("\nconnected to WiFi!\n");
  
  // Connection successful
  Serial.print("[INFO] Connection Successful");
  Serial.print("");  
  printConnectionInformation();
  Serial.println("-----------------------------------------------");
  Serial.println(""); 

  Serial.print("Attempting MQTT connection ....");
  Serial.println(broker);
  if(!mqttClient.connect(broker,mqttPort))
  {
    Serial.print("MQTT connection failed! Error code = ");

    Serial.println(mqttClient.connectError());


    while (1);
  }
  Serial.println("You're connected to the MQTT broker!");
 // Construct the operation message as a JSON string
  String messagePayload = "{\"operation\": \"hello\", \"source_id\": \"arduinoClient\", \"payload\": {\"url\": \"example.com\"}}";
  mqttClient.beginMessage("");
  mqttClient.print(messagePayload);
}



/*
Prints Wifi connection information
*/
void printConnectionInformation() 
{
  // Print Network SSID
  Serial.print("[INFO] SSID: ");
  Serial.println(WiFi.SSID());

  // Print Router's MAC address
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("[INFO] BSSID: ");
  Serial.print(bssid[5], HEX);
  Serial.print(":");
  Serial.print(bssid[4], HEX);
  Serial.print(":");
  Serial.print(bssid[3], HEX);
  Serial.print(":");
  Serial.print(bssid[2], HEX);
  Serial.print(":");
  Serial.print(bssid[1], HEX);
  Serial.print(":");
  Serial.println(bssid[0], HEX);

  // Print received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("[INFO] Signal Strength (RSSI): ");
  Serial.println(rssi);

  // Print encryption type
  byte encryption = WiFi.encryptionType();
  Serial.print("[INFO] Encryption Type: ");
  Serial.println(encryption, HEX);

  // Print WiFi Shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("[INFO] IP Address: ");
  Serial.println(ip);

  // Print MAC address
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("[INFO] MAC Address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);
}
