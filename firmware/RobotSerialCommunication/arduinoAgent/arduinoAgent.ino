
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
#define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)   // Debug print is empty when DEBUG_ENABLED is not defined
#define DEBUG_PRINTLN(...) // Debug println is empty when DEBUG_ENABLED is not defined
#endif
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;     // the WiFi radio's status



using namespace std;
uint8_t packetBuffer[256]; //buffer to hold incoming packet

//--------------------------------------------filtro de media movil simple para estabilizar la lectura de rad/s---------------------------------------

MeanFilter<long> meanFilterRight(5);
MeanFilter<long> meanFilterLeft(5);
const double MAX_OPTIMAL_VEL=15;//rad/S



//operation variables
struct appdata operation_send;
struct appdata *server_operation;

//prototypes//
//TODO: DO a reuqest manager for the opeartions and handle serial and mqtt communication
void do_operation(uint8_t operation);
void op_saludo();
void op_message();
void op_moveRobot();
void op_StopRobot();
void op_vel_robot();
void isrRight();
void isrLeft();
void send(uint8_t operation, byte *data);
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
      Serial.println("Failed to initialize IMU!");
      while (1);
    }
    // Se empieza con los motores parados
    robot.fullStop();
  
    wheelControlerRight.setControlerParam(0.2, 0.05, 0.009);
    wheelControlerRight.setFeedForwardParam(0.0825,0.707);
    wheelControlerLeft.setControlerParam(0.2, 0.05, 0.009);
    wheelControlerLeft.setFeedForwardParam(1.656,0.072);
    // Comunicacion por puerto serie
    Serial1.begin(9600);//for debuggin
    Serial.begin(9600);

    //connect();
    pinMode(led, OUTPUT); 
   DEBUG_PRINTLN("setup ok");
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

const uint8_t INIT_FLAG = 112;

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

  if(currentTime - timeAfter >= SAMPLINGTIME) 
  {

    int auxPWMD = 0, auxPWMI = 0;
    double fD, fI;
    timeStopD = timeStopI = millis();
    //condition for know when the wheel is stoped
    //se define un contador de tiempo para comprobar que las reudas estan paradas
    deltaTimeStopD = timeStopD - timeAfterDebounceRight;
    deltaTimeStopI = timeStopI - timeAfterDebounceLeft;
    meanFilterRight.AddValue(deltaTimeRight);
    meanFilterLeft.AddValue(deltaTimeLeft);
    fI = deltaTimeStopI >= 200 ? 0 : (double)1 / (meanFilterLeft.GetFiltered() * MAX_ENCODER_STEPS) * 1000000;
    fD = deltaTimeStopD >= 200 ? 0 : (double)1 / (meanFilterRight.GetFiltered() * MAX_ENCODER_STEPS) * 1000000;
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

    
    
    op_vel_robot();
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
      //digitalWrite(led, HIGH);
      break;
    case OP_STOP_ROBOT:
      op_StopRobot();
      break;
    case OP_VEL_ROBOT:
      sendDataSerial = true;
      break;
    case OP_CONF_PID:
      op_conf_pid();
    case OP_CONF_FF:
      op_conf_ff();
    default:
      break;
  }
}

void send(uint8_t operation, byte *data) {
  operation_send.op = operation;
  operation_send.len = sizeof(data);
  Serial1.write((char*)&operation_send.op, 1);
  Serial1.write((char*)&operation_send.len, 2);
  Serial1.write((char*)&data, operation_send.len);
  Serial1.flush();
}

void op_saludo() {
  operation_send.op = (uint8_t)OP_HELLO;

  operation_send.len = sizeof (operation_send.data);  /* len */
  Serial1.write((char*)operation_send.data, operation_send.len + HEADER_LEN);
  Serial1.flush();
//  send(ID, OP_HELLO, )
}

void op_message() { }

void op_moveRobot() {
 DEBUG_PRINTLN("move");
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
  DEBUG_PRINT("op stop:");
  DEBUG_PRINTLN(OP_STOP_ROBOT);
  wheelControlerLeft.setSetPoint(0.0);
  wheelControlerRight.setSetPoint(0.0);
  // fullStop(pinMotorI);
  // fullStop(pinMotorD);
  robot.fullStop();
}

void op_vel_robot() {
  DEBUG_PRINT("op vel:");
  DEBUG_PRINTLN(OP_VEL_ROBOT);
  operation_send.InitFlag=INIT_FLAG;
  operation_send.id=1;
  operation_send.op = 5;
  short int a=1;
  doubleToBytes(wRight, &operation_send.data[0]);
  doubleToBytes(wLeft, &operation_send.data[8]);
  DEBUG_PRINT("vel robot--->");
  DEBUG_PRINT(" wRight:");
  DEBUG_PRINT(wRight);
  DEBUG_PRINT(" wLeft:");
  DEBUG_PRINTLN(wLeft);
  /*if(backD) {
    shortToBytes(a, &operation_send.data[16]);
  }
  if(backI) {
    shortToBytes(a, &operation_send.data[18]);
  }*/
  operation_send.len = sizeof(double)*2;
  /*Serial.println(operation_send.op);
  Serial.println(wRight);
  Serial.print("len \t");F
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
  DEBUG_PRINT("conf pid:");
  DEBUG_PRINTLN(OP_CONF_PID);
  double kp_right= bytesToDouble(&server_operation->data[0]);
  double ki_right = bytesToDouble(&server_operation->data[8]);
  double kd_right = bytesToDouble(&server_operation->data[16]);

  double kp_left = bytesToDouble(&server_operation->data[24]);
  double ki_left = bytesToDouble(&server_operation->data[32]);
  double kd_left = bytesToDouble(&server_operation->data[40]);

  wheelControlerRight.setControlerParam(kp_right, ki_right, kd_right);
  wheelControlerLeft.setControlerParam(kp_left, ki_left, kd_left);

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

}
void isrRight() {
  timeBeforeDebounceRight = millis();//tiempo para evitar rebotes
  deltaDebounceRight = timeBeforeDebounceRight - timeAfterDebounceRight;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceRight > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeRight=micros();
    encoder_countRight++;
    if(encoder_countRight == MAX_ENCODER_STEPS)
    {
      encoder_countRight=0;
      wheelTurnCounterRight++;
    }
    deltaTimeRight = startTimeRight - timeAfterRight;
      
    timeAfterRight = micros();
  }
  timeAfterDebounceRight = millis();  
}

void isrLeft() {
  timeBeforeDebounceLeft = millis();//tiempo para evitar rebotes
  deltaDebounceLeft = timeBeforeDebounceLeft - timeAfterDebounceLeft;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceLeft > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeLeft = micros();
    encoder_countLeft++;//se cuenta los pasos de encoder
    if(encoder_countLeft == MAX_ENCODER_STEPS)
    {
      encoder_countLeft=0;
      wheelTurnCounterLeft++;
    
    }
    deltaTimeLeft = startTimeLeft - timeAfterLeft;
    timeAfterLeft = micros();
  }
  timeAfterDebounceLeft = millis();
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
