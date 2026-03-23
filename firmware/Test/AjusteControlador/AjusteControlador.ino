/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  Robot.ino
 * RUTA:     /firmware/Robot.ino
 * REPO:     https://github.com/UCM-237/robotarium-hub
 * ----------------------------------------------------------------------------
*/


#include "common.h"
#include "controler.h"
#include "robot.h"
#ifdef ARDUINO_TYPE_MKR
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include <ArduinoJson.h>
#endif
// --- CONFIGURACIÓN DEL TEST ---
double VELOCIDAD_OBJETIVO = 0.01; // rad/s (ajusta según necesites)
bool TEST_RUEDA_DERECHA = false;   // true para derecha, false para izquierda
bool BACKWARDS= false; // true rueda hacia atras, false hacia adelante
// ------------------------------
// Uncomment only one
//#define  AJUSTEFF
//#define TESTFF
#define AJUSTEPID
//------------------------------
int pwm_output=0;
double w_objetivo=VLMIN;
robot miRobot;
controler PID_RuedaL, PID_RuedaR;


void setup() {
  Serial.begin(9600);
  miRobot.pinSetup();
  miRobot.motorSetup();
  // Paso 1: Objetivo realista (aprox 2 vueltas por segundo)
  VELOCIDAD_OBJETIVO = 15.0; 
  PID_RuedaL.setSetPoint(VELOCIDAD_OBJETIVO);
  PID_RuedaR.setSetPoint(VELOCIDAD_OBJETIVO);
  
  /* Rueda derecha
   *  A=11.07
   *  B=33.58
   *  PWM_MIN=100 w=6 r/s
   *  PWM_MAX=255 w=20 r/s
   */
  PID_RuedaL.setFeedForwardParam(24.1,-155);
  PID_RuedaR.setFeedForwardParam(24.1,-155);
  // Paso 2: Solo proporcional (Kp). Ki y Kd a CERO.
  // Un Kp de 2.0 o 5.0 es un buen inicio para motores de bajo coste.
  PID_RuedaL.setControlerParam(15.0, 1.0,0.0);
  PID_RuedaR.setControlerParam(15.0, 1.0,0.0);
  // Configuración de interrupciones para encoders (necesario para calcular w real)
  attachInterrupt(digitalPinToInterrupt(miRobot.getPinLeftEncoder()), isr_left, RISING);
  attachInterrupt(digitalPinToInterrupt(miRobot.getPinRightEncoder()), isr_right, RISING);

  Serial.println("Objetivo(rad/s),Real(rad/s),PWM");
}

void loop() {
     double w_derecha, w_izquierda;
  static unsigned long lastMillis = 0;
 // --- GESTIÓN DE TIMEOUT (Rueda parada) ---
  unsigned long ahora = micros();
  
  // Si han pasado más de 500ms (500,000 micros) sin pulsos, la velocidad es 0
  if (ahora - startTimeRight > 500000) deltaTimeRight = 0;
  if (ahora - startTimeLeft > 500000) deltaTimeLeft = 0;

  // --- CÁLCULO DE VELOCIDAD ANGULAR (rad/s) ---
  // Fórmula: (2*PI / Pasos_por_vuelta) / (Tiempo_en_segundos)
  
  if (deltaTimeRight > 500) { // Protección contra división por cero y ruido extremo
    w_derecha = (2.0 * PI / 20.0) / (deltaTimeRight / 1000000.0);
  } 
  else {
    w_derecha = 0;
  }

  if (deltaTimeLeft > 500) {
    w_izquierda = (2.0 * PI / 20.0) / (deltaTimeLeft / 1000000.0);
  } else {
    w_izquierda = 0;
  } 
#ifdef AJUSTEFF    
  if(pwm_output>=255)
    pwm_output=255;    
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
    pwm_output+=10;
  
    if (TEST_RUEDA_DERECHA) {
  
            miRobot.moveRightWheel(pwm_output, VELOCIDAD_OBJETIVO, BACKWARDS);
            Serial.print("encoder: ");
            Serial.print(encoder_countRight);
            Serial.print("pwm: ");
            Serial.print(pwm_output);            
            Serial.print(", w_real: ");
            Serial.println(w_derecha);
  }
  else {
            miRobot.moveLeftWheel(pwm_output, VELOCIDAD_OBJETIVO, BACKWARDS);
            Serial.print("pwm: ");
            Serial.print(pwm_output);            
            Serial.print(", w_real: ");
            Serial.println(w_izquierda);
  }
  }

#endif

#ifdef TESTFF


  if(w_objetivo>=20){
    pwm_output=0;
 }
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 1000) {
    lastMillis = millis();
     w_objetivo+=0.1;
     double w_real;

    if (TEST_RUEDA_DERECHA) {
            PID_RuedaR.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaR.feedForward(); 
            miRobot.moveRightWheel(pwm_output, w_objetivo, BACKWARDS);
            Serial.print("pwm: ");
            Serial.print(pwm_output);
            Serial.print(", w_objetivo: ");
            Serial.println(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(w_derecha);
  }
  else  {
            PID_RuedaL.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaL.feedForward(); 
            miRobot.moveLeftWheel(pwm_output, w_objetivo, BACKWARDS);
            Serial.print("pwm: ");
            Serial.print(pwm_output);
            Serial.print(", w_objetivo: ");
            Serial.print(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(w_izquierda);
  }

  }

#endif


#ifdef AJUSTEPID


  if(w_objetivo>=20){
    pwm_output=0;
 }
  // Ejecutamos el bucle de control cada 10ms (100Hz) como en tu Robot.ino original
  if (millis() - lastMillis >= 0.01) {
    lastMillis = millis();
     w_objetivo=10.0;
     int pwm_pid=0;

    if (TEST_RUEDA_DERECHA) {
            PID_RuedaR.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaR.feedForward();
            pwm_output+=PID_RuedaR.pid(w_derecha);
            pwm_output=constrain(pwm_output,MINPWM,MAXPWM);
            miRobot.moveRightWheel(pwm_output, w_objetivo, BACKWARDS);
            Serial.print(", w_objetivo: ");
            Serial.println(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(w_derecha);
  }
  else  {
            PID_RuedaL.setSetPoint(w_objetivo);
            pwm_output= PID_RuedaL.feedForward(); 
            //Serial.print("pwm_FF: ");
            //Serial.print(pwm_output);
            pwm_pid=PID_RuedaL.pid(w_izquierda);
            pwm_output+=pwm_pid;
            pwm_output=constrain(pwm_output,MINPWM,MAXPWM);
            miRobot.moveLeftWheel(pwm_output, w_objetivo, BACKWARDS);
            /*Serial.print("pwm: ");
            Serial.print(pwm_output);*/
            Serial.print("w_objetivo: ");
            Serial.print(w_objetivo);
            Serial.print(", w_real: ");
            Serial.println(w_izquierda);
  }

  }

#endif
}

// --- ISR Necesarias (Copiadas de tu lógica en Robot.ino) ---

void isr_right() {
  unsigned long ahora = micros();
  // Filtro debounce: 1000 micros = 1ms
  if (ahora - timeBeforeDebounceRight > 200) {
    deltaTimeRight = ahora - startTimeRight;
    startTimeRight = ahora;
    encoder_countRight++;
  }
  timeBeforeDebounceRight = ahora;
}

void isr_left() {
  unsigned long ahora = micros();
  if (ahora - timeBeforeDebounceLeft > 200) {
    deltaTimeLeft = ahora - startTimeLeft;
    startTimeLeft = ahora;
    encoder_countLeft++;
  }
  timeBeforeDebounceLeft = ahora;
}
