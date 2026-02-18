/*
 * ----------------------------------------------------------------------------
 * ARCHIVO:  Robot.cpp
 * PROYECTO: Robotarium Hub (UCM)
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Implementación de los métodos de la clase Robot. Contiene la lógica física
 * de movimiento y la gestión de pines mediante registros de Arduino.
 * ----------------------------------------------------------------------------
 */

/*
Boromir, Arwen y Gandalf tiene los pines del motor de la siguiente manera
Motor Izquierdo:
ENABLE  9
IN 1    8
IN 2    7

Motor Derecho:
ENABLE  10
IN 1    11
IN 2    12

En el Arduino MKR el pin 9 no puede usarse para el ENABLE. Así que:

Motor Izquierdo:
ENABLE  10
IN 1    11
IN 2    12

Motor Derecho:
ENABLE  7
IN 1    8
IN 2    9

*/
#include "robot.h"
#define FORWARD HIGH 
#define BACKWARD LOW

// Configuración de pines dinámica según el modelo de placa y puente en H
void robot::pinSetup() {
  #ifdef ARDUINO_TYPE_MKR
    this->pinLeftEncoder = 1;  // Pin interrupción izquierda MKR
    this->pinRightEncoder = 4; // Pin interrupción derecha MKR

    #ifdef H_BRIDGE_BLACK
      // Configuración para el Puente en H Negro (L298N o similar)
      this->pinENA = 5;  this->pinIN1 = 4; 
      this->pinIN3 = 9;  this->pinENB = 8;
    #endif
  
    #ifdef H_BRIDGE_RED
      // Configuración para el Puente en H Rojo
      this->pinENA = 7;  this->pinIN2 = 9;  this->pinIN1 = 8;
      this->pinIN3 = 11; this->pinIN4 = 12; this->pinENB = 10;
    #endif
  #endif
  
  #ifdef ARDUINO_TYPE_NANO
    this->pinLeftEncoder = 2;  this->pinRightEncoder = 3; 
    // (Configuraciones de pines para Nano...)
  #endif

  // Asignación de arrays para facilitar el manejo de motores [ENABLE, IN1, IN2]
  this->pinMotorRight[0] = pinENB; this->pinMotorRight[1] = pinIN3; this->pinMotorRight[2] = pinIN4;
  this->pinMotorLeft[0] = pinENA;  this->pinMotorLeft[1] = pinIN1;  this->pinMotorLeft[2] = pinIN2;
}

// Configura todos los pines de control como salida
void robot::motorSetup() {
    pinMode(this->pinENA, OUTPUT); pinMode(this->pinIN1, OUTPUT); pinMode(this->pinIN2, OUTPUT);
    pinMode(this->pinENB, OUTPUT); pinMode(this->pinIN3, OUTPUT); pinMode(this->pinIN4, OUTPUT);
}

// Mueve una rueda hacia adelante aplicando PWM
void robot::moveForward(const int pinMotor[3], int speed) {
    digitalWrite(pinMotor[1], HIGH);
    digitalWrite(pinMotor[2], LOW);
    analogWrite(pinMotor[0], speed);
}

// Mueve una rueda hacia atrás aplicando PWM
void robot::moveBackward(const int pinMotor[3], int speed) {
    digitalWrite(pinMotor[1], LOW);
    digitalWrite(pinMotor[2], HIGH);
    analogWrite(pinMotor[0], speed);
}

// Freno electrónico: Pone ambos pines de dirección en HIGH
void robot::fullStopRightWheel() {
    digitalWrite(this->pinMotorRight[1], HIGH);
    digitalWrite(this->pinMotorRight[2], HIGH);
    analogWrite(this->pinMotorRight[0], 0);
}

// Lógica de decisión para la rueda derecha
void robot::moveRightWheel(int pwm, double w, bool back) {
    if(pwm == 0 || ((int)w) == 0) {
        fullStopRightWheel(); // Si la velocidad es 0, frenar
    } else {
        if(back) moveBackward(this->pinMotorRight, pwm);
        else moveForward(this->pinMotorRight, pwm);
    }
}
/**
 * Devuelve el ID único asignado al robot.
 * Se utiliza para que el servidor identifique qué robot de la flota está respondiendo.
 */
uint8_t robot::getRobotID()
{
    return this->robotID; // Retorna el valor definido en la parte privada de robot.h
}

/**
 * Devuelve el diámetro total del robot (distancia entre el centro de las dos ruedas).
 * Crucial para cálculos cinemáticos de rotación (OP_TURN_ROBOT).
 */
double robot::getRobotDiameter()
{
    return this->RobotDiameter; // Valor por defecto: 14.5 cm
}

/**
 * Devuelve el radio de la rueda del robot.
 * Se usa para convertir desplazamientos lineales en rotaciones del motor.
 */
double robot::getRobotWheelRadius()
{
    return this->RobotWheelRadius; // Valor por defecto: 3.35 cm
}

/**
 * Detiene ambos motores inmediatamente.
 * Configura los pines de control para frenado activo y pone el PWM a 0.
 */
void robot::fullStop()
{
    fullStopLeftWheel();  // Llama al frenado individual de la rueda izquierda
    fullStopRightWheel(); // Llama al frenado individual de la rueda derecha
}

/**
 * Controla el movimiento de la rueda izquierda.
 * @param pwm Potencia calculada por el PID/FeedForward.
 * @param w Velocidad angular deseada (si es 0, la rueda frena).
 * @param back Booleano que indica si el giro es hacia atrás.
 */
void robot::moveLeftWheel(int pwm, double w, bool back)
{
    // Si la potencia es 0 o la velocidad deseada es 0, detenemos la rueda
    if(pwm == 0 || ((int)w) == 0)
    {
        fullStopLeftWheel();
    } 
    else 
    {
        // Selección de dirección basada en el parámetro 'back'
        if(back) 
        {
            moveBackward(this->pinMotorLeft, pwm); // Mueve motor izquierdo hacia atrás
        } 
        else 
        {
            moveForward(this->pinMotorLeft, pwm);  // Mueve motor izquierdo hacia adelante
        }
    }
}
// Implementación de los métodos de parada individual (Freno activo)

void robot::fullStopLeftWheel()
{
    digitalWrite(this->pinMotorLeft[1], HIGH);
    digitalWrite(this->pinMotorLeft[2], HIGH);
    analogWrite(this->pinMotorLeft[0], 0);
}

/**
 * Retorna el número de pin asignado al encoder de la rueda izquierda.
 * Se utiliza en setup() para configurar la interrupción (attachInterrupt).
 */
int robot::getPinLeftEncoder()
{
    return this->pinLeftEncoder; // Retorna el pin configurado en pinSetup()
}

/**
 * Retorna el número de pin asignado al encoder de la rueda derecha.
 * Permite al sistema leer la velocidad de la rueda derecha mediante pulsos.
 */
int robot::getPinRightEncoder()
{
    return this->pinRightEncoder; // Retorna el pin configurado en pinSetup()
}
