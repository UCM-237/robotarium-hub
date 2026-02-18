/*
 * ----------------------------------------------------------------------------
 * ARCHIVO:  Robot.h
 * PROYECTO: Robotarium Hub (UCM)
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Definición de la clase Robot. Actúa como el HAL (Hardware Abstraction Layer).
 * Contiene la declaración de métodos para control de motores y sensores.
 * ----------------------------------------------------------------------------
 */

 
#pragma once
#include <Arduino.h>

// H-BRIDGE: UNCOMMENT ONLY ONE OPTION
//#define H_BRIDGE_BLACK
#define H_BRIDGE_RED

// ARDUINO TYPE: UNCOMMENT ONLY ONE OPTION
#define ARDUINO_TYPE_MKR
//#define ARDUINO_TYPE_NANO




class robot
{
    public:
        void pinSetup();
        void motorSetup();
        void set_wheel_speed(int wheel, int direction, int speed) ;
        void moveForward(const int pinMotor[3], int speed);
        void moveBackward(const int pinMotor[3], int speed);
        void fullStop();
        void fullStopRightWheel();
        void fullStopLeftWheel();
        void moveRightWheel(int pwm, double w,bool back);
        void moveLeftWheel(int pwm, double w,bool back);
        double getRobotWheelDiameter();
        double getRobotWheelRadius();
        double getRobotDiameter();
        uint8_t getRobotID();
        int getPinLeftEncoder();
        int getPinRightEncoder();
        
       
    private:
        int pinIN1;
        int pinIN2;
        int pinENA;
        int pinIN3;
        int pinIN4;
        int pinENB;
        int pinMotorRight[3];
        int pinMotorLeft[3];
        int pinLeftEncoder;
        int pinRightEncoder;
        

        double RobotWheelDiamter = 6.7;
        double RobotWheelRadius = 3.35;
        double RobotDiameter = 14.5;
        uint8_t robotID=5;
        const int LEFT_WHEEL = 0;
        const int RIGHT_WHEEL = 1;




};
