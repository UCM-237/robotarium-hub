#pragma once
#include <Arduino.h>

class robot
{
    public:
        void pinSetup();
        void motorSetup();
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
       
    private:
        int pinIN1;
        int pinIN2;
        int pinENA;
        int pinIN3;
        int pinIN4;
        int pinENB;
        int pinMotorRight[3];
        int pinMotorLeft[3];

        double RobotWheelDiamter = 6.7;
        double RobotWheelRadius = 3.35;
        double RobotDiameter = 14.5;
        uint8_t robotID;





};
