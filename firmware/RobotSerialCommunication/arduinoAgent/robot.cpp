#include "robot.h"

void robot::pinSetup()
{
    this->pinENA = 12;
    this->pinIN1 = 7;
    this->pinIN2 = 8;

    this->pinIN3 = 9;
    this->pinIN4 = 10;
    this->pinENB = 11;

    this->pinMotorLeft[0] = this->pinENA;
    this->pinMotorLeft[1] = this->pinIN1;
    this->pinMotorLeft[2] = this->pinIN2;

    this->pinMotorRight[0] = this->pinENB;
    this->pinMotorRight[1] = this->pinIN3;
    this->pinMotorRight[2] = this->pinIN4;
}

void robot::motorSetup()
{
    pinMode(this->pinIN1, OUTPUT);
    pinMode(this->pinIN2, OUTPUT);
    pinMode(this->pinENA, OUTPUT);
    pinMode(this->pinIN3, OUTPUT);
    pinMode(this->pinIN4, OUTPUT);
    pinMode(this->pinENB, OUTPUT);
}

void robot::moveForward(const int pinMotor[3], int speed)
{
    digitalWrite(pinMotor[1], HIGH);
    digitalWrite(pinMotor[2], LOW);
    analogWrite(pinMotor[0], speed);
}

void robot::moveBackward(const int pinMotor[3], int speed)
{
    digitalWrite(pinMotor[1], LOW);
    digitalWrite(pinMotor[2], HIGH);
    analogWrite(pinMotor[0], speed);
}

void robot::fullStop()
{
    this->fullStopLeftWheel();
    this->fullStopRightWheel();
}

void robot::fullStopRightWheel()
{
    digitalWrite(this->pinMotorRight[1], HIGH);
    digitalWrite(this->pinMotorRight[2], HIGH);
    analogWrite(this->pinMotorRight[0], 0);
}

void robot::fullStopLeftWheel()
{
    digitalWrite(this->pinMotorLeft[1], HIGH);
    digitalWrite(this->pinMotorLeft[2], HIGH);
    analogWrite(this->pinMotorLeft[0], 0);
}

void robot::moveRightWheel(int pwm, double w, bool back)
{
    if(pwm == 0 || ((int)w) == 0)
    {
        fullStopRightWheel();
    } 
    else 
    {
        if(back) 
        {
            moveBackward(this->pinMotorRight, pwm);
        } 
        else if(!back) 
        {
            moveForward(this->pinMotorRight, pwm);
        }
    }
}

void robot::moveLeftWheel(int pwm, double w, bool back)
{
    if(pwm == 0 || ((int)w) == 0)
    {
        fullStopLeftWheel();
    } 
    else 
    {
        if(back) 
        {
            moveBackward(this->pinMotorLeft, pwm);
        } 
        else if(!back) 
        {
            moveForward(this->pinMotorLeft, pwm);
        }
    }
}
