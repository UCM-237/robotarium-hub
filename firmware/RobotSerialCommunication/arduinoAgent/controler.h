#pragma once
#include <Arduino.h>

#define MINPWM 110
#define MAXPWM 255
class controler
{
    public:
        controler();
        ~controler();
        void setControlerParam(double kp, double ki, double kd);
        int pid(double w);
        void setFeedForwardParam(double A, double B);
        int feedForward();
        void setSetPoint(double setPoint);
        double getSetPoint();
    private:
        unsigned long   currentTime, previousTime=0;;
        double          elapsedTime;
        double          error=0, lastError=0, cumError=0, rateError;
        double         setPoint;
        double          kp, ki, kd;
        int            PWM;
        double feedForwardParam_A;
        double feedForwardParam_B;
        const double maxIntegralError=5;
        

};