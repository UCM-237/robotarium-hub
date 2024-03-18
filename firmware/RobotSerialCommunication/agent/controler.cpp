#include "controler.h"

controler::controler()
{
}

controler::~controler()
{
}

void controler::setControlerParam(double kp, double ki, double kd)
{
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

int controler::pid(double w)
{   
    int output;
    this->currentTime= millis();
    this->elapsedTime = (double)(currentTime - previousTime);
    this->error = this->setPoint - w;
    double aux = abs(this->error);
    if(aux>=0.3)
    {
        this->cumError +=this->error*this->elapsedTime;
        if(this->lastError>0 && this->error<0)
        {
            this->cumError = this->error*this->elapsedTime;
        }
        else if(this->lastError<0 && this->error>0)
        {
            this->cumError = this->error*this->elapsedTime;
        }
        constrain(this->cumError, -MAXCUMERROR,MAXCUMERROR);
        this->rateError = (this->error - this->lastError)/this->elapsedTime;
        output = static_cast<int>(round(this->kp*this->error + this->ki*this->cumError + this->kd*this->rateError));
        this->lastError = this->error;
    }
    this->previousTime = this->currentTime;
    return output;
}

void controler::setFeedForwardParam(double A, double B)
{
    this->feedForwardParam_A = A;
    this->feedForwardParam_B = B;
}

int controler::feedForward()
{
    this->PWM = (this->setPoint !=0.0)? constrain(round((this->setPoint - feedForwardParam_A)/feedForwardParam_B), MINPWM, MAXPWM) : 0;
    return this->PWM;
}

void controler::setSetPoint(double setPoint)
{
    this->setPoint = setPoint;
}
