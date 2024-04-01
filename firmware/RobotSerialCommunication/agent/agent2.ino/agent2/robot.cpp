#include "robot.h"

Robot::Robot()
{
    this->motorSetup();
    this->stopMotor(this->pinMotorI);
    this->stopMotor(this->pinMotorD);
}

Robot::~Robot()
{
}

void Robot::initRobot()
{
    this->motorSetup();
    this->stopMotor(this->pinMotorI);
    this->stopMotor(this->pinMotorD);
    this->wheelControlerRight.setControlerParam(kp_right, ki_right, kd_right);
    this->wheelControlerLeft.setControlerParam(kp_left, ki_left, kd_left);
    wheelControlerRight.setFeedForwardParam(feedForwardParam_A_Right,feedForwardParam_B_Right);
    wheelControlerLeft.setFeedForwardParam(feedForwardParam_A_Left,feedForwardParam_B_Left);
}

#include <cstdint> // Include the necessary header file

void Robot::doOperation(int operation)
{
    switch (operation) 
    {
        case OP_SALUDO:
            this->opHello();
        break;
        case OP_MOVE_WHEEL:
            this->opMoveWheel();
            digitalWrite(this->led, HIGH);
        break;
        case OP_STOP_WHEEL:
            this->opStopWheel();
        break;
        case OP_VEL_ROBOT:
            this->sendDataSerial = true;
        break;
        case OP_CONF_PID:
            this->opConfigPID();
        default:
        break;
    }
    
}

void Robot::sendData(unsigned int operation, byte *data) // Replace "byte" with "std::uint8_t"
{
    operation_send.op = operation;
    operation_send.len = sizeof(data);
    Serial1.write((char*)&operation_send.op, 2);
    Serial1.write((char*)&operation_send.len, 2);
    Serial1.write((char*)&data, operation_send.len);
    Serial1.flush();
}

void Robot::opHello()
{
     operation_send.op = OP_SALUDO;

  operation_send.len = sizeof (operation_send.data);  /* len */
  Serial1.write((char*)operation_send.data, operation_send.len + HEADER_LEN);
  Serial1.flush();
//  send(ID, OP_SALUDO, )
}

void Robot::opMoveWheel()
{
    Serial.println("moveWheel");
    digitalWrite(this->led, LOW);
    this->setpointWRight = bytesToDouble(&server_operation->data[0]);
    this->setpointWLeft = bytesToDouble(&server_operation->data[8]);
    Serial.println(setpointWRight);
    Serial.println(setpointWLeft);
    if(this->setpointWRight < 1 && this->setpointWRight > -1) {
        this->setpointWRight = 0;
    }
    if(this->setpointWLeft < 1 && this->setpointWLeft > -1) {
        this->setpointWLeft = 0;
    }
    if(this->setpointWRight < 0) {
        this->setPointGWD = this->setpointWRight;
        this->setpointWRight = this->setpointWRight*(-1);
        this->backD = true;
    } else if(this->setpointWRight > 0) {
        this->backD = false;
    }
    if(this->setpointWLeft < 0) {
        this->setPointGWI = this->setpointWLeft;
        this->setpointWLeft = this->setpointWLeft*(-1);
        this->backI = true;
    }
    else if(this->setpointWLeft>0) {
        this->backI = false;
    }
    this->PWM_D=this->wheelControlerRight.setSetPoint(setpointWRight);
    this->PWM_I=this->wheelControlerLeft.setSetPoint(setpointWLeft);
    // feedForwardD();
    // feedForwardI();  
    moveWheel(this->PWM_I, this->setpointWLeft, this->pinMotorI, this->backI);
    moveWheel(this->PWM_D, this->setpointWRight, this->pinMotorD, this->backD);
    for(int i=0; i<20; i++){
        this->meanFilterRight.AddValue(deltaTimeD);
        this->meanFilterLeft.AddValue(deltaTimeI);
    }
}

void Robot::opStopWheel()
{
    setpointWRight=0;
    setpointWLeft=0;
    stopMotor(pinMotorI);
    stopMotor(pinMotorD);
}

void Robot::opVelRobot()
{
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

void Robot::opConfigPID()
{
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

void Robot::motorSetup()
{
    pinMode(pinIN1, OUTPUT);
    pinMode(pinIN2, OUTPUT);
    pinMode(pinENA, OUTPUT);
    pinMode(pinIN3, OUTPUT);
    pinMode(pinIN4, OUTPUT);
    pinMode(pinENB, OUTPUT);
}

void Robot::moveForward(const int pinMotor[3], int speed)
{
    digitalWrite(pinMotor[1], HIGH);
    digitalWrite(pinMotor[2], LOW);
    analogWrite(pinMotor[0], speed);
}

void Robot::moveBackward(const int pinMotor[3], int speed)
{
    digitalWrite(pinMotor[1], LOW);
    digitalWrite(pinMotor[2], HIGH);
    analogWrite(pinMotor[0], speed);
}

void Robot::stopMotor(const int pinMotor[3])
{
    digitalWrite(pinMotor[1], HIGH);
    digitalWrite(pinMotor[2], HIGH);
    analogWrite(pinMotor[0], 0);
}

void Robot::moveWheel(int pwm, double w, const int pinMotor[3], bool back)
{
    if(pwm == 0 || ((int)w) == 0)
    {
        stopMotor(pinMotor);
    } 
    else 
    {
        if(back)
        {
            moveBackward(pinMotor, pwm);
        } 
        else if(!back) 
        {
            moveForward(pinMotor, pwm);
        }
    }
}
