#pragma once
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "operation.h"
#include "common.h"
#include "dataStructure.h"
#include "controler.h"
#include <MeanFilterLib.h>
#include <Arduino_LSM6DS3.h>
class Robot
{
public:
    Robot();
    ~Robot();
    void initRobot();
    void doOperation(int operationR);
    void sendData(unsigned int operation, byte *data);
    void opHello();
    void opMoveWheel();
    void opStopWheel();
    void opVelRobot();
    void opConfigPID();
    void motorSetup();
    void moveForward(const int pinMotor[3], int speed);
    void moveBackward(const int pinMotor[3], int speed);
    void stopMotor(const int pinMotor[3]);
    void moveWheel(int pwm, double w, const int pinMotor[3], bool back);

    
    struct appdata operation_send;
    struct appdata *server_operation;
    controler wheelControlerRight;
    controler wheelControlerLeft;
    int led = 13;
    bool sendDataSerial;
    //     ---------------motor setup---------------*/
    const int pinENA = 12;//señal de PWM
    const int pinIN1 = 7;//indica sentido de giro
    const int pinIN2 = 8;//indica sentido de giro

    const int pinIN3 = 9;//indica sentido de giro
    const int pinIN4 = 10;
    const int pinENB = 11;//Señal de PWM

    const int pinMotorI[3] = { pinENA, pinIN1, pinIN2 };
    const int pinMotorD[3] = { pinENB, pinIN3, pinIN4 };
    
    double D=6.7;//cm
    double R=3.35; //cm
    double L=9.5; // cm distancia entre ruedas
    //-----------------contador encoder------------------------------------------------------------------------------------
    const int             N=                  20;//Resolucion encoder       
    const int             encoderLeft =          2;//pin de entrada de encoder derecha
    const int             encoderRight=           15;//pin de entrada de encoder izquierda
    volatile unsigned     encoder_countRight=      0;//cuenta los pulsos de encoder derecha
    int                   encoder_countRight_after=0;
    int                   encoder_countLeft_after=0;
    int                   dif_encoderD=0;
    int                   dif_encoderI=0;
    volatile unsigned     encoder_countI=      0;//cuenta los pulsos de encoder izquierda
    volatile int          vueltaD=             0;//cuenta las vueltas que ha dado la rueda derecha
    volatile int          vueltaI=             0;//cuenta las vueltas que ha dado la rueda izquierda
    int                   valorD=              0;//media de tiempo
    // +++++++++++++++  ++++Constantes del controlador+++++++++++++++
    double                kp_right=0.2, ki_right=0.05, D_right = 0;/*KD_p=0.2, KD_i=0.05, KD_d=0;//    KD_p=0.1, KD_i=0.05, KD_d=0;*/
    double                P_left=0.2, ki_left=0.05, kd_left=0;
    //------------------------feedforward Y PWM----------------------------------------------------//
    int    PWM_D=0;
    int    PWM_I=0;
    double setpointWRight=0;
    double setpointWLeft=0;
    double feedForwardParam_A_Right=0.0825;
    double feedForwardParam_B_Right=0.0707;
    double feedForwardParam_A_Left=1.656;
    double feedForwardParam_B_Left=0.0720;
    //-------------------------GIROSCOPIO------------------------------------------------------------------------------------------------//
    float x, y, z;
    const float ERR_GIROSCOPE=3.05;
    int contD=0;
    int contI=0;
    const int MAXFIT=3;//maximum adjustmen that the gyroscope does to the pwm 
    double setPointGWD;
    double setPointGWI;

    bool backD=false,backI=false;


    MeanFilter<long> meanFilterRight(10);
    MeanFilter<long> meanFilterLeft(10);
};