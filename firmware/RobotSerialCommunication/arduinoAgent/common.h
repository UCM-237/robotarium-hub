#pragma once
#include <math.h>
#include <MeanFilterLib.h>
#include <Arduino_LSM6DS3.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include "operation.h"
const int MAXDATASIZE=255;
const int HEADER_LEN (sizeof(unsigned short)*3);


struct appdata {
  unsigned short InitFlag;
  unsigned short id;
  unsigned short op; //codigo de operacion
  unsigned short len;                       /* longitud de datos */
  unsigned char data [MAXDATASIZE-HEADER_LEN];//datos
};
typedef union { unsigned char b[8]; double i; } mdouble;
typedef union { unsigned char b[4]; float  i; } mfloat;
typedef union { unsigned char b[4]; long   i; } mlong;
typedef union { unsigned char b[2]; short  i; } mshort;
void floatToBytes(float n, unsigned char *b)
{
    int i;
    mfloat x;

    x.i = n;
    for(i=0 ; i<4 ; i++) b[i] = x.b[i];
}

float bytesToFloat(unsigned char *b)
{
    int i;
    mfloat x;

    for(i=0 ; i<4 ; i++) x.b[i] = b[i];
    return(x.i);
}
void doubleToBytes(double n, unsigned char *b)
{
    int i;
    mdouble x;

    x.i = n;
    for(i=0 ; i<8 ; i++) b[i] = x.b[i];
}
double bytesToDouble(unsigned char *b)
{
    int i;
    mdouble x;

    for(i=0 ; i<8 ; i++) x.b[i] = b[i];
    return(x.i);
}
void longToBytes(long n, unsigned char *b)
{
    int i;
    mlong x;

    x.i = n;
    for(i=0 ; i<4 ; i++) b[i] = x.b[i];
}
void shortToBytes(short n, unsigned char *b)
{
    mshort x;

    x.i = n;
    b[1] = x.b[1];
    b[0] = x.b[0];
}
short bytesToShort(unsigned char *b)
{
    mshort x;

    x.b[1] = b[1];
    x.b[0] = b[0];
    return(x.i);
}

//-----------------contador encoder------------------------------------------------------------------------------------
const int             N=                  20;//Resolucion encoder       
const int             encoderLeft =          2;//pin de entrada de encoder derecha
const int             encoderRight=           15;//pin de entrada de encoder izquierda

int                   encoder_countRight_after=0;
int                   encoder_countLeft_after=0;
int                   dif_encoderD=0;
int                   dif_encoderI=0;

volatile int          vueltaD=             0;//cuenta las vueltas que ha dado la rueda derecha
volatile int          vueltaI=             0;//cuenta las vueltas que ha dado la rueda izquierda
int                   valorD=              0;//media de tiempo



volatile unsigned long timeStopD=          0;//se usa para poner a cero los radianes cuando el tiempo es muy grande
volatile unsigned long timeStopI=          0;//se usa para poner a cero los radianes cuando el tiempo es muy grande
volatile unsigned long deltaTimeStopD;
volatile unsigned long deltaTimeStopI;

//------------------------feedforward Y PWM----------------------------------------------------//
int    PWM_Right=0;
int    PWM_Left=0;

double SetpointD,SetpointI,SetpointAnterior=0;//se usa para indicar el valor de referncia es temporal se debera usar uno para cada rueda
bool backD=false,backI=false;
#define MINPWM 100
#define MAXPWM 255
#define LIM_LINEAL 13.5
#define MINSETPOINT 5.5 //this is the minimal speed for every wheel, with that the code avoid the dead zone

//------------------------debounce-----------------------------------------------------------------------------------
//Se usara para evitar que cuenten pulsos debido a rebotes o ruido del sistema.
unsigned long              TIMEDEBOUNCE        =15; //(ms) es el tiempo minimo entre pulsos
volatile unsigned long timeAfterDebounceD= 0;
volatile unsigned long timeBeforeDebounceD=0;
volatile unsigned long deltaDebounceD=     0;

volatile unsigned long timeAfterDebounceI= 0;
volatile unsigned long timeBeforeDebounceI=0;
volatile unsigned long deltaDebounceI=     0;
//----------T-------IEMPO ENTRE INTERRUPCIONES-----------------------------------------------------------------------//
volatile unsigned long startTimeI=         0;
volatile unsigned long timeAfterI=         0;
volatile unsigned      deltaTimeLeft;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned long startTimeD=         0;
volatile unsigned long timeAfterD=         0;
volatile unsigned      deltaTimeRight;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned     encoder_countRight=      0;//cuenta los pulsos de encoder derecha
volatile unsigned     encoder_countI=      0;//cuenta los pulsos de encoder izquierda