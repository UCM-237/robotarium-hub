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
const int MAXDATASIZE=255; // bytes
const int HEADER_LEN =sizeof(int)*5; //bytes


struct appdata {
  int InitFlag;
  int id;
  int op; //codigo de operacion
  int len;                       /* longitud de datos */
  unsigned char data [MAXDATASIZE-HEADER_LEN];//datos
};

//Utilities
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
int bytesToLong(unsigned char *b)
{
    int i;
    mlong x;

    for(i=0 ; i<4 ; i++) x.b[i] = b[i];
    return(x.i);
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
const int             MAX_ENCODER_STEPS =                      20;//Resolucion encoder       
const int             encoderLeft =             2;//pin de entrada de encoder derecha
const int             encoderRight=             15;//pin de entrada de encoder izquierda

int                   encoder_countRight_after=0;
int                   encoder_countLeft_after=0;
int                   dif_encoderD=0;
int                   dif_encoderI=0;

volatile long long int          wheelTurnCounterRight=             0;//cuenta las vueltas que ha dado la rueda derecha
volatile long long int          wheelTurnCounterLeft=             0;//cuenta las vueltas que ha dado la rueda izquierda



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
unsigned long              TIMEDEBOUNCE        =10; //(ms) es el tiempo minimo entre pulsos
volatile unsigned long timeAfterDebounceRight= 0;
volatile unsigned long timeBeforeDebounceRight=0;
volatile unsigned long deltaDebounceRight=     0;

volatile unsigned long timeAfterDebounceLeft= 0;
volatile unsigned long timeBeforeDebounceLeft=0;
volatile unsigned long deltaDebounceLeft=     0;
//----------T-------IEMPO ENTRE INTERRUPCIONES-----------------------------------------------------------------------//
volatile unsigned long startTimeLeft=         0;
volatile unsigned long timeAfterLeft=         0;
volatile unsigned      deltaTimeLeft;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned long startTimeRight=         0;
volatile unsigned long timeAfterRight=         0;
volatile unsigned      deltaTimeRight;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned     encoder_countRight=      0;//cuenta los pulsos de encoder derecha
volatile unsigned     encoder_countLeft=      0;//cuenta los pulsos de encoder izquierda