#include <math.h>
#include "MeanFilterLib.h"
#include <Arduino_LSM6DS3.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#define MAXDATASIZE 255
#define HEADER_LEN (sizeof(unsigned short)*3)
#define ID 1;
struct appdata{

        unsigned short id; //identificador
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
//operacion error
#define OP_ERROR            0xFFFF
//operaciones requeridas por central
#define OP_SALUDO           0x0001
#define OP_MOVE_WHEEL       0x0002

#define OP_STOP_WHEEL       0x0003
#define OP_VEL_ROBOT        0X0005
#define OP_POSITION         0x0008
//operaciones cliente
#define OP_MESSAGE_RECIVE   0x0004

#define OP_STOP_SERIAL      0X0006
#define OP_RUN_SERIAL       0X0007
//----------------parametros RObot-------------------
#define D 6.7//cm
#define R 3.35 //cm
#define L 9.5 // cm distancia entre ruedas


     //     ---------------motor setup---------------*/
const int pinENA = 12;//señal de PWM
const int pinIN1 = 7;//indica sentido de giro
const int pinIN2 = 8;//indica sentido de giro

const int pinIN3 = 9;//indica sentido de giro
const int pinIN4 = 10;
const int pinENB = 11;//Señal de PWM

const int pinMotorI[3] = { pinENA, pinIN1, pinIN2 };
const int pinMotorD[3] = { pinENB, pinIN3, pinIN4 };

//-----------------contador encoder------------------------------------------------------------------------------------
const int             N=                  20;//Resolucion encoder       
const int             encoderI =          2;//pin de entrada de encoder derecha
const int             encoderD=           15;//pin de entrada de encoder izquierda
volatile unsigned     encoder_countD=      0;//cuenta los pulsos de encoder derecha
int                   encoder_countD_after=0;
int                   encoder_countI_after=0;
int                   dif_encoderD=0;
int                   dif_encoderI=0;
volatile unsigned     encoder_countI=      0;//cuenta los pulsos de encoder izquierda
volatile int          vueltaD=             0;//cuenta las vueltas que ha dado la rueda derecha
volatile int          vueltaI=             0;//cuenta las vueltas que ha dado la rueda izquierda
int                   valorD=              0;//media de tiempo

//----------T-------IEMPO ENTRE INTERRUPCIONES-----------------------------------------------------------------------//
volatile unsigned long startTimeI=         0;
volatile unsigned long timeAfterI=         0;
volatile unsigned      deltaTimeI;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned long startTimeD=         0;
volatile unsigned long timeAfterD=         0;
volatile unsigned      deltaTimeD;//diferencia de tiempo entre una interrupcion y otra

volatile unsigned long timeStopD=          0;//se usa para poner a cero los radianes cuando el tiempo es muy grande
volatile unsigned long timeStopI=          0;//se usa para poner a cero los radianes cuando el tiempo es muy grande
volatile unsigned long deltaTimeStopD;
volatile unsigned long deltaTimeStopI;

//------------------------debounce-----------------------------------------------------------------------------------
//Se usara para evitar que cuenten pulsos debido a rebotes o ruido del sistema.
# define               TIMEDEBOUNCE        14.5 //(ms) es el tiempo minimo entre pulsos
volatile unsigned long timeAfterDebounceD= 0;
volatile unsigned long timeBeforeDebounceD=0;
volatile unsigned long deltaDebounceD=     0;

volatile unsigned long timeAfterDebounceI= 0;
volatile unsigned long timeBeforeDebounceI=0;
volatile unsigned long deltaDebounceI=     0;

//---------------------variables odometria------------------------------------------------------------------------------------
double                 distanciaI=0;
double                 distanciaD=0;
double wI,wD;//sirven para medir la velocidad de cada rueda
//---------------------VARIABLES DEL CONTROLADOR PID-----------------------------------------------------------------------------
//+++++++++++ variables internas del controlador++++++++++++++
//++++++++++++++++++rueda derecha+++++++++++++++++
unsigned long         currentTimeD, previousTimeD=0;;
double                elapsedTimeD;
double                errorD=0, lastErrorD=0, cumErrorD=0, rateErrorD;
// ++++++++++rueda izquierda++++++++++++
unsigned long         currentTimeI, previousTimeI=0;;
double                elapsedTimeI;
double                errorI=0, lastErrorI=0, cumErrorI=0, rateErrorI;
// +++++++++++++++  ++++Constantes del controlador+++++++++++++++
double                KD_p=0.2, KD_i=0.09, KD_d=0;//    KD_p=0.1, KD_i=0.05, KD_d=0;
double                KI_p=0.15, KI_i=0.06, KI_d=0;//
double                MAXCUMERROR = 5;
//-------------------------GIROSCOPIO------------------------------------------------------------------------------------------------//
float x, y, z;
const float ERR_GIROSCOPE=3.05;
int contD=0;
int contI=0;
const int MAXFIT=3;//maximum adjustmen that the gyroscope does to the pwm 
double setPointGWD;
double setPointGWI;
//------------------------feedforward Y PWM----------------------------------------------------//
int    PWM_D=0;
int    PWM_I=0;
double setpointWD=0;
double setpointWI=0;
double SetpointD,SetpointI,SetpointAnterior=0;//se usa para indicar el valor de referncia es temporal se debera usar uno para cada rueda
bool backD=false,backI=false;
#define MINPWM 96
#define MAXPWM 255
#define LIM_LINEAL 13.5
#define MINSETPOINT 5.5 //this is the minimal speed for every wheel, with that the code avoid the dead zone
//----------------------sampling time variables----------------------------------------------------------------------------
#define SAMPLINGTIME 5000//us
unsigned long currentTime=0, timeAfter=0;
double elapsedTime=0;
