/*
 * ----------------------------------------------------------------------------
 * PROYECTO: Robotarium Hub (UCM)
 * ARCHIVO:  common.h
 * ----------------------------------------------------------------------------
 * DESCRIPCIÓN:
 * Definición de estructuras de datos compartidas (appdata) y tipos globales.
 * IMPORTANTE: Cualquier cambio en este archivo debe replicarse en el código
 * de la unidad de control (Python/C++ en Raspberry Pi) para evitar fallos de trama.
 * ----------------------------------------------------------------------------
 */
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

// Tamaño máximo del paquete de datos que el Arduino puede procesar por Serial
const int MAXDATASIZE=255; 
// Tamaño de la cabecera calculada como 5 enteros (InitFlag, id, op, len...)
const int HEADER_LEN =sizeof(int)*5; 

// Estructura para el intercambio de mensajes con la Raspberry Pi
struct appdata {
  uint16_t InitFlag; // Marcador de inicio de trama (ej: 112)
  uint16_t id;       // Identificador del robot en el enjambre
  uint16_t op;       // Código de la operación a ejecutar (ver operation.h)
  uint16_t len;      // Longitud real de los datos en el array 'data'
  unsigned char data [MAXDATASIZE-HEADER_LEN]; // Carga útil del mensaje
};

/** * SECCIÓN DE UTILIDADES: UNIONES Y CONVERSIÓN DE BYTES
 * Las uniones permiten compartir el mismo espacio de memoria entre tipos complejos y bytes.
 * Se utilizan para enviar datos numéricos a través del flujo Serial de 8 bits.
 */
typedef union { unsigned char b[8]; double i; } mdouble; // Para números de 64 bits
typedef union { unsigned char b[4]; float  i; } mfloat;  // Para números de 32 bits
typedef union { unsigned char b[4]; long   i; } mlong;   // Para enteros de 32 bits
typedef union { unsigned char b[2]; short  i; } mshort;  // Para enteros de 16 bits

// Convierte un float (4 bytes) a un array de caracteres
void floatToBytes(float n, unsigned char *b) {
    int i; mfloat x; x.i = n;
    for(i=0 ; i<4 ; i++) b[i] = x.b[i];
}

// Recompone un float a partir de 4 bytes recibidos
float bytesToFloat(unsigned char *b) {
    int i; mfloat x;
    for(i=0 ; i<4 ; i++) x.b[i] = b[i];
    return(x.i);
}

// Convierte un double (8 bytes) a un array de caracteres
void doubleToBytes(double n, unsigned char *b) {
    int i; mdouble x; x.i = n;
    for(i=0 ; i<8 ; i++) b[i] = x.b[i];
}

// Recompone un double a partir de 8 bytes recibidos
double bytesToDouble(unsigned char *b) {
    int i; mdouble x;
    for(i=0 ; i<8 ; i++) x.b[i] = b[i];
    return(x.i);
}

// Convierte un long (4 bytes) a bytes
void longToBytes(long n, unsigned char *b) {
    int i; mlong x; x.i = n;
    for(i=0 ; i<4 ; i++) b[i] = x.b[i];
}

// Recompone un long a partir de bytes
int bytesToLong(unsigned char *b) {
    int i; mlong x;
    for(i=0 ; i<4 ; i++) x.b[i] = b[i];
    return(x.i);
}

// Convierte un short (2 bytes) a bytes
void shortToBytes(short n, unsigned char *b) {
    mshort x; x.i = n;
    b[1] = x.b[1]; b[0] = x.b[0];
}

// Recompone un short a partir de bytes
short bytesToShort(unsigned char *b) {
    mshort x; x.b[1] = b[1]; x.b[0] = b[0];
    return(x.i);
}

// --- VARIABLES DE CONTROL DE ENCODER ---
const int MAX_ENCODER_STEPS = 20; // Resolución física: pasos por vuelta del encoder

// Almacenan el estado anterior de los contadores para calcular diferenciales
int encoder_countRight_after=0;
int encoder_countLeft_after=0;
int dif_encoderD=0; // Pulsos acumulados en el último ciclo (derecha)
int dif_encoderI=0; // Pulsos acumulados en el último ciclo (izquierda)

// Contadores totales de vueltas de rueda (volátiles para interrupciones)
volatile long long int wheelTurnCounterRight = 0;
volatile long long int wheelTurnCounterLeft = 0;

// Gestión de tiempos para detección de parada (timeout)
volatile unsigned long timeStopD= 0;
volatile unsigned long timeStopI= 0;
volatile unsigned long deltaTimeStopD;
volatile unsigned long deltaTimeStopI;

// --- CONTROL DE MOTORES Y PWM ---
int PWM_Right=0; // Valor actual de PWM enviado al motor derecho
int PWM_Left=0;  // Valor actual de PWM enviado al motor izquierdo

// Setpoints de velocidad y banderas de dirección (back = marcha atrás)
double SetpointD, SetpointI, SetpointAnterior=0;
bool backD=false, backI=false;

// Constantes de configuración de potencia y límites operativos
#define MINPWM 100      // PWM mínimo para que el motor se mueva
#define MAXPWM 255      // PWM máximo (100% potencia)
#define LIM_LINEAL 13.5 
#define MINSETPOINT 5.5 // Velocidad mínima en rad/s para evitar la zona muerta

// --- FILTRO DE DEBOUNCE (ANTI-REBOTES) ---
// Evita lecturas erróneas por ruido eléctrico en las interrupciones del encoder
unsigned long TIMEDEBOUNCE = 12; // Tiempo mínimo (ms) entre pulsos válidos
volatile unsigned long timeAfterDebounceRight= 0;
volatile unsigned long timeBeforeDebounceRight=0;
volatile unsigned long deltaDebounceRight= 0;

volatile unsigned long timeAfterDebounceLeft= 0;
volatile unsigned long timeBeforeDebounceLeft=0;
volatile unsigned long deltaDebounceLeft= 0;

// --- TIEMPOS ENTRE INTERRUPCIONES PARA CÁLCULO DE VELOCIDAD ---
// Se usan para medir el periodo entre pulsos y derivar la velocidad angular
volatile unsigned long startTimeLeft= 0;
volatile unsigned long timeAfterLeft= 0;
volatile unsigned deltaTimeLeft; // Tiempo entre flancos en la rueda izquierda

volatile unsigned long startTimeRight= 0;
volatile unsigned long timeAfterRight= 0;
volatile unsigned deltaTimeRight; // Tiempo entre flancos en la rueda derecha

volatile unsigned encoder_countRight= 0; // Contador de pulsos brutos (derecha)
volatile unsigned encoder_countLeft= 0;  // Contador de pulsos brutos (izquierda)
