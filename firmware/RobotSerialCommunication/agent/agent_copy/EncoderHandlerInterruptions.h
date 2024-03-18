/**
 * @file EncoderHandler.h
*/

#pragma once
#include <Arduino.h>

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
void isrRight() {
  timeBeforeDebounceD = millis();//tiempo para evitar rebotes
  deltaDebounceD = timeBeforeDebounceD - timeAfterDebounceD;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceD > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeD=micros();
    encoder_countRight++;
      deltaTimeRight = startTimeD - timeAfterD;
      
      timeAfterD = micros();
  }
  timeAfterDebounceD = millis();  
}

void isrLeft() {
  timeBeforeDebounceI = millis();//tiempo para evitar rebotes
  deltaDebounceI = timeBeforeDebounceI - timeAfterDebounceI;// tiempo que ha pasdo entre interrupcion e interrupcion
  if(deltaDebounceI > TIMEDEBOUNCE) {//condicion para evitar rebotes
    //se empieza a contar el tiempo que ha pasado entre una interrupcion "valida" y otra.    
    startTimeI = micros();
    encoder_countI++;//se cuenta los pasos de encoder
    deltaTimeLeft = startTimeI - timeAfterI;
    encoder_countI = 0;
    timeAfterI = micros();
  }
  timeAfterDebounceI = millis();
}
