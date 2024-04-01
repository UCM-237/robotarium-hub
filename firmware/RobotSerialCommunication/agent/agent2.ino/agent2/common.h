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

