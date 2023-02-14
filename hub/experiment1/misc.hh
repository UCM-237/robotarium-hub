#ifndef MISC_H
#define MISC_H 1

#include <string>
#include <iostream>
#include <string>
#include <vector>
#include "tinyxml2.h"
#include <math.h>
#include <unistd.h>

typedef union { unsigned char b[8]; double i; } mdouble;
typedef union { unsigned char b[4]; float  i; } mfloat;
typedef union { unsigned char b[4]; long   i; } mlong;
typedef union { unsigned char b[2]; short  i; } mshort;

void   floatToBytes     (float  n, unsigned char *b);
void   doubleToBytes    (double n, unsigned char *b);
void   longToBytes      (long   n, unsigned char *b);
void   shortToBytes     (short  n, unsigned char *b);
float  bytesToFloat     (unsigned char  *b);
double bytesToDouble    (unsigned char  *b);
long   bytesToLong      (unsigned char  *b);
short  bytesToShort     (unsigned char  *b);
void   floatToBytes_inv (float  n, unsigned char *b);
void   doubleToBytes_inv(double n, unsigned char *b);
void   longToBytes_inv  (long   n, unsigned char *b);
void   shortToBytes_inv (short  n, unsigned char *b);
float  bytesToFloat_inv (unsigned char  *b);
double bytesToDouble_inv(unsigned char  *b);
long   bytesToLong_inv  (unsigned char  *b);
short  bytesToShort_inv (unsigned char  *b);

#endif
