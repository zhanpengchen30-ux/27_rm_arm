#ifndef _LADRC_H
#define _LADRC_H

#include "main.h"

typedef struct
{
    float h;
    float r;
    float w0;
    float wc;
    float b0;
    float u;
}para_t;

typedef struct
{
    float v1;
    float v2;
    float z1;
    float z2;
    float z3;
}process_t;

typedef struct 
{
    para_t para;
    process_t process;
}ladrc_t;

void ADRC_Init(ladrc_t *LADRC,float h,float r,float w0,float wc,float b0);
void ladrc_cal(ladrc_t *LADRC,float set,float get);

#endif
