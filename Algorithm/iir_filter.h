#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include "main.h"

void int_Batwolf(int16_t *input,int16_t *output,int16_t* x);
void float_Batwolf(float *input,float *output,float* y);
void ClearFilter(float filter[],int size);

#endif


