#ifndef _RAMP_H
#define _RAMP_H


#include <stdio.h>
#include <stdint.h>

extern float RAMP_float( float final, float now, float ramp );
extern float RampInc_float( float *buffer, float now, float ramp );

#endif
