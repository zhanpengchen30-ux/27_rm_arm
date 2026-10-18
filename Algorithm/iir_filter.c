#include "iir_filter.h"
#include "string.h"

/**
 * @brief			二阶波巴特沃斯滤波
 * @param[out]		input : 输入数据
 * @param[in]		output : 原始数据
 */

void int_Batwolf(int16_t *input,int16_t *output,int16_t* x)
{
	
	x[0]=0.067f*(*input)+1.14f*x[1]-0.412f*x[2];
    *output=x[0]+x[1]*2.0f+x[2];
     	
    x[2]=x[1];
	
	x[1]=x[0];
	
}


void float_Batwolf(float *input,float *output,float* y)
{
	
	y[0]=0.067f*(*input)+1.14f*y[1]-0.412f*y[2];
    *output=y[0]+y[1]*2.0f+y[2];
     	
    y[2]=y[1];
	
	y[1]=y[0];
	
}

/**
 * @brief			将滤波系数清零
 */
void ClearFilter(float filter[],int size)
{
	memset(filter,0,size * sizeof(int));
}

