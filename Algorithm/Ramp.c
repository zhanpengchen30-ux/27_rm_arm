#include "Ramp.h"


/**
  * @brief  浮点数斜坡函数
  * @param  final: 最终目标值
  * @param  now:   当前值
  * @param  ramp:  每调用一次的最大变化量 (斜率)
  * @retval        计算后的新当前值
  */
float RAMP_float(float final, float now, float ramp)
{
    float buffer = final - now;

    // 限制斜坡步长不能为负
    if(ramp < 0.0f)
		{
        ramp = -ramp;
    }

    if(buffer > ramp)
		{
        now += ramp;
    }
		else if(buffer < -ramp)
		{
        now -= ramp;
    }
		else
		{
        now = final;
    }
    
    return now;
}





/**
  * @brief  斜坡函数,使目标输出值缓慢等于指针输入值
  * @param  要在当前输出量上累加的值,目标输出量,递增快慢
  * @retval 目标输出量           
	*/
float RampInc_float( float *buffer, float now, float ramp )
{
		if (*buffer > 0)
		{
				if (*buffer > ramp)
				{  
						now     += ramp;
					  *buffer -= ramp;
				}   
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		else
		{
				if (*buffer < -ramp)
				{
						now     += -ramp;
					  *buffer -= -ramp;
				}
				else
				{
						now     += *buffer;
					  *buffer  = 0;
				}
		}
		
		return now;
}

