/**
***************************************************************************
* @file       bsp_buzzer.c/h
* @author
* @version V1.0
* @date 2023/12/10
* @brief
*
*****************************************************************************
* @attention
*
*****************************************************************************
*/

#include "bsp_buzzer.h"

Buzzer_t Buzzer_Control;

//初始化buzzer
void Buzzer_Init(void)
{
	//使能控制buzzer的PWM
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
	//初始时为无声状态
	Buzzer_Control.mode = ZERO_TIME;
	//占空比
	Buzzer_Control.buzzer_value = 0;
	Buzzer_Control.status = OFF;
	Buzzer_Control.continue_times = 0;
	Buzzer_Control.times = 0;
}

void Buzzer_Processing(void)
{
  switch (Buzzer_Control.mode)
  {
  case ZERO_TIME:
    //占空比
    Buzzer_Control.buzzer_value = 0;
	Buzzer_Control.continue_times = 0;
    Buzzer_Control.status = OFF;
	Buzzer_Control.times = 0;
    break;
  case ONE_TIME:
    //响一声持续1s
    Buzzer_Control.continue_times++;
	 if (Buzzer_Control.times >= 1)
    {
      Buzzer_Control.mode = ZERO_TIME;
    }
	//这是一周周期性思想， T = 2 *  BUZZER_CONTINUE_TIMES
    if ((Buzzer_Control.continue_times / BUZZER_CONTINUE_TIMES) % 2 == 0 && Buzzer_Control.status == OFF)
    {
      Buzzer_Control.buzzer_value = BUZZER_VAL;
	  Buzzer_Control.status = ON;
    }
		if((Buzzer_Control.continue_times / BUZZER_CONTINUE_TIMES) % 2 == 1 && Buzzer_Control.status == ON)
		{
			Buzzer_Control.buzzer_value = 0;
			Buzzer_Control.status = OFF;
			Buzzer_Control.times += 1;
		}
    break;
	case HEAPS_TIME:
		Buzzer_Control.buzzer_value = LOW_BUZZER_VAL;
		Buzzer_Control.continue_times = 0;
		Buzzer_Control.status = OFF;			//保证他是从OFF开始 
		Buzzer_Control.times = 0;
		break;
  default:
		Buzzer_Control.continue_times++;
	    if (Buzzer_Control.times == 3)
		{
			Buzzer_Control.mode = ZERO_TIME;
		}
		if ((Buzzer_Control.continue_times / BUZZER_CONTINUE_TIMES) % 2 == 0 && Buzzer_Control.status == OFF)
		{
			Buzzer_Control.buzzer_value = LOW_BUZZER_VAL;
			Buzzer_Control.status = ON;
		}
		if((Buzzer_Control.continue_times / BUZZER_CONTINUE_TIMES) % 2 == 1 && Buzzer_Control.status == ON)
		{
			Buzzer_Control.buzzer_value = 0;
			Buzzer_Control.status = OFF;
			Buzzer_Control.times += 1;
		}
    break;
  }
  __HAL_TIM_SetCompare(&htim12, TIM_CHANNEL_2, Buzzer_Control.buzzer_value); 
}

void Set_BuzzerMode(buzzer_mode_u buzzer_mode)
{
	Buzzer_Control.mode = buzzer_mode;
}

buzzer_mode_u Get_BuzzerMode(void)
{
	return Buzzer_Control.mode;
}


