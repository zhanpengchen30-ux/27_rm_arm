#include "pid.h"
#include "iir_filter.h"
#include "INS_task.h"

pid_struct_t motor_pid_shoot[2];
pid_Cascade_t motor_pid_Cas_Yaw;
pid_Cascade_t motor_pid_Cas_Pitch;

pid_feedforward_t motor_pid_feedforward_Yaw;

float yaw_speed_filter[3] = {0.0f};
float yaw_target_filter[3] = {0.0f};
float yaw_current_filter[3] = {0.0f};

float pitch_speed_filter[3] = {0.0f};
float pitch_target_filter[3] = {0.0f};

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/**
  * @brief PID参数初始化
  */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}






//位置式pid计算
float PID_Calculate(PID_Controller *pid, float target, float measured )
{
    pid->target = target;
    pid->measured = measured;
    pid->error = pid->target - pid->measured;

    pid->p_out = pid->Kp * pid->error;

    pid->i_out += pid->Ki * pid->error;

    if(pid->i_out > pid->Integral_Limit) pid->i_out = pid->Integral_Limit;//积分限幅
    else if(pid->i_out < -pid->Integral_Limit) pid->i_out = -pid->Integral_Limit;

    pid->d_out = pid->Kd * (pid->error - pid->pre_error);

    float output = pid->p_out + pid->i_out + pid->d_out;

    if(output > pid->MaxOutput) {output = pid->MaxOutput;}//输出限幅
    else if(output < -pid->MaxOutput) {output = -pid->MaxOutput;}

    pid->pre_error = pid->error;
    return output;
}






/**
  * @brief  PID位置式计算
  */
float pid_calc(pid_struct_t *pid, float tar, float now)//位置式
{
  pid->tar = tar;
  pid->now = now;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
  
  pid->p_out  = pid->kp * pid->err[0];//比例部分
  pid->i_out += pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);//微分部分
  LimitMax(pid->i_out,pid->i_max);//限幅
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;//控制量
  LimitMax(pid->output,pid->out_max);
  return pid->output;
}

/**
  * @brief  PID增量式计算
  */
float pid_calc_shoot(pid_struct_t *pid, float tar, float now)//增量式
{
	pid->tar = tar;
  pid->now = now;
	pid->err[2] = pid->err[1];
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->tar - pid->now;
	
	pid->p_out  = pid->kp * (pid->err[0] - pid->err[1]);//比例部分
  pid->i_out  = pid->ki * pid->err[0];//积分部分
  pid->d_out  = pid->kd * (pid->err[0] - 2*pid->err[1] + pid->err[2]);//微分部分
	
	LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);//限幅
	
  pid->delta_u = pid->p_out + pid->i_out + pid->d_out;//控制量
	pid->delta_out = pid->last_delta_out + pid->delta_u;
	
	LIMIT_MIN_MAX(pid->delta_out, -pid->out_max, pid->out_max);
	pid->last_delta_out = pid->delta_out;
  return pid->delta_out;
}




