 #ifndef _PID_H
#define _PID_H

#include "main.h"

typedef struct _pid_struct_t
{
  float kp;
  float ki;
  float kd;
  float i_max;
  float out_max;
  
  float tar;      
  float now;      
  float err[3];   //本次误差和上一次误差

  float p_out;
  float i_out;
  float d_out;
  float output;
	
	float delta_u;						//本次增量值
	float delta_out;					//本次增量式输出 = 本次增量加上一次增量
	float last_delta_out;
}pid_struct_t;

typedef struct
{
	pid_struct_t outer;
	pid_struct_t inner;
	float output;
}pid_Cascade_t;

typedef struct
{
	pid_struct_t angle_pid;
	pid_struct_t speed_pid;
	pid_struct_t current_pid;
	float output;
}pid_feedforward_t;

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);
												
extern pid_struct_t motor_pid_shoot[2];
extern pid_Cascade_t motor_pid_Cas_Yaw;
extern pid_Cascade_t motor_pid_Cas_Pitch;
							
extern pid_feedforward_t motor_pid_feedforward_Yaw;
							
extern float yaw_speed_filter[3];
extern float yaw_target_filter[3];
extern float yaw_current_filter[3];
							
extern float pitch_speed_filter[3];
extern float pitch_target_filter[3];
							
float pid_calc(pid_struct_t *pid, float tar, float now);
float pid_calc_shoot(pid_struct_t *pid, float tar, float now);
float pid_calc_cloud(pid_struct_t *pid, float tar, float now);
float pid_CascadeCalc_yaw(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow);
float pid_CascadeCalc_pitch(pid_Cascade_t *pid,float angleTar,float angleNow,float speedNow);
void motor_pid_init(void);
							
float pid_pid_feedforward_yaw(pid_feedforward_t *pid,float angleTar,float angleNow,float speedNow,float currentNow);
	
#endif
