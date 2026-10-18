#ifndef RC_TASK_H
#define RC_TASK_H

#include "main.h"
#include "bsp_dr16.h"

// 机器人运行状态枚举
typedef enum
{
    ROBOT_STATE_DISABLED = 0, // 失能模式（电机断电、停止输出）
    ROBOT_STATE_ENABLED,      // 启动使能模式（正常控制）
} Robot_State_e;

// 机器人指令结构体（供底盘、云台、发射机构读取）
typedef struct
{
    Robot_State_e state;      // 当前运行状态
    
    // 目标控制量（摇杆映射结果）
    float vx_target;          // 底盘前后线速度 (m/s)
    float vy_target;          // 底盘左右线速度 (m/s)
    float wz_target;          // 底盘自转角速度 (rad/s)
    float pitch_target;       // 云台目标俯仰角
    float yaw_target;         // 云台目标航向角
} Robot_Cmd_t;

// 获取当前机器人指令（对外接口）
const Robot_Cmd_t* Robot_Get_Cmd(void);

// 任务函数原型声明（匹配 CMSIS-RTOS V2）
void RC_Task(void *argument);

#endif /* RC_TASK_H */
