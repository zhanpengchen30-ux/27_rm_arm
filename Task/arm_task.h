#ifndef ARM_TASK_H
#define ARM_TASK_H

#include "main.h"

// 机械臂任务入口
void Arm_Task(void *argument);

// 监控任务入口：喂 IWDG + 控制环挂死自动复位
void Mon_Task(void *argument);

#endif
