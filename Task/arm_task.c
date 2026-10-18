#include "arm_task.h"
#include "arm.h"
#include "dm_motor.h"
#include "bsp_dwt.h"

void Arm_Task(void *argument)
{
    // 初始化达妙电机
    DM_Motor_Init();

    // 初始化机械臂
    Arm_Init();

    // 控制频率 500Hz = 2ms
    uint32_t period = 2;

    while(1)
    {
        // 跑控制循环
        Arm_ControlLoop();

        // 周期延时
        osDelay(period);
    }
}
