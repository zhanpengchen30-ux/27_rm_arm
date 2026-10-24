#include "arm_task.h"
#include "arm.h"
#include "dm_motor.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

void Arm_Task(void *argument)
{
    // 初始化达妙电机
    DM_Motor_Init();

    // 初始化机械臂
    Arm_Init();

    /* Fixed 2ms cadence. Per-joint delays inside Arm_ControlLoop used to make
       the real period depend on loop work and task scheduling. */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2);

    while(1)
    {
        // 跑控制循环
        Arm_ControlLoop();

        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
