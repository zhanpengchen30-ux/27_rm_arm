#include "arm_task.h"
#include "arm.h"
#include "dm_motor.h"
#include "bsp_dwt.h"
#include "cmsis_os2.h"

void Arm_Task(void *argument)
{
    DM_Motor_Init();
    Arm_Init();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2);

    while(1)
    {
        Arm_ControlLoop();
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}