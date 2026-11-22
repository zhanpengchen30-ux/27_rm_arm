#include "arm_task.h"
#include "arm.h"
#include "dm_motor.h"
#include "bsp_dwt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

void Arm_Task(void *argument)
{
    // 初始化机械臂（内部已含 DM_Motor_Init，别在这里重复调）
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

/* ===== 监控任务：喂硬件看门狗 + 控制环挂死判定 ==========================
 * IWDG 6s：比 Flash 保存的最坏全片停顿(~2s)长，保存期间不会误复位；
 * 调度器或本任务自己死掉时，硬件看门狗直接复位整机。
 * 控制环心跳停 500ms（且不在 Flash 窗口内）= 挂死：先失能全部电机再复位。 */
void Mon_Task(void *argument)
{
    /* 调试时冻结 IWDG（H723 的 IWDG 在 APB4FZ1）：不冻结的话停在断点上 6s 就被复位 */
    __HAL_DBGMCU_FREEZE_IWDG1();

    /* 工程没带 HAL 的 iwdg 驱动，直接操作寄存器：
       LSI≈32kHz，/64 = 500Hz，RLR=3000 → 6s 超时；启动后无法关闭。 */
    IWDG1->KR  = 0x5555u;              /* 解锁 PR/RLR */
    IWDG1->PR  = 4u;                   /* 分频 /64 */
    IWDG1->RLR = 3000u;                /* 3000/500Hz = 6s */
    /* 同步正常几百µs完成；标志卡住也必须启动（按已生效配置跑），
       无限等在这里 = 整机永远失去看门狗保护 */
    for(uint32_t t = 0; (IWDG1->SR & 0x03u) != 0u && t < 1000000u; t++);
    IWDG1->KR  = 0xCCCCu;              /* 开始计数 */

    while(1)
    {
        osDelay(50);
        IWDG1->KR = 0xAAAAu;           /* 喂狗 */
        uint32_t now = HAL_GetTick();
        if(g_arm_loop_tick == 0u)
        {
            if(now > 10000u) Arm_EmergencyStopAndReset();   /* 10s 还进不了控制环 = 初始化挂死 */
            continue;
        }
        if(g_arm_busy_until != 0u && (int32_t)(now - g_arm_busy_until) < 0) continue;
        if(now - g_arm_loop_tick > 500u) Arm_EmergencyStopAndReset();
    }
}
