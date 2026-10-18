#include "rc_task.h"
#include "cmsis_os2.h"
#include "bsp_dr16.h"
#include "bsp_buzzer.h"  // 状态切换时可以用蜂鸣器提示

static Robot_Cmd_t robot_cmd = {
    .state = ROBOT_STATE_DISABLED
};

// 开机安全上锁标志：开机时即使遥控器已经是“双中”，也必须先打一次“双下”确认后才能启动！
static uint8_t safety_lock = 1; 

/**
  * @brief  获取机器人当前控制指令
  */
const Robot_Cmd_t* Robot_Get_Cmd(void)
{
    return &robot_cmd;
}

/**
  * @brief  遥控器指令处理任务（周期 10ms）
  */
void RC_Task(void *argument)
{
    // 等待系统各硬件外设就绪
    osDelay(200);

    for (;;)
    {
        // 1. 获取遥控器最新解包数据
        const DR16_Data_t *rc = DR16_Get_Data();

        // 2. 安全保护：遥控器离线检测（掉线强制失能）
        if (!DR16_IsOnline())
        {
            robot_cmd.state = ROBOT_STATE_DISABLED;
            safety_lock = 1; // 掉线后重新上锁，重新连上时必须再打一次双下
        }
        else
        {
            // 拨杆逻辑判断
if (rc->rc.s[0] == DR16_SW_DOWN && rc->rc.s[1] == DR16_SW_DOWN)
{
    robot_cmd.state = ROBOT_STATE_DISABLED;
    safety_lock = 0;
}
else if (rc->rc.s[0] == DR16_SW_MID && rc->rc.s[1] == DR16_SW_MID)
{
    if (safety_lock == 0)
    {
        robot_cmd.state = ROBOT_STATE_ENABLED;
    }
}
            // 其他挡位组合（如左中右下等）：可根据需要扩展其他模式，否则默认保持当前
        }

        // 4. 根据状态处理控制输出
        if (robot_cmd.state == ROBOT_STATE_DISABLED)
        {
            // 失能状态：控制量强制归零，防止机构动作
            robot_cmd.vx_target = 0.0f;
            robot_cmd.vy_target = 0.0f;
            robot_cmd.wz_target = 0.0f;
        }
        else if (robot_cmd.state == ROBOT_STATE_ENABLED)
        {
            // 使能状态：将摇杆原始量（-660 ~ 660）映射为实际物理控制量
            // 例如：最大线速度 3.0 m/s，最大自转角速度 6.0 rad/s
            robot_cmd.vx_target = (float)rc->rc.ch[1] * (3.0f / 660.0f);  // 前后
            robot_cmd.vy_target = -(float)rc->rc.ch[0] * (3.0f / 660.0f); // 左右
            robot_cmd.wz_target = -(float)rc->rc.ch[2] * (6.0f / 660.0f); // 旋转
        }

        // 遥控器 DBUS 刷新周期约为 14ms，任务以 10ms 周期执行
        osDelay(10);
    }
}
