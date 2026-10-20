#include "arm.h"
#include <string.h>
#include <math.h>
#include "bsp_dwt.h"
#include "usart.h"
#include "bsp_dr16.h"
#include "dm_motor.h"
#include "cmsis_os2.h"

static uint8_t zigbee_rx_byte;
Arm_t arm; // 导出供 Watch 查看
static uint8_t zigbee_buf[32];
static uint8_t zigbee_idx = 0;

#define JOINT_MAX_VEL       2.5f    // 主从跟随最大角速度 rad/s
#define ZIGBEE_TIMEOUT_MS   200     // 主手掉线超时判定（毫秒）

// ==================== 1. 主从方向映射与安全电流 ====================

// 关节方向映射：J2, J3, J4 按真机实测设为 -1.0f
static const float ARM_DIR[ARM_JOINT_NUM] = {
    1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f
};

// 各关节软限位（⚠️注意：J2 的最小限位根据实际机械结构设置，提前5度刹停，防止肉身撞墙堵转！）
static const float ARM_Q_MIN[ARM_JOINT_NUM] = { -3.10f, -2.80f, -0.65f, -3.14f, -3.14f, -3.14f};
static const float ARM_Q_MAX[ARM_JOINT_NUM] = { 0.05f,  0.05f,  1.90f,  3.14f,  3.14f,  3.14f};

// 各关节出力电流上限（J2 大臂给 1.8A 托举，其他给 1.2A）
static const float ARM_CUR_LIMIT[ARM_JOINT_NUM] = {
    1.2f,  // J1 底座
    1.8f,  // J2 大臂 (抗重力)
    1.5f,  // J3 小臂
    1.0f,  // J4 腕部
    1.0f,  // J5 腕部
    1.0f   // J6 腕部
};

static float Arm_ClampJoint(int i, float v)
{
    if(v < ARM_Q_MIN[i]) return ARM_Q_MIN[i];
    if(v > ARM_Q_MAX[i]) return ARM_Q_MAX[i];
    return v;
}

// ==================== 2. 自定义控制器串口解析（15字节完美匹配） ====================
void Arm_Zigbee_ParseByte(uint8_t byte)
{
    if(zigbee_idx == 0)
    {
        if(byte == 0xAA)
        {
            zigbee_buf[zigbee_idx++] = byte;
        }
        return;
    }

    zigbee_buf[zigbee_idx++] = byte;

    // 匹配 15 字节协议
    if(zigbee_idx == 15)
    {
        if(zigbee_buf[14] == 0x55) 
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                int16_t angle_raw = (int16_t)(zigbee_buf[1 + i*2] | (zigbee_buf[2 + i*2] << 8));
                arm.target_joint[i] = angle_raw * 0.001f;
            }

            arm.active = zigbee_buf[13]; // 读取发送端 DEADMAN 标志
            arm.connected = 1;
            arm.recv_cnt++;
            arm.last_rx_tick = HAL_GetTick();
        }
        zigbee_idx = 0;
        return;
    }
    // 兼容 14 字节协议
    else if(zigbee_idx == 14 && zigbee_buf[13] == 0x55)
    {
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            int16_t angle_raw = (int16_t)(zigbee_buf[1 + i*2] | (zigbee_buf[2 + i*2] << 8));
            arm.target_joint[i] = angle_raw * 0.001f;
        }
        arm.active = 1;
        arm.connected = 1;
        arm.recv_cnt++;
        arm.last_rx_tick = HAL_GetTick();
        zigbee_idx = 0;
        return;
    }

    if(zigbee_idx >= 16)
    {
        zigbee_idx = 0;
    }
}

// 串口中断回调
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART10)
    {
        Arm_Zigbee_ParseByte(zigbee_rx_byte);
        HAL_UART_Receive_IT(&huart10, &zigbee_rx_byte, 1);
    }
}

// ==================== 3. 初始化 ====================
void Arm_Init(void)
{
    DM_Motor_Init();

    // 开启 5V 收发器供电
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_15 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);

    memset(&arm, 0, sizeof(arm));
    arm.state = ARM_STATE_DISABLED; 
    arm.last_state = ARM_STATE_DISABLED;
    arm.q_offset_valid = 0;

    HAL_UART_Receive_IT(&huart10, &zigbee_rx_byte, 1);
}

// ==================== 4. 遥控器与主从随动状态机 ====================
static float arm_lock_pos[ARM_JOINT_NUM] = {0};

static void Arm_UpdateState(void)
{
    const DR16_Data_t *rc = DR16_Get_Data();

    // 1. 遥控器离线安全保护
    if(!DR16_IsOnline())
    {
        arm.state = ARM_STATE_DISABLED;
    }
    else
    {
        // 2. 遥控器右拨杆三档
        if(rc->rc.s[0] == DR16_SW_DOWN)      // 下档：彻底泄力
        {
            arm.state = ARM_STATE_DISABLED;
        }
        else if(rc->rc.s[0] == DR16_SW_MID) // 中档：原地焊死
        {
            arm.state = ARM_STATE_LOCKED;
        }
        else if(rc->rc.s[0] == DR16_SW_UP)  // 上档：无线主从跟随
        {
            if(arm.connected)
                arm.state = ARM_STATE_FOLLOWING;
            else
                arm.state = ARM_STATE_LOCKED;
        }
    }

    // 3. 通信超时检测
    if(arm.connected && (HAL_GetTick() - arm.last_rx_tick) > ZIGBEE_TIMEOUT_MS)
    {
        arm.connected = 0;
        arm.active = 0;
    }

    // 4. 状态跳变边沿处理
    const int motor_num = ARM_USE_GRIPPER ? DM_MOTOR_NUM : ARM_JOINT_NUM;

    if(arm.state != arm.last_state)
    {
        // 切入【使能】
        if(arm.last_state == ARM_STATE_DISABLED && arm.state != ARM_STATE_DISABLED)
        {
            for(int i = 0; i < motor_num; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) {
                    DM_Motor_ClearError(m->hfdcan, m->can_id);
                    osDelay(1);
                }
            }
            osDelay(2);

            for(int i = 0; i < motor_num; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) {
                    DM_Motor_Enable(m->hfdcan, m->can_id);
                    if(i < ARM_JOINT_NUM && m->rx_cnt > 0)
                    {
                        arm_lock_pos[i]   = m->position;
                        arm.cmd_smooth[i] = m->position;
                    }
                    osDelay(1);
                }
            }
        }
        // 切入【中档锁死】
        else if(arm.state == ARM_STATE_LOCKED)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m && m->rx_cnt > 0)
                {
                    arm_lock_pos[i]   = m->position;
                    arm.cmd_smooth[i] = m->position;
                }
            }
            arm.q_offset_valid = 0;
        }
        // 切入【失能】
        else if(arm.state == ARM_STATE_DISABLED)
        {
            for(int i = 0; i < motor_num; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) {
                    DM_Motor_Disable(m->hfdcan, m->can_id);
                    osDelay(1);
                }
            }
            arm.q_offset_valid = 0;
        }
        arm.last_state = arm.state;
    }

    // 5. 核心离合对齐：切入跟随瞬间捕获相对偏置
    if(arm.state == ARM_STATE_FOLLOWING)
    {
        if(!arm.q_offset_valid && arm.connected && arm.recv_cnt > 0)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m && m->rx_cnt > 0)
                {
                    arm.q_offset[i]   = ARM_DIR[i] * arm.target_joint[i] - m->position;
                    arm.cmd_smooth[i] = m->position;
                }
            }
            arm.q_offset_valid = 1;
        }
    }
    else
    {
        arm.q_offset_valid = 0;
    }
}

// 斜坡滤波（防甩臂）
static float Arm_RateLimit(float target, float current)
{
    float err = target - current;
    if(err > ARM_MAX_STEP) return current + ARM_MAX_STEP;
    if(err < -ARM_MAX_STEP) return current - ARM_MAX_STEP;
    return target;
}

// ==================== 5. 控制主循环 ====================
void Arm_ControlLoop(void)
{
    // 1. 工业级自动自愈巡检：后台静默守护全臂（凡是 error!=1 自动拉绿拉活）
    static uint32_t t_heal = 0;
    if(HAL_GetTick() - t_heal > 200)
    {
        t_heal = HAL_GetTick();
        if(arm.state == ARM_STATE_LOCKED || arm.state == ARM_STATE_FOLLOWING)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m && m->error != 1) 
                {
                    DM_Motor_ClearError(m->hfdcan, m->can_id);
                    osDelay(1);
                    DM_Motor_Enable(m->hfdcan, m->can_id);
                    osDelay(1);
                }
            }
        }
    }

    // 2. 刷新状态机
    Arm_UpdateState();

    // 3. 下档彻底失能退出
    if(arm.state == ARM_STATE_DISABLED) return;

    // 4. 全臂 6 轴位置控制闭环下发
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || m->hfdcan == NULL) continue;

        // 【上档：真正的主从实时跟随】
        if(arm.state == ARM_STATE_FOLLOWING && arm.q_offset_valid && m->rx_cnt > 0)
        {
            float raw_target = ARM_DIR[i] * arm.target_joint[i] - arm.q_offset[i];
            float clamped    = Arm_ClampJoint(i, raw_target); // 软件软限位温柔刹停，绝不撞墙
            arm.cmd_smooth[i] = Arm_RateLimit(clamped, arm.cmd_smooth[i]);
        }
        // 【中档：全臂原地焊死】
        else
        {
            arm.cmd_smooth[i] = arm_lock_pos[i];
        }

        // 下发 PVT 报文
        DM_Motor_PVT_Control(m->hfdcan, m->can_id,
                             arm.cmd_smooth[i], JOINT_MAX_VEL, ARM_CUR_LIMIT[i]);
        
        osDelay(1); // 1ms 间隔防总线塞爆
    }
}

Arm_t* Arm_Get(void)
{
    return &arm;
}
