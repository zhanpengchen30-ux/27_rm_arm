#include "arm.h"
#include <string.h>
#include "bsp_dwt.h"
#include "usart.h"
#include "bsp_dr16.h"
#include "dm_motor.h"
#include "cmsis_os2.h"

static uint8_t zigbee_rx_byte;
static Arm_t arm;
static uint8_t zigbee_buf[ZIGBEE_FRAME_LEN];
static uint8_t zigbee_idx = 0;

#define GRIPPER_OPEN_POS    0.0f    // rad
#define GRIPPER_CLOSE_POS   2.0f    // rad

#define JOINT_MAX_VEL       2.0f    // 关节最大运行角速度 rad/s
#define GRIPPER_CUR         1.0f    // 夹爪闭环电流

// ==================== 1. 各关节标定表、限位与电流限幅 ====================

// 关节主从方向映射：J2 经过实测必须为 -1.0f（向上抬），其他关节若反向直接改负号
static const float ARM_DIR[ARM_JOINT_NUM] = {
    1.0f, -1.0f, 1.0f, 1.0f, 1.0f, 1.0f
};

// 关节机械软限位（单位 rad，保护机构防止撞车身/打限位/绞线）
static const float ARM_Q_MIN[ARM_JOINT_NUM] = {-2.80f, -1.20f, -2.80f, -2.60f, -2.80f, -2.80f};
static const float ARM_Q_MAX[ARM_JOINT_NUM] = { 2.80f,  1.20f,  2.80f,  2.60f,  2.80f,  2.80f};

// 各关节闭环出力电流上限（绝不能为 0！J2 大臂需要抗重力给 1.8A，其他给 1.0~1.2A）
static const float ARM_CUR_LIMIT[ARM_JOINT_NUM] = {
    1.2f,  // J1: 4340 底座偏航
    1.8f,  // J2: J8009 大臂俯仰（托举整臂重力）
    1.5f,  // J3: 4340 小臂俯仰
    1.0f,  // J4: 4310 腕部翻滚
    1.0f,  // J5: 4310 腕部俯仰
    1.0f   // J6: 6220 腕部偏航
};

static float Arm_ClampJoint(int i, float v)
{
    if(v < ARM_Q_MIN[i]) return ARM_Q_MIN[i];
    if(v > ARM_Q_MAX[i]) return ARM_Q_MAX[i];
    return v;
}

// ==================== 2. 初始化 ====================
void Arm_Init(void)
{
    // 1. 初始化电机驱动底层结构体
    DM_Motor_Init();

    // 2. 开启板载 5V 电源（给 CAN 收发器供电）
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
    arm.gripper_state = GRIPPER_OPEN;
    arm.target_gripper = GRIPPER_OPEN_POS;
    arm.state = ARM_STATE_DISABLED; 
    arm.last_state = ARM_STATE_DISABLED;
    arm.q_offset_valid = 0;

    HAL_UART_Receive_IT(&huart10, &zigbee_rx_byte, 1);
}

// ==================== 3. 串口解析（主手数据） ====================
void Arm_Zigbee_ParseByte(uint8_t byte)
{
    zigbee_buf[zigbee_idx++] = byte;

    if(zigbee_idx == 1 && byte != ZIGBEE_HEAD)
    {
        zigbee_idx = 0;
        return;
    }

    if(zigbee_idx >= ZIGBEE_FRAME_LEN)
    {
        if(zigbee_buf[ZIGBEE_FRAME_LEN - 1] == ZIGBEE_TAIL)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                int16_t angle_raw = (int16_t)(zigbee_buf[1 + i*2] | (zigbee_buf[2 + i*2] << 8));
                arm.target_joint[i] = angle_raw * 0.001f;
            }
            arm.active = zigbee_buf[ZIGBEE_ACTIVE_OFFSET];
            arm.recv_cnt++;
            arm.connected = 1;
            arm.last_rx_tick = HAL_GetTick();
        }
        zigbee_idx = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART10)
    {
        Arm_Zigbee_ParseByte(zigbee_rx_byte);
        HAL_UART_Receive_IT(&huart10, &zigbee_rx_byte, 1);
    }
}

static void Arm_ReadSwitches(void)
{
    // 自定义按键读取预留
}

// ==================== 4. 夹爪控制 ====================
#if ARM_USE_GRIPPER
static void Arm_ControlGripper(void)
{
    if(arm.state == ARM_STATE_DISABLED) return;

    if(arm.sw_open == GPIO_PIN_RESET && arm.gripper_state != GRIPPER_OPEN)
    {
        arm.gripper_state = GRIPPER_OPEN;
        arm.target_gripper = GRIPPER_OPEN_POS;
        arm.gripper_cmd_recv = 1;
    }
    if(arm.sw_close == GPIO_PIN_RESET && arm.gripper_state != GRIPPER_CLOSE)
    {
        arm.gripper_state = GRIPPER_CLOSE;
        arm.target_gripper = GRIPPER_CLOSE_POS;
        arm.gripper_cmd_recv = 1;
    }

    float pos = arm.target_gripper;
    if(!arm.gripper_cmd_recv)
    {
        DM_Motor_t *g = DM_Motor_Get(DM_MOTOR_NUM - 1);
        if(g && g->rx_cnt > 0)
        {
            pos = g->position;
        }
    }

    DM_Motor_PVT_Control(&hfdcan1, DM_GRIPPER_ID, pos, 1.0f, GRIPPER_CUR);
}
#else
static void Arm_ControlGripper(void)
{
    // 夹爪未启用，空实现
}
#endif

// ==================== 5. 核心状态机与安全离合 ====================
static void Arm_UpdateState(void)
{
    const DR16_Data_t *rc = DR16_Get_Data();

    // 1. 遥控器安全断连检测
    if(!DR16_IsOnline())
    {
        arm.state = ARM_STATE_DISABLED;
    }
    else
    {
        // 2. 遥控器右侧三档拨杆 s[0]
        if(rc->rc.s[0] == DR16_SW_DOWN)      // 下档：强制全臂失能泄力
        {
            arm.state = ARM_STATE_DISABLED;
        }
        else if(rc->rc.s[0] == DR16_SW_MID) // 中档：全臂原地焊死（保命安全档）
        {
            arm.state = ARM_STATE_LOCKED;
        }
        else if(rc->rc.s[0] == DR16_SW_UP)  // 上档：进入主从跟随
        {
            if(arm.connected && arm.active)
                arm.state = ARM_STATE_FOLLOWING;
            else
                arm.state = ARM_STATE_LOCKED; // 主手掉线自动退回锁定档
        }
    }

    // 3. Zigbee 通信超时检测
    if(arm.connected && (HAL_GetTick() - arm.last_rx_tick) > ARM_RX_TIMEOUT_MS)
    {
        arm.connected = 0;
        arm.active = 0;
    }

    // 4. 状态跳变动作（边沿触发）
    const int motor_num = ARM_USE_GRIPPER ? DM_MOTOR_NUM : ARM_JOINT_NUM;

    if(arm.state != arm.last_state)
    {
        // 【进入使能】：从失能切入锁死/跟随
        if(arm.last_state == ARM_STATE_DISABLED && arm.state != ARM_STATE_DISABLED)
        {
            // 第一遍：挨个清错
            for(int i = 0; i < motor_num; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) {
                    DM_Motor_ClearError(m->hfdcan, m->can_id);
                    osDelay(1);
                }
            }
            osDelay(2);

            // 第二遍：挨个使能并同步当前实际位置为初始目标（核心防暴冲）
            for(int i = 0; i < motor_num; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) {
                    DM_Motor_Enable(m->hfdcan, m->can_id);
                    if(i < ARM_JOINT_NUM && m->rx_cnt > 0)
                    {
                        arm.cmd_smooth[i] = m->position;
                    }
                    osDelay(1);
                }
            }
        }
        // 【进入失能】：彻底泄力
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
        }
        arm.last_state = arm.state;
    }

    // 5. 离合对齐逻辑（进入跟随档时的平滑捕捉）
    if(arm.state != ARM_STATE_FOLLOWING)
    {
        arm.q_offset_valid = 0; // 退出跟随，偏置作废
    }
    else if(!arm.q_offset_valid)
    {
        // 确保所有参与跟随的关节均已建立双向反馈
        int all_motors_online = (arm.recv_cnt > 0);
        for(int i = 0; i < ARM_JOINT_NUM && all_motors_online; i++)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(!m || m->rx_cnt == 0) all_motors_online = 0;
        }

        if(all_motors_online)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                // 捕获主从绝对差值：从手期望 = 主手目标 - 相对偏置
                arm.q_offset[i]   = ARM_DIR[i] * arm.target_joint[i] - m->position;
                arm.cmd_smooth[i] = m->position;
            }
            arm.q_offset_valid = 1; // 偏置锁定，开始平滑跟随
        }
    }
}

// 斜坡限速（防关节剧烈突变甩臂）
static float Arm_RateLimit(float target, float current)
{
    float err = target - current;
    if(err > ARM_MAX_STEP) return current + ARM_MAX_STEP;
    if(err < -ARM_MAX_STEP) return current - ARM_MAX_STEP;
    return target;
}

// ==================== 6. 机械臂控制主循环 ====================
void Arm_ControlLoop(void)
{
    // 1. 刷新遥控器与整臂状态机
    Arm_UpdateState();

    // 2. 失能状态下：绝对不发任何控制帧
    if(arm.state == ARM_STATE_DISABLED) return;

    // 3. 运行中个别电机掉线/红灯的自动保活机制（只针对失能态 error==0 的电机发）
    static uint32_t keepalive_tick = 0;
    if(HAL_GetTick() - keepalive_tick > 200)
    {
        keepalive_tick = HAL_GetTick();
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(m && (m->rx_cnt == 0 || m->error == 0)) 
            {
                DM_Motor_Enable(m->hfdcan, m->can_id);
            }
        }
    }

    Arm_ReadSwitches();
    Arm_ControlGripper();

    // 4. 全臂 6 轴位置控制下发
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || m->hfdcan == NULL) continue;

        // 仅在跟随状态、偏置已锁定且电机有反馈时计算主手跟随
        if(arm.state == ARM_STATE_FOLLOWING && arm.q_offset_valid && m->rx_cnt > 0)
        {
            float raw_target = ARM_DIR[i] * arm.target_joint[i] - arm.q_offset[i];
            float clamped_target = Arm_ClampJoint(i, raw_target);
            arm.cmd_smooth[i] = Arm_RateLimit(clamped_target, arm.cmd_smooth[i]);
        }
        else
        {
            // 锁死档（中档）或等待对齐期间：目标恒等于当前物理位置，全臂原地焊死！
            if(m->rx_cnt > 0)
            {
                arm.cmd_smooth[i] = m->position;
            }
        }

        // 下发 PVT 帧（电流限幅严格按照 ARM_CUR_LIMIT 表给足，确保有力矩！）
        DM_Motor_PVT_Control(m->hfdcan, m->can_id,
                             arm.cmd_smooth[i], JOINT_MAX_VEL, ARM_CUR_LIMIT[i]);
        
        // 关键防爆时序：每个关节下发之间间隔 1ms，防止 H7 FDCAN TX FIFO 塞爆
        osDelay(1); 
    }
}

Arm_t* Arm_Get(void)
{
    return &arm;
}
