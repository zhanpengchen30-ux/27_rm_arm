#include "arm.h"
#include <string.h>
#include "bsp_dwt.h"
#include "usart.h"
#include "bsp_dr16.h" // 接入大疆遥控器

static uint8_t zigbee_rx_byte;
static Arm_t arm;
static uint8_t zigbee_buf[ZIGBEE_FRAME_LEN];
static uint8_t zigbee_idx = 0;

#define GRIPPER_OPEN_POS    0.0f    // rad
#define GRIPPER_CLOSE_POS   2.0f    // rad

#define JOINT_MAX_VEL       1.5f    
#define JOINT_CUR           0.0f    
#define GRIPPER_CUR         0.0f    

void Arm_Init(void)
{
    // 1. 彻底补全电机的 q_max、dq_max 和 can_id 参数
    DM_Motor_Init(); 

    // 2. 保持原有的收发器供电使能 ...
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

// 修复：修正字节解析偏移
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
            // 修复：数据段从索引 1 开始，每次 +2
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

// 夹爪控制
// ARM_USE_GRIPPER = 0 时整段不参与编译：既不读 GPIO，也不给夹爪电机下发任何帧
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
            pos = g->position; // 开机未操作时保持当前
        }
    }

    DM_Motor_PVT_Control(&hfdcan1, DM_GRIPPER_ID, pos, 1.0f, GRIPPER_CUR);
}
#else
static void Arm_ControlGripper(void)
{
    // 夹爪未上机，空实现
}
#endif

// ---------------- 主从跟随用的标定表 ----------------

// 关节机械行程（单位 rad）——必须按真机实测替换！下面是占位值，直接上电会顶到硬限位
static const float ARM_Q_MIN[ARM_JOINT_NUM] = {-2.80f, -1.20f, -2.80f, -2.60f, -2.80f, -2.80f};
static const float ARM_Q_MAX[ARM_JOINT_NUM] = { 2.80f,  1.20f,  2.80f,  2.60f,  2.80f,  2.80f};

// 主从方向映射：+1 同向，-1 反向
// 第一次联调时逐关节试，哪个反着走就把对应位改成 -1，不用动主手代码
static const float ARM_DIR[ARM_JOINT_NUM] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

static float Arm_ClampJoint(int i, float v)
{
    if(v < ARM_Q_MIN[i]) return ARM_Q_MIN[i];
    if(v > ARM_Q_MAX[i]) return ARM_Q_MAX[i];
    return v;
}

// 修复：全面打通大疆遥控器状态机
static void Arm_UpdateState(void)
{
    const DR16_Data_t *rc = DR16_Get_Data();

    // 1. 遥控器离线检测（遥控器一关，强制彻底失能）
    if(!DR16_IsOnline())
    {
        arm.state = ARM_STATE_DISABLED;
    }
    else
    {
        // 2. 检查遥控器右拨杆 s[0]
        if(rc->rc.s[0] == DR16_SW_DOWN) // 下档：强制失能泄力
        {
            arm.state = ARM_STATE_DISABLED;
        }
        else if(rc->rc.s[0] == DR16_SW_MID) // 中档：原地锁死
        {
            arm.state = ARM_STATE_LOCKED;
        }
        else if(rc->rc.s[0] == DR16_SW_UP) // 上档：允许跟随
        {
            // 还需要 Zigbee 在线且主手 deadman 激活
            if(arm.connected && arm.active)
                arm.state = ARM_STATE_FOLLOWING;
            else
                arm.state = ARM_STATE_LOCKED;
        }
    }

    // 3. Zigbee 超时检测
    if(arm.connected && (HAL_GetTick() - arm.last_rx_tick) > ARM_RX_TIMEOUT_MS)
    {
        arm.connected = 0;
        arm.active = 0;
    }

    // 4. 硬件使能 / 失能边沿动作
    {
        const int motor_num = ARM_USE_GRIPPER ? DM_MOTOR_NUM : ARM_JOINT_NUM;

        if(arm.state != arm.last_state)
        {
            if(arm.last_state == ARM_STATE_DISABLED && arm.state != ARM_STATE_DISABLED)
            {
                // 第一遍：清锁存错误（0xFB）
                // 达妙电机有错误锁存，不清错的话发再多使能帧都不会变绿
                for(int i = 0; i < motor_num; i++)
                {
                    DM_Motor_t *m = DM_Motor_Get(i);
                    DM_Motor_ClearError(m->hfdcan, m->can_id);
                }

                // 第二遍：使能（0xFC）并同步初值
                for(int i = 0; i < motor_num; i++)
                {
                    DM_Motor_t *m = DM_Motor_Get(i);
                    DM_Motor_Enable(m->hfdcan, m->can_id);
                    if(i < ARM_JOINT_NUM && m->rx_cnt > 0)
                    {
                        arm.cmd_smooth[i] = m->position; // 核心防暴冲同步
                    }
                }
            }
            else if(arm.state == ARM_STATE_DISABLED)
            {
                // 从运行切入失能：彻底泄力
                for(int i = 0; i < motor_num; i++)
                {
                    DM_Motor_t *m = DM_Motor_Get(i);
                    DM_Motor_Disable(m->hfdcan, m->can_id);
                }
            }

            // 5. 离合偏移不在这里捕获，见函数末尾的延迟捕获
            arm.last_state = arm.state;
        }
    }

    // 6. 使能兜底重试
    //    使能帧只在状态边沿发一次，万一那一帧丢了/电机当时还没上电，就永远不会亮。
    //    这里每 200ms 检查一次：有错误位、或者根本没回包的电机，补发"清错+使能"。
    if(arm.state == ARM_STATE_LOCKED || arm.state == ARM_STATE_FOLLOWING)
    {
        static uint32_t retry_tick = 0;
        if(HAL_GetTick() - retry_tick > 200)
        {
            retry_tick = HAL_GetTick();
            const int n = ARM_USE_GRIPPER ? DM_MOTOR_NUM : ARM_JOINT_NUM;
            for(int i = 0; i < n; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m->error != 0 || m->rx_cnt == 0)
                {
                    DM_Motor_ClearError(m->hfdcan, m->can_id);
                    DM_Motor_Enable(m->hfdcan, m->can_id);
                }
            }
        }
    }

    // 5. 离合捕获（延迟版，覆盖“使能后电机还没回包”的情况）
    //    只有 6 个电机全部有反馈、且至少收到过一帧主手数据，才认为偏移有效。
    //    在此之前从手保持不动，避免用假数据算出错误偏移后甩臂。
    if(arm.state != ARM_STATE_FOLLOWING)
    {
        arm.q_offset_valid = 0;
    }
    else if(!arm.q_offset_valid)
    {
        int ready = (arm.recv_cnt > 0);
        for(int i = 0; i < ARM_JOINT_NUM && ready; i++)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(!m || m->rx_cnt == 0) ready = 0;
        }

        if(ready)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                arm.q_offset[i]   = ARM_DIR[i] * arm.target_joint[i] - m->position;
                arm.cmd_smooth[i] = m->position;
            }
            arm.q_offset_valid = 1;
        }
    }
}

static float Arm_RateLimit(float target, float current)
{
    float err = target - current;
    if(err > ARM_MAX_STEP) return current + ARM_MAX_STEP;
    if(err < -ARM_MAX_STEP) return current - ARM_MAX_STEP;
    return target;
}

void Arm_ControlLoop(void)
{
    // 1. 自动使能：红灯时补发使能拉绿
    static uint32_t t_en = 0;
    if (HAL_GetTick() - t_en > 100) 
    {
        t_en = HAL_GetTick();
        for (int i = 0; i < ARM_JOINT_NUM; i++) 
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if (m && m->error == 0) 
            {
                DM_Motor_Enable(m->hfdcan, m->can_id);
            }
        }
    }

    // 2. 双管齐下控制 2 号电机 J8009（大幅度摆动测试）
    DM_Motor_t *m2 = DM_Motor_Get(1);
    if (m2 != NULL && m2->hfdcan != NULL && m2->error == 1) 
    {
        // 目标位置：在 ±0.8 弧度之间平滑转动
        float target_pos = 0.8f * sinf(HAL_GetTick() * 0.001f); 

        // 方案 A：PVT 控制帧（给 0x302，前馈电流给 1.5A，绝不能给 0！）
        DM_Motor_PVT_Control(m2->hfdcan, m2->can_id, target_pos, 2.0f, 1.5f);
        osDelay(1);

        // 方案 B：MIT 控制帧（直发 0x02，kp=40, kd=1.5，如果电机在 MIT 模式它会立刻响应这个！）
        DM_Motor_MIT_Control(m2->hfdcan, m2->can_id, target_pos, 0.0f, 40.0f, 1.5f, 0.0f);
    }

    osDelay(2); // 保持控制周期
}

Arm_t* Arm_Get(void)
{
    return &arm;
}
