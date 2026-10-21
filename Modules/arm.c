#include "arm.h"
#include <string.h>
#include <math.h>
#include "bsp_dwt.h"
#include "usart.h"
#include "bsp_dr16.h"
#include "dm_motor.h"
#include "cmsis_os2.h"

static uint8_t zigbee_rx_byte;
Arm_t arm;
static uint8_t zigbee_buf[32];
static uint8_t zigbee_idx = 0;

#define JOINT_MAX_VEL       4.0f

#ifndef ARM_RX_TIMEOUT_MS
#define ARM_RX_TIMEOUT_MS   500
#endif
#define ZIGBEE_TIMEOUT_MS   ARM_RX_TIMEOUT_MS

#define ARM_LINK_HOLD_MS    1500
#define ARM_LIMIT_DRAG      1

volatile uint32_t zb_bad_tail   = 0;
volatile uint32_t zb_resync     = 0;
volatile uint32_t zb_restart    = 0;
volatile uint32_t zb_gap_max_ms = 0;

static const float ARM_DIR[ARM_JOINT_NUM] = {
    1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f
};

static const float ARM_Q_MIN[ARM_JOINT_NUM] = { -3.10f, -2.80f, -0.65f, -10.0f, -10.0f, -10.0f};
static const float ARM_Q_MAX[ARM_JOINT_NUM] = {  0.05f,  0.05f,  1.90f,  10.0f,  10.0f,  10.0f};

static const float ARM_CUR_LIMIT[ARM_JOINT_NUM] = {
    0.7f,  0.7f,  0.7f,  0.7f,  0.7f,  0.7f
};

static float Arm_ClampJoint(int i, float v)
{
    if(v < ARM_Q_MIN[i]) return ARM_Q_MIN[i];
    if(v > ARM_Q_MAX[i]) return ARM_Q_MAX[i];
    return v;
}

static void Arm_DecodeFrame(uint8_t active)
{
    uint32_t now = HAL_GetTick();
    if(arm.recv_cnt > 0)
    {
        uint32_t gap = now - arm.last_rx_tick;
        if(gap > zb_gap_max_ms) zb_gap_max_ms = gap;
    }
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        int16_t angle_raw = (int16_t)(zigbee_buf[1 + i*2] | (zigbee_buf[2 + i*2] << 8));
        arm.target_joint[i] = angle_raw * 0.001f;
    }
    arm.active = active;
    arm.connected = 1;
    arm.recv_cnt++;
    arm.last_rx_tick = now;
}

static void Arm_ZigbeeResync(void)
{
    uint8_t n = zigbee_idx;
    uint8_t i;
    for(i = 1; i < n; i++)
    {
        if(zigbee_buf[i] == 0xAA) break;
    }
    zb_resync++;
    if(i >= n) { zigbee_idx = 0; return; }
    memmove(zigbee_buf, &zigbee_buf[i], n - i);
    zigbee_idx = n - i;
}

void Arm_Zigbee_ParseByte(uint8_t byte)
{
    if(zigbee_idx == 0)
    {
        if(byte == 0xAA) zigbee_buf[zigbee_idx++] = byte;
        return;
    }
    zigbee_buf[zigbee_idx++] = byte;

    if(zigbee_idx == 15)
    {
        if(zigbee_buf[14] == 0x55)
        {
            Arm_DecodeFrame(zigbee_buf[13]);
            zigbee_idx = 0;
        }
        else
        {
            zb_bad_tail++;
            Arm_ZigbeeResync();
        }
        return;
    }
    if(zigbee_idx >= 16) zigbee_idx = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART10)
    {
        Arm_Zigbee_ParseByte(zigbee_rx_byte);
        HAL_UART_Receive_IT(&huart10, &zigbee_rx_byte, 1);
    }
}

void Arm_Zigbee_RxRestart(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart10);
    __HAL_UART_CLEAR_NEFLAG(&huart10);
    __HAL_UART_CLEAR_FEFLAG(&huart10);
    HAL_UART_AbortReceive(&huart10);
    zigbee_idx = 0;
    zb_restart++;
    (void)HAL_UART_Receive_IT(&huart10, &zigbee_rx_byte, 1);
}

void Arm_Init(void)
{
    DM_Motor_Init();

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

static float arm_lock_pos[ARM_JOINT_NUM] = {0};

static void Arm_UpdateState(void)
{
    const DR16_Data_t *rc = DR16_Get_Data();

    if(!DR16_IsOnline())
    {
        arm.state = ARM_STATE_DISABLED;
    }
    else
    {
        if(rc->rc.s[0] == DR16_SW_DOWN)
        {
            arm.state = ARM_STATE_DISABLED;
        }
        else if(rc->rc.s[0] == DR16_SW_MID)
        {
            arm.state = ARM_STATE_LOCKED;
        }
        else if(rc->rc.s[0] == DR16_SW_UP)
        {
            if(arm.connected)
                arm.state = ARM_STATE_FOLLOWING;
            else if(arm.state == ARM_STATE_FOLLOWING && arm.q_offset_valid &&
                    (HAL_GetTick() - arm.last_rx_tick) < ARM_LINK_HOLD_MS)
            {
                arm.state = ARM_STATE_FOLLOWING;
            }
            else
                arm.state = ARM_STATE_LOCKED;
        }
    }

    if(arm.connected && (HAL_GetTick() - arm.last_rx_tick) > ZIGBEE_TIMEOUT_MS)
    {
        arm.connected = 0;
        arm.active = 0;
    }

    const int motor_num = ARM_USE_GRIPPER ? DM_MOTOR_NUM : ARM_JOINT_NUM;

    if(arm.state != arm.last_state)
    {
        if(arm.last_state == ARM_STATE_DISABLED && arm.state != ARM_STATE_DISABLED)
        {
            for(int i = 0; i < motor_num; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) { DM_Motor_ClearError(m->hfdcan, m->can_id); osDelay(1); }
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
        else if(arm.state == ARM_STATE_DISABLED)
        {
            for(int i = 0; i < motor_num; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) { DM_Motor_Disable(m->hfdcan, m->can_id); osDelay(1); }
            }
            arm.q_offset_valid = 0;
        }
        arm.last_state = arm.state;
    }

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

static float Arm_RateLimit(float target, float current)
{
    float err = target - current;
    if(err > ARM_MAX_STEP) return current + ARM_MAX_STEP;
    if(err < -ARM_MAX_STEP) return current - ARM_MAX_STEP;
    return target;
}

void Arm_ControlLoop(void)
{
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

    Arm_UpdateState();

    if(arm.state == ARM_STATE_DISABLED) return;

    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || m->hfdcan == NULL) continue;

        if(arm.state == ARM_STATE_FOLLOWING && arm.q_offset_valid && m->rx_cnt > 0)
        {
            if(arm.connected)
            {
                float raw_target = ARM_DIR[i] * arm.target_joint[i] - arm.q_offset[i];
                float clamped    = Arm_ClampJoint(i, raw_target);
#if ARM_LIMIT_DRAG
                if(clamped != raw_target)
                {
                    arm.q_offset[i] += (raw_target - clamped);
                }
#endif
                arm.cmd_smooth[i] = Arm_RateLimit(clamped, arm.cmd_smooth[i]);
            }
        }
        else
        {
            arm.cmd_smooth[i] = arm_lock_pos[i];
        }

        DM_Motor_PVT_Control(m->hfdcan, m->can_id,
                             arm.cmd_smooth[i], JOINT_MAX_VEL, ARM_CUR_LIMIT[i]);

        osDelay(1);
    }
}

Arm_t* Arm_Get(void)
{
    return &arm;
}