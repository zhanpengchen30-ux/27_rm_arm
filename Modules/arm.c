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

#define ARM_LIMIT_DRAG      1
#define ARM_GAP_CATCHUP_MS  100u
#define ARM_GAP_RECLUTCH_MS 500u
#define ARM_CATCHUP_SPEED_RAD_S 1.5f

/* PVT velocity field (u16) is the speed the inner loop may use to reach the
   position target. Feeding 0 at standstill removes its ability to correct a
   gravity sag (J2 slowly drops), feeding a constant 4 makes it ring.
   Use the commanded speed with a floor that still allows holding. */
#define ARM_VEL_MAX_RAD_S   4.0f
#define ARM_VEL_MIN_RAD_S   1.5f
static float arm_prev_cmd[ARM_JOINT_NUM] = {0};
static volatile uint8_t arm_catchup_mask = 0;
static volatile uint8_t arm_reclutch_req = 0;
static uint8_t arm_q_offset_mask = 0;

volatile uint32_t zb_bad_tail   = 0;
volatile uint32_t zb_resync     = 0;
volatile uint32_t zb_restart    = 0;
volatile uint32_t zb_gap_max_ms = 0;
volatile uint32_t zb_reclutch_cnt = 0;

/* Jump filter: a single frame whose target moves more than ARM_JUMP_MAX_RAD is
   treated as a link glitch / master re-zero. It is only accepted once the next
   frame confirms it, so the slave never lunges at full speed after a dropout. */
#define ARM_JUMP_MAX_RAD   0.15f
static uint8_t  arm_jump_hold[ARM_JOINT_NUM] = {0};
static float    arm_jump_pend[ARM_JOINT_NUM] = {0};
volatile uint32_t zb_jump_reject = 0;

/* ---- master frame interpolation ------------------------------------------
   The master sends every 10ms while the loop runs every 2ms. Applying a whole
   frame's delta inside one 2ms tick gives a ~5rad/s burst followed by ~8ms of
   standstill - the arm buzzes at 100Hz. Spread each frame's delta over
   ARM_INTERP_TICKS ticks so the command becomes a continuous ramp. */
#define ARM_INTERP_TICKS 5u
static float    arm_interp_target[ARM_JOINT_NUM] = {0};
static float    arm_interp_step[ARM_JOINT_NUM]   = {0};
static uint32_t arm_interp_left = 0;
static uint32_t arm_last_frame_cnt = 0;

static void Arm_InterpResetJoint(int i)
{
    arm_interp_target[i] = arm.target_joint[i];
    arm_interp_step[i]   = 0.0f;
}

static void Arm_InterpResetAll(void)
{
    for(int i = 0; i < ARM_JOINT_NUM; i++) Arm_InterpResetJoint(i);
    arm_interp_left = 0;
}

static const float ARM_DIR[ARM_JOINT_NUM] = {
    1.0f, -1.0f, -1.0f, -1.0f, 1.0f, 1.0f
};

static const float ARM_Q_MIN[ARM_JOINT_NUM] = { -3.10f, -2.80f, -0.65f, -10.0f, -10.0f, -10.0f};
static const float ARM_Q_MAX[ARM_JOINT_NUM] = {  0.05f,  0.05f,  1.90f,  10.0f,  10.0f,  10.0f};

static const float ARM_CUR_LIMIT[ARM_JOINT_NUM] = {
    0.7f,  1.8f,  0.7f,  0.7f,  0.7f,  0.7f
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
    uint8_t long_gap = 0;
    uint8_t reacquire = (active && !arm.active);
    if(arm.recv_cnt > 0)
    {
        uint32_t gap = now - arm.last_rx_tick;
        if(gap > zb_gap_max_ms) zb_gap_max_ms = gap;
        if(gap > ARM_GAP_RECLUTCH_MS)
        {
            arm_reclutch_req = 1;
            long_gap = 1;
        }
        else if(gap > ARM_GAP_CATCHUP_MS)
        {
            /* Use gentle catch-up until each joint reaches its target. */
            arm_catchup_mask = (uint8_t)((1u << ARM_JOINT_NUM) - 1u);
        }
    }
    if(reacquire) arm_reclutch_req = 1;
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        int16_t angle_raw = (int16_t)(zigbee_buf[1 + i*2] | (zigbee_buf[2 + i*2] << 8));
        float nq = angle_raw * 0.001f;
        float d  = nq - arm.target_joint[i];

        if(long_gap || reacquire || fabsf(d) <= ARM_JUMP_MAX_RAD)
        {
            arm.target_joint[i] = nq;
            arm_jump_hold[i] = 0;
        }
        else if(arm_jump_hold[i] == 0)
        {
            arm_jump_pend[i] = nq;      /* suspicious: keep old target this frame */
            arm_jump_hold[i] = 1;
            zb_jump_reject++;
        }
        else if(fabsf(nq - arm_jump_pend[i]) <= ARM_JUMP_MAX_RAD)
        {
            arm.target_joint[i] = nq;   /* confirmed by the next frame: accept */
            arm_jump_hold[i] = 0;
        }
        else
        {
            arm_jump_pend[i] = nq;
            zb_jump_reject++;
        }
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
    arm_q_offset_mask = 0;
    arm_catchup_mask = 0;
    arm_reclutch_req = 0;

    HAL_UART_Receive_IT(&huart10, &zigbee_rx_byte, 1);
}

static float arm_lock_pos[ARM_JOINT_NUM] = {0};

static uint8_t Arm_CaptureFollowOffset(void)
{
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        uint8_t bit = (uint8_t)(1u << i);
        DM_Motor_t *m = DM_Motor_Get(i);
        if((arm_q_offset_mask & bit) || m == NULL || m->rx_cnt == 0) continue;
        arm.q_offset[i]   = ARM_DIR[i] * arm.target_joint[i] - m->position;
        arm.cmd_smooth[i] = m->position;
        arm_q_offset_mask |= bit;
    }
    arm.q_offset_valid = (arm_q_offset_mask == ((1u << ARM_JOINT_NUM) - 1u));
    return arm.q_offset_valid;
}

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
            if(arm.connected && arm.active)
                arm.state = ARM_STATE_FOLLOWING;
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
            arm_q_offset_mask = 0;
            arm_catchup_mask = 0;
        }
        else if(arm.state == ARM_STATE_DISABLED)
        {
            for(int i = 0; i < motor_num; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) { DM_Motor_Disable(m->hfdcan, m->can_id); osDelay(1); }
            }
            arm.q_offset_valid = 0;
            arm_q_offset_mask = 0;
            arm_catchup_mask = 0;
        }
        arm.last_state = arm.state;
    }

    if(arm.state == ARM_STATE_FOLLOWING)
    {
        if(!arm.q_offset_valid && arm.connected && arm.recv_cnt > 0)
        {
            (void)Arm_CaptureFollowOffset();
        }
    }
    else
    {
        arm.q_offset_valid = 0;
        arm_q_offset_mask = 0;
        arm_catchup_mask = 0;
    }
}

static float Arm_RateLimit(float target, float current, float max_step)
{
    float err = target - current;
    if(err > max_step) return current + max_step;
    if(err < -max_step) return current - max_step;
    return target;
}

void Arm_ControlLoop(void)
{
    static uint32_t t_heal = 0;
    static uint8_t heal_enable_mask = 0;
    static uint32_t heal_enable_due[ARM_JOINT_NUM] = {0};
    uint32_t now = HAL_GetTick();

    /* Keep the 2ms control task non-blocking. Queue enable 2ms after a clear
       command instead of sleeping inside the control loop. */
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        uint8_t bit = (uint8_t)(1u << i);
        if((heal_enable_mask & bit) && (int32_t)(now - heal_enable_due[i]) >= 0)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(m) DM_Motor_Enable(m->hfdcan, m->can_id);
            heal_enable_mask &= (uint8_t)~bit;
        }
    }

    if((uint32_t)(now - t_heal) >= 200u)
    {
        t_heal = now;
        if(arm.state == ARM_STATE_LOCKED || arm.state == ARM_STATE_FOLLOWING)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                uint8_t bit = (uint8_t)(1u << i);
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m && !(heal_enable_mask & bit) &&
                   (m->rx_cnt == 0 || m->error != 0))
                {
                    DM_Motor_ClearError(m->hfdcan, m->can_id);
                    heal_enable_due[i] = now + 2u;
                    heal_enable_mask |= bit;
                }
            }
        }
    }

    Arm_UpdateState();

    /* After a long outage, treat the newly received master pose as the new
       clutch point. Shorter gaps keep following, with reduced slew instead. */
    if(arm_reclutch_req)
    {
        arm_reclutch_req = 0;
        if(arm.state == ARM_STATE_FOLLOWING && arm.connected && arm.active)
        {
            arm.q_offset_valid = 0;
            arm_q_offset_mask = 0;
            arm_catchup_mask = 0;
            Arm_InterpResetAll();
            if(Arm_CaptureFollowOffset()) zb_reclutch_cnt++;
        }
    }

    if(arm.state == ARM_STATE_DISABLED) return;

    /* spread each new master frame over ARM_INTERP_TICKS control ticks */
    if(arm.recv_cnt != arm_last_frame_cnt)
    {
        arm_last_frame_cnt = arm.recv_cnt;
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            arm_interp_step[i] = (arm.target_joint[i] - arm_interp_target[i]) / (float)ARM_INTERP_TICKS;
        }
        arm_interp_left = ARM_INTERP_TICKS;
    }
    if(arm_interp_left > 0)
    {
        for(int i = 0; i < ARM_JOINT_NUM; i++) arm_interp_target[i] += arm_interp_step[i];
        arm_interp_left--;
    }

    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || m->hfdcan == NULL) continue;

        uint8_t joint_bit = (uint8_t)(1u << i);
        if(arm.state == ARM_STATE_FOLLOWING && arm.connected && arm.active && m->rx_cnt > 0 &&
           !(arm_q_offset_mask & joint_bit))
        {
            /* Capture each joint independently; one late motor must not leave
               the remaining joints running with an implicit zero offset. */
            arm.q_offset[i]   = ARM_DIR[i] * arm.target_joint[i] - m->position;
            arm.cmd_smooth[i] = m->position;
            arm_prev_cmd[i]   = m->position;   /* no velocity kick at clutch */
            Arm_InterpResetJoint(i);
            arm_q_offset_mask |= joint_bit;
            arm.q_offset_valid =
                (arm_q_offset_mask == ((1u << ARM_JOINT_NUM) - 1u));
        }

        if(arm.state == ARM_STATE_FOLLOWING && arm.connected && arm.active &&
           (arm_q_offset_mask & joint_bit) && m->rx_cnt > 0)
        {
            if(arm.connected)
            {
                float raw_target = ARM_DIR[i] * arm_interp_target[i] - arm.q_offset[i];
                float clamped    = Arm_ClampJoint(i, raw_target);
#if ARM_LIMIT_DRAG
                if(clamped != raw_target)
                {
                    arm.q_offset[i] += (raw_target - clamped);
                }
#endif
                float max_step = ARM_MAX_STEP;
                float target_error = clamped - arm.cmd_smooth[i];
                if((arm_catchup_mask & joint_bit) && fabsf(target_error) <= 0.05f)
                {
                    arm_catchup_mask &= (uint8_t)~joint_bit;
                }
                if(arm_catchup_mask & joint_bit)
                    max_step = ARM_CATCHUP_SPEED_RAD_S * ARM_CTRL_PERIOD_S;
                arm.cmd_smooth[i] = Arm_RateLimit(clamped, arm.cmd_smooth[i], max_step);
            }
        }
        else if(arm.state != ARM_STATE_FOLLOWING)
        {
            /* While following, a temporary link/enable loss must hold the
               last commanded pose; arm_lock_pos can be an older clutch pose. */
            arm.cmd_smooth[i] = arm_lock_pos[i];
        }

        /* commanded speed with a floor so the inner loop can still hold the pose */
        float vmag = fabsf(arm.cmd_smooth[i] - arm_prev_cmd[i]) / ARM_CTRL_PERIOD_S;
        arm_prev_cmd[i] = arm.cmd_smooth[i];
        if(vmag < ARM_VEL_MIN_RAD_S) vmag = ARM_VEL_MIN_RAD_S;
        if(vmag > ARM_VEL_MAX_RAD_S) vmag = ARM_VEL_MAX_RAD_S;

        DM_Motor_PVT_Control(m->hfdcan, m->can_id,
                             arm.cmd_smooth[i], vmag, ARM_CUR_LIMIT[i]);

    }
}

Arm_t* Arm_Get(void)
{
    return &arm;
}
