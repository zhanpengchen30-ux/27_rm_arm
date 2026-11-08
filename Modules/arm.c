#include "arm.h"
#include <string.h>
#include <math.h>
#include "bsp_dwt.h"
#include "usart.h"
#include "bsp_dr16.h"
#include "dm_motor.h"
#include "bsp_can.h"
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

/* 使能/清错/自愈巡检覆盖的电机数。夹爪打开时含第 7 个（索引 ARM_JOINT_NUM）。
   注意区分：ARM_JOINT_NUM 只用于"有主从映射关系"的 6 个关节（限位、离合
   偏移、插值），夹爪不参与那套逻辑，只共用使能与自愈。 */
#if ARM_USE_GRIPPER
#define ARM_MOTOR_NUM   DM_MOTOR_NUM
#else
#define ARM_MOTOR_NUM   ARM_JOINT_NUM
#endif

/* 越限策略：已改为"纯限位"（不再吃偏移），保留定义仅作历史说明 */
/* #define ARM_LIMIT_DRAG   1 */
#define ARM_GAP_CATCHUP_MS  100u
#define ARM_GAP_RECLUTCH_MS 500u
#define ARM_CATCHUP_SPEED_RAD_S 1.5f

/* PVT velocity field (u16) is the speed the inner loop may use to reach the
   position target. Feeding 0 at standstill removes its ability to correct a
   gravity sag (J2 slowly drops), feeding a constant 4 makes it ring.
   Use the commanded speed with a floor that still allows holding. */
#define ARM_VEL_MAX_RAD_S   4.0f
#define ARM_VEL_MIN_RAD_S   0.4f
static float arm_prev_cmd[ARM_JOINT_NUM] = {0};

/* Jitter diagnostics: per-joint peak-to-peak over a 1s window, 500Hz sampled.
   dbg_tgt_pp  > 0  => the command itself is moving (master noise / link)
   dbg_pos_pp  > 0 while tgt_pp ~ 0 => the motor's own loop is oscillating */
static float dbg_tgt_min[ARM_JOINT_NUM] = {1e9f,1e9f,1e9f,1e9f,1e9f,1e9f};
static float dbg_tgt_max[ARM_JOINT_NUM] = {-1e9f,-1e9f,-1e9f,-1e9f,-1e9f,-1e9f};
static float dbg_pos_min[ARM_JOINT_NUM] = {1e9f,1e9f,1e9f,1e9f,1e9f,1e9f};
static float dbg_pos_max[ARM_JOINT_NUM] = {-1e9f,-1e9f,-1e9f,-1e9f,-1e9f,-1e9f};
volatile float dbg_tgt_pp[ARM_JOINT_NUM] = {0};
volatile float dbg_pos_pp[ARM_JOINT_NUM] = {0};
static volatile uint8_t arm_catchup_mask = 0;
static volatile uint8_t arm_reclutch_req = 0;
static uint8_t arm_q_offset_mask = 0;

static uint8_t Arm_FbFresh(const DM_Motor_t *m);   /* 定义在后面：反馈 50ms 内有新帧 */
static void Slot_FlashLoad(void);                  /* 矿位点位表，上电从 Flash 扇区6 恢复 */

/* ===== 抽臂 / 取矿（走到预设工作位）========================================
 * 主手按键把 active 的 bit1~bit2 当命令码发过来：
 *   0 = 交还跟随   1 = 只走到工作位(抽臂)   2 = 一键取矿：工作位→取矿位，到位才合爪
 * 本机把 6 个关节走到下面记录的电机绝对位置 —— 不看主手角度，也不走主从偏移。
 * 两组位置来自 2026-11-05 实测：摆好姿态后抄 dm_motors[0..5].position。
 * 命令码回 0 时请求重新吸合(reclutch)：以当前位置为基准继续跟主手增量，不会跳。
 * 只在遥控器上档(FOLLOWING)且主手在线时生效；中/下档命令码无效。
 * 老主手只发 0/1，bit1~bit2 恒为 0 ⇒ 行为与改动前完全一致。
 * ======================================================================== */

volatile float g_deploy_pos[ARM_JOINT_NUM] = {   /* 工作位（抽臂） */
    -0.0165939f, -0.8249407f, -0.9119177f,
     0.2397575f, -0.4194326f, -2.7750435f
};
volatile float g_mine_pos[ARM_JOINT_NUM] = {     /* 取矿位 */
    -0.5640116f, -1.2048903f, -0.6399250f,
     0.4510946f, -0.9763870f, -2.6010914f
};
volatile float   g_deploy_speed = 0.5f;   /* rad/s：走 60° 约 2s */
volatile uint8_t g_deploy_state = 0;      /* 0=跟随 1/3=行走 2=工作位保持 4=取矿位保持(已合爪) */
volatile uint8_t arm_deploy_cmd = 0;      /* 收到的命令码，Watch 核对用 */
static   uint8_t arm_deploy_hold = 0;     /* 各关节到位标志 */

/* 命令码 3 = 主手按键发来的 500ms 脉冲：一键循环 工作位→轨迹首点→回放。
   每段到位按电机实际位置进容差并连续保持判定，比按指令位置判更准。 */
volatile uint8_t g_cycle_state = 0;       /* 0=空闲 1=去工作位 2=去轨迹首点 3=回放中 4=回放完保持 */
volatile uint8_t g_cycle_err = 0;         /* 1=没录轨迹，按键不生效 */
static   uint8_t cycle_hold = 0;
static   float   cycle_goal[ARM_JOINT_NUM];

#define ARM_CYCLE_TOL        0.05f        /* 循环每段单轴实际到位容差(rad) */
#define ARM_CYCLE_HOLD_TICKS 50u          /* 到位连续保持 100ms 才切下一段 */

#define ARM_TEACH_SAMPLE_TICKS   5u       /* 2ms控制环，每5拍记录一次 = 100Hz */
#define ARM_TEACH_MAX_SAMPLES    3000u    /* 最长30s */
#define ARM_TEACH_START_TOL      0.12f    /* 回放前各轴与起点的允许偏差(rad) */
#define ARM_TEACH_PATH_TOL       0.20f    /* 倒放前各轴与最近轨迹点的允许偏差(rad) */
#define ARM_TEACH_END_TOL        0.08f    /* 合爪前各轴实际到位误差(rad) */
#define ARM_TEACH_END_HOLD_TICKS 50u      /* 到位连续保持100ms */

#define ARM_TEACH_EMPTY     0u
#define ARM_TEACH_RECORDING 1u
#define ARM_TEACH_READY     2u
#define ARM_TEACH_REPLAY    3u
#define ARM_TEACH_DONE      4u
#define ARM_TEACH_RETURN    5u

#define ARM_TEACH_ERR_MODE     1u
#define ARM_TEACH_ERR_DEPLOY   2u
#define ARM_TEACH_ERR_START    3u
#define ARM_TEACH_ERR_FULL     4u
#define ARM_TEACH_ERR_BUSY     5u
#define ARM_TEACH_ERR_SHORT    6u
#define ARM_TEACH_ERR_FEEDBACK 7u
#define ARM_TEACH_ERR_PATH     8u

volatile uint8_t  g_teach_req = 0;
volatile uint8_t  g_teach_state = ARM_TEACH_EMPTY;
volatile uint8_t  g_teach_error = 0;
volatile uint16_t g_teach_count = 0;
volatile uint16_t g_teach_index = 0;
volatile float    g_teach_speed = 1.0f;

volatile uint32_t g_arm_loop_tick = 0;    /* 控制环最近一次运行的时间戳，监控任务判挂死用 */
volatile uint32_t g_arm_busy_until = 0;   /* Flash 操作窗口(全片取指停顿)，窗口内监控不判挂死 */
static float teach_path[ARM_TEACH_MAX_SAMPLES][ARM_JOINT_NUM];
static uint8_t teach_grip_path[ARM_TEACH_MAX_SAMPLES];
static float teach_replay_goal[ARM_JOINT_NUM];
static float teach_replay_cursor = 0.0f;
static uint8_t teach_replay_grip = 0;
static uint8_t teach_sample_div = 0;
static uint8_t teach_end_hold = 0;
static uint8_t teach_auto_replay = 0;

/* 越限状态（只读观察用）：
   mask 某位为 1 = 该关节主手已越过从手限位、从手正停在限位上；
   excursion = 主手超出限位的角度（rad）。回到 0 即恢复严格同步。 */
volatile uint8_t arm_limit_mask = 0;
volatile float   arm_limit_excursion[ARM_JOINT_NUM] = {0};

volatile uint32_t zb_bad_tail   = 0;
volatile uint32_t zb_resync     = 0;
volatile uint32_t zb_restart    = 0;
volatile uint32_t zb_gap_max_ms = 0;
volatile uint32_t zb_reclutch_cnt = 0;

/* ---- 帧格式（与主手 Apps/zigbee.c 严格一致）----------------------------
   18 字节：AA | q0..q5 (12B) | active | grip | seq | crc8 | 55
   active: bit0 = deadman，bit1~bit2 = 命令码：0跟随/1工作位/2取矿/3一键循环(按键500ms脉冲)
           bit3~bit4 保留未用
   crc8 = CRC-8/ATM(poly 0x07)，覆盖 buf[0..15]
   seq 每帧 +1，用于统计无线丢帧（zb_seq_lost）
   长度与各字段下标只在 arm.h 里定义一份，别在这里再写一遍。        */
volatile uint32_t zb_crc_err  = 0;   /* CRC 校验失败次数 */
volatile uint32_t zb_seq_lost = 0;   /* 按序号推算出的丢帧数 */
static uint8_t zigbee_last_seq   = 0;
static uint8_t zigbee_seq_valid  = 0;

static uint8_t Zigbee_Crc8(const uint8_t *d, uint8_t n)
{
    uint8_t crc = 0x00u;
    for(uint8_t i = 0; i < n; i++)
    {
        crc ^= d[i];
        for(uint8_t b = 0; b < 8u; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

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

static const float ARM_Q_MIN[ARM_JOINT_NUM] = { -3.10f, -2.80f,  -2.40f, -10.0f, -10.0f, -10.0f};
static const float ARM_Q_MAX[ARM_JOINT_NUM] = {  0.05f,  0.05f,  0.23f,  10.0f,  10.0f,  10.0f};

/* 电流限值：额定电流的标幺值（PVT 帧内 ×10000，范围 0~10000），不是安培。
   0.7 = 额定的 70%。 */
static const float ARM_CUR_LIMIT[ARM_JOINT_NUM] = {
    0.7f,  0.7f,  0.7f,  0.7f,  0.7f,  0.7f
};

static float Arm_ClampJoint(int i, float v)
{
    if(v < ARM_Q_MIN[i]) return ARM_Q_MIN[i];
    if(v > ARM_Q_MAX[i]) return ARM_Q_MAX[i];
    return v;
}

/* 夹爪行程（rad），实测机械限位 -1.72 / +2.10。
   两个值同时是软限位，Watch 里填错也只会停在区间边界。 */
volatile float g_grip_open  = -1.72f;
volatile float g_grip_close = 2.10f;

/* 夹爪下发周期。二值动作不需要 500Hz，10ms 一帧可明显降低 CAN1 负载。 */
#define GRIP_TX_PERIOD_MS   10u
#define GRIP_TX_PERIOD_S    0.010f

/* 每拍最大步进(rad)，10ms 一拍 ⇒ 数值 ×100 即 rad/s。
   0.15 已到 GRIP_MAX_LAG 上限，再大也不会更快。 */
volatile float g_grip_step  = 0.15f;

/* 夹爪电流限值：同为标幺值，上限 1.0（= 满额定）。 */
volatile float g_grip_cur = 0.9f;

/* 目标相对实测位置的最大超前(rad)。夹住物体后目标走不动，
   钳住误差即钳住夹持力，避免长时间堵转升温。 */
#define GRIP_MAX_LAG        0.15f

/* 首帧反馈到达时把目标对齐到实际位置，避免从 0 起算导致上电猛甩。 */
static uint8_t arm_grip_synced = 0;

/* 夹爪控制模式基址，0x300 = 力位混合(PVT)。
   Arm_SendCmd 会把四种基址都发一遍，这里只决定先发哪个。 */
volatile uint32_t g_grip_id_base = 0x300u;

/* 发一条命令帧到第 i 台电机。
   夹爪模式未最终确认，0xFB/0xFC/0xFD 这类命令帧对四种基址各发一遍 ——
   不含位置数据，命中或落空都不会让电机乱动。控制帧(PVT)仍固定走 0x300。 */
static void Arm_SendCmd(DM_Motor_t *m, int i, uint32_t cmd)
{
#if ARM_USE_GRIPPER
    if(i == ARM_JOINT_NUM)
    {
        static const uint32_t bases[4] = {0x000u, 0x100u, 0x200u, 0x300u};
        DM_Motor_CmdAt(m->hfdcan, g_grip_id_base, m->can_id, cmd);
        for(int b = 0; b < 4; b++)
        {
            if(bases[b] != g_grip_id_base)
                DM_Motor_CmdAt(m->hfdcan, bases[b], m->can_id, cmd);
        }
        return;
    }
#endif
    DM_Motor_CmdAt(m->hfdcan, 0x300u, m->can_id, cmd);
}

/* ===== 紧急停 / 紧急停并自动复位 ======================================
 * 给 HardFault、控制环挂死监控调用：故障上下文里调度器可能已死，
 * 全程只用轮询 CAN 发送和裸延时，禁止 osDelay/互斥量。
 * Error_Handler 只停不复位（启动期失败不该进复位循环）。           */
void Arm_EmergencyStop(void)
{
    __disable_irq();
    for(int i = 0; i < ARM_JOINT_NUM + ARM_USE_GRIPPER; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m != NULL && m->hfdcan != NULL) Arm_SendCmd(m, i, DM_CMD_DISABLE);
    }
}

void Arm_EmergencyStopAndReset(void)
{
    Arm_EmergencyStop();
    for(volatile int d = 0; d < 400000; d++);   /* 几 ms，等失能帧上总线 */
    NVIC_SystemReset();
}

static void Arm_DecodeFrame(uint8_t active, uint8_t grip, uint8_t seq)
{
    uint32_t now = HAL_GetTick();
    uint8_t deadman = (uint8_t)(active & 0x01u);
    uint8_t long_gap = 0;
    uint8_t reacquire = (deadman && !arm.active);

    arm.gripper_cmd_recv = grip;
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
    arm.active = deadman;
    arm_deploy_cmd = (uint8_t)((active >> 1) & 0x03u);
    arm.connected = 1;

    /* 用序号统计真实丢帧：正常应为 +1 */
    if(zigbee_seq_valid)
    {
        uint8_t adv = (uint8_t)(seq - zigbee_last_seq);
        if(adv > 1u) zb_seq_lost += (uint32_t)(adv - 1u);
    }
    zigbee_last_seq  = seq;
    zigbee_seq_valid = 1;

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

    if(zigbee_idx == ZIGBEE_FRAME_LEN)
    {
        uint8_t crc_calc = Zigbee_Crc8(zigbee_buf, (uint8_t)(ZIGBEE_FRAME_LEN - 2u)); /* [0..15] */
        if(zigbee_buf[ZIGBEE_FRAME_LEN - 1u] == 0x55 &&
           crc_calc == zigbee_buf[ZIGBEE_FRAME_LEN - 2u])
        {
            Arm_DecodeFrame(zigbee_buf[ZIGBEE_ACTIVE_OFFSET],
                            zigbee_buf[ZIGBEE_GRIP_OFFSET],
                            zigbee_buf[ZIGBEE_SEQ_OFFSET]);
            zigbee_idx = 0;
        }
        else
        {
            if(zigbee_buf[ZIGBEE_FRAME_LEN - 1u] != 0x55) zb_bad_tail++;
            else                                          zb_crc_err++;
            Arm_ZigbeeResync();
        }
        return;
    }
    if(zigbee_idx >= ZIGBEE_FRAME_LEN + 1u) zigbee_idx = 0;
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

static void Teach_FlashLoad(void);   /* 定义在示教函数段后面 */

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

    Teach_FlashLoad();
    Slot_FlashLoad();
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

/* 使能序列跨多个 osDelay，期间物理档位可能拨回下档：每一步都重新确认 */
static uint8_t Arm_EnableAllowed(void)
{
    return (DR16_IsOnline() && DR16_Get_Data()->rc.s[0] != DR16_SW_DOWN) ? 1u : 0u;
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

    if(arm.state != arm.last_state)
    {
        if(arm.last_state == ARM_STATE_DISABLED && arm.state != ARM_STATE_DISABLED)
        {
            /* 途中拨回下档即中止；已使能的由下一拍 DISABLED 分支统一失能 */
            for(int i = 0; i < ARM_MOTOR_NUM; i++)
            {
                if(!Arm_EnableAllowed()) break;
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) { Arm_SendCmd(m, i, DM_CMD_CLEAR_ERR); osDelay(1); }
            }
            if(Arm_EnableAllowed())
            {
                osDelay(2);
                for(int i = 0; i < ARM_MOTOR_NUM; i++)
                {
                    if(!Arm_EnableAllowed()) break;
                    DM_Motor_t *m = DM_Motor_Get(i);
                    if(m) {
                        Arm_SendCmd(m, i, DM_CMD_ENABLE);
                        if(i < ARM_JOINT_NUM && Arm_FbFresh(m))
                        {
                            arm_lock_pos[i]   = m->position;
                            arm.cmd_smooth[i] = m->position;
                        }
                        osDelay(1);
                    }
                }
            }
        }
        else if(arm.state == ARM_STATE_LOCKED)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m && Arm_FbFresh(m))
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
            for(int i = 0; i < ARM_MOTOR_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) { Arm_SendCmd(m, i, DM_CMD_DISABLE); osDelay(1); }
            }
            arm.q_offset_valid = 0;
            arm_q_offset_mask = 0;
            arm_catchup_mask = 0;
            arm_grip_synced = 0;
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

/* ===== 跟随限速 / 减速带（Watch 可调）====================================
 * 旧行为：匀速怼到目标再瞬间钉住，急停时加速度是阶跃 → 靠电机内部 PD 硬吃
 * 惯性，表现为过冲 + 回抽。开减速带后按剩余距离提前收速 v<=sqrt(2*a*d)。
 * g_decel_en=0 时逐拍结果与改动前完全一致；g_slew_max_rad_s 默认 3.0 也等于
 * 原来写死的 ARM_MAX_STEP，两个开关都能单独退回旧行为做对照。
 * ======================================================================== */
volatile float   g_slew_max_rad_s = 3.0f;                /* 跟随速度上限(rad/s) */
volatile float   g_decel_rad_s2   = 20.0f;               /* 减速带加速度(rad/s^2) */
volatile uint8_t g_decel_en       = 0;                   /* 0=旧行为 1=开减速带 */
volatile float   g_trk_err_peak[ARM_JOINT_NUM] = {0};    /* 实测-指令峰值(rad)，写0清零 */

static float Arm_RateLimit(float target, float current, float max_step)
{
    float err = target - current;
    if(err > max_step) return current + max_step;
    if(err < -max_step) return current - max_step;
    return target;
}

/* 反馈新鲜度：50ms 没新帧 = 反馈丢失(线松/电机掉电/总线故障)。
   丢反馈的关节不再信它的位置、也不再给它发 PVT，避免按冻结值或 0 位姿灌电流。 */
static uint8_t Arm_FbFresh(const DM_Motor_t *m)
{
    return (m->rx_cnt > 0u && (HAL_GetTick() - m->last_rx_tick) < 50u) ? 1u : 0u;
}

static uint8_t Arm_TeachFeedbackReady(void)
{
    if(!arm.q_offset_valid) return 0;
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || m->hfdcan == NULL || !Arm_FbFresh(m)) return 0;
    }
    return 1;
}

static void Arm_TeachStoreSample(void)
{
    if(g_teach_count >= ARM_TEACH_MAX_SAMPLES) return;
    for(int i = 0; i < ARM_JOINT_NUM; i++)
        teach_path[g_teach_count][i] = arm.cmd_smooth[i];
    teach_grip_path[g_teach_count] = arm.gripper_cmd_recv ? 1u : 0u;
    g_teach_index = g_teach_count;
    g_teach_count++;
}

static void Arm_TeachFinishRecording(void)
{
    Arm_TeachStoreSample();
    g_teach_state = (g_teach_count >= 2u) ? ARM_TEACH_READY : ARM_TEACH_EMPTY;
    g_teach_index = 0;
    if(g_teach_count < 2u) g_teach_error = ARM_TEACH_ERR_SHORT;
}

static void Arm_TeachStopReplay(void)
{
    g_teach_state = (g_teach_count >= 2u) ? ARM_TEACH_READY : ARM_TEACH_EMPTY;
    g_teach_index = 0;
    teach_end_hold = 0;
    teach_auto_replay = 0;
    arm.q_offset_valid = 0;
    arm_q_offset_mask = 0;
    arm_reclutch_req = 1;
}

static uint8_t Arm_TeachStartReturn(uint8_t auto_replay)
{
    uint32_t best_index = 0;
    float best_score = 1e30f;
    for(uint32_t n = 0; n < g_teach_count; n++)
    {
        float score = 0.0f;
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            float d = arm.cmd_smooth[i] - teach_path[n][i];
            score += d * d;
        }
        if(score < best_score)
        {
            best_score = score;
            best_index = n;
        }
    }
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        if(fabsf(arm.cmd_smooth[i] - teach_path[best_index][i]) > ARM_TEACH_PATH_TOL)
        {
            g_teach_error = auto_replay ? ARM_TEACH_ERR_START : ARM_TEACH_ERR_PATH;
            return 0;
        }
    }
    if(g_teach_speed < 0.2f) g_teach_speed = 0.2f;
    if(g_teach_speed > 2.0f) g_teach_speed = 2.0f;
    teach_replay_cursor = (float)best_index;
    g_teach_index = (uint16_t)best_index;
    teach_end_hold = 0;
    teach_auto_replay = auto_replay;
    g_teach_state = ARM_TEACH_RETURN;
    return 1;
}

static void Arm_TeachHandleRequest(void)
{
    uint8_t running = (arm.state == ARM_STATE_FOLLOWING && arm.connected && arm.active);
    uint8_t motion = (g_teach_state == ARM_TEACH_REPLAY || g_teach_state == ARM_TEACH_DONE || g_teach_state == ARM_TEACH_RETURN);

    if(g_cycle_state == 0u && (!running || arm_deploy_cmd != 0u))
    {
        if(g_teach_state == ARM_TEACH_RECORDING) Arm_TeachFinishRecording();
        else if(motion) Arm_TeachStopReplay();
        if(g_teach_state != ARM_TEACH_EMPTY)
            g_teach_error = running ? ARM_TEACH_ERR_DEPLOY : ARM_TEACH_ERR_MODE;
    }
    motion = (g_teach_state == ARM_TEACH_REPLAY || g_teach_state == ARM_TEACH_DONE || g_teach_state == ARM_TEACH_RETURN);

    uint8_t req = g_teach_req;
    if(req == 0u) return;
    g_teach_req = 0;
    g_teach_error = 0;

    /* 一键循环占用臂期间，Watch 示教请求一律报忙；req=2 当作全停 */
    if(g_cycle_state != 0u)
    {
        if(req == 2u)
        {
            if(g_cycle_state == 3u) Arm_TeachStopReplay();
            g_cycle_state = 0u;
            arm_reclutch_req = 1;
        }
        else g_teach_error = ARM_TEACH_ERR_BUSY;
        return;
    }

    if(req == 1u)
    {
        if(!running)
            g_teach_error = ARM_TEACH_ERR_MODE;
        else if(arm_deploy_cmd != 0u)
            g_teach_error = ARM_TEACH_ERR_DEPLOY;
        else if(!Arm_TeachFeedbackReady())
            g_teach_error = ARM_TEACH_ERR_FEEDBACK;
        else
        {
            if(motion)
            {
                Arm_TeachStopReplay();
                arm_catchup_mask = 0;
                Arm_InterpResetAll();
                arm_reclutch_req = 0;  /* 本拍关节循环会重新捕获偏移 */
            }
            g_teach_count = 0;
            g_teach_index = 0;
            teach_replay_cursor = 0.0f;
            teach_replay_grip = 0;
            teach_sample_div = ARM_TEACH_SAMPLE_TICKS - 1u;
            teach_end_hold = 0;
            teach_auto_replay = 0;
            g_teach_state = ARM_TEACH_RECORDING;
        }
    }
    else if(req == 2u)
    {
        if(g_teach_state == ARM_TEACH_RECORDING) Arm_TeachFinishRecording();
        else if(motion) Arm_TeachStopReplay();
    }
    else if(req == 3u)
    {
        if(g_teach_state == ARM_TEACH_RECORDING || g_teach_state == ARM_TEACH_REPLAY ||
           g_teach_state == ARM_TEACH_RETURN)
            g_teach_error = ARM_TEACH_ERR_BUSY;
        else if(g_teach_count < 2u)
            g_teach_error = ARM_TEACH_ERR_SHORT;
        else if(!running)
            g_teach_error = ARM_TEACH_ERR_MODE;
        else if(arm_deploy_cmd != 0u)
            g_teach_error = ARM_TEACH_ERR_DEPLOY;
        else if(!Arm_TeachFeedbackReady())
            g_teach_error = ARM_TEACH_ERR_FEEDBACK;
        else
        {
            uint8_t at_start = 1;
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                if(fabsf(arm.cmd_smooth[i] - teach_path[0][i]) > ARM_TEACH_START_TOL)
                {
                    at_start = 0;
                    break;
                }
            }
            if(at_start)
            {
                if(g_teach_speed < 0.2f) g_teach_speed = 0.2f;
                if(g_teach_speed > 2.0f) g_teach_speed = 2.0f;
                teach_replay_cursor = 0.0f;
                g_teach_index = 0;
                teach_end_hold = 0;
                teach_auto_replay = 0;
                g_teach_state = ARM_TEACH_REPLAY;
            }
            else (void)Arm_TeachStartReturn(1u);
        }
    }
    else if(req == 4u)
    {
        if(g_teach_state == ARM_TEACH_RECORDING || g_teach_state == ARM_TEACH_REPLAY ||
           g_teach_state == ARM_TEACH_RETURN)
            g_teach_error = ARM_TEACH_ERR_BUSY;
        else if(g_teach_count < 2u)
            g_teach_error = ARM_TEACH_ERR_SHORT;
        else if(!running)
            g_teach_error = ARM_TEACH_ERR_MODE;
        else if(arm_deploy_cmd != 0u)
            g_teach_error = ARM_TEACH_ERR_DEPLOY;
        else if(!Arm_TeachFeedbackReady())
            g_teach_error = ARM_TEACH_ERR_FEEDBACK;
        else (void)Arm_TeachStartReturn(0u);
    }
}

static void Arm_TeachPrepareReplay(void)
{
    float last = (float)(g_teach_count - 1u);
    if(g_teach_state == ARM_TEACH_REPLAY)
    {
        teach_replay_cursor += g_teach_speed / (float)ARM_TEACH_SAMPLE_TICKS;
        if(teach_replay_cursor > last) teach_replay_cursor = last;
    }
    else if(g_teach_state == ARM_TEACH_RETURN)
    {
        teach_replay_cursor -= g_teach_speed / (float)ARM_TEACH_SAMPLE_TICKS;
        if(teach_replay_cursor < 0.0f) teach_replay_cursor = 0.0f;
    }
    if(teach_replay_cursor > last) teach_replay_cursor = last;

    uint32_t index = (uint32_t)teach_replay_cursor;
    float frac = 0.0f;
    if(index >= g_teach_count - 1u) index = g_teach_count - 1u;
    else frac = teach_replay_cursor - (float)index;
    g_teach_index = (uint16_t)index;
    teach_replay_grip = teach_grip_path[index];

    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        teach_replay_goal[i] = teach_path[index][i];
        if(frac > 0.0f)
            teach_replay_goal[i] += frac * (teach_path[index + 1u][i] - teach_path[index][i]);
    }
}

static void Arm_TeachTickAfterJoints(void)
{
    if(g_teach_state == ARM_TEACH_RECORDING)
    {
        if(++teach_sample_div >= ARM_TEACH_SAMPLE_TICKS)
        {
            teach_sample_div = 0;
            Arm_TeachStoreSample();
            if(g_teach_count >= ARM_TEACH_MAX_SAMPLES)
            {
                g_teach_state = ARM_TEACH_READY;
                g_teach_index = 0;
                g_teach_error = ARM_TEACH_ERR_FULL;
            }
        }
    }

    if(g_teach_state == ARM_TEACH_RETURN && teach_replay_cursor <= 0.0f)
    {
        uint8_t all_at_start = 1;
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(m == NULL || !Arm_FbFresh(m) || fabsf(m->position - teach_path[0][i]) > ARM_TEACH_END_TOL)
            {
                all_at_start = 0;
                break;
            }
        }
        if(all_at_start)
        {
            if(++teach_end_hold >= ARM_TEACH_END_HOLD_TICKS)
            {
                g_teach_error = 0;
                if(teach_auto_replay)
                {
                    teach_auto_replay = 0;
                    teach_replay_cursor = 0.0f;
                    g_teach_index = 0;
                    teach_end_hold = 0;
                    g_teach_state = ARM_TEACH_REPLAY;
                }
                else Arm_TeachStopReplay();
            }
        }
        else teach_end_hold = 0;
    }

    if(g_teach_state == ARM_TEACH_REPLAY &&
       teach_replay_cursor >= (float)(g_teach_count - 1u))
    {
        uint8_t all_at_end = 1;
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(m == NULL || !Arm_FbFresh(m) || fabsf(m->position - teach_path[g_teach_count - 1u][i]) > ARM_TEACH_END_TOL)
            {
                all_at_end = 0;
                break;
            }
        }
        if(all_at_end)
        {
            if(++teach_end_hold >= ARM_TEACH_END_HOLD_TICKS) g_teach_state = ARM_TEACH_DONE;
        }
        else teach_end_hold = 0;
    }
}

/* ===== 按键一键循环：工作位 → 轨迹首点 → 回放 ============================
 * 命令码 3 是主手按键的 500ms 脉冲：上升沿时空闲则启动循环；
 * 循环中(含回放完保持)再收到脉冲 = 交还跟随。
 * 档位离开上档、无线断开、或 Watch 起了抽臂/取矿(cmd 1/2)：立即中止并重新吸合。
 * 启动不要求主手使能(deadman)，和抽臂/取矿同一套安全约定：
 * 中止手段 = 再按一下、降档、无线超时。                                  */
static void Arm_CycleTick(void)
{
    static uint8_t cmd3_last = 0;
    uint8_t cmd3 = (arm_deploy_cmd == 3u) ? 1u : 0u;
    uint8_t pulse = (cmd3 && !cmd3_last);
    cmd3_last = cmd3;

    /* 空闲时按键归矿位流程；已经有循环在跑就让它把这一段收尾，别半路撒手 */
    if(g_slot_enable && g_cycle_state == 0u && pulse) return;

    if(g_cycle_state != 0u &&
       (arm.state != ARM_STATE_FOLLOWING || !arm.connected ||
        (arm_deploy_cmd != 0u && arm_deploy_cmd != 3u)))
    {
        if(g_cycle_state == 3u) Arm_TeachStopReplay();
        g_cycle_state = 0u;
        arm_reclutch_req = 1;
        return;
    }

    if(pulse)
    {
        if(g_cycle_state != 0u)                 /* 循环中再按 = 交还跟随 */
        {
            if(g_cycle_state == 3u) Arm_TeachStopReplay();
            g_cycle_state = 0u;
            arm_reclutch_req = 1;
            return;
        }
        if(g_teach_count < 2u)                  /* 没录轨迹：只报错不动作 */
        {
            g_cycle_err = 1u;
            return;
        }
        g_cycle_err = 0u;
        if(g_teach_state == ARM_TEACH_RECORDING) Arm_TeachFinishRecording();
        else if(g_teach_state == ARM_TEACH_REPLAY || g_teach_state == ARM_TEACH_RETURN)
            Arm_TeachStopReplay();
        for(int i = 0; i < ARM_JOINT_NUM; i++) cycle_goal[i] = g_deploy_pos[i];
        cycle_hold = 0;
        g_cycle_state = 1u;
        return;
    }

    if(g_cycle_state == 1u || g_cycle_state == 2u)
    {
        uint8_t all_at = 1;
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(m == NULL || !Arm_FbFresh(m) || fabsf(m->position - cycle_goal[i]) > ARM_CYCLE_TOL)
            {
                all_at = 0;
                break;
            }
        }
        if(!all_at) { cycle_hold = 0; return; }
        if(++cycle_hold < ARM_CYCLE_HOLD_TICKS) return;
        cycle_hold = 0;
        if(g_cycle_state == 1u)
        {
            for(int i = 0; i < ARM_JOINT_NUM; i++) cycle_goal[i] = teach_path[0][i];
            g_cycle_state = 2u;
        }
        else
        {
            teach_replay_cursor = 0.0f;
            g_teach_index = 0;
            teach_end_hold = 0;
            teach_auto_replay = 0;
            g_teach_state = ARM_TEACH_REPLAY;
            g_cycle_state = 3u;
        }
    }
    else if(g_cycle_state == 3u && g_teach_state == ARM_TEACH_DONE)
    {
        g_cycle_state = 4u;
    }
}

/* ===== 轨迹掉电保存（内部 Flash 最后一个扇区）=============================
 * H723 的 1MB Flash 分 8 个 128KB 扇区，程序只占约 76KB，借 7 号扇区存轨迹。
 * 布局：32B 头(magic/count/crc32) + count*25B(6 个 float + 1 个夹爪字节)。
 * 擦除+写入期间全片取指停顿约 1~2s，所以只允许下档(电机已失能)时操作，
 * Watch 写 g_teach_save_req：1=保存当前轨迹，2=删除 Flash 轨迹(只写坏头，毫秒级)。
 * 头最后写：中途断电整份保存作废，不会留下半截轨迹。
 * 上电自动加载有效轨迹，直接进 state=2 就绪。                            */
#define TEACH_FLASH_SECTOR   7u
#define TEACH_FLASH_ADDR     0x080E0000u
#define TEACH_FLASH_MAGIC    0x54454143u
#define TEACH_SAMPLE_BYTES   25u

volatile uint8_t g_teach_save_req = 0;
volatile uint8_t g_teach_save_err = 0;   /* 0=ok 1=不在下档 2=Flash操作失败 3=没轨迹 4=加载校验不过 */
volatile uint8_t g_teach_saved = 0;      /* 1=Flash 里有有效轨迹 */

static uint32_t Teach_Crc32(const uint8_t *d, uint32_t n)
{
    uint32_t crc = 0xFFFFFFFFu;
    for(uint32_t i = 0; i < n; i++)
    {
        crc ^= d[i];
        for(int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88321u : crc >> 1;
    }
    return ~crc;
}

static uint8_t Teach_FeedByte(uint32_t pos)
{
    uint32_t sample = pos / TEACH_SAMPLE_BYTES;
    uint32_t off    = pos % TEACH_SAMPLE_BYTES;
    if(off < 24u) return ((const uint8_t *)&teach_path[sample][0])[off];
    return teach_grip_path[sample];
}

static void Teach_FlashSave(void)
{
    uint32_t payload = (uint32_t)g_teach_count * TEACH_SAMPLE_BYTES;

    /* 擦+写期间全片取指停顿约 1~2s：监控任务在此窗口内不判挂死 */
    g_arm_busy_until = HAL_GetTick() + 8000u;

    uint32_t crc = 0xFFFFFFFFu;
    for(uint32_t p = 0; p < payload; p++)
    {
        crc ^= Teach_FeedByte(p);
        for(int b = 0; b < 8; b++)
            crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88321u : crc >> 1;
    }
    crc = ~crc;

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef er = {0};
    er.TypeErase    = FLASH_TYPEERASE_SECTORS;
    er.Banks        = FLASH_BANK_1;
    er.Sector       = TEACH_FLASH_SECTOR;
    er.NbSectors    = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_err = 0;
    if(HAL_FLASHEx_Erase(&er, &sector_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        g_arm_busy_until = 0;
        g_teach_save_err = 2;
        return;
    }

    static uint8_t chunk[32] __ALIGNED(32);
    uint32_t addr = TEACH_FLASH_ADDR + 32u;
    uint32_t p = 0;
    while(p < payload)
    {
        for(int i = 0; i < 32; i++)
            chunk[i] = (p < payload) ? Teach_FeedByte(p++) : 0xFFu;
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)chunk) != HAL_OK)
        {
            HAL_FLASH_Lock();
            g_arm_busy_until = 0;
            g_teach_save_err = 2;
            return;
        }
        addr += 32u;
    }

    static uint32_t hdr[8] __ALIGNED(32);
    for(int i = 0; i < 8; i++) hdr[i] = 0xFFFFFFFFu;
    hdr[0] = TEACH_FLASH_MAGIC;
    hdr[1] = g_teach_count;
    hdr[2] = crc;
    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, TEACH_FLASH_ADDR, (uint32_t)hdr) != HAL_OK)
    {
        HAL_FLASH_Lock();
        g_arm_busy_until = 0;
        g_teach_save_err = 2;
        return;
    }
    HAL_FLASH_Lock();
    g_arm_busy_until = 0;
    g_teach_saved = 1;
    g_teach_save_err = 0;
}

static void Teach_FlashDelete(void)
{
    static uint32_t zero[8] __ALIGNED(32) = {0};
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                               TEACH_FLASH_ADDR, (uint32_t)zero);
    HAL_FLASH_Lock();
    g_teach_save_err = (st == HAL_OK) ? 0u : 2u;
    if(st == HAL_OK) g_teach_saved = 0;
}

static void Teach_FlashLoad(void)
{
    const uint32_t *hdr = (const uint32_t *)TEACH_FLASH_ADDR;
    if(hdr[0] != TEACH_FLASH_MAGIC) return;
    uint32_t count = hdr[1];
    if(count < 2u || count > ARM_TEACH_MAX_SAMPLES)
    {
        g_teach_save_err = 4;
        return;
    }
    uint32_t payload = count * TEACH_SAMPLE_BYTES;
    const uint8_t *src = (const uint8_t *)(TEACH_FLASH_ADDR + 32u);
    if(Teach_Crc32(src, payload) != hdr[2])
    {
        g_teach_save_err = 4;
        return;
    }
    for(uint32_t s = 0; s < count; s++)
    {
        const uint8_t *q = src + s * TEACH_SAMPLE_BYTES;
        for(int i = 0; i < ARM_JOINT_NUM; i++)
            memcpy(&teach_path[s][i], q + i * 4u, 4u);
        teach_grip_path[s] = q[24];
    }
    g_teach_count = (uint16_t)count;
    g_teach_index = 0;
    g_teach_state = ARM_TEACH_READY;
    g_teach_saved = 1;
}

/* ===== 单臂两矿位：点位映射 + 顺序执行 + 异常处理 ==========================
 * 本臂负责矿位 g_slot_base 和 g_slot_base+1（臂A=1,2；臂B=3,4）。
 * 顺序固定、取放互为逆序（车上后放的先取，LIFO）：
 *   放矿 槽0→槽1  =  臂A 先放1再放2 / 臂B 先放3再放4（"34"）
 *   取矿 槽1→槽0  =  臂A 先取2再取1（"21"）/ 臂B 先取4再取3
 *
 * 一次完整作业 = 手动取、自动放：
 *   g_slot_cmd=2（或主手按键）：自动走到"该取的矿位"的取矿点，到位立刻交还跟随，
 *     操作手手动微调对矿并按夹爪键合爪；
 *   g_slot_cmd=1（主手按键上升沿）：回工作位 → 走到该矿位的放矿点 → 张爪卸矿并保持
 *     ARM_SLOT_OPEN_MS → 回工作位 → 交还跟随。
 * 点位存的是 6 台电机的绝对角（与 g_deploy_pos 同一套约定），不看主手角度、
 * 不走主从偏移，所以录一次就能反复走到同一个姿态。
 * 每步到位判据 = 实测位置进容差 + 连续保持 + 反馈新鲜；任一关节反馈丢失或
 * 单步超时一律中止并交还，绝不带着走歪的轨迹继续。
 * g_slot_enable=0 时本层完全不介入，行为回到纯跟随 + 旧的按键循环。
 * ======================================================================== */
#define ARM_SLOT_ARRIVE_TOL   0.03f     /* 单轴到位容差(rad) */
#define ARM_SLOT_HOLD_TICKS   50u       /* 到位后连续保持 100ms 才算数 */
#define ARM_SLOT_OPEN_MS      500u      /* 放矿位张爪保持时间 */
#define ARM_SLOT_WORK_TOL     0.05f     /* 判断"已经在标准交接位"的容差，可省一步 */
#define ARM_SLOT_SETTLE_TOL   0.02f     /* 录点位时实测与指令的最大差(臂必须静止) */

#define ARM_SLOT_ST_IDLE      0u
#define ARM_SLOT_ST_GO_WORK   1u        /* 先把矿带回标准交接位 */
#define ARM_SLOT_ST_GO_PLACE  2u        /* 走向放矿点 */
#define ARM_SLOT_ST_OPEN      3u        /* 张爪卸矿 + 保持 */
#define ARM_SLOT_ST_BACK      4u        /* 回标准交接位 */
#define ARM_SLOT_ST_GO_TAKE   5u        /* 走向取矿点，到位交还手动 */

#define ARM_SLOT_ERR_NONE      0u
#define ARM_SLOT_ERR_MODE      1u       /* 不在上档 / 主手掉线 / 被别的自动命令打断 */
#define ARM_SLOT_ERR_POSE      2u       /* 点位没录、NaN 或超出软限位 */
#define ARM_SLOT_ERR_BUSY      3u       /* 抽臂 / 一键循环 / 示教回放占用中 */
#define ARM_SLOT_ERR_SEQ       4u       /* 顺序或库存冲突（放满/取空/该位已有矿） */
#define ARM_SLOT_ERR_FEEDBACK  5u       /* 反馈不新鲜：拒绝启动，行走中则中止 */
#define ARM_SLOT_ERR_TIMEOUT   6u       /* 单步超时：被卡住、被矿顶住、走不到位 */
#define ARM_SLOT_ERR_SETTLE    7u       /* 录点位时臂还在动 */

volatile uint8_t g_slot_enable   = 1u;   /* 0=本层不介入 */
volatile uint8_t g_slot_base     = 1u;   /* 1=本臂管矿位1,2  3=本臂管矿位3,4 */
volatile uint8_t g_slot_cmd      = 0u;   /* 1=自动放矿 2=走到下一取矿点 3=中止 */
volatile uint8_t g_slot_state    = ARM_SLOT_ST_IDLE;
volatile uint8_t g_slot_err      = ARM_SLOT_ERR_NONE;
volatile uint8_t g_slot_target   = 0u;   /* 本流程针对的槽下标，矿位号 = base + target */
volatile uint8_t g_slot_occupied[ARM_SLOT_NUM] = {0u, 0u};
volatile int8_t  g_slot_place_ptr = 0;   /* 放矿指针 0→1 */
volatile int8_t  g_slot_take_ptr  = 1;   /* 取矿指针 1→0（后放先取） */
volatile int8_t  g_slot_carry     = -1;  /* 手上这颗矿来自哪个槽，-1=不知道(纯手动夹的) */
volatile uint8_t g_slot_force     = 0u;  /* 1=忽略库存冲突（人工确认过再用） */
volatile uint8_t g_slot_pose_cmd  = 0u;  /* 1=录取矿点 2=录放矿点 3=清除本槽 */
volatile uint8_t g_slot_pose_sel  = 0u;  /* 录入/清除的槽下标 */
volatile uint8_t g_slot_saved     = 0u;  /* Flash 里有点位表 */
volatile uint8_t g_slot_save_req  = 0u;  /* 1=存Flash 2=删除；仅下档生效 */
volatile uint8_t g_slot_save_err  = 0u;  /* 0=ok 1=不在下档 2=Flash失败 4=加载校验不过 */

Arm_SlotPose_t arm_slot[ARM_SLOT_NUM];
static float    slot_goal[ARM_JOINT_NUM];
static uint8_t  slot_hold = 0;
static uint32_t slot_step_deadline = 0;
static uint32_t slot_dwell_until = 0;

static uint8_t Arm_SlotPoseOk(const volatile float *p)
{
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        if(!(p[i] == p[i])) return 0u;                                  /* NaN */
        if(p[i] < ARM_Q_MIN[i] || p[i] > ARM_Q_MAX[i]) return 0u;       /* 超软限位 */
    }
    return 1u;
}

static uint8_t Arm_SlotFbOk(void)
{
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || !Arm_FbFresh(m)) return 0u;
    }
    return 1u;
}

/* 抽臂/一键循环/示教回放各自都是完整的动作序列，同时跑必然互相抢 cmd_smooth */
static uint8_t Arm_SlotBusy(void)
{
    if(arm_deploy_cmd == 1u || arm_deploy_cmd == 2u || g_cycle_state != 0u) return 1u;
    if(g_teach_state == ARM_TEACH_RECORDING || g_teach_state == ARM_TEACH_REPLAY ||
       g_teach_state == ARM_TEACH_RETURN) return 1u;
    return 0u;
}

/* 本步超时按实际距离和当前速度算，再留 3 倍余量：
   写死一个秒数在 g_deploy_speed=0.05 时会造成慢速动作必然超时。 */
static void Arm_SlotGoalSet(const volatile float *pose)
{
    float worst = 0.0f;
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        slot_goal[i] = pose[i];
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m != NULL)
        {
            float d = fabsf(Arm_ClampJoint(i, pose[i]) - arm.cmd_smooth[i]);
            if(d > worst) worst = d;
        }
    }
    float speed = (g_deploy_speed > 0.05f) ? g_deploy_speed : 0.05f;
    uint32_t est = (uint32_t)((worst / speed) * 1000.0f);
    if(est < 2000u) est = 2000u;
    slot_step_deadline = HAL_GetTick() + est * 3u;
    slot_hold = 0;
}

/* 交还控制权：以当前位置重新吸合，主手不会感觉到跳变 */
static void Arm_SlotStop(uint8_t err)
{
    g_slot_state = ARM_SLOT_ST_IDLE;
    slot_hold = 0;
    g_slot_err = err;
    arm_reclutch_req = 1;
}

/* 0=还在走 1=已到位并保持 2=单步超时 */
static uint8_t Arm_SlotArrived(void)
{
    if((int32_t)(HAL_GetTick() - slot_step_deadline) >= 0) return 2u;
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || !Arm_FbFresh(m) ||
           fabsf(m->position - slot_goal[i]) > ARM_SLOT_ARRIVE_TOL)
        {
            slot_hold = 0;
            return 0u;
        }
    }
    return (++slot_hold >= ARM_SLOT_HOLD_TICKS) ? 1u : 0u;
}

static uint8_t Arm_SlotAtWorkPose(void)
{
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || !Arm_FbFresh(m)) return 0u;
        if(fabsf(m->position - g_deploy_pos[i]) > ARM_SLOT_WORK_TOL) return 0u;
    }
    return 1u;
}

/* 录点位：上档跟随 + 主手在线 + 六轴反馈新鲜 + 臂确实静止，取电机绝对角。
   正在走的姿态录进去没有复现价值，所以静止判据不过就报错不写表。 */
static uint8_t Arm_SlotCapture(float *dst)
{
    if(arm.state != ARM_STATE_FOLLOWING || !arm.connected) return ARM_SLOT_ERR_MODE;
    if(!Arm_SlotFbOk()) return ARM_SLOT_ERR_FEEDBACK;
    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(fabsf(m->position - arm.cmd_smooth[i]) > ARM_SLOT_SETTLE_TOL)
            return ARM_SLOT_ERR_SETTLE;
        dst[i] = m->position;
    }
    if(!Arm_SlotPoseOk(dst)) return ARM_SLOT_ERR_POSE;
    return ARM_SLOT_ERR_NONE;
}

static void Arm_SlotPoseHandler(void)
{
    if(g_slot_pose_cmd == 0u) return;
    uint8_t cmd = g_slot_pose_cmd;
    g_slot_pose_cmd = 0u;
    g_slot_err = ARM_SLOT_ERR_NONE;

    if(g_slot_pose_sel >= ARM_SLOT_NUM)
    {
        g_slot_err = ARM_SLOT_ERR_POSE;
        return;
    }
    uint8_t s = g_slot_pose_sel;

    if(cmd == 1u)
    {
        uint8_t e = Arm_SlotCapture(arm_slot[s].take);
        if(e != ARM_SLOT_ERR_NONE) { g_slot_err = e; return; }
        arm_slot[s].take_valid = 1u;
    }
    else if(cmd == 2u)
    {
        uint8_t e = Arm_SlotCapture(arm_slot[s].place);
        if(e != ARM_SLOT_ERR_NONE) { g_slot_err = e; return; }
        arm_slot[s].place_valid = 1u;
    }
    else if(cmd == 3u)
    {
        arm_slot[s].take_valid = 0u;
        arm_slot[s].place_valid = 0u;
        g_slot_occupied[s] = 0u;
    }
}

/* 一轮 = 放满两个位、再按逆序取空。两头都走完才允许开新一轮；
   一半一半说明流程被打断过，必须人工确认（Watch 改指针或 g_slot_force=1）。 */
static uint8_t Arm_SlotRoundReady(void)
{
    if(g_slot_place_ptr < (int8_t)ARM_SLOT_NUM && g_slot_take_ptr >= 0) return 1u;
    if(g_slot_place_ptr >= (int8_t)ARM_SLOT_NUM && g_slot_take_ptr < 0)
    {
        g_slot_place_ptr = 0;
        g_slot_take_ptr = (int8_t)ARM_SLOT_NUM - 1;
        return 1u;
    }
    return 0u;
}

static void Arm_SlotStartPlace(void)
{
    if(!Arm_SlotRoundReady())
    {
        g_slot_err = ARM_SLOT_ERR_SEQ;         /* 放取指针不同步，先人工核对 */
        return;
    }
    if(g_slot_place_ptr >= (int8_t)ARM_SLOT_NUM)
    {
        g_slot_err = ARM_SLOT_ERR_SEQ;         /* 两个位都放过了，该去取矿 */
        return;
    }
    if(!Arm_SlotPoseOk(g_deploy_pos))
    {
        g_slot_err = ARM_SLOT_ERR_POSE;        /* 工作位本身非法，回不去就没意义 */
        return;
    }
    uint8_t s = (uint8_t)g_slot_place_ptr;
    if(!arm_slot[s].place_valid || !Arm_SlotPoseOk(arm_slot[s].place))
    {
        g_slot_err = ARM_SLOT_ERR_POSE;        /* 这个矿位的放矿点还没录 */
        return;
    }
    if(!g_slot_force && g_slot_occupied[s])
    {
        g_slot_err = ARM_SLOT_ERR_SEQ;         /* 该位已有矿：先取走，或纠正库存 */
        return;
    }
    g_slot_target = s;
    g_slot_err = ARM_SLOT_ERR_NONE;
    if(Arm_SlotAtWorkPose())
    {
        Arm_SlotGoalSet(arm_slot[s].place);
        g_slot_state = ARM_SLOT_ST_GO_PLACE;
    }
    else
    {
        Arm_SlotGoalSet(g_deploy_pos);
        g_slot_state = ARM_SLOT_ST_GO_WORK;
    }
}

static void Arm_SlotStartTake(void)
{
    if(!Arm_SlotRoundReady())
    {
        g_slot_err = ARM_SLOT_ERR_SEQ;
        return;
    }
    if(g_slot_take_ptr < 0)
    {
        g_slot_err = ARM_SLOT_ERR_SEQ;         /* 该取的两个位都取空了 */
        return;
    }
    uint8_t s = (uint8_t)g_slot_take_ptr;
    if(!arm_slot[s].take_valid || !Arm_SlotPoseOk(arm_slot[s].take))
    {
        g_slot_err = ARM_SLOT_ERR_POSE;        /* 这个矿位的取矿点还没录 */
        return;
    }
    if(!g_slot_force && !g_slot_occupied[s])
    {
        g_slot_err = ARM_SLOT_ERR_SEQ;         /* 库存显示这个位是空的 */
        return;
    }
    g_slot_target = s;
    g_slot_err = ARM_SLOT_ERR_NONE;
    Arm_SlotGoalSet(arm_slot[s].take);
    g_slot_state = ARM_SLOT_ST_GO_TAKE;
}

/* 控制环每拍调用一次，放在 Arm_CycleTick 之后，和它们用同一套到位判据 */
static void Arm_SlotTick(void)
{
    static uint8_t btn_last = 0;
    uint32_t now = HAL_GetTick();

    if(!g_slot_enable) return;

    /* 控制权被拿走或链路断：立即中止，不推进指针、不改库存 */
    if(g_slot_state != ARM_SLOT_ST_IDLE &&
       (arm.state != ARM_STATE_FOLLOWING || !arm.connected || Arm_SlotBusy()))
    {
        Arm_SlotStop(ARM_SLOT_ERR_MODE);
        return;
    }

    Arm_SlotPoseHandler();

    /* 主手按键(PH10)发来的命令码 3 上升沿 = 放矿流程开关；Watch 写 g_slot_cmd 同义 */
    uint8_t btn = (arm_deploy_cmd == 3u) ? 1u : 0u;
    uint8_t pulse = (btn && !btn_last);
    btn_last = btn;

    uint8_t cmd = g_slot_cmd;
    if(cmd == 0u && pulse) cmd = 1u;

    if(cmd != 0u)
    {
        g_slot_cmd = 0u;
        g_slot_err = ARM_SLOT_ERR_NONE;

        if(g_slot_state != ARM_SLOT_ST_IDLE)
        {
            Arm_SlotStop(ARM_SLOT_ERR_NONE);   /* 流程中再按 = 停住交还，矿不丢 */
            return;
        }
        if(arm.state != ARM_STATE_FOLLOWING || !arm.connected)
        {
            g_slot_err = ARM_SLOT_ERR_MODE;
            return;
        }
        if(Arm_SlotBusy())    { g_slot_err = ARM_SLOT_ERR_BUSY;     return; }
        if(!Arm_SlotFbOk())   { g_slot_err = ARM_SLOT_ERR_FEEDBACK; return; }

        if(cmd == 1u)      Arm_SlotStartPlace();
        else if(cmd == 2u) Arm_SlotStartTake();
        return;
    }

    /* 行走段反馈丢了不再靠超时兜底，直接中止并报错 */
    if((g_slot_state == ARM_SLOT_ST_GO_WORK || g_slot_state == ARM_SLOT_ST_GO_PLACE ||
        g_slot_state == ARM_SLOT_ST_BACK || g_slot_state == ARM_SLOT_ST_GO_TAKE) &&
       !Arm_SlotFbOk())
    {
        Arm_SlotStop(ARM_SLOT_ERR_FEEDBACK);
        return;
    }

    switch(g_slot_state)
    {
    case ARM_SLOT_ST_GO_WORK:
    {
        uint8_t r = Arm_SlotArrived();
        if(r == 2u) { Arm_SlotStop(ARM_SLOT_ERR_TIMEOUT); return; }
        if(r != 1u) return;
        Arm_SlotGoalSet(arm_slot[g_slot_target].place);
        g_slot_state = ARM_SLOT_ST_GO_PLACE;
        return;
    }
    case ARM_SLOT_ST_GO_PLACE:
    {
        uint8_t r = Arm_SlotArrived();
        if(r == 2u) { Arm_SlotStop(ARM_SLOT_ERR_TIMEOUT); return; }
        if(r != 1u) return;
        slot_dwell_until = now + ARM_SLOT_OPEN_MS;
        g_slot_state = ARM_SLOT_ST_OPEN;
        return;
    }
    case ARM_SLOT_ST_OPEN:
        if((int32_t)(now - slot_dwell_until) < 0) return;
        g_slot_occupied[g_slot_target] = 1u;
        if(g_slot_carry >= 0 && (uint8_t)g_slot_carry < ARM_SLOT_NUM)
            g_slot_occupied[g_slot_carry] = 0u;    /* 这颗矿的来源位空出来了 */
        g_slot_carry = -1;
        g_slot_place_ptr = (int8_t)(g_slot_place_ptr + 1);
        Arm_SlotGoalSet(g_deploy_pos);
        g_slot_state = ARM_SLOT_ST_BACK;
        return;

    case ARM_SLOT_ST_BACK:
    {
        uint8_t r = Arm_SlotArrived();
        if(r == 2u) { Arm_SlotStop(ARM_SLOT_ERR_TIMEOUT); return; }
        if(r != 1u) return;
        Arm_SlotStop(ARM_SLOT_ERR_NONE);           /* 一整趟放矿完成 */
        return;
    }
    case ARM_SLOT_ST_GO_TAKE:
    {
        uint8_t r = Arm_SlotArrived();
        if(r == 2u) { Arm_SlotStop(ARM_SLOT_ERR_TIMEOUT); return; }
        if(r != 1u) return;
        /* 到位即交还：微调对矿和合爪是手动的，合完再按按键走放矿流程 */
        g_slot_carry = (int8_t)g_slot_target;
        g_slot_take_ptr = (int8_t)(g_slot_take_ptr - 1);
        Arm_SlotStop(ARM_SLOT_ERR_NONE);
        return;
    }
    default:
        return;
    }
}

/* ===== 矿位点位表掉电保存（Flash 扇区 6，与轨迹扇区 7 分开擦除）===========
 * 布局：32B 头(magic/槽数/crc/本臂矿位号/三个指针) + 2×56B 记录，补到 128B。
 * 头最后写：中途断电整份作废，不会留下半截点位。
 * 擦+写有全片取指停顿，所以只允许下档(电机已失能)时操作。                  */
#define SLOT_FLASH_SECTOR    6u
#define SLOT_FLASH_ADDR      0x080C0000u
#define SLOT_FLASH_MAGIC     0x534C5431u    /* "SLT1" */
#define SLOT_REC_BYTES       56u            /* take24 + place24 + take_valid + place_valid + occupied + 保留 */
#define SLOT_IMG_BYTES       128u           /* 2×56=112 → 补到 4 个 32B flashword */

static void Slot_FlashWriteHeader(uint32_t magic, uint32_t crc)
{
    static uint32_t hdr[8] __ALIGNED(32);
    for(int i = 0; i < 8; i++) hdr[i] = 0xFFFFFFFFu;
    hdr[0] = magic;
    hdr[1] = ARM_SLOT_NUM;
    hdr[2] = crc;
    hdr[3] = g_slot_base;
    hdr[4] = (uint32_t)g_slot_place_ptr + 1u;    /* int8 可能为 -1，偏移后存 */
    hdr[5] = (uint32_t)g_slot_take_ptr  + 1u;
    hdr[6] = (uint32_t)g_slot_carry     + 1u;
    (void)HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, SLOT_FLASH_ADDR, (uint32_t)hdr);
}

static void Slot_FlashSave(void)
{
    static uint8_t img[SLOT_IMG_BYTES] __ALIGNED(32);

    g_arm_busy_until = HAL_GetTick() + 3000u;

    memset(img, 0xFF, sizeof(img));
    for(uint8_t s = 0; s < ARM_SLOT_NUM; s++)
    {
        uint8_t *r = img + s * SLOT_REC_BYTES;
        memcpy(r, arm_slot[s].take,  sizeof(arm_slot[s].take));
        memcpy(r + 24u, arm_slot[s].place, sizeof(arm_slot[s].place));
        r[48u] = arm_slot[s].take_valid;
        r[49u] = arm_slot[s].place_valid;
        r[50u] = g_slot_occupied[s];
    }
    uint32_t crc = Teach_Crc32(img, ARM_SLOT_NUM * SLOT_REC_BYTES);

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef er = {0};
    er.TypeErase    = FLASH_TYPEERASE_SECTORS;
    er.Banks        = FLASH_BANK_1;
    er.Sector       = SLOT_FLASH_SECTOR;
    er.NbSectors    = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    uint32_t sector_err = 0;
    if(HAL_FLASHEx_Erase(&er, &sector_err) != HAL_OK)
    {
        HAL_FLASH_Lock();
        g_arm_busy_until = 0;
        g_slot_save_err = 2;
        return;
    }
    uint32_t addr = SLOT_FLASH_ADDR + 32u;
    for(uint32_t w = 0; w < SLOT_IMG_BYTES / 32u; w++)
    {
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, addr, (uint32_t)&img[w * 32u]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            g_arm_busy_until = 0;
            g_slot_save_err = 2;
            return;
        }
        addr += 32u;
    }
    Slot_FlashWriteHeader(SLOT_FLASH_MAGIC, crc);
    HAL_FLASH_Lock();
    g_arm_busy_until = 0;
    g_slot_saved = 1u;
    g_slot_save_err = 0u;
}

static void Slot_FlashDelete(void)
{
    static uint32_t zero[8] __ALIGNED(32) = {0};
    HAL_FLASH_Unlock();
    HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                                            SLOT_FLASH_ADDR, (uint32_t)zero);
    HAL_FLASH_Lock();
    g_slot_save_err = (st == HAL_OK) ? 0u : 2u;
    if(st == HAL_OK) g_slot_saved = 0u;
}

static void Slot_FlashLoad(void)
{
    const uint32_t *hdr = (const uint32_t *)SLOT_FLASH_ADDR;
    if(hdr[0] != SLOT_FLASH_MAGIC) return;
    if(hdr[1] != ARM_SLOT_NUM) { g_slot_save_err = 4; return; }

    const uint8_t *img = (const uint8_t *)(SLOT_FLASH_ADDR + 32u);
    if(Teach_Crc32(img, ARM_SLOT_NUM * SLOT_REC_BYTES) != hdr[2])
    {
        g_slot_save_err = 4;
        return;
    }
    for(uint8_t s = 0; s < ARM_SLOT_NUM; s++)
    {
        const uint8_t *r = img + s * SLOT_REC_BYTES;
        memcpy(arm_slot[s].take,  r,      sizeof(arm_slot[s].take));
        memcpy(arm_slot[s].place, r + 24u, sizeof(arm_slot[s].place));
        arm_slot[s].take_valid  = r[48u];
        arm_slot[s].place_valid = r[49u];
        g_slot_occupied[s]      = r[50u];
        /* CRC 只保证字节没花，NaN/越界的点位照样不能用 */
        if(!Arm_SlotPoseOk(arm_slot[s].take))  arm_slot[s].take_valid = 0u;
        if(!Arm_SlotPoseOk(arm_slot[s].place)) arm_slot[s].place_valid = 0u;
    }
    g_slot_base = (hdr[3] == 3u) ? 3u : 1u;
    if(hdr[4] <= ARM_SLOT_NUM + 1u) g_slot_place_ptr = (int8_t)hdr[4] - 1;
    if(hdr[5] <= ARM_SLOT_NUM + 1u) g_slot_take_ptr  = (int8_t)hdr[5] - 1;
    if(hdr[6] <= ARM_SLOT_NUM + 1u) g_slot_carry     = (int8_t)hdr[6] - 1;
    g_slot_saved = 1u;
}

/* ===== 电机总线扫描（诊断用，Watch 控制）=====
 * 某台电机不回帧时，确认它在哪条总线、哪个 ID、哪种模式下才应答。
 * 用法：遥控器打中档，设 g_scan_bus(1/2/3) 后把 g_scan_req 置 1；
 *       每 2ms 发一帧，4 基址 × ID 1~15 = 60 帧，约 120ms 扫完。
 * 读结果：can_rx_id_raw[] / can_rx_bus_cnt[] / can_rx_last_id[] 有无新增。
 * 必须打中档：扫描会对 ID 1~15 发使能帧，下档时会把关节意外上电。 */
volatile uint8_t  g_scan_req  = 0;
volatile uint8_t  g_scan_bus  = 1;
volatile uint32_t g_scan_base = 0;
volatile uint8_t  g_scan_id   = 0;

static void Arm_BusScanTick(void)
{
    static const uint32_t bases[4] = {0x000u, 0x100u, 0x200u, 0x300u};
    static uint8_t bi = 0, id = 1, started_bus = 0;

    /* 退出扫描时必须把 2/3 路停回去：悬空收发器开着会往共用的 RX 回调里
       灌噪声帧。记录"实际开起来的是哪一路"，这样扫描中途改 g_scan_bus
       也不会把错误的一路留在运行状态。 */
    if(!g_scan_req || arm.state != ARM_STATE_LOCKED)
    {
        if(started_bus) { BSP_FDCAN_StopAux(started_bus); started_bus = 0; }
        if(!g_scan_req) { bi = 0; id = 1; }
        return;
    }

    FDCAN_HandleTypeDef *h = (g_scan_bus == 2u) ? &hfdcan2
                           : (g_scan_bus == 3u) ? &hfdcan3 : &hfdcan1;
    uint8_t want_aux = (g_scan_bus == 2u || g_scan_bus == 3u) ? g_scan_bus : 0u;
    if(started_bus != want_aux)
    {
        if(started_bus) BSP_FDCAN_StopAux(started_bus);
        if(want_aux)    BSP_FDCAN_StartAux(want_aux);
        started_bus = want_aux;
    }

    g_scan_base = bases[bi];
    g_scan_id   = id;
    DM_Motor_CmdAt(h, bases[bi], id, DM_CMD_ENABLE);

    if(++id > 15u)
    {
        id = 1u;
        if(++bi >= 4u)
        {
            bi = 0u;
            if(started_bus) { BSP_FDCAN_StopAux(started_bus); started_bus = 0; }
            g_scan_req = 0u;
        }
    }
}

/* Watch 参数体检：非有限或越界就回退默认值（调试器能写进 NaN） */
static float Arm_SaneF(volatile float *p, float lo, float hi, float dflt)
{
    float v = *p;
    if(!(v == v) || v < lo || v > hi)
    {
        *p = dflt;
        return dflt;
    }
    return v;
}

void Arm_ControlLoop(void)
{
    g_arm_loop_tick = HAL_GetTick();

    /* Watch 误写防护：NaN/越界一律回退默认，别让非法值进 PVT 帧 */
    Arm_SaneF(&g_grip_open,   -5.0f, 5.0f, -1.72f);
    Arm_SaneF(&g_grip_close,  -5.0f, 5.0f,  2.10f);
    Arm_SaneF(&g_grip_step,  0.001f, 1.0f,  0.15f);
    Arm_SaneF(&g_grip_cur,    0.0f,  1.0f,  0.9f);
    Arm_SaneF(&g_deploy_speed, 0.05f, 3.0f, 0.5f);
    Arm_SaneF(&g_teach_speed,  0.2f,  2.0f, 1.0f);
    Arm_SaneF(&g_slew_max_rad_s, 0.1f,  6.0f,  3.0f);
    Arm_SaneF(&g_decel_rad_s2,   1.0f, 80.0f, 20.0f);

    static uint32_t t_heal = 0;
    static uint8_t heal_enable_mask = 0;
    static uint32_t heal_enable_due[ARM_MOTOR_NUM] = {0};
    uint32_t now = HAL_GetTick();

    /* 1s jitter window: publish peak-to-peak, then restart the window */
    {
        static uint32_t t_pp = 0;
        if((uint32_t)(now - t_pp) >= 1000u)
        {
            t_pp = now;
            for(int i = 0; i < ARM_JOINT_NUM; i++)
            {
                if(dbg_tgt_min[i] > dbg_tgt_max[i]) { dbg_tgt_pp[i] = 0.0f; dbg_pos_pp[i] = 0.0f; }
                else
                {
                    dbg_tgt_pp[i] = dbg_tgt_max[i] - dbg_tgt_min[i];
                    dbg_pos_pp[i] = dbg_pos_max[i] - dbg_pos_min[i];
                }
                dbg_tgt_min[i] =  1e9f; dbg_tgt_max[i] = -1e9f;
                dbg_pos_min[i] =  1e9f; dbg_pos_max[i] = -1e9f;
            }
        }
    }

    /* Keep the 2ms control task non-blocking. Queue enable 2ms after a clear
       command instead of sleeping inside the control loop.
       失能即取消排队中的使能：否则 DISABLED 分支发完失能帧又被队列重新上电。 */
    if(arm.state == ARM_STATE_DISABLED) heal_enable_mask = 0;
    else for(int i = 0; i < ARM_MOTOR_NUM; i++)
    {
        uint8_t bit = (uint8_t)(1u << i);
        if((heal_enable_mask & bit) && (int32_t)(now - heal_enable_due[i]) >= 0)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(m) Arm_SendCmd(m, i, DM_CMD_ENABLE);
            heal_enable_mask &= (uint8_t)~bit;
        }
    }

    if((uint32_t)(now - t_heal) >= 200u)
    {
        t_heal = now;
        if(arm.state == ARM_STATE_LOCKED || arm.state == ARM_STATE_FOLLOWING)
        {
            for(int i = 0; i < ARM_MOTOR_NUM; i++)
            {
                uint8_t bit = (uint8_t)(1u << i);
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m && !(heal_enable_mask & bit) &&
                   (m->rx_cnt == 0 || m->error != 1))
                {
                    Arm_SendCmd(m, i, DM_CMD_CLEAR_ERR);
                    heal_enable_due[i] = now + 2u;
                    heal_enable_mask |= bit;
                }
            }
        }
    }

    Arm_UpdateState();

    Arm_BusScanTick();

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

    Arm_TeachHandleRequest();

    /* 命令码 2 = 一键取矿序列：先到工作位，到位后自动接着下探取矿位，中间不等第二次按键 */
    static uint8_t deploy_seq_phase = 0;   /* 0=正走向工作位  1=正走向取矿位 */
    static uint8_t deploy_last = 0;
    static uint8_t deploy_code_last = 0;

    /* 保存/删除只在下档做：擦 Flash 全片停顿 1~2s，电机失能时才无害 */
    if(g_teach_save_req != 0u)
    {
        uint8_t req = g_teach_save_req;
        g_teach_save_req = 0;
        if(arm.state != ARM_STATE_DISABLED) g_teach_save_err = 1;
        else if(req == 2u) Teach_FlashDelete();
        else if(req == 1u)
        {
            if(g_teach_count < 2u) g_teach_save_err = 3;
            else Teach_FlashSave();
        }
    }

    /* 矿位点位表同样只在下档保存（Flash 扇区6，独立于轨迹扇区7） */
    if(g_slot_save_req != 0u)
    {
        uint8_t req = g_slot_save_req;
        g_slot_save_req = 0;
        if(arm.state != ARM_STATE_DISABLED) g_slot_save_err = 1;
        else if(req == 2u) Slot_FlashDelete();
        else if(req == 1u) Slot_FlashSave();
    }

    if(arm.state == ARM_STATE_DISABLED)
    {
        /* 失能 = 放弃所有自动占用：录制收尾、回放停止、循环/抽臂序列清零。
           失能期间臂泄力可能被搬动，残留状态带进下次使能会从错误起点续走。 */
        if(g_teach_state == ARM_TEACH_RECORDING) Arm_TeachFinishRecording();
        else if(g_teach_state != ARM_TEACH_EMPTY && g_teach_state != ARM_TEACH_READY)
            Arm_TeachStopReplay();
        g_cycle_state = 0u;
        deploy_last = 0;
        deploy_seq_phase = 0;
        if(g_slot_state != ARM_SLOT_ST_IDLE) Arm_SlotStop(ARM_SLOT_ERR_MODE);
        return;
    }

    /* spread each new master frame over ARM_INTERP_TICKS control ticks */
    if(arm.recv_cnt != arm_last_frame_cnt)
    {
        arm_last_frame_cnt = arm.recv_cnt;
        /* ISR 逐轴写 target_joint：这里取六轴必须原子快照，
           否则半新半旧的位姿会被算进插值斜坡（偶发单轴错位） */
        float tgt[ARM_JOINT_NUM];
        __disable_irq();
        for(int i = 0; i < ARM_JOINT_NUM; i++) tgt[i] = arm.target_joint[i];
        __enable_irq();
        for(int i = 0; i < ARM_JOINT_NUM; i++)
            arm_interp_step[i] = (tgt[i] - arm_interp_target[i]) / (float)ARM_INTERP_TICKS;
        arm_interp_left = ARM_INTERP_TICKS;
    }
    if(arm_interp_left > 0)
    {
        for(int i = 0; i < ARM_JOINT_NUM; i++) arm_interp_target[i] += arm_interp_step[i];
        arm_interp_left--;
    }

    /* 抽臂/取矿生效条件：遥控器上档 + 主手在线 + 命令码 1/2（3 是循环脉冲，另走 Arm_CycleTick） */
    uint8_t in_deploy = ((arm_deploy_cmd == 1u || arm_deploy_cmd == 2u) && arm.connected &&
                         arm.state == ARM_STATE_FOLLOWING) ? 1u : 0u;
    volatile float *deploy_goal = (arm_deploy_cmd == 2u && deploy_seq_phase == 1u)
                                ? g_mine_pos : g_deploy_pos;
    if(in_deploy && (!deploy_last || deploy_code_last != arm_deploy_cmd))
    {
        arm_deploy_hold = 0;      /* cmd_smooth 保持上一拍，从当前位置起步 */
        deploy_seq_phase = 0;     /* 序列从头开始 */
    }
    if(!in_deploy && deploy_last)
    {
        arm_reclutch_req = 1;     /* 交还跟随：以当前位置重新吸合，不跳 */
        deploy_seq_phase = 0;
    }
    deploy_last = in_deploy;
    deploy_code_last = arm_deploy_cmd;

    uint8_t in_cycle_move = ((g_cycle_state == 1u || g_cycle_state == 2u) &&
                             arm.state == ARM_STATE_FOLLOWING && arm.connected) ? 1u : 0u;
    /* 循环驱动的回放/保持不要求主手使能：按键序列期间主手可能不在零力模式 */
    uint8_t in_teach = ((g_teach_state == ARM_TEACH_REPLAY || g_teach_state == ARM_TEACH_DONE || g_teach_state == ARM_TEACH_RETURN) &&
                        arm.state == ARM_STATE_FOLLOWING && arm.connected &&
                        (arm.active || g_cycle_state != 0u) && !in_deploy) ? 1u : 0u;
    /* 矿位流程：in_slot_hold = 流程占着臂（含张爪保持，此时谁都不许动目标）；
       in_slot_move = 正在走向某个点位的行走段。抽臂/循环优先，见下面的分支顺序。 */
    uint8_t in_slot_move = ((g_slot_state == ARM_SLOT_ST_GO_WORK || g_slot_state == ARM_SLOT_ST_GO_PLACE ||
                             g_slot_state == ARM_SLOT_ST_BACK || g_slot_state == ARM_SLOT_ST_GO_TAKE) &&
                            arm.state == ARM_STATE_FOLLOWING && arm.connected) ? 1u : 0u;
    uint8_t in_slot_hold = (g_slot_enable && g_slot_state != ARM_SLOT_ST_IDLE) ? 1u : 0u;

    /* 自动运动期间任一关节反馈丢失 = 整段冻结：目标和游标都不前进，
       免得其余关节继续走把轨迹走歪；反馈恢复后从冻结点继续 */
    uint8_t auto_fb_ok = 1;
    if(in_deploy || in_cycle_move || in_teach || in_slot_move)
    {
        for(int i = 0; i < ARM_JOINT_NUM; i++)
        {
            DM_Motor_t *m = DM_Motor_Get(i);
            if(m == NULL || !Arm_FbFresh(m)) { auto_fb_ok = 0; break; }
        }
        if(!auto_fb_ok) arm_deploy_hold = 0;   /* 冻结期间不允许判到位 */
    }
    if(in_teach && auto_fb_ok) Arm_TeachPrepareReplay();

    for(int i = 0; i < ARM_JOINT_NUM; i++)
    {
        DM_Motor_t *m = DM_Motor_Get(i);
        if(m == NULL || m->hfdcan == NULL) continue;

        uint8_t joint_bit = (uint8_t)(1u << i);
        /* 反馈超时的关节本拍跳过：不算目标、不发 PVT，
           电机自己保持/进通讯丢失保护，比按冻结值或 0 位姿灌电流安全 */
        if(!Arm_FbFresh(m)) continue;
        if(in_deploy && auto_fb_ok)
        {
            /* 走本机记录的电机绝对位置，与主手角度、q_offset 无关 */
            if(m->rx_cnt > 0)
            {
                float goal = Arm_ClampJoint(i, deploy_goal[i]);
                float step = g_deploy_speed * ARM_CTRL_PERIOD_S;
                arm.cmd_smooth[i] = Arm_RateLimit(goal, arm.cmd_smooth[i], step);
                if(fabsf(arm.cmd_smooth[i] - goal) <= 0.001f) arm_deploy_hold |= joint_bit;
                else                                          arm_deploy_hold &= (uint8_t)~joint_bit;
            }
        }
        else if(in_cycle_move && auto_fb_ok && m->rx_cnt > 0)
        {
            /* 循环行走段：走工作位/轨迹首点，速度同抽臂 */
            float goal = Arm_ClampJoint(i, cycle_goal[i]);
            float step = g_deploy_speed * ARM_CTRL_PERIOD_S;
            arm.cmd_smooth[i] = Arm_RateLimit(goal, arm.cmd_smooth[i], step);
        }
        else if(in_teach && auto_fb_ok && m->rx_cnt > 0)
        {
            float goal = Arm_ClampJoint(i, teach_replay_goal[i]);
            arm.cmd_smooth[i] = Arm_RateLimit(goal, arm.cmd_smooth[i], ARM_MAX_STEP);
        }
        else if(in_slot_move && auto_fb_ok && m->rx_cnt > 0)
        {
            /* 矿位行走段：走点位表的电机绝对角，速度同抽臂 */
            float goal = Arm_ClampJoint(i, slot_goal[i]);
            float step = g_deploy_speed * ARM_CTRL_PERIOD_S;
            arm.cmd_smooth[i] = Arm_RateLimit(goal, arm.cmd_smooth[i], step);
        }
        else if(arm.state == ARM_STATE_FOLLOWING && arm.connected && arm.active && m->rx_cnt > 0 &&
           !(arm_q_offset_mask & joint_bit) && !in_slot_hold)
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

        if(!in_deploy && !in_teach && !in_cycle_move && !in_slot_hold && arm.state == ARM_STATE_FOLLOWING && arm.connected && arm.active &&
           (arm_q_offset_mask & joint_bit) && m->rx_cnt > 0)
        {
            if(arm.connected)
            {
                float raw_target = ARM_DIR[i] * arm_interp_target[i] - arm.q_offset[i];
                float clamped    = Arm_ClampJoint(i, raw_target);

                /* 越限：从手停在限位，q_offset 一律不改。
                   旧逻辑把超出量累加进 q_offset（"吃偏移"），主手一过限再回来，
                   主从对应关系就永久变了 —— 这正是"反方向角度就改变"的原因。
                   现在只记录越限量，主手退回到限位内立刻恢复严格同步。 */
                if(clamped != raw_target)
                {
                    arm_limit_mask |= (uint8_t)(1u << i);
                    arm_limit_excursion[i] = raw_target - clamped;
                }
                else
                {
                    arm_limit_mask &= (uint8_t)~(1u << i);
                    arm_limit_excursion[i] = 0.0f;
                }
                float max_step = g_slew_max_rad_s * ARM_CTRL_PERIOD_S;
                float target_error = clamped - arm.cmd_smooth[i];
                if((arm_catchup_mask & joint_bit) && fabsf(target_error) <= 0.05f)
                {
                    arm_catchup_mask &= (uint8_t)~joint_bit;
                }
                if(arm_catchup_mask & joint_bit)
                    max_step = ARM_CATCHUP_SPEED_RAD_S * ARM_CTRL_PERIOD_S;
                if(g_decel_en)
                {
                    /* 剩余距离越近越慢，急停时不再是阶跃减速 */
                    float v_allow = sqrtf(2.0f * g_decel_rad_s2 *
                                          fabsf(clamped - arm.cmd_smooth[i]));
                    float ds;
                    if(v_allow > g_slew_max_rad_s) v_allow = g_slew_max_rad_s;
                    if(v_allow < 0.05f) v_allow = 0.05f;   /* 别把收尾拖成龟速 */
                    ds = v_allow * ARM_CTRL_PERIOD_S;
                    if(ds < max_step) max_step = ds;
                }
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

        /* jitter tracking (500Hz) */
        {
            float trk = fabsf(m->position - arm.cmd_smooth[i]);
            if(trk > g_trk_err_peak[i]) g_trk_err_peak[i] = trk;
        }
        if(arm.target_joint[i] < dbg_tgt_min[i]) dbg_tgt_min[i] = arm.target_joint[i];
        if(arm.target_joint[i] > dbg_tgt_max[i]) dbg_tgt_max[i] = arm.target_joint[i];
        if(m->position < dbg_pos_min[i]) dbg_pos_min[i] = m->position;
        if(m->position > dbg_pos_max[i]) dbg_pos_max[i] = m->position;

        DM_Motor_PVT_Control(m->hfdcan, m->can_id,
                             arm.cmd_smooth[i], vmag, ARM_CUR_LIMIT[i]);

    }

    Arm_TeachTickAfterJoints();
    Arm_CycleTick();
    Arm_SlotTick();

    /* 一键取矿序列：工作位到位后自动接下一段，不停在中间。
       到位标志必须在切段之后算：切段这一拍旧段的"已到位"不能带进新段，
       否则 g_deploy_state 直接跳 4，夹爪在去取矿位的半路上就提前闭合。 */
    if(in_deploy && arm_deploy_cmd == 2u && deploy_seq_phase == 0u &&
       arm_deploy_hold == ((1u << ARM_JOINT_NUM) - 1u))
    {
        deploy_seq_phase = 1u;
        arm_deploy_hold  = 0;   /* 下一拍 deploy_goal 自动切到 g_mine_pos */
    }

    uint8_t deploy_all_hold = (arm_deploy_hold == ((1u << ARM_JOINT_NUM) - 1u)) ? 1u : 0u;

    if(!in_deploy)
        g_deploy_state = 0u;
    else if(arm_deploy_cmd == 1u)
        g_deploy_state = deploy_all_hold ? 2u : 1u;
    else
        g_deploy_state = (deploy_seq_phase == 1u) ? (deploy_all_hold ? 4u : 3u) : 1u;

#if ARM_USE_GRIPPER
    /* ===== 夹爪：第 7 个电机（索引 ARM_JOINT_NUM）=====
     * 控制帧必须无条件下发 —— 达妙电机使能后收不到控制帧会判通讯丢失、自行
     * 退出使能（表现为"绿一下又变红"）。拿到首帧反馈前电流限幅发 0，
     * 既保活又不产生力矩，目标停在 0 也不会猛甩。
     * 中档保持位置；上档按主手按键开合（0=张开 1=闭合）。 */
    {
        static uint32_t t_grip_tx = 0;
        DM_Motor_t *gm = DM_Motor_Get(ARM_JOINT_NUM);

        /* 清错+使能由上面 200ms 统一巡检队列负责（覆盖全部 7 台、判据 error!=1、
           不阻塞控制环），这里不再单独补，避免对正常使能中的夹爪反复清错。 */

        if(gm != NULL && gm->hfdcan != NULL &&
           (uint32_t)(now - t_grip_tx) >= GRIP_TX_PERIOD_MS)
        {
            t_grip_tx = now;

            float cur = 0.0f;

            if(gm->rx_cnt > 0)
            {
                if(!arm_grip_synced)
                {
                    arm.target_gripper = gm->position;
                    arm_grip_synced = 1;
                }
                cur = g_grip_cur;

                /* 只有上档且主手在线使能时才推动目标；中档保持上一次的目标不动。
                   循环行走段(去工作位/去轨迹首点)冻结目标：可能正夹着矿，强制张开会掉矿。 */
                uint8_t slot_grip_freeze = (g_slot_enable && (g_slot_state == ARM_SLOT_ST_GO_WORK ||
                                             g_slot_state == ARM_SLOT_ST_GO_PLACE ||
                                             g_slot_state == ARM_SLOT_ST_BACK)) ? 1u : 0u;
                uint8_t slot_grip_open   = (g_slot_enable && g_slot_state == ARM_SLOT_ST_OPEN) ? 1u : 0u;

                if(arm.state == ARM_STATE_FOLLOWING && arm.connected && arm.active &&
                   !in_cycle_move && !slot_grip_freeze)
                {
                    /* 一键取矿的整个序列期间，夹爪一律张到最大(g_grip_open)：
                       主手启动时就发了 grip=1，中途合会在下探路上把矿推开。
                       只有走到取矿位(state=4)这一拍才放行，实现"最后一瞬间才夹" */
                    uint8_t grip_req = arm.gripper_cmd_recv;
                    if(arm_deploy_cmd == 2u && g_deploy_state != 4u) grip_req = 0u;
                    if(in_teach)
                        grip_req = (g_teach_state == ARM_TEACH_RETURN) ? 0u : teach_replay_grip;
                    /* 矿位流程：只有停在放矿位这一拍才允许张开，路上绝不松爪 */
                    if(slot_grip_open) grip_req = 0u;

                    float want = grip_req ? g_grip_close : g_grip_open;
                    float err  = want - arm.target_gripper;

                    if(fabsf(err) > 1e-4f)
                    {
                        float step = (err > 0.0f) ? g_grip_step : -g_grip_step;
                        if(fabsf(err) < g_grip_step) step = err;
                        arm.target_gripper += step;
                    }
                }

                /* 软限位：目标不出 g_grip_open / g_grip_close 划出的区间（谁大谁小都行，
                   方向反了对调两个值即可）。 */
                float lo = (g_grip_open < g_grip_close) ? g_grip_open  : g_grip_close;
                float hi = (g_grip_open < g_grip_close) ? g_grip_close : g_grip_open;
                if(arm.target_gripper < lo) arm.target_gripper = lo;
                if(arm.target_gripper > hi) arm.target_gripper = hi;

                /* 限幅跟踪误差 = 限幅夹持力，同时防止堵转满电流死顶 */
                float lag = arm.target_gripper - gm->position;
                if(lag >  GRIP_MAX_LAG) arm.target_gripper = gm->position + GRIP_MAX_LAG;
                if(lag < -GRIP_MAX_LAG) arm.target_gripper = gm->position - GRIP_MAX_LAG;
            }

            float gvel = g_grip_step / GRIP_TX_PERIOD_S;
            if(gvel > ARM_VEL_MAX_RAD_S) gvel = ARM_VEL_MAX_RAD_S;
            if(gvel < ARM_VEL_MIN_RAD_S) gvel = ARM_VEL_MIN_RAD_S;

            DM_Motor_PVT_Control(gm->hfdcan, gm->can_id,
                                 arm.target_gripper, gvel, cur);
        }
    }
#endif
}

Arm_t* Arm_Get(void)
{
    return &arm;
}
