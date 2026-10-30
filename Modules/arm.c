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
    1.0f, -1.0f, -1.0f, -1.0f, 1.0f, -1.0f
};

static const float ARM_Q_MIN[ARM_JOINT_NUM] = { -3.10f, -2.80f,  -2.40f, -10.0f, -10.0f, -10.0f};
static const float ARM_Q_MAX[ARM_JOINT_NUM] = {  0.05f,  0.05f,  0.23f,  10.0f,  10.0f,  10.0f};

/* 电流限值。注意单位：PVT 帧里这一项是**额定电流的标幺值**（DM_Motor_PVT_Control
   内部 ×10000 后装进 u16，协议有效范围 0~10000），不是安培。
   0.7 = 各自额定电流的 70%，不是 0.7A。 */
static const float ARM_CUR_LIMIT[ARM_JOINT_NUM] = {
    0.7f,  0.7f,  0.7f,  0.7f,  0.7f,  0.7f
};

static float Arm_ClampJoint(int i, float v)
{
    if(v < ARM_Q_MIN[i]) return ARM_Q_MIN[i];
    if(v > ARM_Q_MAX[i]) return ARM_Q_MAX[i];
    return v;
}

/* 夹爪开/合目标位置（rad）。
   2026-10-30 串口 dump 读到电机当前位置 Output Position = 1.7238，
   所以默认值放在它附近，避免第一次下发时从 0 猛甩过去。
   方向未知：先小步试，看夹爪往哪边走，反了就把两个值对调。
   两个值定义的区间同时也是目标的软限位，Watch 里填错也只会停在区间边界。 */
volatile float g_grip_open  = -1.72f;
volatile float g_grip_close = 2.10f;

/* 夹爪专用的下发周期。开合是二值动作，不需要跟关节一样 500Hz ——
   第 7 台电机进 2ms 环会把 CAN1 负载从约 72% 推到约 84%（达妙一发一收，
   回帧省不掉）。10ms 一帧把负载压回约 74%。 */
#define GRIP_TX_PERIOD_MS   10u
#define GRIP_TX_PERIOD_S    0.010f

/* 每个下发周期的最大步进(rad)。0.01 rad / 10ms = 1 rad/s。 */
volatile float g_grip_step  = 0.01f;

/* 夹爪电流限值，单位同 ARM_CUR_LIMIT —— **额定电流的标幺值**，不是安培。
   4310 额定 2.5A，所以 0.9 ≈ 2.25A，1.0 就已经是满额定。
   协议上限是 1.0（内部 ×10000 后不能超 10000），夹不紧往上调但别超 1.0。 */
volatile float g_grip_cur = 0.9f;

/* 目标相对实测位置的最大超前量(rad)。夹住东西后目标走不动，误差会一路顶到
   电流限幅并长时间堵转（4310 升温很快）。把误差钳住就等于把夹持力钳住：
   既保住夹持力，又不会满电流死顶。顺带兜住 Watch 里把 g_grip_close 填成
   离谱数值的情况 —— 目标永远跑不出实测位置 ±这个值。 */
#define GRIP_MAX_LAG        0.15f

/* 第一次拿到夹爪反馈时把目标对齐到实际位置，否则 target_gripper 从 0 起算，
   上电后第一帧就会命令它猛甩一大段。失能时清零，下次使能重新对齐。 */
static uint8_t arm_grip_synced = 0;

/* 夹爪命令帧的首选控制模式基址。6 个关节已确认是"力位混合"(0x300)，
   夹爪是后装的电机，串口 dump 读到 Control Mode: 4（力位混合），所以 0x300
   应该是对的。但使能/清错/失能帧仍会把四种基址都发一遍（见 Arm_SendCmd），
   这里只决定先发哪一个 —— 万一 dump 读错了也不至于全盘落空。
   想单独试某一种模式，在 Watch 里把这个值改成 0x000/0x100/0x200/0x300 即可。 */
volatile uint32_t g_grip_id_base = 0x300u;

/* 发一条命令帧到第 i 台电机。
 *
 * 前 6 个关节一直用 0x300+id，工作正常。
 * 第 7 台（夹爪）的控制模式虽然从串口 dump 里读到 Control Mode: 4，
 * 但为了不再靠猜，使能/清错/失能这类命令帧对夹爪把四种模式的基址各发一遍
 * （0x000=MIT 0x100=位置速度 0x200=速度 0x300=力位混合）。
 * 这些是 0xFB/0xFC/0xFD 特殊命令帧，不含位置数据，所以命中或落空都不会让
 * 电机乱动；哪个基址生效看 dm_motors[6].rx_cnt 涨不涨就知道了。
 * 控制帧（PVT）仍固定走 0x300。 */
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

static void Arm_DecodeFrame(uint8_t active, uint8_t grip, uint8_t seq)
{
    uint32_t now = HAL_GetTick();
    uint8_t long_gap = 0;
    uint8_t reacquire = (active && !arm.active);

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
    arm.active = active;
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

    if(arm.state != arm.last_state)
    {
        if(arm.last_state == ARM_STATE_DISABLED && arm.state != ARM_STATE_DISABLED)
        {
            for(int i = 0; i < ARM_MOTOR_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) { Arm_SendCmd(m, i, DM_CMD_CLEAR_ERR); osDelay(1); }
            }
            osDelay(2);
            for(int i = 0; i < ARM_MOTOR_NUM; i++)
            {
                DM_Motor_t *m = DM_Motor_Get(i);
                if(m) {
                    Arm_SendCmd(m, i, DM_CMD_ENABLE);
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

static float Arm_RateLimit(float target, float current, float max_step)
{
    float err = target - current;
    if(err > max_step) return current + max_step;
    if(err < -max_step) return current - max_step;
    return target;
}

/* ===== 电机总线扫描（纯诊断，Watch 控制）==============================
 * 用途：某个电机完全不回帧时，确认它到底在哪条总线、哪个 CAN ID、
 *       哪种控制模式下才肯应答。
 * 用法：遥控器拨到中档（LOCKED），在 Watch 里设
 *         g_scan_bus = 1/2/3      要扫哪一路 FDCAN
 *         g_scan_req = 1          开始
 *       每个 2ms 周期只发一帧，4 种基址 x CAN ID 1~15 共 60 帧，约 120ms 扫完，
 *       结束后 g_scan_req 自动回 0（扫 2/3 路时会顺便把该路停回去）。
 * 读结果：扫 1 路看 can_rx_id_raw[] 哪个下标新增；
 *         扫 2/3 路看 can_rx_bus_cnt[bus] 是否增长、can_rx_last_id[bus] 是多少。
 *       三路都没有新 ID 出现 -> 电机不在任何一条 CAN 上（没接/没供电/波特率不符）。
 * 为什么限定中档：扫描会对 ID 1~15 发使能帧，中档下 6 个关节本来就是使能并
 *       锁位的，重复使能不改变任何状态；下档失能时扫会把关节意外上电。 */
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

void Arm_ControlLoop(void)
{
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
       command instead of sleeping inside the control loop. */
    for(int i = 0; i < ARM_MOTOR_NUM; i++)
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

        /* jitter tracking (500Hz) */
        if(arm.target_joint[i] < dbg_tgt_min[i]) dbg_tgt_min[i] = arm.target_joint[i];
        if(arm.target_joint[i] > dbg_tgt_max[i]) dbg_tgt_max[i] = arm.target_joint[i];
        if(m->position < dbg_pos_min[i]) dbg_pos_min[i] = m->position;
        if(m->position > dbg_pos_max[i]) dbg_pos_max[i] = m->position;

        DM_Motor_PVT_Control(m->hfdcan, m->can_id,
                             arm.cmd_smooth[i], vmag, ARM_CUR_LIMIT[i]);

    }

#if ARM_USE_GRIPPER
    /* ===== 夹爪：第 7 个电机（索引 ARM_JOINT_NUM = 6）=====
     * 使能/清错走上面 200ms 的自愈巡检，条件和 6 个关节一致。
     *
     * 关键：控制帧必须像关节一样**无条件下发**，不能等 rx_cnt > 0。
     * 达妙电机使能后长时间收不到控制帧会判通讯丢失自己退出使能，于是
     * "清错→使能→绿一下→掉回红→200ms 后重来"，表现就是中档上档都没使能。
     * 拿到第一帧反馈之前电流限值发 0：有保活、会回帧、没有力矩，
     * 所以也不会因为目标还停在 0 而猛甩。
     *
     * 中档(LOCKED)：保持当前位置，和关节一样有保持力。
     * 上档(FOLLOWING)：按主手按键开合，0 = 张开，1 = 闭合。 */
    {
        static uint32_t t_grip_tx = 0;
        DM_Motor_t *gm = DM_Motor_Get(ARM_JOINT_NUM);

        /* 夹爪清错重试（2026-10-30）。
           达妙电机有错误位锁存：error != 0 时怎么发使能都不生效（LED 红灯常亮）。
           原来的清错只在 "DISABLED -> 其它" 那一次状态切换时发，一旦电机在上电
           顺序里错过那一帧（或先报错再使能），就再也清不掉了。
           这里改成：只要 error 还不是 0，就每 500ms 补一次 清错 + 使能。
           顺带说明：dm_motors[].enabled 这个字段反馈报文里没有，ParseFeedback
           从不写它，所以它恒为 0 —— 判断有没有真使能要看电机 LED。 */
        if(gm != NULL && gm->hfdcan != NULL && gm->error != 0)
        {
            static uint32_t grip_clr_tick = 0;
            if((HAL_GetTick() - grip_clr_tick) > 500u)
            {
                grip_clr_tick = HAL_GetTick();
                Arm_SendCmd(gm, ARM_JOINT_NUM, DM_CMD_CLEAR_ERR);
                osDelay(1);
                Arm_SendCmd(gm, ARM_JOINT_NUM, DM_CMD_ENABLE);
            }
        }

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

                /* 只有上档且主手在线使能时才推动目标；中档保持上一次的目标不动。 */
                if(arm.state == ARM_STATE_FOLLOWING && arm.connected && arm.active)
                {
                    float want = arm.gripper_cmd_recv ? g_grip_close : g_grip_open;
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
