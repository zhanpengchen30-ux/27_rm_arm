#include "ore_job.h"
#include <string.h>

/* ============================================================================
 * 一条臂的作业拓扑（工作位是唯一交接点）：
 *
 *            矿位 A / B
 *                ↑ ↓
 *   展开 ←→  工作位  ←→  收纳位
 *
 *   每臂 6 段示教轨迹：展开、收纳、去 A、回、去 B、回。
 *   存与取共用"去/回"两段，只有夹爪时序相反：
 *       存：到矿位 → 张开（放） → 回工作位
 *       取：到矿位 → 闭合（夹） → 回工作位
 *
 *   顺序序列（可在 g_order 改）：
 *       1 号臂：矿位 2 → 1      2 号臂：矿位 3 → 4
 *   即 arm0 的 order = {1,0}（先操作下标 1，再下标 0），arm1 = {0,1}。
 * ==========================================================================*/

#define ORE_T_SEG_MS      12000u   /* 单段轨迹超时 */
#define ORE_T_GRIP_MS      2500u   /* 夹爪动作超时 */
#define ORE_T_START_MS      200u   /* 起步保护：播放后先等 200ms 再判结束 */

#define ORE_OP_NONE   0u
#define ORE_OP_STORE  1u
#define ORE_OP_PICK   2u

typedef struct
{
    uint8_t slot[ORE_SLOT_PER_ARM];
    uint8_t last_idx;
    uint8_t holding;
    uint8_t state;
    uint8_t fault;
    uint8_t target;
    uint8_t at_work;
    uint8_t op;
} ore_ctx_t;

static const OreJob_Hw_t *g_hw;
static ore_ctx_t          g_ctx[ORE_ARM_NUM];
static OreArmStatus_t     g_pub[ORE_ARM_NUM];

static uint8_t  g_sel;
static uint8_t  g_key_last;
static uint8_t  g_seg_run[ORE_ARM_NUM];
static uint8_t  g_grip_sent[ORE_ARM_NUM];
static uint32_t g_t_seg[ORE_ARM_NUM];
static uint32_t g_t_grip[ORE_ARM_NUM];
static uint32_t g_fake_ms;

/* 存矿优先顺序：[0] 是第一个存的槽。1 号臂先 2 后 1，2 号臂先 3 后 4。 */
static const uint8_t g_order[ORE_ARM_NUM][ORE_SLOT_PER_ARM] =
{
    { 1u, 0u },
    { 0u, 1u }
};

/* -------------------------------------------------------------------------- */

static uint32_t now_ms(void)
{
    if(g_hw != 0 && g_hw->Now != 0) return g_hw->Now();
    return g_fake_ms;
}

static void publish(uint8_t a)
{
    ore_ctx_t      *c = &g_ctx[a];
    OreArmStatus_t *p = &g_pub[a];

    for(uint8_t i = 0; i < ORE_SLOT_PER_ARM; i++) p->slot[i] = c->slot[i];
    p->last_idx = c->last_idx;
    p->holding  = c->holding;
    p->state    = c->state;
    p->fault    = c->fault;
    p->target   = c->target;
    p->at_work  = c->at_work;
}

static void fail(uint8_t a, uint8_t err)
{
    g_ctx[a].state = ORE_JOB_FAULT;
    g_ctx[a].fault = err;
    g_ctx[a].op    = ORE_OP_NONE;
    g_seg_run[a]   = 0u;
    g_grip_sent[a] = 0u;
    if(g_hw != 0 && g_hw->StopSegment != 0) g_hw->StopSegment(a);
}

static uint8_t arm_busy(uint8_t a)
{
    uint8_t s = g_ctx[a].state;
    return (uint8_t)((s != ORE_JOB_IDLE && s != ORE_JOB_FAULT) ? 1u : 0u);
}

static uint8_t play(uint8_t a, uint8_t seg)
{
    if(g_hw == 0 || g_hw->PlaySegment == 0) return 0u;
    if(g_hw->PlaySegment(a, seg, ORE_PLAY_SPEED) == 0u) return 0u;

    g_seg_run[a] = 1u;
    g_t_seg[a]   = now_ms();
    return 1u;
}

/* 1 = 本段结束（含超时置故障后返回 0） */
static uint8_t seg_done(uint8_t a)
{
    uint32_t t = now_ms();

    if((uint32_t)(t - g_t_seg[a]) < ORE_T_START_MS) return 0u;
    if(g_hw == 0 || g_hw->SegmentBusy == 0)         return 1u;

    if(g_hw->SegmentBusy(a) != 0u)
    {
        if((uint32_t)(t - g_t_seg[a]) > ORE_T_SEG_MS) fail(a, ORE_ERR_SEG_TIMEOUT);
        return 0u;
    }
    return 1u;
}

/* 0/1 = 传感器读数；0xFF = 没有传感器 */
static uint8_t slot_sensor(uint8_t a, uint8_t idx)
{
    if(g_hw == 0 || g_hw->SlotSensor == 0) return 0xFFu;
    return g_hw->SlotSensor(a, idx);
}

/* 找下一个要存的位置：返回 0/1；-1 = 全满；-2 = 有未知槽需先确认 */
static int8_t find_empty(uint8_t a)
{
    const ore_ctx_t *c = &g_ctx[a];
    uint8_t unknown = 0u;

    for(uint8_t k = 0; k < ORE_SLOT_PER_ARM; k++)
    {
        uint8_t idx = g_order[a][k];
        if(c->slot[idx] == ORE_SLOT_EMPTY)   return (int8_t)idx;
        if(c->slot[idx] == ORE_SLOT_UNKNOWN) unknown = 1u;
    }
    return (unknown != 0u) ? (int8_t)-2 : (int8_t)-1;
}

/* 找下一个要取的位置：返回 0/1；-1 = 全空；-2 = 有未知槽 */
static int8_t find_take(uint8_t a)
{
    const ore_ctx_t *c = &g_ctx[a];
    uint8_t unknown = 0u;

    for(uint8_t k = 0; k < ORE_SLOT_PER_ARM; k++)
    {
#if ORE_TAKE_POLICY_FIFO
        uint8_t idx = g_order[a][k];
#else
        uint8_t idx = g_order[a][(uint8_t)(ORE_SLOT_PER_ARM - 1u - k)];
#endif
        if(c->slot[idx] == ORE_SLOT_FULL)    return (int8_t)idx;
        if(c->slot[idx] == ORE_SLOT_UNKNOWN) unknown = 1u;
    }
    return (unknown != 0u) ? (int8_t)-2 : (int8_t)-1;
}

static uint8_t next_full(uint8_t a)
{
    for(uint8_t k = 0; k < ORE_SLOT_PER_ARM; k++)
    {
        uint8_t idx = g_order[a][k];
        if(g_ctx[a].slot[idx] == ORE_SLOT_FULL) return idx;
    }
    return 0xFFu;
}

/* -------------------------------------------------------------------------- */

static void start_move(uint8_t a, uint8_t op)
{
    ore_ctx_t *c = &g_ctx[a];
    int8_t idx;

    if(arm_busy(a) != 0u) { c->fault = ORE_ERR_BUSY; return; }

    if(op == ORE_OP_STORE)
    {
        if(c->holding == 0u) { c->fault = ORE_ERR_NO_GRIP; return; }
        idx = find_empty(a);
        if(idx == (int8_t)-1) { c->fault = ORE_ERR_SLOT_FULL; return; }
    }
    else
    {
        if(c->holding != 0u) { c->fault = ORE_ERR_NO_GRIP; return; }
        idx = find_take(a);
        if(idx == (int8_t)-1) { c->fault = ORE_ERR_SLOT_EMPTY; return; }
    }

    if(idx == (int8_t)-2) { c->fault = ORE_ERR_UNKNOWN_SLOT; return; }

    c->target      = (uint8_t)idx;
    c->op          = op;
    c->fault       = ORE_OK;
    c->state       = (c->at_work != 0u) ? (uint8_t)ORE_JOB_TO_SLOT
                                        : (uint8_t)ORE_JOB_GO_WORK;
    g_seg_run[a]   = 0u;
    g_grip_sent[a] = 0u;
}

static void start_deploy(uint8_t a)
{
    ore_ctx_t *c = &g_ctx[a];

    if(arm_busy(a) != 0u) { c->fault = ORE_ERR_BUSY; return; }
    if(c->holding != 0u)  { c->fault = ORE_ERR_NO_GRIP; return; }  /* 持矿禁止展开 */

    c->op         = ORE_OP_NONE;
    c->fault      = ORE_OK;
    c->state      = ORE_JOB_GO_WORK;
    g_seg_run[a]  = 0u;
}

static void start_stow(uint8_t a)
{
    ore_ctx_t *c = &g_ctx[a];

    if(arm_busy(a) != 0u) { c->fault = ORE_ERR_BUSY; return; }
    if(c->holding != 0u)  { c->fault = ORE_ERR_NO_GRIP; return; }  /* 空爪才能收纳 */

    c->op         = ORE_OP_NONE;
    c->fault      = ORE_OK;
    c->state      = ORE_JOB_STOWING;
    g_seg_run[a]  = 0u;
}

/* -------------------------------------------------------------------------- */

static void job_tick(uint8_t a)
{
    ore_ctx_t *c = &g_ctx[a];

    switch(c->state)
    {
    case ORE_JOB_GO_WORK:
        if(g_seg_run[a] == 0u && play(a, (uint8_t)ORE_SEG_DEPLOY) == 0u)
        {
            fail(a, ORE_ERR_NO_TRAJ);
            return;
        }
        if(seg_done(a) == 0u) return;

        g_seg_run[a] = 0u;
        c->at_work   = 1u;
        c->state     = (c->op != ORE_OP_NONE) ? (uint8_t)ORE_JOB_TO_SLOT
                                              : (uint8_t)ORE_JOB_IDLE;
        break;

    case ORE_JOB_TO_SLOT:
    {
        uint8_t seg = (c->target == 0u) ? (uint8_t)ORE_SEG_TO_SLOT0
                                        : (uint8_t)ORE_SEG_TO_SLOT1;

        if(g_seg_run[a] == 0u && play(a, seg) == 0u)
        {
            fail(a, ORE_ERR_NO_TRAJ);
            return;
        }
        if(seg_done(a) == 0u) return;

        g_seg_run[a]   = 0u;
        g_grip_sent[a] = 0u;
        g_t_grip[a]    = now_ms();
        c->state       = ORE_JOB_GRIP;
        break;
    }

    case ORE_JOB_GRIP:
    {
        uint8_t settled = 1u;
        uint8_t grabbed = 1u;
        uint8_t s;

        if(g_grip_sent[a] == 0u)
        {
            if(g_hw != 0 && g_hw->GripCmd != 0)
                g_hw->GripCmd(a, (uint8_t)((c->op == ORE_OP_PICK) ? 1u : 0u));
            g_grip_sent[a] = 1u;
            g_t_grip[a]    = now_ms();
            return;
        }
        if(g_hw != 0 && g_hw->GripSettled != 0)
            settled = g_hw->GripSettled(a, &grabbed);

        if(settled == 0u)
        {
            if((uint32_t)(now_ms() - g_t_grip[a]) > ORE_T_GRIP_MS)
                fail(a, ORE_ERR_GRIP_TIMEOUT);
            return;
        }

        if(c->op == ORE_OP_STORE)
        {
            s = slot_sensor(a, c->target);
            if(s == 0u)                     /* 传感器说没放进去 */
            {
                c->slot[c->target] = (uint8_t)ORE_SLOT_UNKNOWN;
                c->holding         = 0u;
                fail(a, ORE_ERR_NO_GRIP);
                return;
            }
            c->slot[c->target] = (uint8_t)ORE_SLOT_FULL;
            c->last_idx        = c->target;
            c->holding         = 0u;
        }
        else
        {
            if(grabbed == 0u)               /* 没夹到东西 */
            {
                c->slot[c->target] = (uint8_t)ORE_SLOT_UNKNOWN;
                c->holding         = 0u;
                fail(a, ORE_ERR_NO_GRIP);
                return;
            }
            s = slot_sensor(a, c->target);
            if(s == 1u)                     /* 传感器说矿还在槽里 */
            {
                c->slot[c->target] = (uint8_t)ORE_SLOT_UNKNOWN;
                fail(a, ORE_ERR_NO_GRIP);
                return;
            }
            c->slot[c->target] = (uint8_t)ORE_SLOT_EMPTY;
            c->holding         = 1u;
            c->last_idx        = next_full(a);
        }

        g_seg_run[a] = 0u;
        c->state     = ORE_JOB_BACK;
        break;
    }

    case ORE_JOB_BACK:
    {
        uint8_t seg = (c->target == 0u) ? (uint8_t)ORE_SEG_FROM_SLOT0
                                        : (uint8_t)ORE_SEG_FROM_SLOT1;

        if(g_seg_run[a] == 0u && play(a, seg) == 0u)
        {
            fail(a, ORE_ERR_NO_TRAJ);
            return;
        }
        if(seg_done(a) == 0u) return;

        g_seg_run[a] = 0u;
        c->at_work   = 1u;
        c->op        = ORE_OP_NONE;
        c->state     = ORE_JOB_IDLE;
        break;
    }

    case ORE_JOB_STOWING:
        if(g_seg_run[a] == 0u && play(a, (uint8_t)ORE_SEG_STOW) == 0u)
        {
            fail(a, ORE_ERR_NO_TRAJ);
            return;
        }
        if(seg_done(a) == 0u) return;

        g_seg_run[a] = 0u;
        c->at_work   = 0u;
        c->op        = ORE_OP_NONE;
        c->state     = ORE_JOB_IDLE;
        break;

    default:
        break;
    }
}

/* -------------------------------------------------------------------------- */

void OreJob_Init(const OreJob_Hw_t *hw)
{
    memset(g_ctx,      0, sizeof(g_ctx));
    memset(g_pub,      0, sizeof(g_pub));
    memset(g_seg_run,  0, sizeof(g_seg_run));
    memset(g_grip_sent,0, sizeof(g_grip_sent));
    memset(g_t_seg,    0, sizeof(g_t_seg));
    memset(g_t_grip,   0, sizeof(g_t_grip));

    g_hw       = hw;
    g_sel      = 0u;
    g_key_last = 0u;
    g_fake_ms  = 0u;

    for(uint8_t a = 0; a < ORE_ARM_NUM; a++)
    {
        g_ctx[a].slot[0]  = (uint8_t)ORE_SLOT_UNKNOWN;
        g_ctx[a].slot[1]  = (uint8_t)ORE_SLOT_UNKNOWN;
        g_ctx[a].last_idx = 0xFFu;
        g_ctx[a].state    = ORE_JOB_IDLE;
        g_ctx[a].fault    = ORE_OK;
        g_ctx[a].at_work  = 0u;
        publish(a);
    }
}

void OreJob_Tick(void)
{
    g_fake_ms += 2u;              /* 无 Now 回调时按 2ms 控制环估算 */

    for(uint8_t a = 0; a < ORE_ARM_NUM; a++)
    {
        job_tick(a);
        publish(a);
    }
}

void OreJob_SetKeyMap(uint8_t key_map)
{
    uint8_t rise = (uint8_t)(key_map & (uint8_t)(~g_key_last));

    g_key_last = key_map;

    /* 停止键最高优先级，且不受左右选择影响：两条臂一起停 */
    if((rise & ORE_KEY_STOP) != 0u)
    {
        OreJob_Stop();
        return;
    }

    /* 选臂只切目标，不启动动作；被选臂或原臂正在运行时拒绝切换 */
    if((rise & ORE_KEY_SEL) != 0u)
    {
        uint8_t want = ((key_map & ORE_KEY_SEL) != 0u) ? 1u : 0u;
        if(arm_busy(want) == 0u && arm_busy(g_sel) == 0u) g_sel = want;
        return;
    }

    if((rise & ORE_KEY_STORE)  != 0u) { start_move(g_sel, ORE_OP_STORE); return; }
    if((rise & ORE_KEY_PICK)   != 0u) { start_move(g_sel, ORE_OP_PICK);  return; }
    if((rise & ORE_KEY_DEPLOY) != 0u) { start_deploy(g_sel);             return; }
    if((rise & ORE_KEY_STOW)   != 0u) { start_stow(g_sel);               return; }
}

void OreJob_Stop(void)
{
    for(uint8_t a = 0; a < ORE_ARM_NUM; a++)
    {
        ore_ctx_t *c = &g_ctx[a];

        if(arm_busy(a) == 0u) continue;

        if(g_hw != 0 && g_hw->StopSegment != 0) g_hw->StopSegment(a);

        /* 停在半路：涉及的那个槽状态不可信，宁可标未知让人核对 */
        if(c->state == ORE_JOB_TO_SLOT ||
           c->state == ORE_JOB_GRIP    ||
           c->state == ORE_JOB_BACK)
        {
            c->slot[c->target] = (uint8_t)ORE_SLOT_UNKNOWN;
        }

        c->state       = ORE_JOB_IDLE;
        c->op          = ORE_OP_NONE;
        c->fault       = ORE_ERR_STOPPED;
        c->at_work     = 0u;      /* 停在半路，下次动作先回工作位 */
        g_seg_run[a]   = 0u;
        g_grip_sent[a] = 0u;
        publish(a);
    }
}

void OreJob_ConfirmSlot(uint8_t arm, uint8_t idx, OreSlotState_t s)
{
    if(arm >= ORE_ARM_NUM || idx >= ORE_SLOT_PER_ARM) return;
    if(s == ORE_SLOT_UNKNOWN) return;

    g_ctx[arm].slot[idx] = (uint8_t)s;

    if(s == ORE_SLOT_FULL)
    {
        g_ctx[arm].last_idx = idx;
    }
    else if(g_ctx[arm].last_idx == idx)
    {
        g_ctx[arm].last_idx = next_full(arm);
    }
    g_ctx[arm].fault = ORE_OK;
    publish(arm);
}

void OreJob_SetHolding(uint8_t arm, uint8_t holding)
{
    if(arm >= ORE_ARM_NUM) return;
    g_ctx[arm].holding = (holding != 0u) ? 1u : 0u;
    publish(arm);
}

void OreJob_NotifyManual(uint8_t arm)
{
    if(arm >= ORE_ARM_NUM) return;
    g_ctx[arm].at_work = 0u;
    publish(arm);
}

void OreJob_ClearFault(uint8_t arm)
{
    if(arm >= ORE_ARM_NUM) return;
    if(g_ctx[arm].state != ORE_JOB_FAULT) return;

    g_ctx[arm].state = ORE_JOB_IDLE;
    g_ctx[arm].fault = ORE_OK;
    publish(arm);
}

const OreArmStatus_t *OreJob_Status(uint8_t arm)
{
    if(arm >= ORE_ARM_NUM) return 0;
    return &g_pub[arm];
}

uint8_t OreJob_SelectedArm(void)
{
    return g_sel;
}
