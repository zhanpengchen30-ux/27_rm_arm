#ifndef ORE_JOB_H
#define ORE_JOB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * 矿位作业状态机（纯逻辑，不碰硬件）
 *
 * 设计前提：
 *   1. 每条臂有 2 个矿位，全部动作以"工作位"为交接点（见 ore_job.c 顶部图）；
 *   2. 每臂 6 条示教轨迹：展开 / 收纳 / 去矿位A / 回 / 去矿位B / 回；
 *   3. 存矿与取矿共用同一对"去/回"轨迹，只有夹爪时序不同；
 *   4. 槽位状态只有 空 / 满 / 未知 三种，未知必须人工确认，绝不默认成空。
 *
 * 本文件不 include 任何 HAL/FreeRTOS 头，可直接在 PC 上编译做逻辑测试；
 * 硬件动作全部通过 OreJob_Hw_t 回调注入。
 * ==========================================================================*/

#define ORE_ARM_NUM       2u          /* 机械臂数量 */
#define ORE_SLOT_PER_ARM  2u          /* 每臂矿位数 */

/* 取矿顺序策略：
 *   1 = FIFO：按存入顺序取（臂1 存 2→1，则取 2→1），与"统一顺序序列 21/34"一致
 *   0 = LIFO：后进先出（两个都有矿时取最后存入的那个）
 *
 * 注：原始需求里"取 2→1"和"两个都有矿时取第二个位置"这两句是矛盾的，
 *     默认按 FIFO 实现；改这一个宏即可切换，不要在别处写死顺序。 */
#define ORE_TAKE_POLICY_FIFO   1

/* 轨迹播放速度（倍速），1.0 = 示教原速 */
#define ORE_PLAY_SPEED         1.0f

/* ---- 段编号（每臂一套，共 6 段）---------------------------------------- */
typedef enum
{
    ORE_SEG_DEPLOY      = 0,   /* 收纳位 → 工作位 */
    ORE_SEG_STOW        = 1,   /* 工作位 → 收纳位 */
    ORE_SEG_TO_SLOT0    = 2,   /* 工作位 → 矿位 A（1 号臂=矿位1，2 号臂=矿位3） */
    ORE_SEG_FROM_SLOT0  = 3,   /* 矿位 A → 工作位 */
    ORE_SEG_TO_SLOT1    = 4,   /* 工作位 → 矿位 B（1 号臂=矿位2，2 号臂=矿位4） */
    ORE_SEG_FROM_SLOT1  = 5,   /* 矿位 B → 工作位 */
    ORE_SEG_NUM         = 6
} OreSeg_t;

/* ---- 槽位状态 ---------------------------------------------------------- */
typedef enum
{
    ORE_SLOT_UNKNOWN = 0,      /* 掉矿/取消/重启后：必须人工确认 */
    ORE_SLOT_EMPTY   = 1,
    ORE_SLOT_FULL    = 2
} OreSlotState_t;

/* ---- 作业状态 ---------------------------------------------------------- */
typedef enum
{
    ORE_JOB_IDLE = 0,
    ORE_JOB_GO_WORK,           /* 先去工作位（不在工作位时才走） */
    ORE_JOB_TO_SLOT,           /* 去目标矿位 */
    ORE_JOB_GRIP,              /* 夹爪动作 + 到位/夹持确认 */
    ORE_JOB_BACK,              /* 回工作位 */
    ORE_JOB_STOWING,           /* 收纳中 */
    ORE_JOB_FAULT
} OreJobState_t;

/* ---- 错误码 ------------------------------------------------------------ */
enum
{
    ORE_OK = 0,
    ORE_ERR_BUSY = 1,          /* 该臂正在执行别的动作 */
    ORE_ERR_NO_TRAJ,           /* 该段轨迹没示教 */
    ORE_ERR_SEG_TIMEOUT,       /* 段执行超时（卡住/反馈丢失） */
    ORE_ERR_GRIP_TIMEOUT,      /* 夹爪没有在规定时间内到位 */
    ORE_ERR_NO_GRIP,           /* 取矿没夹到（或存矿时手上没矿） */
    ORE_ERR_SLOT_FULL,         /* 两个矿位都满了 */
    ORE_ERR_SLOT_EMPTY,        /* 两个矿位都空 */
    ORE_ERR_UNKNOWN_SLOT,      /* 目标槽状态未知，先确认 */
    ORE_ERR_STOPPED            /* 被停止键中止 */
};

/* ---- 按键位图（复用主手 grip 字节，18 字节帧长度不变）------------------
 * bit0 = 夹爪手动（保持原语义，旧主手只发 0/1 时行为不变）
 * bit1 = 存矿    bit2 = 取矿    bit3 = 展开
 * bit4 = 收纳    bit5 = 停止    bit6 = 左右选择（电平：0=臂1 1=臂2）
 * bit7 = 保留
 * 说明：bit1~bit6 是"电平"，本模块内部取上升沿，主手按住不会连发。
 * ---------------------------------------------------------------------- */
#define ORE_KEY_GRIP     0x01u
#define ORE_KEY_STORE    0x02u
#define ORE_KEY_PICK     0x04u
#define ORE_KEY_DEPLOY   0x08u
#define ORE_KEY_STOW     0x10u
#define ORE_KEY_STOP     0x20u
#define ORE_KEY_SEL      0x40u

/* ---- 硬件接口（由 arm.c 适配层实现）------------------------------------ */
typedef struct
{
    /* 播放某臂某段轨迹：返回 0 = 没示教/不可用，1 = 已启动 */
    uint8_t  (*PlaySegment)(uint8_t arm, uint8_t seg, float speed);
    /* 段是否还在执行：1 = 执行中，0 = 已结束（要求"结束"包含到位判定） */
    uint8_t  (*SegmentBusy)(uint8_t arm);
    /* 中止该臂当前段（可传 NULL，表示不支持中止） */
    void     (*StopSegment)(uint8_t arm);
    /* 夹爪命令：close = 1 闭合夹紧，0 张开释放 */
    void     (*GripCmd)(uint8_t arm, uint8_t close);
    /* 夹爪是否动作结束；grabbed 输出"是否夹到东西"（无传感器时可恒填 1） */
    uint8_t  (*GripSettled)(uint8_t arm, uint8_t *grabbed);
    /* 矿位检测（可选，无传感器传 NULL）：0=空 1=有矿 2=无传感器 */
    uint8_t  (*SlotSensor)(uint8_t arm, uint8_t idx);
    /* 毫秒时基（传 NULL 则内部用计数器，仅测试用） */
    uint32_t (*Now)(void);
} OreJob_Hw_t;

/* ---- 对外状态（只读）--------------------------------------------------- */
typedef struct
{
    uint8_t slot[ORE_SLOT_PER_ARM];   /* OreSlotState_t */
    uint8_t last_idx;                 /* 最后存入的槽，0xFF=无 */
    uint8_t holding;                  /* 1=夹爪持矿 */
    uint8_t state;                    /* OreJobState_t */
    uint8_t fault;                    /* 错误码 */
    uint8_t target;                   /* 当前操作的槽 */
    uint8_t at_work;                  /* 1=在工作位（可信） */
} OreArmStatus_t;

void    OreJob_Init(const OreJob_Hw_t *hw);

/* 2ms 控制环里每拍调一次 */
void    OreJob_Tick(void);

/* 每拍喂主手 key_map（含 bit0 夹爪位），内部做上升沿检测 */
void    OreJob_SetKeyMap(uint8_t key_map);

/* 停止两条臂的全部自动动作（停止键语义：保力矩、保夹持、不失能） */
void    OreJob_Stop(void);

/* 人工纠正：把某槽手动设为 空/满（UNKNOWN 的唯一出口） */
void    OreJob_ConfirmSlot(uint8_t arm, uint8_t idx, OreSlotState_t s);

/* 人工声明"现在手上有/没有矿"（重启或手动夹取后必须调用） */
void    OreJob_SetHolding(uint8_t arm, uint8_t holding);

/* 人工接管主从跟随：工作位标志失效，下次自动动作会先回工作位 */
void    OreJob_NotifyManual(uint8_t arm);

/* 清故障（UNKNOWN_SLOT 清掉后仍需 OreJob_ConfirmSlot 才能继续） */
void    OreJob_ClearFault(uint8_t arm);

const OreArmStatus_t *OreJob_Status(uint8_t arm);
uint8_t OreJob_SelectedArm(void);

#ifdef __cplusplus
}
#endif

#endif /* ORE_JOB_H */
