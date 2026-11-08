#ifndef ARM_H
#define ARM_H

#include "main.h"
#include "dm_motor.h"

#define ARM_JOINT_NUM   6

/* 本臂负责的矿位数量（矿位号 = g_slot_base + 槽下标） */
#define ARM_SLOT_NUM    2u

// 夹爪：0 = 完全不驱动（不使能、不下发控制帧）
//       1 = 使能第 7 个电机并按主手按键开合（2026-10-30 打开）
#define ARM_USE_GRIPPER 1

/* 夹爪开/合目标位置（rad），Watch 里可实时改。
   未知量程时先用小步长试：把 g_grip_close 从 0 慢慢加大，观察夹爪行程，
   找到刚好夹紧又不变形的角度后再写死。方向反了就对调两个值。
   这两个值同时也是目标的软限位。 */
extern volatile float g_grip_open;
extern volatile float g_grip_close;
extern volatile float g_grip_step;   /* 每个夹爪下发周期(10ms)的最大步进(rad) */

/* 夹爪电流限值：**额定电流的标幺值**，不是安培（PVT 帧内部 ×10000，上限 1.0）。
   4310 额定 2.5A，所以 0.9 ≈ 2.25A。夹不紧往上调，但别超过 1.0。 */
extern volatile float g_grip_cur;

/* 上档且主手使能：1=重新录制并替换旧轨迹，2=停止，3=回起点后回放，4=仅回起点；100Hz/30s。
   示教操作只走 Watch 写本变量；主手按键(PH10)已改为一键循环，见 g_cycle_state。
   轨迹只在 RAM，复位即丢；要留就下档写 g_teach_save_req=1 存 Flash，上电自动加载。 */
extern volatile uint8_t  g_teach_req;
extern volatile uint8_t  g_cycle_state;   /* 0=空闲 1=去工作位 2=去轨迹首点 3=回放中 4=回放完保持 */
extern volatile uint8_t  g_cycle_err;     /* 1=没录轨迹，按键不生效 */
extern volatile uint32_t g_arm_loop_tick;   /* 控制环心跳时间戳 */
extern volatile uint32_t g_arm_busy_until;  /* Flash 操作窗口，监控不判挂死 */
void Arm_EmergencyStop(void);               /* 只失能全部电机，不复位 */
void Arm_EmergencyStopAndReset(void);       /* 失能全部电机后系统复位 */
extern volatile uint8_t  g_teach_save_req;  /* 1=存Flash 2=删除；仅下档生效，结果看 g_teach_save_err */
extern volatile uint8_t  g_teach_save_err;  /* 0=ok 1=不在下档 2=Flash失败 3=没轨迹 4=加载校验不过 */
extern volatile uint8_t  g_teach_saved;     /* 1=Flash 里有有效轨迹 */
extern volatile uint8_t  g_teach_state;   /* 0=无轨迹 1=记录 2=就绪 3=正放 4=末端保持 5=倒放回起点 */
extern volatile uint8_t  g_teach_error;
extern volatile uint16_t g_teach_count;
extern volatile uint16_t g_teach_index;
extern volatile float    g_teach_speed;   /* 回放倍速，允许0.2~2.0，1.0为原速 */

/* ===== 跟随抖动对照实验（默认值=改动前的行为，写 Watch 才生效）===== */
extern volatile float   g_slew_max_rad_s;   /* 跟随速度上限，默认3.0(=旧 ARM_MAX_STEP) */
extern volatile float   g_decel_rad_s2;     /* 减速带加速度，默认20 */
extern volatile uint8_t g_decel_en;         /* 0=旧行为(匀速怼+瞬间钉住) 1=开减速带 */
extern volatile float   g_trk_err_peak[6];  /* 实测-指令 峰值跟踪误差(rad)；写0清零后重测 */

// Zigbee接收帧格式 (18字节)
// 0xAA + 6*int16(0.001rad) + active + grip + seq + crc8 + 0x55
// 2026-10-30: 主手加夹爪按键，grip 插在 active 后面，帧 17 -> 18
#define ZIGBEE_HEAD          0xAA
#define ZIGBEE_TAIL          0x55
#define ZIGBEE_ACTIVE_OFFSET 13
#define ZIGBEE_GRIP_OFFSET   14
#define ZIGBEE_SEQ_OFFSET    15
#define ZIGBEE_FRAME_LEN     18u

#define ARM_RX_TIMEOUT_MS    500

#define ARM_CTRL_PERIOD_S    (0.002f)
#define ARM_MAX_SPEED_RAD_S  (3.0f)
#define ARM_MAX_STEP         (ARM_MAX_SPEED_RAD_S * ARM_CTRL_PERIOD_S)

typedef enum {
    ARM_STATE_DISABLED  = 0, // 彻底失能泄力（遥控器下档或关机）
    ARM_STATE_LOCKED    = 1, // 原地锁死保持（遥控器中档）
    ARM_STATE_FOLLOWING = 2  // 实时平滑跟随（遥控器上档 + 主手使能）
} ArmState_e;

typedef enum {
    GRIPPER_OPEN = 0,
    GRIPPER_CLOSE = 1
} GripperState_e;

/* 单臂两矿位：每个槽一个取矿点 + 一个放矿点，存 6 台电机的绝对角(rad)。
   和 g_deploy_pos 同一套约定：不看主手角度、不走主从偏移，能反复走到同一姿态。 */
typedef struct
{
    float take[ARM_JOINT_NUM];       // 取矿点（手动合爪前自动走到的姿态）
    float place[ARM_JOINT_NUM];      // 放矿点（车上这个槽的卸矿姿态）
    uint8_t take_valid;              // 0=没录 / 1=已录且合法
    uint8_t place_valid;
} Arm_SlotPose_t;

/* 顺序：放矿 槽0→槽1，取矿 槽1→槽0（后放先取）。
   矿位号 = g_slot_base + 槽下标：臂A base=1(矿位1,2)，臂B base=3(矿位3,4)。 */
extern volatile uint8_t g_slot_enable;    /* 0=矿位流程不介入，退回纯跟随+旧按键循环 */
extern volatile uint8_t g_slot_base;      /* 1=本臂管矿位1,2  3=管矿位3,4 */
extern volatile uint8_t g_slot_cmd;       /* 1=自动放矿 2=走到下一取矿点 3=中止 */
extern volatile uint8_t g_slot_state;     /* 0空闲 1去工作位 2去放矿位 3张爪卸矿 4回工作位 5去取矿位 */
extern volatile uint8_t g_slot_err;       /* 1档位 2点位 3被占用 4顺序/库存 5反馈 6超时 7没静止 */
extern volatile uint8_t g_slot_target;    /* 本流程针对的槽下标 */
extern volatile uint8_t g_slot_occupied[ARM_SLOT_NUM];
extern volatile int8_t  g_slot_place_ptr; /* 放矿指针 0→1 */
extern volatile int8_t  g_slot_take_ptr;  /* 取矿指针 1→0 */
extern volatile int8_t  g_slot_carry;     /* 手上这颗矿来自哪个槽，-1=不知道 */
extern volatile uint8_t g_slot_force;     /* 1=忽略库存冲突（人工确认过） */
extern volatile uint8_t g_slot_pose_cmd;  /* 1=录取矿点 2=录放矿点 3=清除本槽 */
extern volatile uint8_t g_slot_pose_sel;  /* 录入/清除的槽下标 */
extern volatile uint8_t g_slot_saved;
extern volatile uint8_t g_slot_save_req;  /* 1=存Flash 2=删除；仅下档生效 */
extern volatile uint8_t g_slot_save_err;  /* 0=ok 1=不在下档 2=Flash失败 4=加载校验不过 */
extern Arm_SlotPose_t arm_slot[ARM_SLOT_NUM];

typedef struct
{
    float target_joint[ARM_JOINT_NUM];
    float target_gripper;                // 修复：必须是 float
    uint8_t active;                      

    float cmd_smooth[ARM_JOINT_NUM];     
    float q_offset[ARM_JOINT_NUM];       // 主从离合偏移：每次进入跟随瞬间捕获，之后只跟随增量
    uint8_t q_offset_valid;              // 偏移是否已有效捕获

    GPIO_PinState sw_open;               
    GPIO_PinState sw_close;              

    GripperState_e gripper_state;        
    uint8_t gripper_cmd_recv;            

    ArmState_e state;                    
    ArmState_e last_state;               
    uint8_t connected;                   
    uint32_t last_rx_tick;               
    uint32_t recv_cnt;                   
} Arm_t;

void Arm_Init(void);
void Arm_Zigbee_ParseByte(uint8_t byte);
void Arm_Zigbee_RxRestart(void);
void Arm_ControlLoop(void);
Arm_t* Arm_Get(void);

#endif
