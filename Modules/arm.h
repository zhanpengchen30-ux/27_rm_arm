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
extern volatile float g_grip_max_lag;/* 目标超前实测位置的上限(rad)，同时决定开合速度与夹持力 */
extern volatile float g_grip_vel_max;/* 夹爪 PVT 速度字段上限(rad/s)，默认 6，可实时改 */

/* 运行时方向修正：默认全 +1，等于只用编译期 ARM_DIR；某轴反了就写 -1，立刻生效。 */
extern volatile int8_t g_dir_sign[ARM_JOINT_NUM];

/* 各轴 PVT 电流前馈（不是限流）：默认 0.7。承重轴托不住/手能推得动就往上加，
   1.2 是历史上的"够托举"值，1.8 触发过流保护；上限 2.0 由 SaneF 兜底。 */
extern volatile float g_cur_ff[ARM_JOINT_NUM];

/* 夹爪电流限值：**额定电流的标幺值**，不是安培（PVT 帧内部 ×10000，上限 1.0）。
   4310 额定 2.5A，所以 0.9 ≈ 2.25A。夹不紧往上调，但别超过 1.0。 */
extern volatile float g_grip_cur;

/* 上档且主手使能：1=重新录制并替换旧轨迹，2=停止，3=回起点后回放，4=仅回起点；100Hz/30s。
   示教操作只走 Watch 写本变量；主手按键(PH10)已改为一键循环，见 g_cycle_state。
   轨迹只在 RAM，复位即丢；要留就下档写 g_teach_save_req=1 存 Flash，上电自动加载。 */
extern volatile uint8_t  g_teach_req;
extern volatile uint8_t  g_cycle_state;   /* 0=空闲 1=去工作位 2=去轨迹首点 3=回放中 4=回放完保持 */
extern volatile uint8_t  g_cycle_err;     /* 1=没录轨迹 2=工作位未标定 */
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
extern volatile float   g_trk_vel_gain;     /* vmag 追赶增益：vmag += g*|实测-指令|，0=旧行为 */
extern volatile float   g_trk_err_peak[6];  /* 实测-指令 峰值跟踪误差(rad)；写0清零后重测（伺服误差） */
extern volatile float   g_lag_err_peak[6];  /* 主手期望-实测 峰值滞后(rad)；写0清零后重测（总滞后） */

// Zigbee接收帧格式 (19字节，2026-11-09 四键协议)
// 0xAB + 6*int16(0.001rad) + active + action + event_id + seq + crc8 + 0x55
//   active : bit0=deadman, bit1~2=Watch 调试命令码(抽臂/循环，正常0)
//   action : 0=无事件 1=STOW收纳/展开 2=ORE矿位作业 3=HOME归位/接管 4=GRIP夹爪翻转
//   event_id: 每次按下递增(1..255)，从手按它去重，重复帧只执行一次
//   crc8   : CRC-8/ATM(0x07)，覆盖 [0..16]
// 帧头改成 0xAB：旧 0xAA 主手帧整帧拒绝，主从必须配套烧录。
#define ZIGBEE_HEAD          0xAB
#define ZIGBEE_TAIL          0x55
#define ZIGBEE_ACTIVE_OFFSET 13
#define ZIGBEE_ACTION_OFFSET 14
#define ZIGBEE_EVENT_OFFSET  15
#define ZIGBEE_SEQ_OFFSET    16
#define ZIGBEE_FRAME_LEN     19u

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

/* 单臂两矿位：每个槽一组取矿点（取矿点 + 上方过渡点 + 合爪后抬出点）
   和一组放矿点（放矿点 + 放矿点上方过渡点），存 6 台电机的绝对角(rad)。
   和 g_deploy_pos 同一套约定：不看主手角度、不走主从偏移，能反复走到同一姿态。 */
typedef struct
{
    float take[ARM_JOINT_NUM];        // 取矿点（夹爪伸进矿里、还没合爪的姿态）
    float take_above[ARM_JOINT_NUM];  // 取矿点上方过渡点：去程先到这再下探，回程抬出后也经这
    float take_lift[ARM_JOINT_NUM];   // 抬出点：合爪夹住矿后先抬到这，再回工作位
    float place[ARM_JOINT_NUM];       // 放矿点（车上这个槽的卸矿姿态）
    float place_above[ARM_JOINT_NUM]; // 放矿点上方过渡点（先到这再下探，没录就直接去放矿点）
    uint8_t take_valid;               // 0=没录 / 1=已录且合法
    uint8_t take_above_valid;
    uint8_t take_lift_valid;
    uint8_t place_valid;
    uint8_t above_valid;
} Arm_SlotPose_t;

/* 单臂固定顺序作业：放矿位2 → 放矿位1 → 取矿位1 → 取矿位2，做完一轮自动回到第一步。
   矿位号 = g_slot_base + 槽下标（base=1 → 矿位1,2）。一次 ORE 按键只做当前这一步，
   行走/夹放期间再按不排队不跳步；只有成功完成才推进 g_slot_step。 */
extern volatile uint8_t g_slot_enable;    /* 0=矿位流程不介入 */
extern volatile uint8_t g_slot_base;      /* 1=本臂管矿位1,2  3=管矿位3,4 */
extern volatile uint8_t g_slot_cmd;       /* Watch调试：1=放矿 2=取矿 3=中止；ORE按键自动按step选 */
extern volatile uint8_t g_slot_state;     /* 0空闲 1去工作位 2去放矿位 3张爪卸矿 4回工作位 5去取矿位 6合爪确认 7等夹爪张开 8去放矿点上方 9去取矿点上方 10抬出 11回程经取矿上方 */
extern volatile uint8_t g_slot_err;       /* 1档位 2点位 3被占用 4顺序/库存 5反馈 6超时 7没静止 8夹持未确认 */
extern volatile uint8_t g_slot_target;    /* 本流程针对的槽下标 */
extern volatile uint8_t g_slot_step;      /* 0=放槽1 1=放槽0 2=取槽0 3=取槽1，完成+1模4 */
extern volatile uint8_t g_slot_occupied[ARM_SLOT_NUM];
extern volatile int8_t  g_slot_carry;     /* 手上这颗矿来自哪个槽，-1=手动夹的/不知道 */
extern volatile uint8_t g_slot_force;     /* 1=人工override：忽略库存冲突，且夹持窗口未标定时信任操作手 */
extern volatile uint8_t g_slot_auto_follow; /* 1=取矿趟回工作位后也自动交还跟随(默认)；0=夹着矿保持住 */
extern volatile uint8_t g_slot_pose_cmd;  /* 1=录取矿点 2=录放矿点 3=清除本槽 4=录放矿点上方 5=录取矿点上方 6=录抬出点(矿在夹爪里) */
extern volatile uint8_t g_slot_pose_sel;  /* 录入/清除的槽下标 */
extern volatile uint8_t g_slot_saved;
extern volatile uint8_t g_slot_save_req;  /* 1=存Flash 2=删除；仅下档生效 */
extern volatile uint8_t g_slot_save_err;  /* 0=ok 1=不在下档 2=Flash失败 4=加载校验不过 */
extern Arm_SlotPose_t arm_slot[ARM_SLOT_NUM];

/* 夹爪夹矿确认窗口(rad)：夹住矿时夹爪停在 open 与 close 之间（矿的厚度），
   空合到底会冲到 close。实测把"夹住矿"的角度范围填进 min/max（max>min 才生效）。
   未标定(max<=min)时自动夹持确认一律要求 g_slot_force=1，否则报 err=8 不动作。 */
extern volatile float g_grip_ore_min;
extern volatile float g_grip_ore_max;

/* ===== HOME 归位/接管 + STOW 收纳/展开 =====
   g_home_state: 0=跟随主手 1=去工作位 2=工作位保持(暂停跟随) 3=去收纳位 4=收纳位保持。
   非0一律暂停跟随，自动运动仍照常运行；HOME 在工作位保持时再按一下才重新吸合跟随。
   收纳位必须先用 g_stow_pose_cmd=1 标定（静止时录当前姿态），未标定拒绝收纳动作。 */
extern volatile uint8_t g_home_state;
extern volatile uint8_t g_home_err;       /* 0=ok 1=忙 2=收纳位未标定/非法 3=反馈/超时 4=档位/链路 */
extern volatile float   g_stow_pos[ARM_JOINT_NUM];
extern volatile uint8_t g_stow_valid;     /* 1=收纳位已标定且合法 */
extern volatile uint8_t g_stow_pose_cmd;  /* 1=录当前姿态为收纳位 2=清除标定 */

/* ===== 工作位：所有自动动作的交接点 =====
   不再写死。上电 g_deploy_valid=0（未标定），一切需要工作位的动作直接拒绝。
   Watch 写 g_deploy_pose_cmd=1 录当前姿态，=2 清除；成功/失败看 g_home_err：
   0=ok 2=点位非法/未标定 3=臂没停稳或反馈不新鲜 4=不在上档。 */
extern volatile float   g_deploy_pos[ARM_JOINT_NUM];
extern volatile uint8_t g_deploy_valid;
extern volatile uint8_t g_deploy_pose_cmd;

/* ===== 姿态快照（B 方案）：随手存一个姿态，掉电可留，随时走回去 =====
   g_snap_cmd: 1=把当前六轴角度存成快照  2=清除  3=走到快照姿态
   存完只是 RAM；要掉电保留，照常在下档写 g_slot_save_req=1（与点位表同一扇区）。
   录点位之前想先"试摆"一个姿态，用这个最省事。 */
extern volatile float   g_snap_pos[ARM_JOINT_NUM];
extern volatile uint8_t g_snap_valid;
extern volatile uint8_t g_snap_cmd;

/* ===== 调试汇总：Watch 里只看这一个，不用展开 dm_motors[0..6] =====
   全部平铺成标量（不用数组），Watch 打开就是 6 行角度，不会折叠。
   字段后缀 1~6 对应 J1~J6。
     q1~q6   实测角度，单位 rad
     d1~d6   同一路实测角换算成度（deg），方便肉眼判断
     c1~c6   下发指令 cmd_smooth（PID 前的位置指令），rad
     t1~t6   主手目标（未插值、未限幅，用来判断主手是否在动），rad
     grip_pos / grip_cmd   夹爪实测与期望
     err_max / err_max_deg   六轴最大 实测减指令 的绝对值，正常跟随应小于 0.05 rad
     state   0=失能 1=锁死 2=跟随
     home    0=跟随 1=去工作位 2=工作位保持 3=去收纳位 4=收纳位保持
     q_off   1=已捕获主从偏移（真正开始跟随），0=还没接管
     deploy_ok / stow_ok   工作位 / 收纳位是否已标定
     m_err_mask   哪几轴报错（bit0=J1 ... bit5=J6），m_err 是第一个错误码
     slot_v   点位有效位：bit0 槽0取 bit1 槽0放 bit2 槽1取 bit3 槽1放
     slot_occ   库存：bit0 槽0 bit1 槽1
*/
typedef struct
{
    float q1, q2, q3, q4, q5, q6;
    float d1, d2, d3, d4, d5, d6;
    float c1, c2, c3, c4, c5, c6;
    float t1, t2, t3, t4, t5, t6;
    float grip_pos, grip_cmd;
    float err_max, err_max_deg;

    uint8_t state, home, slot, teach;
    uint8_t connected, active, q_off;
    uint8_t deploy_ok, stow_ok, snap_ok;
    uint8_t m_err_mask, m_err, fb_mask, lock_mask;
    uint16_t lock_recover;
    float    j4_tx_pos;
    uint16_t j4_tx_vel_raw, j4_tx_cur_raw;
    uint32_t j4_tx_ok;
    uint8_t slot_err, home_err, slot_step, slot_occ, slot_v;
} ArmDbg_t;

extern volatile ArmDbg_t g_arm_dbg;

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
    uint8_t gripper_cmd_recv;            // 从手自持的夹爪期望：0开1合。GRIP按键翻转、自动流程改写

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
