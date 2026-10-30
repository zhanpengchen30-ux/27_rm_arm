#ifndef ARM_H
#define ARM_H

#include "main.h"
#include "dm_motor.h"

#define ARM_JOINT_NUM   6

// 夹爪：0 = 完全不驱动（不使能、不下发控制帧）
//       1 = 使能第 7 个电机并按主手按键开合（2026-10-30 打开）
#define ARM_USE_GRIPPER 1

/* 夹爪开/合目标位置（rad），Watch 里可实时改。
   未知量程时先用小步长试：把 g_grip_close 从 0 慢慢加大，观察夹爪行程，
   找到刚好夹紧又不变形的角度后再写死。方向反了就对调两个值。 */
extern volatile float g_grip_open;
extern volatile float g_grip_close;
extern volatile float g_grip_step;   /* 每个 2ms 控制周期的最大步进(rad) */

// Zigbee接收帧格式 (18字节)
// 0xAA + 6*int16(0.001rad) + active + grip + seq + crc8 + 0x55
// 2026-10-30: 主手加夹爪按键，grip 插在 active 后面，帧 17 -> 18
#define ZIGBEE_HEAD          0xAA
#define ZIGBEE_TAIL          0x55
#define ZIGBEE_ACTIVE_OFFSET 13
#define ZIGBEE_GRIP_OFFSET   14
#define ZIGBEE_SEQ_OFFSET    15
#define ZIGBEE_FRAME_LEN     18

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
