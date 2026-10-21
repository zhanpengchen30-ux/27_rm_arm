#ifndef ARM_H
#define ARM_H

#include "main.h"
#include "dm_motor.h"

#define ARM_JOINT_NUM   6

// 夹爪暂时不上机：0 = 完全不驱动夹爪电机（不使能、不下发控制帧）
// 注意：主手侧的开关还没进通信协议，夹爪状态目前传不过来
#define ARM_USE_GRIPPER 0

// Zigbee接收帧格式 (15字节)
// 0xAA + 6*int16(0.001rad) + active(1B) + 0x55
#define ZIGBEE_HEAD          0xAA
#define ZIGBEE_TAIL          0x55
#define ZIGBEE_ACTIVE_OFFSET 13   
#define ZIGBEE_FRAME_LEN     15   

#define ARM_RX_TIMEOUT_MS    200

#define ARM_CTRL_PERIOD_S    (0.006f)
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
