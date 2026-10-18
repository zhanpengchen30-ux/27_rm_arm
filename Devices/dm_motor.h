#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "main.h"
#include "fdcan.h"
#include <math.h>

// 达妙电机数量
#define DM_MOTOR_NUM    7

// 电机CAN_ID定义（按你的实际电机改）
#define DM_J1_ID    0x01
#define DM_J2_ID    0x02
#define DM_J3_ID    0x03
#define DM_J4_ID    0x04
#define DM_J5_ID    0x05
#define DM_J6_ID    0x06
#define DM_GRIPPER_ID 0x07

// 达妙电机模式命令
#define DM_CMD_CLEAR_ERR    0xFB
#define DM_CMD_ENABLE       0xFC
#define DM_CMD_DISABLE      0xFD
#define DM_CMD_SET_ZERO     0xFE

// 达妙电机数据结构
typedef struct
{
    uint8_t can_id;         // CAN ID
    FDCAN_HandleTypeDef *hfdcan;  // 挂在哪路CAN

    // 限幅（按电机型号配置）
    float q_max;            // 位置限幅 rad
    float dq_max;           // 速度限幅 rad/s
    float tau_max;          // 力矩限幅 N·m

    // 反馈
    float position;         // 当前位置 rad
    float velocity;         // 当前速度 rad/s
    float torque;           // 当前力矩 N·m
    uint8_t error;          // 错误码
    float mos_temp;         // MOS温度
    float rotor_temp;       // 绕组温度

    // 目标
    float target_pos;       // 目标位置 rad
    float target_vel;       // 目标速度 rad/s

    // 状态
    uint8_t enabled;        // 是否使能
    uint32_t rx_cnt;        // 接收计数
} DM_Motor_t;

// 初始化所有达妙电机
void DM_Motor_Init(void);

// 使能单个电机
void DM_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

// 清除锁存错误（0xFB）。达妙电机有错误位锁存，不清掉怎么发使能都不会亮
void DM_Motor_ClearError(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

// 调试用：读 FDCAN 错误计数，tx_err 一路涨 = 总线上没人 ACK（没接线/没供电/CANH-L 反）
void DM_Motor_GetBusError(FDCAN_HandleTypeDef *hfdcan, uint8_t *tx_err, uint8_t *rx_err, uint8_t *lec);

// 失能单个电机
void DM_Motor_Disable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

// 设当前角度为零点（0x300+ID，0xFE帧）
void DM_Motor_SetZero(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id);

// 位置速度模式控制（0x100帧）
void DM_Motor_PosVel_Control(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                              float pos, float vel);

// MIT模式控制（PVT五参数）
void DM_Motor_MIT_Control(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                           float pos, float vel, float kp, float kd, float tau);

// PVT模式控制（Hybrid，0x300+ID帧：位置float4字节 + 速度×100 2字节 + 电流×10000 2字节）
void DM_Motor_PVT_Control(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                           float pos, float vel, float cur);

// 解析CAN反馈（在Can_Decode_Task里调用）
void DM_Motor_ParseFeedback(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, uint8_t *data);

// 获取电机数据指针
DM_Motor_t* DM_Motor_Get(uint8_t index);
// 导出全局电机数组，让所有文件和 Keil Watch 都能直接识别结构体
extern DM_Motor_t dm_motors[DM_MOTOR_NUM];
#endif
