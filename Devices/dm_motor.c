#include "dm_motor.h"
#include <string.h>

// 7 motor instances
 DM_Motor_t dm_motors[DM_MOTOR_NUM];

// float to uint (scaled)
static uint16_t float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if(x < x_min) x = x_min;
    if(x > x_max) x = x_max;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// uint to float (scaled)
static float uint_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return (float)x * span / ((float)((1 << bits) - 1)) + x_min;
}

volatile uint32_t can_tx_fifo_full = 0;
volatile uint32_t can_tx_fail = 0;
// Send one CAN frame
static void DM_CAN_Send(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, uint8_t *data)
{
    FDCAN_TxHeaderTypeDef txHeader;
    txHeader.Identifier = std_id;
    txHeader.IdType = FDCAN_STANDARD_ID;
    txHeader.TxFrameType = FDCAN_DATA_FRAME;
    txHeader.DataLength = FDCAN_DLC_BYTES_8;
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF;
    txHeader.FDFormat = FDCAN_CLASSIC_CAN;
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    // Do not stall the 2ms control loop behind a saturated CAN queue. The
    // counters below expose repeated drops so bus load can be checked.
    uint32_t timeout = 20;
    while(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0 && timeout--) {
        for(volatile int i = 0; i < 20; i++);
    }
    if(HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0)
    {
        if(HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, data) != HAL_OK) can_tx_fail++;
    } else { can_tx_fifo_full++; }
}

// 7 motors, types from bottom: J1=4340, J2=J8009, J3=4340, J4=4310, J5=4310, J6=6220, Gripper=4310
void DM_Motor_Init(void)
{
    memset(dm_motors, 0, sizeof(dm_motors));

    // J1: 4340
    dm_motors[0].can_id = DM_J1_ID;
    dm_motors[0].hfdcan = &hfdcan1;
    dm_motors[0].q_max = 12.5f;
    dm_motors[0].dq_max = 8.0f;
    dm_motors[0].tau_max = 28.0f;

    // J2: J8009
    dm_motors[1].can_id = DM_J2_ID;
    dm_motors[1].hfdcan = &hfdcan1;
    dm_motors[1].q_max = 12.5f;
    dm_motors[1].dq_max = 45.0f;
    dm_motors[1].tau_max = 54.0f;

    // J3: 4340
    dm_motors[2].can_id = DM_J3_ID;
    dm_motors[2].hfdcan = &hfdcan1;
    dm_motors[2].q_max = 12.5f;
    dm_motors[2].dq_max = 8.0f;
    dm_motors[2].tau_max = 28.0f;

    // J4: 4310
    dm_motors[3].can_id = DM_J4_ID;
    dm_motors[3].hfdcan = &hfdcan1;
    dm_motors[3].q_max = 12.5f;
    dm_motors[3].dq_max = 50.0f;
    dm_motors[3].tau_max = 10.0f;

    // J5: 4310
    dm_motors[4].can_id = DM_J5_ID;
    dm_motors[4].hfdcan = &hfdcan1;
    dm_motors[4].q_max = 12.5f;
    dm_motors[4].dq_max = 50.0f;
    dm_motors[4].tau_max = 10.0f;

    // J6: 6220
    dm_motors[5].can_id = DM_J6_ID;
    dm_motors[5].hfdcan = &hfdcan1;
    dm_motors[5].q_max = 12.5f;
    dm_motors[5].dq_max = 45.0f;
    dm_motors[5].tau_max = 10.0f;

    // Gripper: 4310
    dm_motors[6].can_id = DM_GRIPPER_ID;
    dm_motors[6].hfdcan = &hfdcan1;
    dm_motors[6].q_max = 12.5f;
    dm_motors[6].dq_max = 50.0f;
    dm_motors[6].tau_max = 10.0f;
}

// Enable motor (PVT/Hybrid mode: frame ID = 0x300 + motor_id)
void DM_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_ENABLE};
    DM_CAN_Send(hfdcan, 0x300 + motor_id, data);
}

// 清除锁存错误 (0xFB)
// 达妙电机有错误锁存：欠压/过流/过温/编码器一旦报过，错误位会一直挂着，
// 此时无论发多少次使能帧都不会变绿，必须先清错。
void DM_Motor_ClearError(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_CLEAR_ERR};
    DM_CAN_Send(hfdcan, 0x300 + motor_id, data);
}

// 调试用：读 FDCAN 错误状态
// lec: 0=无错 1=填充 2=格式 3=ACK 4=位1 5=位0 6=CRC
//      lec==3(ACK错) 且 tx_err 一直涨 → 总线上没有其他节点应答（没接线/没供电/CANH-L 反）
void DM_Motor_GetBusError(FDCAN_HandleTypeDef *hfdcan, uint8_t *tx_err, uint8_t *rx_err, uint8_t *lec)
{
    FDCAN_ErrorCountersTypeDef ec;
    HAL_FDCAN_GetErrorCounters(hfdcan, &ec);
    *tx_err = (uint8_t)ec.TxErrorCnt;
    *rx_err = (uint8_t)ec.RxErrorCnt;
    *lec    = (uint8_t)((hfdcan->Instance->PSR >> 4) & 0x07);
}

// Disable motor (PVT/Hybrid mode: frame ID = 0x300 + motor_id)
void DM_Motor_Disable(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_DISABLE};
    DM_CAN_Send(hfdcan, 0x300 + motor_id, data);
}

// Set current angle as zero (PVT/Hybrid mode: frame ID = 0x300 + motor_id, 0xFE)
void DM_Motor_SetZero(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, DM_CMD_SET_ZERO};
    DM_CAN_Send(hfdcan, 0x300 + motor_id, data);
}

// Position-velocity mode (0x100 frame), kept as backup
void DM_Motor_PosVel_Control(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                              float pos, float vel)
{
    uint8_t data[8];
    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &vel, 4);
    DM_CAN_Send(hfdcan, 0x100 + motor_id, data);
}

// MIT mode (PVT five-parameter), kept as backup
void DM_Motor_MIT_Control(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                           float pos, float vel, float kp, float kd, float tau)
{
    // find motor for its limits
    float q_max = 12.5f, dq_max = 30.0f, tau_max = 10.0f;
    for(int i = 0; i < DM_MOTOR_NUM; i++)
    {
        if(dm_motors[i].can_id == motor_id && dm_motors[i].hfdcan == hfdcan)
        {
            q_max = dm_motors[i].q_max;
            dq_max = dm_motors[i].dq_max;
            tau_max = dm_motors[i].tau_max;
            break;
        }
    }

    uint16_t q_uint   = float_to_uint(pos,  -q_max,   q_max,   16);
    uint16_t dq_uint  = float_to_uint(vel,  -dq_max,  dq_max,  12);
    uint16_t kp_uint  = float_to_uint(kp,   0,        500,     12);
    uint16_t kd_uint  = float_to_uint(kd,   0,        5,       12);
    uint16_t tau_uint = float_to_uint(tau, -tau_max, tau_max,  12);

    uint8_t data[8];
    data[0] = (q_uint >> 8) & 0xFF;
    data[1] = q_uint & 0xFF;
    data[2] = dq_uint >> 4;
    data[3] = ((dq_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F);
    data[4] = kp_uint & 0xFF;
    data[5] = kd_uint >> 4;
    data[6] = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F);
    data[7] = tau_uint & 0xFF;

    DM_CAN_Send(hfdcan, motor_id, data);
}

// PVT/Hybrid mode: frame ID = 0x300 + motor_id
void DM_Motor_PVT_Control(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id,
                           float pos, float vel, float cur)
{
    uint8_t data[8];
    uint8_t *pbuf = (uint8_t*)&pos;                 // float 4 字节
    uint16_t u16_vel = (uint16_t)(vel * 100.0f);   // 速度
    uint16_t u16_cur = (uint16_t)(cur * 10000.0f); // 力矩/电流
    uint8_t *vbuf = (uint8_t*)&u16_vel;
    uint8_t *ibuf = (uint8_t*)&u16_cur;

    data[0] = pbuf[0];
    data[1] = pbuf[1];
    data[2] = pbuf[2];
    data[3] = pbuf[3];
    data[4] = vbuf[0];
    data[5] = vbuf[1];
    data[6] = ibuf[0];
    data[7] = ibuf[1];

    DM_CAN_Send(hfdcan, 0x300 + motor_id, data);
}

// Parse feedback (Master ID = 0x11~0x17, low 4 bits = motor CAN ID)
void DM_Motor_ParseFeedback(FDCAN_HandleTypeDef *hfdcan, uint32_t can_id, uint8_t *data)
{
    uint8_t motor_id = can_id & 0x0F;

    for(int i = 0; i < DM_MOTOR_NUM; i++)
    {
        if(dm_motors[i].can_id == motor_id && dm_motors[i].hfdcan == hfdcan)
        {
            uint8_t err = data[0] >> 4;
            uint16_t pos_uint   = (data[1] << 8) | data[2];
            uint16_t vel_uint   = (data[3] << 4) | (data[4] >> 4);
            uint16_t tau_uint   = ((data[4] & 0x0F) << 8) | data[5];

            dm_motors[i].error = err;
            dm_motors[i].position = uint_to_float(pos_uint, -dm_motors[i].q_max, dm_motors[i].q_max, 16);
            dm_motors[i].velocity = uint_to_float(vel_uint, -dm_motors[i].dq_max, dm_motors[i].dq_max, 12);
            dm_motors[i].torque   = uint_to_float(tau_uint, -dm_motors[i].tau_max, dm_motors[i].tau_max, 12);
            dm_motors[i].mos_temp = data[6];
            dm_motors[i].rotor_temp = data[7];
            dm_motors[i].rx_cnt++;
            break;
        }
    }
}

// Get motor pointer by index
DM_Motor_t* DM_Motor_Get(uint8_t index)
{
    if(index < DM_MOTOR_NUM)
        return &dm_motors[index];
    return NULL;
}
