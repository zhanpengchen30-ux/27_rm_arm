#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "cmsis_os2.h"


extern osMessageQueueId_t canRxQueueHandle;

/* 诊断变量，Keil Watch 直接看 */
extern volatile uint32_t can_rx_id_raw[32];  // 按原始 CAN ID 统计的收帧数
extern volatile uint32_t can_rx_drop;        // 解码队列满丢帧数
extern volatile uint32_t can_rx_bus_cnt[4];  // 分总线收帧数，下标 1/2/3 = FDCAN1/2/3
extern volatile uint32_t can_rx_last_id[4];  // 各总线最后一帧的 ID
extern volatile uint32_t can_rx_other_cnt;   // FDCAN1 上非 0x11~0x16 的帧数
extern volatile uint32_t can_rx_other_id;    // 上面那种帧的最后一帧 ID

/* 临时启停 FDCAN2/3，只给总线扫描用。平时这两路必须保持停止：
   悬空的收发器会收噪声，在共用的 RX 回调里产生幽灵帧。 */
void BSP_FDCAN_StartAux(uint8_t bus);
void BSP_FDCAN_StopAux(uint8_t bus);

//FDCAN 配置
void BSP_FDCAN_Config(void);

//CAN 接收中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

//CAN 接收解码任务
void Can_Decode_Task(void const *argument);


#endif
