#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"
#include "cmsis_os2.h"


extern osMessageQueueId_t canRxQueueHandle;

//FDCAN配置
void BSP_FDCAN_Config(void);

//CAN接收中断回调函数
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

//CAN接收解码任务
void Can_Decode_Task(void const *argument);


#endif
