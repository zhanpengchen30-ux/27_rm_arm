#ifndef BSP_DR16_H
#define BSP_DR16_H

#include "main.h"
#include "cmsis_os2.h"
#include <stdbool.h>


// DBUS协议参数
#define DR16_UART_HANDLE     huart5
#define RC_FRAME_LENGTH      18u
#define DR16_RX_BUF_NUM      36u  //双缓冲区长度，配合Circular DMA使用

// 遥控器数值范围
#define DR16_CH_VALUE_MIN    364
#define DR16_CH_VALUE_OFFSET 1024
#define DR16_CH_VALUE_MAX    1684

//离线判定超时阈值
#define DR16_TIMEOUT_MS      50u      


// 开关状态
#define DR16_SW_UP           1
#define DR16_SW_MID          3
#define DR16_SW_DOWN         2


// 遥控器数据结构体
typedef struct
{
    struct
    {
        int16_t ch[5]; // 0-3: 摇杆, 4: 拨轮
        uint8_t s[2];  // 0: 右拨杆, 1: 左拨杆
    } rc;
    
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    
    struct
    {
        uint16_t v;
    } key;
		bool is_online;          // 在线状态标志位
} DR16_Data_t;



extern osMessageQueueId_t rcQueueHandle;


//遥控器初始化
void DR16_Init(void);

//获取处理后的遥控器数据
const DR16_Data_t* DR16_Get_Data(void);

//判断DR16是否在线
bool DR16_IsOnline(void);

//接收回调
void DR16_RX_Callback_Handler(uint16_t Size);

//串口错误复位函数
void DR16_Reset(void);

#endif
