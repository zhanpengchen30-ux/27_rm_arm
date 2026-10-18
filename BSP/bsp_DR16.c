#include "bsp_DR16.h"
#include "usart.h"
#include "string.h"
#include "stdlib.h"

#define ABS(x) ((x) > 0 ? (x) : -(x))
#define DR16_FRAME_LENGTH 18

static DR16_Data_t dr16_data = {0};         // 遥控器全局数据
static uint32_t dr16_last_update_tick = 0; // 上一次接收时间

// 接收缓冲区：32字节对齐，尺寸向上取整到32字节（适配H7 Cache Line 32字节刷新）
__attribute__((aligned(32))) static uint8_t sbus_rx_buf[32]; 

/**
  * @brief  遥控器初始化
  */
void DR16_Init(void)
{
    // 开启空闲中断 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&DR16_UART_HANDLE, sbus_rx_buf, DR16_FRAME_LENGTH);
    // 禁用 DMA 半传输中断（避免多余中断打扰）
    __HAL_DMA_DISABLE_IT(DR16_UART_HANDLE.hdmarx, DMA_IT_HT);
}

/**
  * @brief  判断DR16是否在线
  */
bool DR16_IsOnline(void)
{
    if (dr16_last_update_tick == 0) return false;
    return ((osKernelGetTickCount() - dr16_last_update_tick) <= DR16_TIMEOUT_MS);
}

/**
  * @brief  获取 DR16 最新数据
  */
const DR16_Data_t* DR16_Get_Data(void)
{
    // 离线归零保护
    if (!DR16_IsOnline())
    {
        memset(&dr16_data.rc, 0, sizeof(dr16_data.rc));
        memset(&dr16_data.mouse, 0, sizeof(dr16_data.mouse));
        memset(&dr16_data.key, 0, sizeof(dr16_data.key));
    }
    
    return &dr16_data;
}

/**
  * @brief  DBUS 协议底层解码
  */
static void sbus_to_rc(volatile const uint8_t *sbus_buf)
{
    DR16_Data_t rc_raw = {0}; 

    // 1. 基础通道解码
    rc_raw.rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;
    rc_raw.rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff;
    rc_raw.rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff;
    rc_raw.rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff;
    rc_raw.rc.s[0]  = ((sbus_buf[5] >> 4) & 0x0003);
    rc_raw.rc.s[1]  = ((sbus_buf[5] >> 6) & 0x0003);
    
    rc_raw.mouse.x = (int16_t)(sbus_buf[6] | (sbus_buf[7] << 8));
    rc_raw.mouse.y = (int16_t)(sbus_buf[8] | (sbus_buf[9] << 8));
    rc_raw.mouse.z = (int16_t)(sbus_buf[10] | (sbus_buf[11] << 8));
    rc_raw.mouse.press_l = sbus_buf[12];
    rc_raw.mouse.press_r = sbus_buf[13];
    rc_raw.key.v = sbus_buf[14] | (sbus_buf[15] << 8);
    rc_raw.rc.ch[4] = (sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07ff; // 拨轮

    // 2. 减去摇杆中位偏移 (1024)
    for (int i = 0; i < 5; i++)
    {
        rc_raw.rc.ch[i] -= DR16_CH_VALUE_OFFSET;
    }

    // 3. 异常检测（必须在限幅之前进行！）
    // 正常遥控器回传范围 -660 ~ 660，如果超出过大说明数据帧严重受损
    if (ABS(rc_raw.rc.ch[0]) > 700 || ABS(rc_raw.rc.ch[1]) > 700 ||
        ABS(rc_raw.rc.ch[2]) > 700 || ABS(rc_raw.rc.ch[3]) > 700)
    {
        return; // 丢弃该异常帧，不更新数据
    }

    // 4. 幅值限幅保护
    for (int i = 0; i < 5; i++)
    {
        if (rc_raw.rc.ch[i] > 660)  rc_raw.rc.ch[i] = 660;
        if (rc_raw.rc.ch[i] < -660) rc_raw.rc.ch[i] = -660;
    }

    // 5. 摇杆全通道死区滤波（消除中位微小机械回弹抖动）
    for (int i = 0; i < 4; i++)
    {
        if (ABS(rc_raw.rc.ch[i]) <= 10)
        {
            rc_raw.rc.ch[i] = 0;
        }
    }

    // 6. 更新全局数据与时间戳
    dr16_data = rc_raw;
    dr16_last_update_tick = osKernelGetTickCount();
}

/**
  * @brief  接收回调处理函数（在 HAL_UARTEx_RxEventCallback 中调用）
  * @param  Size: 本次接收到的数据长度
  */
void DR16_RX_Callback_Handler(uint16_t Size)
{
    // 严格校验是否为标准 18 字节 DBUS 帧
    if (Size == DR16_FRAME_LENGTH)
    {
        // STM32H7 Cache 刷新：使无效 Cache，保证 CPU 读取内存中的 DMA 新数据
        SCB_InvalidateDCache_by_Addr((uint32_t *)sbus_rx_buf, 32);

        // 数据解析
        sbus_to_rc(sbus_rx_buf);
    }

    // 关键修复：必须重新重启下一次 DMA 接收！
    HAL_UARTEx_ReceiveToIdle_DMA(&DR16_UART_HANDLE, sbus_rx_buf, DR16_FRAME_LENGTH);
    __HAL_DMA_DISABLE_IT(DR16_UART_HANDLE.hdmarx, DMA_IT_HT);
}

/**
  * @brief  遥控器串口异常重置（当发生串口过载、噪声、帧错误时调用）
  */
void DR16_Reset(void)
{
    // 先终止可能异常挂起的接收
    HAL_UART_AbortReceive(&DR16_UART_HANDLE);

    // 重新开启空闲中断 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&DR16_UART_HANDLE, sbus_rx_buf, DR16_FRAME_LENGTH);
    __HAL_DMA_DISABLE_IT(DR16_UART_HANDLE.hdmarx, DMA_IT_HT);
}
