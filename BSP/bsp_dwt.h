#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "main.h"

/**
 * @brief  初始化 DWT 硬件定时器
 * @note   必须在系统时钟配置完成后调用一次
 */
void DWT_Init(void);

/**
 * @brief  获取自上次调用后经过的时间 (单位: 秒)
 * @note   这是一个“打点计时器”，用于测量任务的实际运行周期 dt
 * @param  last_cnt: 指向用于存储上次计数值的变量的指针 (uint32_t)
 * @return float: 时间间隔 (s)
 */
float DWT_Get_Delta_T(uint32_t *last_cnt);

/**
 * @brief  获取当前 DWT 计数器的原始值
 * @return uint32_t: 当前的时钟周期计数值
 */
uint32_t DWT_Get_Counter(void);

/**
 * @brief  计算两个时间戳之间的耗时 (单位: 微秒)
 * @note   这是一个“秒表”，用于测量一段代码的执行耗时
 * @param  start_cnt: DWT_Get_Counter() 获取的开始时间戳
 * @param  end_cnt: DWT_Get_Counter() 获取的结束时间戳
 * @return float: 耗时 (us)
 */
float DWT_Calc_Time_us(uint32_t start_cnt, uint32_t end_cnt);

#endif
