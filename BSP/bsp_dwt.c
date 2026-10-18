#include "bsp_dwt.h"


static float CPU_FREQ_Hz_f = 0.0f;
static float CPU_FREQ_us_f = 0.0f;


void DWT_Init(void)
{
    //使能 DWT 外设
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    //获取时钟频率
    CPU_FREQ_Hz_f = (float)HAL_RCC_GetHCLKFreq();
    CPU_FREQ_us_f = CPU_FREQ_Hz_f / 1000000.0f;
}


float DWT_Get_Delta_T(uint32_t *last_cnt)
{
    volatile uint32_t now_cnt = DWT->CYCCNT;
    
    //计算差值
    uint32_t delta_ticks = now_cnt - *last_cnt;
    
    //更新时间戳
    *last_cnt = now_cnt;
    
    //换算成秒
    return (float)delta_ticks / CPU_FREQ_Hz_f;
}


uint32_t DWT_Get_Counter(void)
{
    return DWT->CYCCNT;
}


float DWT_Calc_Time_us(uint32_t start_cnt, uint32_t end_cnt)
{
    uint32_t delta_ticks = end_cnt - start_cnt;
    return (float)delta_ticks / CPU_FREQ_us_f;
}
