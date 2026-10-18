#ifndef __SMC_H
#define __SMC_H

#include <stdint.h>
#include <math.h>

typedef struct {
    // --- 信号输入 ---
    float ref_pos;      // 目标位置
    float fdb_pos;      // 反馈位置
    float ref_spd;      // 目标速度
    float fdb_spd;      // 反馈速度

    // --- 调参参数 ---
    float c;            // 滑模面系数 (类似 P，决定响应快慢)
    float k_exp;        // 指数趋近率 (负责收敛速度，在大误差时发力)
    float epsilon;      // 等速趋近率 (负责克服静摩擦和快速趋近)
    float beta;         // 干扰观测器增益 (类似 I，消除重力和阻力)
    
    // --- 限制与滤波 ---
    float boundary;     // 饱和函数边界 (防抖动，越小越硬，越大越软)
    float out_max;      // 输出限幅
    float lp_tau;       // 干扰观测器的低通滤波系数 (推荐 0.01f)
    float ts;           // 控制周期 (秒) (推荐 0.001f)

    // --- 内部状态 (不要手动改) ---
    float err_pos;
    float err_spd;
    float s;            // 滑模面值
    float dist_est;     // 原始干扰估计
    float dist_est_lp;  // 滤波后的干扰估计
    float output;       // 最终输出
    
} SMC_Handle_t;

// 初始化函数
void SMC_Init(SMC_Handle_t *pSMC, float C, float K, float Epsilon, float Beta, float Boundary, float OutMax, float LpTau, float Ts);

// 计算函数
float SMC_Calc(SMC_Handle_t *pSMC);

#endif
