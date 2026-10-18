#include "smc.h"
#include "string.h"


/**
 * @brief  饱和函数 (Saturation Function)
 * @note   用于替代符号函数 sgn(s) 以减少抖动
 * @param  x: 输入值
 * @param  delta: 边界层厚度
 * @return float: -1.0 到 1.0 之间的值
 */
static float sat_func(float x, float delta)
{
    if (delta <= 0.0f) return (x > 0.0f) ? 1.0f : -1.0f;
    
    if (x > delta) return 1.0f;
    else if (x < -delta) return -1.0f;
    else return x / delta;
}




//低通滤波器：让干扰补偿更平滑，电机更安静
static float low_pass_filter(float input, float prev_output, float ts, float tau) {
    if (tau <= 0.0f) return input;
    float alpha = ts / (tau + ts);
    return alpha * input + (1.0f - alpha) * prev_output;
}




//初始化SMC控制器参数
void SMC_Init(SMC_Handle_t *pSMC, float C, float K, float Epsilon, float Beta, float Boundary, float OutMax, float LpTau, float Ts) {
    if (pSMC == 0) return;
    
    memset(pSMC, 0, sizeof(SMC_Handle_t));

    pSMC->c = C;
    pSMC->k_exp = K;
    pSMC->epsilon = Epsilon;
    pSMC->beta = Beta;
    pSMC->boundary = Boundary;
    pSMC->out_max = OutMax;
    pSMC->lp_tau = LpTau;
    pSMC->ts = Ts;
}



/**
 * @brief  执行滑模控制计算
 * @note   建议放在定时器中断或控制循环中调用
 * @param  pSMC: 控制器结构体指针
 * @return float: 控制量输出 (电流/扭矩/电压)
 */
float SMC_Calc(SMC_Handle_t *pSMC)
{
    if(pSMC == 0) return 0.0f;

    //1. 计算误差
    pSMC->err_pos = pSMC->ref_pos - pSMC->fdb_pos;
    pSMC->err_spd = pSMC->ref_spd - pSMC->fdb_spd;

    //2. 计算滑模面 s = c * e + e_dot
    pSMC->s = (pSMC->c * pSMC->err_pos) + pSMC->err_spd;

    //3. 干扰观测器
    // 离散积分：d_new = d_old + (beta * s * dt)
    pSMC->dist_est += pSMC->beta * pSMC->s * pSMC->ts;

    //观测器限幅 (防止积分饱和)
    if (pSMC->dist_est > pSMC->out_max) pSMC->dist_est = pSMC->out_max;
    else if (pSMC->dist_est < -pSMC->out_max) pSMC->dist_est = -pSMC->out_max;

    //观测器滤波 (让力矩更柔和)
    pSMC->dist_est_lp = low_pass_filter(pSMC->dist_est, pSMC->dist_est_lp, pSMC->ts, pSMC->lp_tau);

    //4. 计算趋近律控制量
    float sign_s = sat_func(pSMC->s, pSMC->boundary);
    float reach = (pSMC->epsilon * sign_s) + (pSMC->k_exp * pSMC->s) + (pSMC->c * pSMC->err_spd);

    //5. 总输出 = 趋近控制 + 干扰补偿
    float total_out = reach + pSMC->dist_est_lp;

    //6. 输出限幅
    if (total_out > pSMC->out_max) total_out = pSMC->out_max;
    else if (total_out < -pSMC->out_max) total_out = -pSMC->out_max;

    pSMC->output = total_out;
    return pSMC->output;
}

