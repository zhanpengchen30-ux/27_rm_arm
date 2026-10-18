/**
  ******************************************************************************
  * @file			ladrc.c
  * @version		
  * @date			
  * @brief   		#PID稳定裕度较大，但动态裕度较小，如果被控对象处于动态变化过程中，需要改动PID来达到控制优化的效果
                    #LADRC以PD控制器为基础，加入TD进行输入信号的滤波，将输入信号分为过渡曲线及过渡曲线的微分量，简单的理解为缓慢的向目标点靠近而不阶跃，不超调。
                    #抗扰部分采用LESO对被控对象输出量与控制器控制量进行观测，得到估计状态z1、z2及扰动补偿量z3
  *******************************************************************************
**/
/* -----------------------------------LADRC参数及整定----------------------------------- */
/*
 * 参数含义
 * @h: 步长，根据执行时间进行定义
 * @r：跟踪速度，r越大响应越大，过大r会引入噪声造成高频震荡
 * @w0：w0观测器带宽，其值会影响线性扩张状态观测器的观测精度，当选择较高值时，观测的精度也会提高，但是噪声信号也同样会被放大。
 * @wc：线性反馈率，Kp=wc^2,Kd=3wc,本质是KD控制器
 * @b0：控制量增益，反应控制量对系统的描述
 *
 * 参数整定
 * 1.首先确定b0，确定b0的量纲，b0在实际工程中表现为：过大不收敛且大幅度震荡，过小则高频振荡
 * 2.整定b0时w0和wc可以尽量给大一些
 * 3.确定b0近似大小后开始整定w0和wc，wc对应扰动观测，若存在超调则增大wc；w0对应线性反馈，系统反馈较小时可增大w0
 * 4.r参数最后整定，可近似给200左右，若想增加响应速度可以继续增大至出现高频抖动后略微减小
 * 
*/

#include "LADRC.h"

/**
 * @description: LADRC参数初始化
 * @param 
 * @return 
 */
void ladrc_init(ladrc_t *LADRC,float h,float r,float w0,float wc,float b0)
{
    LADRC->para.h  = h;    //定时时间及时间步长
    LADRC->para.r  = r;    //跟踪速度参数
    LADRC->para.w0 = w0;   //观测器带宽
    LADRC->para.wc = wc;   //状态误差反馈率带宽
    LADRC->para.b0 = b0;
}

/**
 * @description: LADRC跟踪微分部分
 * @param 设置期望Expect(v0)
 * @return 计算过渡过程中的v1(t)、微分信号v2(t)
 */
void LADRC_TD(ladrc_t *LADRC,float Expect) //Expect值即为set值（set）
{
    float fh= -LADRC->para.r*LADRC->para.r*(LADRC->process.v1-Expect)-2*LADRC->para.r*LADRC->process.v2;
    LADRC->process.v1+=LADRC->process.v2*LADRC->para.h;
    LADRC->process.v2+=fh*LADRC->para.h;
}

/**
 * @description: 扩张状态观测器
 * @param 输入为状态量（get）
 * @return 
 */
void LADRC_ESO(ladrc_t *LADRC,float FeedBack)//Feedback即为状态量（get）
{
    float Beita_01=3*LADRC->para.w0;
    float Beita_02=3*LADRC->para.w0*LADRC->para.w0;
    float Beita_03=LADRC->para.w0*LADRC->para.w0*LADRC->para.w0;
    float e= LADRC->process.z1-FeedBack;
    LADRC->process.z1+= (LADRC->process.z2 - Beita_01*e)*LADRC->para.h;
    LADRC->process.z2+= (LADRC->process.z3 - Beita_02*e + LADRC->para.b0*LADRC->para.u)*LADRC->para.h;
    LADRC->process.z3+=-Beita_03*e*LADRC->para.h;
}

/**
 * @description: 控制量计算
 * @param e1 e2 为beita1 beita2
 *        Kp Kd 为fal函数所得值
 * @return 
 */
void LADRC_LF(ladrc_t *LADRC) //输出为LADRC->u
{
    float Kp=LADRC->para.wc*LADRC->para.wc;
    float Kd=3*LADRC->para.wc;
    float e1=LADRC->process.v1-LADRC->process.z1;
    float e2=LADRC->process.v2-LADRC->process.z2;
    float u0=Kp*e1+Kd*e2;
    LADRC->para.u=(u0-LADRC->process.z3)/LADRC->para.b0;
}

/**
 * @description: ADRC总函数，调用此函数即可
 * @param set值与get值
 * @return LADRC->u
 */
void ladrc_cal(ladrc_t *LADRC,float set,float get)
{
    LADRC_TD(LADRC,set);          //期望量
    LADRC_ESO(LADRC,get);         //状态量
    LADRC_LF(LADRC);
}

/**
 * @description: ADRC clear
 * @param set值与get值
 * @return LADRC->u
 */
 void ladrc_clear(ladrc_t *LADRC)
{
    LADRC->process.v1 = 0;
    LADRC->process.v2 = 0;
    LADRC->process.z1 = 0;
    LADRC->process.z2 = 0;
    LADRC->process.z3 = 0;
    LADRC->para.u  = 0;
}

