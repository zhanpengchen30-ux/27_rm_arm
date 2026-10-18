/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "fdcan.h"

extern osMessageQueueId_t canRxQueueHandle;
extern osMessageQueueId_t canTxQueueHandle;

#define TRUE  1
#define FALSE 0
#define PI 3.14159265358979f

// CAN 接收队列数据结构体
typedef struct
{
    FDCAN_HandleTypeDef* hfdcan;
    uint32_t can_id;
    uint8_t data[8];
} CAN_Rx_Message_t;

// CAN 发送队列数据结构体
typedef struct
{
    uint8_t can_bus;     // 1:FDCAN1, 2:FDCAN2, 3:FDCAN3
    uint32_t can_id;
    uint8_t data[8];
    uint8_t DLC;         // 数据长度代码 (Data Length Code)
} CAN_Tx_Message_t;

// PID 控制器结构体
typedef struct
{
    float Kp, Ki, Kd;

    float MaxOutput;
    float Integral_Limit;

    float integral_separation_threshold; // 积分分离阈值
    float dead_band;                     // 死区

    float target;
    float measured;
    float error;
    float pre_error;
    float pre_measured;
    float err_beforeLast;

    float p_out;
    float i_out;
    float d_out;
    float output;
} PID_Controller;

// 电机反馈数据结构体
typedef struct
{
    uint16_t angle;
    int16_t actual_speed;
    int16_t current;
    uint8_t temperature;

    float position; // 位置 (rad)
    float velocity; // 速度 (rad/s)
    float torque;   // 力矩 (N·m)
} Motor_Feedback;

// 电机综合控制结构体
typedef struct
{
    uint16_t id;                  // 电机ID
    FDCAN_HandleTypeDef* hfdcan;  // 所属CAN句柄
    Motor_Feedback feedback;      // 反馈数据

    PID_Controller pid_angle;     // 位置环PID
    float target_world_angle;     // 目标绝对角度

    float target_total_angle;     // 目标多圈累计角度 (连续绝对值)
    int32_t round_count;          // 累计圈数
    uint16_t last_feedback_angle; // 上一次单圈机械角度 (0-8191)
    float current_total_angle;    // 当前多圈累计角度
    float mechanical_degree;      // 当前机械角度 (度)

    PID_Controller pid_speed;     // 速度环PID
    float target_speed;           // 速度环目标速度

    int16_t target_current;       // 目标电流值
    float target_torque;          // 目标力矩

    // 达妙 (DM) 电机专用字段
    uint16_t state;               // 电机状态字
    float last_feedback_position; // 上一次反馈位置 (rad, -12.5 ~ 12.5)
    float current_total_angle_rad;// 当前累计角度 (rad)
    float current_total_angle_deg;// 当前累计角度 (deg)

    uint32_t cnt;
} Motor_Controller;

// 机器人指示灯 LED 状态枚举
typedef enum {
    LED_STATE_BOOTING,        // 开机初始化中 (黄色)
    LED_STATE_DISABLED,       // 电机失能中   (红色)
    LED_STATE_ENABLED,        // 正常使能运行 (绿色)
    LED_STATE_CLIMB_FRONT,    // 特殊状态-前爬 (蓝色)
    LED_STATE_CLIMB_BACK,     // 特殊状态-后爬 (紫色)
    LED_STATE_BASE_HIT_READY, // 击打准备状态
    LED_STATE_RESET           // 复位中
} LED_State_e;

extern volatile LED_State_e led_state;

// 机器人参数初始化
extern void Robot_Param_Init(void);

// FDCAN 通信底层配置
extern void BSP_FDCAN_Config(void);

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define POWER_5V_Pin GPIO_PIN_15
#define POWER_5V_GPIO_Port GPIOC
#define ACC_CS_Pin GPIO_PIN_0
#define ACC_CS_GPIO_Port GPIOC
#define GYRO_CS_Pin GPIO_PIN_3
#define GYRO_CS_GPIO_Port GPIOC
#define ACC_INT_Pin GPIO_PIN_10
#define ACC_INT_GPIO_Port GPIOE
#define ACC_INT_EXTI_IRQn EXTI15_10_IRQn
#define GYRO_INT_Pin GPIO_PIN_12
#define GYRO_INT_GPIO_Port GPIOE
#define GYRO_INT_EXTI_IRQn EXTI15_10_IRQn
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOE
#define LCD_BLK_Pin GPIO_PIN_10
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_RES_Pin GPIO_PIN_11
#define LCD_RES_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_10
#define LCD_DC_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
