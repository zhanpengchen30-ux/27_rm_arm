/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
#include "INS_task.h"
#include "bsp_DR16.h"
#include "bsp_buzzer.h"
#include "arm_task.h"

//#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */
//RC_ctrl_t *rc; 
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */



/* USER CODE END Variables */
/* Definitions for All_Init */
osThreadId_t All_InitHandle;
const osThreadAttr_t All_Init_attributes = {
  .name = "All_Init",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
uint32_t IMU_Buffer[ 1024 ];
osStaticThreadDef_t IMUControlBlock;
const osThreadAttr_t INSTask_attributes = {
  .name = "INSTask",
  .cb_mem = &IMUControlBlock,
  .cb_size = sizeof(IMUControlBlock),
  .stack_mem = &IMU_Buffer[0],
  .stack_size = sizeof(IMU_Buffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for CanDecodeTask */
osThreadId_t CanDecodeTaskHandle;
const osThreadAttr_t CanDecodeTask_attributes = {
  .name = "CanDecodeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for ArmTask */
osThreadId_t ArmTaskHandle;
const osThreadAttr_t ArmTask_attributes = {
  .name = "ArmTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rcTask */
osThreadId_t rcTaskHandle;
const osThreadAttr_t rcTask_attributes = {
  .name = "rcTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for rcQueue */
osMessageQueueId_t rcQueueHandle;
const osMessageQueueAttr_t rcQueue_attributes = {
  .name = "rcQueue"
};
/* Definitions for canRxQueue */
osMessageQueueId_t canRxQueueHandle;
const osMessageQueueAttr_t canRxQueue_attributes = {
  .name = "canRxQueue"
};
/* Definitions for imuBinarySem01 */
osSemaphoreId_t imuBinarySem01Handle;
const osSemaphoreAttr_t imuBinarySem01_attributes = {
  .name = "imuBinarySem01"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void All_Init_Run(void *argument);
extern void IMU_task(void *argument);
extern void Can_Decode_Task(void *argument);
extern void Arm_Task(void *argument);
void RC_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of imuBinarySem01 */
  imuBinarySem01Handle = osSemaphoreNew(1, 1, &imuBinarySem01_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of rcQueue */
  rcQueueHandle = osMessageQueueNew (1, 36, &rcQueue_attributes);

  /* creation of canRxQueue */
  canRxQueueHandle = osMessageQueueNew (20, sizeof(CAN_Rx_Message_t), &canRxQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of All_Init */
  All_InitHandle = osThreadNew(All_Init_Run, NULL, &All_Init_attributes);

  /* creation of INSTask */
  INSTaskHandle = osThreadNew(IMU_task, NULL, &INSTask_attributes);

  /* creation of CanDecodeTask */
  CanDecodeTaskHandle = osThreadNew(Can_Decode_Task, NULL, &CanDecodeTask_attributes);

  /* creation of rcTask */
  rcTaskHandle = osThreadNew(RC_Task, NULL, &rcTask_attributes);

  /* creation of ArmTask */
  ArmTaskHandle = osThreadNew(Arm_Task, NULL, &ArmTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_All_Init_Run */
/**
  * @brief  Function implementing the All_Init thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_All_Init_Run */
void All_Init_Run(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN All_Init_Run */
	
	osDelay(100);

	MX_USB_DEVICE_Init();
	
	taskENTER_CRITICAL();  //�����ٽ���
	
	BSP_FDCAN_Config();
	
	DR16_Init();
	
	taskEXIT_CRITICAL();   //�˳��ٽ���
	
	vTaskDelete(NULL);  
  /* USER CODE END All_Init_Run */
}

/* USER CODE BEGIN Header_RC_Task */
/**
* @brief Function implementing the rcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RC_Task */
__weak void RC_Task(void *argument)
{
  /* USER CODE BEGIN RC_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END RC_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

