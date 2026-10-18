#include "main.h"
#include "cmsis_os.h"
#include "BMI088driver.h"
#include "gpio.h"
#include "tim.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "INS_task.h"
#include "MahonyAHRS.h"
#include "bsp_buzzer.h"
#include "pid.h"


#define USE_HARDCODED_BIAS 1         //0:开机静态采样零漂  1:硬编码零偏，开机秒启动

#define cheat TRUE                   //作弊模式 去掉较小的gyro值
#define correct_Time_define 10000     //上电去0飘 10000次取平均
#define temp_times 300               //探测温度阈值
float RefTemp = 45.0f;               //目标温度




#if (USE_HARDCODED_BIAS == 1)
    //硬编码零偏
    volatile uint8_t imu_ready_flag = 1; 
    uint8_t attitude_flag = 3;//跳过温控等待和采样，直接进解算

    float gyro_correct[3]={-6.73255545e-06f, 4.34518988e-05f,  0.00308083929f}; //roll,pitch,yaw
#else
    //静态采样零漂
    volatile uint8_t imu_ready_flag = 0; 
    uint8_t attitude_flag = 0; 
    float gyro_correct[3] = {0.0f, 0.0f, 0.0f}; 
#endif




pid_struct_t Temperature_PID={0};
float gyro[3], accel[3], temp;
float roll,pitch,yaw=0;
float total_yaw;
float last_yaw;
int32_t turnCounts;
uint32_t correct_times=0;

static void INS_Init(void)
{
	
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf初始化
		pid_init(&Temperature_PID,1600,50,40,200,2000); //加热pidlimit
	
		Mahony_Init(1000);  //mahony姿态解算初始化
    // imu heat init
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    while(BMI088_init());  //陀螺仪初始化
}

uint32_t temp_temperature=0;
static void IMU_Temperature_Ctrl()
{
	  pid_calc(&Temperature_PID,RefTemp,temp); //温度pid
		temp_temperature=(uint32_t)Temperature_PID.output; 
		if(Temperature_PID.output<0)
		{
			temp_temperature=0;
		}

    htim3.Instance->CCR4 = temp_temperature;
}
/***
 * @brief: INS_TASK(void const * argument)
 * @param: argument - 任务参数
 * @retval: void
 * @details: IMU姿态控制任务函数
 
*/
static uint8_t first_mahony=0; 
static void INS_Task(void)
{
    static uint32_t count = 0;

    //1. 读取原始数据
		BMI088_read(gyro, accel, &temp);
	
	  // 2.首次运行初始化四元数 (防止一开始数据没准备好)
		if(first_mahony==0)
		{
			first_mahony++;
			MahonyAHRSinit(accel[0],accel[1],accel[2],0,0,0);  
		}
		// 3.进入正常解算状态
		if(attitude_flag==3)  //ekf的姿态解算
		{				
			gyro[0]-=gyro_correct[0];   //减去陀螺仪0飘
			gyro[1]-=gyro_correct[1];
			gyro[2]-=gyro_correct[2];
			
			
			#if cheat              //作弊 可以让yaw很稳定 去掉比较小的值
				if(fabsf(gyro[2])<0.003f)
					gyro[2]=0;
			#endif
			//===========================================================================
				//ekf姿态解算部分
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
			IMU_QuaternionEKF_Update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2]);
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
			//===============================================================================	
						
			//=================================================================================
			//mahony姿态解算部分
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
			Mahony_update(gyro[0],gyro[1],gyro[2],accel[0],accel[1],accel[2],0,0,0);
			Mahony_computeAngles(); //角度计算   移植到别的平台需要替换掉对应的arm_atan2_f32 和 arm_asin
			//HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
			//=============================================================================
			//ekf获取姿态角度函数
			pitch=-Get_Roll() + 180.0f; //获得pitch
			roll=Get_Pitch() + 180.0f;//获得roll
			yaw=Get_Yaw() + 180.0f;//获得yaw
				
			total_yaw = Get_Total_Yaw() + 180.0f;
			
			
			imu_ready_flag = 1;//数据准备就绪
		}
		else if(attitude_flag==2)   //状态1 开始1000次的陀螺仪0飘初始化
		{
				//gyro correct
				gyro_correct[0]+=	gyro[0];
				gyro_correct[1]+=	gyro[1];
				gyro_correct[2]+=	gyro[2];
				correct_times++;
				if(correct_times>=correct_Time_define)
				{
					gyro_correct[0]/=correct_Time_define;
					gyro_correct[1]/=correct_Time_define;
					gyro_correct[2]/=correct_Time_define;
					attitude_flag=3; //go to 2 state
					
//					Set_BuzzerMode(ZERO_TIME);//陀螺仪校准结束，蜂鸣器不叫
					imu_ready_flag = 0;
				}
		}

    // temperature control
		count++;
    if(count >= 10)
    {
   			count = 0;
			
				if(attitude_flag < 2)
				{
					attitude_flag = 1;
					
//					Set_BuzzerMode(ONE_TIME);//陀螺仪校准中，蜂鸣器持续叫
				}
        // 100hz 的温度控制pid
        IMU_Temperature_Ctrl();
				
				
				
				static uint32_t total_wait_time = 0;
				total_wait_time++; 
        //注意：这里进一次是 10ms
        //1000次 = 10秒, 2000次 = 20秒
				
            float current_tolerance = 0.5f;

            if(total_wait_time < 3000) 
            {
                current_tolerance = 0.5f;
            }
            else if (total_wait_time < 4000) 
            {
                current_tolerance = 3.0f; 
            }
            else 
            {
                current_tolerance = 20.0f;
            }

            static uint32_t temp_Ticks = 0;
            
						if((fabsf(temp-RefTemp)<current_tolerance) && attitude_flag==1)//接近目标温度 开始计数
						{
							temp_Ticks++;
			
							if(temp_Ticks > temp_times)
							{
								attitude_flag= 2;//go to correct state
							}
						}				
            else
            {
                temp_Ticks = 0; 
            }	
    }
}



void IMU_task(void *argument)
{
	  osDelay (500);
	
    INS_Init();
	
	  TickType_t xLastWakeTime;
	
	  const TickType_t xFrequency = pdMS_TO_TICKS(1); 
	
	  xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for (;;)
    {
        INS_Task();
			
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


/**
************************************************************************
* @brief:      	HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
* @param:       GPIO_Pin - 触发中断的GPIO引脚
* @retval:     	void
* @details:    	GPIO外部中断回调函数，处理加速度计和陀螺仪中断
************************************************************************
**/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ACC_INT_Pin)
    {
    }
    else if(GPIO_Pin == GYRO_INT_Pin)
    {

    }

}


