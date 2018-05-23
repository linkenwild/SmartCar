/*********************2017-2017, NJUT, Edu.************************************ 
FileName: angle.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    计算姿态角 使用文件 mpu6050.c kalman_filter.c中文件
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/10     1.0     文件创建   
  *          Smartcar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                   No  Pin assignment                      |
  *          +-----------------------------+-----------------------------+
  *          |      FunctionPin            |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |                             |                             |
  *          +-----------------------------+-----------------------------+
******************************************************************************/ 
#include "board.h"
#include "fsl_pit.h"

#include "led.h"
#include "pit.h"
#include "mpu6050.h"
#include "Kalman_filter.h"
#include "angle.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* 定时器 定时时间选择依据 
CCD 扫描图像曝光时间 10ms 2幅图像更新一次 25ms
*/
#define TEST_ANGLE_TIME 100000   //定义测量时间宽度 100 ms

/*******************************************************************************s
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
* Function Name  : angle_init
* Description    : 姿态倾角测量初始化 定時時間 TEST_ANGLE_TIME
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void angle_init(void)
{
  mpu6050_init(); 
}

/*******************************************************************************
* Function Name  : angle_get
* Description    : FTM1 脉冲宽度  阻塞
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void angle_get(float *angle_temp, float *angle_final)
{
        
        int16_t accel[3];//加速度原始值
        int16_t gyro[3]; //角速度原始值
        float angle_temp_test[3]; //由加速度计算的倾斜角度
        float angle_final_test[3];//最终倾斜角度
  
        mpu6050_read_accel(accel);
        mpu6050_read_gyro(gyro);
        Angle_Calcu(accel, gyro,angle_temp_test, angle_final_test);
        for(int i=0; i<3;i++)
        {
          angle_temp[0] = angle_temp_test[0]; 
          angle_temp[1] = angle_temp_test[1]; 
          angle_temp[2] = angle_temp_test[2]; 
        }
          
         for(int i=0; i<3;i++)
        {
          angle_final[0] = angle_final_test[0]; 
          angle_final[1] = angle_final_test[1]; 
          angle_final[2] = angle_final_test[2]; 
        }
}

