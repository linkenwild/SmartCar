/*********************2017-2017, NJUT, Edu.************************************ 
FileName: angle.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.30
Description:    ������̬�� ʹ���ļ� mpu6050.c kalman_filter.c���ļ�
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/10     1.0     �ļ�����   
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
/* ��ʱ�� ��ʱʱ��ѡ������ 
CCD ɨ��ͼ���ع�ʱ�� 10ms 2��ͼ�����һ�� 25ms
*/
#define TEST_ANGLE_TIME 100000   //�������ʱ���� 100 ms

/*******************************************************************************s
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
* Function Name  : angle_init
* Description    : ��̬��ǲ�����ʼ�� ���r�r�g TEST_ANGLE_TIME
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
* Description    : FTM1 ������  ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void angle_get(float *angle_temp, float *angle_final)
{
        
        int16_t accel[3];//���ٶ�ԭʼֵ
        int16_t gyro[3]; //���ٶ�ԭʼֵ
        float angle_temp_test[3]; //�ɼ��ٶȼ������б�Ƕ�
        float angle_final_test[3];//������б�Ƕ�
  
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

