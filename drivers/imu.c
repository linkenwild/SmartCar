/******************************2017-2017, NJTECH, Edu.************************** 
FileName: imu.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.30
Description:    ��̬���� IMU ʹ�� PIT3��ʱ������ʱ��
          �������������ֵ������̬���㡣�õ�Ŀ������ĸ����Ǻͺ���� �ͺ����
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     �ļ�����   
  *          SmartCar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     NO Pin assignment                     |
  *          +-----------------------------+-----------------------------+
  *          |                             |                             |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 
#include "board.h"
#include "fsl_pit.h"

#include "pit.h"
#include "mpu6050.h"
#include <math.h>
#include "imu.h"
#include "delay.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define M_PI  (float)3.1415926535
#define Kp 0.8f   // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki 0.01f   // integral gain governs rate of convergence of gyroscope biases

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile float exInt, eyInt, ezInt;     // ������
volatile float q0, q1, q2, q3;          // ȫ����Ԫ��
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now;      // �������ڼ��� ��λ us

volatile uint32_t usCounter = 0U;
float last_ex=0,last_ey=0,last_ez=0;

/*******************************************************************************
* Function Name  : PIT3_IRQHandler
* Description    : ��ʱ��3 �жϷ�����
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
  /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_3 , kPIT_TimerFlag);
    /*��������Լ��Ĵ���*/
    usCounter++;
}

/*******************************************************************************
* Function Name  : micros �ڲ�����
* Description    : ��ȡϵͳ���е�ʱ�� ���ϵ翪ʼ��ʱ  ��λ us
* Input          : None
* Output         : None
* Return         : ����ʱ��
*******************************************************************************/
static uint32_t micros(void)
{ 	
  return usCounter;
}

/*******************************************************************************
* Function Name  : imu_init 
* Description    : ��ʼ��IMU��أ������� ��Ԫ�� �������� ϵͳʱ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void imu_init(void)
{	 
	mpu6050_init();
	delay_ms(50);
	PitConfig(kPIT_Chnl_3, 1U);
  	q0 = 1.0f;
  	q1 = 0.0f;
  	q2 = 0.0f;
  	q3 = 0.0f;
  	exInt = 0.0;
  	eyInt = 0.0;
  	ezInt = 0.0;
  	lastUpdate = micros();
  	now = micros();;
}

/*******************************************************************************
* Function Name  : IMU_getValues �ڲ�����
* Description    : ��ȡ���ٶ� ������  �ĵ�ǰֵ 
* Input          : None
* Output         : values �����ŵ������׵�ַ
* Return         : None
*******************************************************************************/
static void IMU_getValues(float * values) 
{  
  int16_t accel_gyro_val[6];
  int i;
  /*��ȡ���ٶ�*/
  mpu6050_read_accel(accel_gyro_val);
  /*��ȡ������*/
  mpu6050_read_gyro(&accel_gyro_val[3]);
  for(i = 0; i<6; i++) 
  {
    if(i < 3) 
    {
      values[i] = (float) accel_gyro_val[i];
    }
    else 
    {
      /*�����Ѿ������̸ĳ��� 2000��ÿ��  16.4 ��Ӧ 1��ÿ��*/
      values[i] = ((float) accel_gyro_val[i]) / 16.4f; //ת�ɶ�ÿ��
    }
  }
}

/*******************************************************************************
* Function Name  : IMU_AHRSupdate �ڲ�����
* Description    : ����AHRS ������Ԫ��
* Input          : ��ǰ�Ĳ���ֵ g���� 
* Output         : None
* Return         : None
*******************************************************************************/
static void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az)//, float mx, float my, float mz) 
{
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez,halfT;

  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;          
  
  now = micros();  //��ȡʱ��
  if(now < lastUpdate)//��ʱ��������ˡ�
  { 
    halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);//������
  }
  else	
  {
    halfT =  ((float)(now - lastUpdate) / 2000000.0f);
  }
  lastUpdate = now;	//����ʱ��

  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

//  norm = sqrt(mx*mx + my*my + mz*mz);  //��       
//  mx = mx / norm;
//  my = my / norm;
//  mz = mz / norm;

  /* compute reference direction of flux*/
//  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);//��
//  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
//  bx = sqrt((hx*hx) + (hy*hy));
//  bz = hz;     
  
  /* estimated direction of gravity and flux (v and w)*/
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
//  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);//��
//  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ; //+ (my*wz - mz*wy);//������
  ey = (az*vx - ax*vz) ; //+ (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) ; //+ (mx*wy - my*wx);

//if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
  exInt = exInt + ex * Ki ;// * halfT;//halfT��
  eyInt = eyInt + ey * Ki ;// * halfT;	
  ezInt = ezInt + ez * Ki ;// * halfT;

  /* adjusted gyroscope measurements*/
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

//  last_ex = ex;//��
//  last_ey = ey;
//  last_ez = ez;
  }

  /* integrate quaternion rate and normalise*/
  q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
  q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
  q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
  q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  
  
  /* normalise quaternion */
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
}

/*******************************************************************************
* Function Name  : IMU_getQ �ڲ�����
* Description    : ������Ԫ�� ���ص�ǰ����Ԫ����ֵ
* Input          : ��Ҫ�����Ԫ���������׵�ַ
* Output         : None
* Return         : None
*******************************************************************************/
static void IMU_getQ(float * q) 
{
  float mygetqval[6];	//���ڴ�Ŵ�����ת�����������
  IMU_getValues(mygetqval);	 
  //�������ǵĲ���ֵת�ɻ���ÿ��
  //���ٶȱ��� ADCֵ������Ҫת��
  IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2]);//,mygetqval[6],mygetqval[7],mygetqval[8]);
  
  q[0] = q0; //���ص�ǰֵ
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

/*******************************************************************************
* Function Name  : imu_getYawPitchRoll 
* Description    : ������Ԫ�� ���ص�ǰ��������̬����
* Input          : ��Ҫ�����̬�ǵ������׵�ַ
* Output         : None
* Return         : None
*******************************************************************************/
void imu_getYawPitchRoll(float * angles) 
{
  float q[4]; //����Ԫ��
  IMU_getQ(q);
  
  //angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  angles[0] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
  //if(angles[0]<0)angles[0]+=360.0f;
}

