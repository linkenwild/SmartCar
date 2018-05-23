/******************************2017-2017, NJTECH, Edu.************************** 
FileName: imu.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    姿态解算 IMU 使用 PIT3定时器计算时间
          将传感器的输出值进行姿态解算。得到目标载体的俯仰角和横滚角 和航向角
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     文件创建   
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
volatile float exInt, eyInt, ezInt;     // 误差积分
volatile float q0, q1, q2, q3;          // 全局四元数
volatile float integralFBhand,handdiff;
volatile uint32_t lastUpdate, now;      // 采样周期计数 单位 us

volatile uint32_t usCounter = 0U;
float last_ex=0,last_ey=0,last_ez=0;

/*******************************************************************************
* Function Name  : PIT3_IRQHandler
* Description    : 定时器3 中断服务函数
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
  /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_3 , kPIT_TimerFlag);
    /*这里添加自己的代码*/
    usCounter++;
}

/*******************************************************************************
* Function Name  : micros 内部函数
* Description    : 读取系统运行的时间 从上电开始计时  单位 us
* Input          : None
* Output         : None
* Return         : 运行时间
*******************************************************************************/
static uint32_t micros(void)
{ 	
  return usCounter;
}

/*******************************************************************************
* Function Name  : imu_init 
* Description    : 初始化IMU相关：传感器 四元数 积分清零 系统时间
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
* Function Name  : IMU_getValues 内部函数
* Description    : 读取加速度 陀螺仪  的当前值 
* Input          : None
* Output         : values 结果存放的数组首地址
* Return         : None
*******************************************************************************/
static void IMU_getValues(float * values) 
{  
  int16_t accel_gyro_val[6];
  int i;
  /*读取加速度*/
  mpu6050_read_accel(accel_gyro_val);
  /*读取陀螺仪*/
  mpu6050_read_gyro(&accel_gyro_val[3]);
  for(i = 0; i<6; i++) 
  {
    if(i < 3) 
    {
      values[i] = (float) accel_gyro_val[i];
    }
    else 
    {
      /*这里已经将量程改成了 2000度每秒  16.4 对应 1度每秒*/
      values[i] = ((float) accel_gyro_val[i]) / 16.4f; //转成度每秒
    }
  }
}

/*******************************************************************************
* Function Name  : IMU_AHRSupdate 内部函数
* Description    : 更新AHRS 更新四元数
* Input          : 当前的测量值 g三向 
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
  
  now = micros();  //读取时间
  if(now < lastUpdate)//定时器溢出过了。
  { 
    halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);//半周期
  }
  else	
  {
    halfT =  ((float)(now - lastUpdate) / 2000000.0f);
  }
  lastUpdate = now;	//更新时间

  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

//  norm = sqrt(mx*mx + my*my + mz*mz);  //无       
//  mx = mx / norm;
//  my = my / norm;
//  mz = mz / norm;

  /* compute reference direction of flux*/
//  hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);//无
//  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);         
//  bx = sqrt((hx*hx) + (hy*hy));
//  bz = hz;     
  
  /* estimated direction of gravity and flux (v and w)*/
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
//  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);//无
//  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
//  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);  
  
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ; //+ (my*wz - mz*wy);//后面无
  ey = (az*vx - ax*vz) ; //+ (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) ; //+ (mx*wy - my*wx);

//if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
  {
  exInt = exInt + ex * Ki ;// * halfT;//halfT无
  eyInt = eyInt + ey * Ki ;// * halfT;	
  ezInt = ezInt + ez * Ki ;// * halfT;

  /* adjusted gyroscope measurements*/
  gx = gx + Kp*ex + exInt;
  gy = gy + Kp*ey + eyInt;
  gz = gz + Kp*ez + ezInt;

//  last_ex = ex;//无
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
* Function Name  : IMU_getQ 内部函数
* Description    : 更新四元数 返回当前的四元数组值
* Input          : 将要存放四元数的数组首地址
* Output         : None
* Return         : None
*******************************************************************************/
static void IMU_getQ(float * q) 
{
  float mygetqval[6];	//用于存放传感器转换结果的数组
  IMU_getValues(mygetqval);	 
  //将陀螺仪的测量值转成弧度每秒
  //加速度保持 ADC值　不需要转换
  IMU_AHRSupdate(mygetqval[3] * M_PI/180, mygetqval[4] * M_PI/180, mygetqval[5] * M_PI/180,
   mygetqval[0], mygetqval[1], mygetqval[2]);//,mygetqval[6],mygetqval[7],mygetqval[8]);
  
  q[0] = q0; //返回当前值
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

/*******************************************************************************
* Function Name  : imu_getYawPitchRoll 
* Description    : 更新四元数 返回当前解算后的姿态数据
* Input          : 将要存放姿态角的数组首地址
* Output         : None
* Return         : None
*******************************************************************************/
void imu_getYawPitchRoll(float * angles) 
{
  float q[4]; //　四元数
  IMU_getQ(q);
  
  //angles[0] = -atan2(2 * q[1] * q[2] + 2 * q[0] * q[3], -2 * q[2]*q[2] - 2 * q[3] * q[3] + 1)* 180/M_PI; // yaw
  angles[0] = -asin(-2 * q[1] * q[3] + 2 * q[0] * q[2])* 180/M_PI; // pitch
  angles[1] = atan2(2 * q[2] * q[3] + 2 * q[0] * q[1], -2 * q[1] * q[1] - 2 * q[2] * q[2] + 1)* 180/M_PI; // roll
  //if(angles[0]<0)angles[0]+=360.0f;
}

