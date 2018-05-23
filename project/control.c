#define OPTICAL_ENCODE_CONSTANT 100 
#define SPEED_CONTRO_PERIOD 100
#define CAR_SPEED_CONSTANT 1000.0/SPEED_CONTROL_PEIORD/OPTICAL_ENCODE_CONSTANT

//==============================================================================
//angle
#define VOLTAGE_GRAVITY 1
#define  GRAVITY_OFFSET 1
#define  GRAVITY_ANGLE_RATIO 1
#define  VOLTAGE_GYRO 1
#define  GYROSCOPE_OFFSET 1
#define  GYROSCOPE_ANGLE_RATIO 1
#define  GRAVITY_ADJUST_TIME_CONSTANT  1
#define  GYROSCOPE_ANGLE_SIGMA_FREQUENCY 1
#define  CAR_ANGLE_SET 1
#define  ANGLE_CONTROL_P 1
#define  CAR_ANGLE_SPEED_SET 1
#define  ANGLE_CONTROL_D 1
#define  ANGLE_CONTROL_OUT_MAX 1
#define  ANGLE_CONTROL_OUT_MIN 1
float  g_fGravityAngle ;
float g_fGyroscopeAngleSpeed  ;
float g_fCarAngle  ;
float g_fGyroscopeAngleIntegral  ;
float g_fCarAngle  ;
float g_fGyroscopeAngleSpeed  ;
float  g_fAngleControlOut ;

void AngleCalculate(void) 
{
  float fDeltaValue;
  g_fGravityAngle = (VOLTAGE_GRAVITY - GRAVITY_OFFSET) *  GRAVITY_ANGLE_RATIO;
  // g_fGravityAngle = 0;
  g_fGyroscopeAngleSpeed = (VOLTAGE_GYRO - GYROSCOPE_OFFSET) *  GYROSCOPE_ANGLE_RATIO;
  g_fCarAngle = g_fGyroscopeAngleIntegral;
  fDeltaValue = (g_fGravityAngle - g_fCarAngle) / GRAVITY_ADJUST_TIME_CONSTANT;
  g_fGyroscopeAngleIntegral += (g_fGyroscopeAngleSpeed + fDeltaValue) / GYROSCOPE_ANGLE_SIGMA_FREQUENCY;
}

void AngleControl(void)
{
  float fValue;
  fValue = (CAR_ANGLE_SET - g_fCarAngle) * ANGLE_CONTROL_P + (CAR_ANGLE_SPEED_SET - g_fGyroscopeAngleSpeed) * ANGLE_CONTROL_D;
  if(fValue > ANGLE_CONTROL_OUT_MAX)
  fValue = ANGLE_CONTROL_OUT_MAX;
  else if(fValue < ANGLE_CONTROL_OUT_MIN)
  fValue = ANGLE_CONTROL_OUT_MIN;
  g_fAngleControlOut = fValue;
}
//==============================================================================
//speed
#define  CAR_SPEED_CONSTANT 1
#define  CAR_SPEED_SETfDelta 1
#define  SPEED_CONTROL_P 1
#define  SPEED_CONTROL_I 1
#define  SPEED_CONTROL_PERIOD 1
float g_fCarSpeed  ;
float g_fCarSpeed  ;
float g_fSpeedControlIntegral  ;
float g_fSpeedControlOutOld  ;
float g_fSpeedControlOutNew  ;
float g_fSpeedControlOut  ;
int  g_nLeftMotorPulseSigma ;
int  g_nRightMotorPulseSigma ;
int  g_nSpeedControlPeriod ;

void SpeedControl(void) 
{
  float fDelta;
  float fP, fI;
  g_fCarSpeed = (g_nLeftMotorPulseSigma + g_nRightMotorPulseSigma) / 2;
  g_nLeftMotorPulseSigma = g_nRightMotorPulseSigma = 0;
  g_fCarSpeed *= CAR_SPEED_CONSTANT;
  fDelta = CAR_SPEED_SETfDelta - g_fCarSpeed;
  fP = fDelta * SPEED_CONTROL_P;
  fI = fDelta * SPEED_CONTROL_I;
  g_fSpeedControlIntegral += fI;
  g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  g_fSpeedControlOutNew = fP + g_fSpeedControlIntegral;
}

void SpeedControlOutput(void) 
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld;
  g_fSpeedControlOut = fValue * (g_nSpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD +  g_fSpeedControlOutOld;
}
//==============================================================================
//direction
#define DIRECTION_CONTROL_COUNT  1
#define LEFT_RIGHT_MINIMUM  1
#define  DIR_CONTROL_P 1
#define  DIR_CONTROL_D 1
#define  DIR_CONTROL_D_VALUE 1
#define  DIRECTION_OFFSET 1
#define  DIRECTION_CONTROL_PERIOD 1
float  g_fLeftVoltageSigma ;
float  g_fRightVoltageSigma ;
float  g_fDirectionControlOutOld ;
float  g_fDirectionControlOutNew ;
float  g_fDirectionControlOut ;
int  g_nDirectionControlPeriod ;

void DirectionControl(void) 
{
  float fLeftRightAdd, fLeftRightSub, fValue;
  float fDValue;
  int nLeft, nRight;
  nLeft = (int)(g_fLeftVoltageSigma /= DIRECTION_CONTROL_COUNT);
  nRight = (int)(g_fRightVoltageSigma /= DIRECTION_CONTROL_COUNT);
  g_fLeftVoltageSigma = 0;
  g_fRightVoltageSigma = 0;
  fLeftRightAdd = nLeft + nRight;
  fLeftRightSub = nLeft - nRight;
  g_fDirectionControlOutOld = g_fDirectionControlOutNew;
  if(fLeftRightAdd < LEFT_RIGHT_MINIMUM) 
  {
    g_fDirectionControlOutNew = 0;
  } 
  else   
  {
    fValue = fLeftRightSub * DIR_CONTROL_P / fLeftRightAdd;
    fDValue = DIR_CONTROL_D_VALUE-DIRECTION_OFFSET;
    fDValue *= DIR_CONTROL_D;
    fValue += fDValue;
    g_fDirectionControlOutNew = fValue;
  }
}

void DirectionControlOutput(void) 
{
  float fValue;
  fValue = g_fDirectionControlOutNew - g_fDirectionControlOutOld;
  g_fDirectionControlOut = fValue * (g_nDirectionControlPeriod + 1) /DIRECTION_CONTROL_PERIOD + g_fDirectionControlOutOld;
}
//==============================================================================
//motor
float g_fLeftMotorOut  ;
float g_fRightMotorOut  ;

void SetMotorVoltage(float fLeftVoltage, float fRightVoltage) 
{
  short nPeriod;
  int nOut;
  if(fLeftVoltage > 0) 
  {
    PWMSet(PWM1, 0);
    PWMSet(PWM2, fLeftVoltage);
  } 
  else 
  {
    PWMSet(PWM1, fLeftVoltage);
    PWMSet(PWM2, 0);
  }
  if(fRightVoltage > 0) 
  {
    PWMSet(PWM3, 0);
    PWMSet(PWM4, fRightVoltage);
  } 
  else 
  {
    PWMSet(PWM3, fRightVoltage);
    PWMSet(PWM4, 0);
  }
}

void MotorOutput(void) 
{
  float fLeft, fRight;
  fLeft = g_fAngleControlOut - g_fSpeedControlOut - g_fDirectionControlOut;
  fRight = g_fAngleControlOut - g_fSpeedControlOut + g_fDirectionControlOut;
  g_fLeftMotorOut = fLeft;
  g_fRightMotorOut = fRight;
  MotorSpeedOut();
}

void MotorSpeedOut(void) 
{
  float fLeftVal, fRightVal;
  fLeftVal = g_fLeftMotorOut;
  fRightVal = g_fRightMotorOut;
  if(fLeftVal > 0)
  fLeftVal += MOTOR_OUT_DEAD_VAL;
  else if(fLeftVal < 0)
  fLeftVal -= MOTOR_OUT_DEAD_VAL;
  if(fRightVal > 0)
  fRightVal += MOTOR_OUT_DEAD_VAL;
  else if(fRightVal < 0)
  fRightVal -= MOTOR_OUT_DEAD_VAL;
  if(fLeftVal > MOTOR_OUT_MAX)
  fLeftVal = MOTOR_OUT_MAX;
  if(fLeftVal < MOTOR_OUT_MIN)
  fLeftVal = MOTOR_OUT_MIN;
  if(fRightVal > MOTOR_OUT_MAX)
  fRightVal = MOTOR_OUT_MAX;
  if(fRightVal < MOTOR_OUT_MIN)
  fRightVal = MOTOR_OUT_MIN;
  SetMotorVoltage(fLeftVal, fRightVal);
}

#define INPUT_VOLTAGE_AVERAGE 1
#define SPEED_CONTROL_COUNT 1

#define CONTROL_PERIOD 5
int  g_n1MSEventCount ;

int  g_nSpeedControlCount ;
int  g_nDirectionControlCount ;

void TI1_OnInterrupt(void)
{
  int i;
  g_nSpeedControlPeriod ++;
  SpeedControlOutput();
  g_nDirectionControlPeriod ++;
  DirectionControlOutput();
  if(g_n1MSEventCount >= CONTROL_PERIOD)
  {
    g_n1MSEventCount = 0;
    GetMotorPulse(); 
  } 
  else if(g_n1MSEventCount == 1) 
  {
    for(i = 0; i < INPUT_VOLTAGE_AVERAGE; i ++)
    SampleInputVoltage();
  }
  else if(g_n1MSEventCount == 2)
  {
    GetInputVoltageAverage();
    AngleCalculate();
    AngleControl();
    MotorOutput();
  } 
  else if(g_n1MSEventCount == 3)
  {
    g_nSpeedControlCount ++;
    if(g_nSpeedControlCount >= SPEED_CONTROL_COUNT) 
    {
    SpeedControl();
    g_nSpeedControlCount = 0;
    g_nSpeedControlPeriod = 0;
    }
  } 
  else if(g_n1MSEventCount == 4) 
  {
    g_nDirectionControlCount ++;
    DirectionVoltageSigma();
    if(g_nDirectionControlCount >= DIRECTION_CONTROL_COUNT) 
    {
      DirectionControl();
      g_nDirectionControlCount = 0;
      g_nDirectionControlPeriod = 0;
    }
  }
}

/*视频资料（93分钟）： http://www.vcan123.com/thread-1245-1-1.html*/

uint8_t  g_n_pixel_pt0[129]={0x00}; //0-127有效，128点无效
uint8_t  g_n_pixel_pt1[129]={0x00}; //0-127有效，128点无效
float g_fthreshold0, g_fthreshold1;
uint32_t speed1, speed2;

float angle_temp[3]; //由加速度计算的倾斜角度
float angle_final[3];//最终倾斜角度

static void control(void)
{
  char str[64];
  int i;
  
  float Pitch;
  short gyro_y, accel_y;

  LED_Init();
  Key_init();
  Jumper_Init();
  ccd0_init();
  ccd1_init();
  
  i2c_oled_init();
  
  PWM_Init(PWM1);
  PWM_Init(PWM2);
  PWM_Init(PWM3);
  PWM_Init(PWM4);
  
  speed1_quad_init();
  speed2_quad_init();
  
/*------------------ Kalman计算 初始化---------------- */
  /**/
  angle_init();
  
/*--------------------------DMP计算初始化------------------------ */
  /*
  while(mpu_dmp_init())
  {
    PRINTF("MPU6050 DMP error\r\n");
  }*/
  
  PitConfig(kPIT_Chnl_0, 1000);//1ms进入中断
  
  while(1)
  {         
      LEDTog(LED1);
      
      /*--------------------------DMP计算------------------------ */
      /*每10ms运行
      if(g_nMPU_DMPCount == 0)
      {
        if(mpu_dmp_get_data(&Pitch, &gyro_y, &accel_y)==0)   //时间 1.03ms
        {

        }
        else
        {
          PRINTF("NO Pitch, Roll!!!\r\n");
          delay_ms(5);
        }
      }      */
      /*DMP计算结束-------------------------------- */
      
      /*--------------------------按键显示 辅助功能------------------------ */
      if( 0x00 == GPIO_ReadPinInput(GPIOE, 11U)) //按键S2第3个灯亮 OLED显示 串口打印
      {
          /*Kalman 打印
          sprintf(str, " %f-%f  %f-%f  %f-%f  \r\n",angle_temp[0],angle_final[0], angle_temp[1],angle_final[1],angle_temp[2],angle_final[2] );
          PRINTF(str);*/
          
          /*DMP打印
          sprintf(str, "Pitch = %f  gyro_y=%d , accel_y=%d\r\n", Pitch,  gyro_y, accel_y);
          PRINTF(str);*/
          
          /*OLED 显示*/
          oled_display_ccd_line (g_n_pixel_pt0, g_fthreshold0, g_n_pixel_pt1, g_fthreshold1);
          //SendImageData(g_n_pixel_pt0);
          
          /*速度打印
          sprintf(str, "speed1 = %d, speed2 = %d  . \r\n", speed1, speed2);
          PRINTF(str);*/
          
          //delay_short(100000);
      }
      else
              delay(); //简单延时

  }
}
/*******************************************************************************
* Function Name  : PIT0_IRQHandler
* Description    : 定时器0 中断服务函数 每 1 ms 进入中断
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PIT0_IRQHandler(void)
{
  static  int nCcd0_exposure_counter = 0;  //如果曝光时间不够 这个变量要用
  static  int nCcd1_exposure_counter = 0;  //如果曝光时间不够 这个变量要用
  
  /* Clear interrupt flag.*/
  PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
  LEDTog(LED2);
  
  /*每 1ms 计数脉冲个数*/
  speed_quad_get(&speed1, &speed2);  
  
  g_n1MSEventCount++; 
  
  if(g_n1MSEventCount >= CONTROL_PERIOD) //CONTROL_PERIOD=5
  {
    g_n1MSEventCount=0;
  }
  
  if(g_n1MSEventCount == 0)
  {
    if(0 == nCcd0_exposure_counter)
      g_fthreshold0 = CCD0_ImageCapture_Data(g_n_pixel_pt0);//CCD0 采集 760us
    else
      angle_get(angle_temp, angle_final);    //角度采集 Kalman计算
  }
  else if(g_n1MSEventCount == 1)
  {
    if(0 == nCcd0_exposure_counter)   
      CCD0_exposure(); //CCD0 曝光 380us
    else
      angle_get(angle_temp, angle_final);  //角度采集 Kalman计算
    
    nCcd0_exposure_counter++;
    if(nCcd0_exposure_counter > 1) nCcd0_exposure_counter = 0;
    
    //SpeedControl(); //计算出最后的速度控制量

  }
  else if(g_n1MSEventCount == 2) 
  {
    if(0 == nCcd1_exposure_counter)    
      g_fthreshold1 = CCD1_ImageCapture_Data(g_n_pixel_pt1);//CCD1 采集 800us
    else
      angle_get(angle_temp, angle_final);  //角度采集 Kalman计算
    
  }
  else if(g_n1MSEventCount == 3)
  {
    if(0 == nCcd1_exposure_counter)
      CCD1_exposure();//CCD1 曝光 380us
     else
      angle_get(angle_temp, angle_final); //角度采集 Kalman计算
     
    nCcd1_exposure_counter++;
    if(nCcd1_exposure_counter > 1) nCcd1_exposure_counter = 0;
    
    //DirectionControl(); //计算出最后的方向控制量
  }
  else if(g_n1MSEventCount == 4)//角度控制后要输出PWM
  {
    /*------------------ Kalman计算 ---------------- */
    /*一共758us   I2C 读两次3个数432us   Kalman 计算 257us 拷备6个数65us    */   
    angle_get(angle_temp, angle_final);    
    /* Kalman计算结束-------------------------------*/
      
    //AngleControl();  //先进行角度计算，再计算出最后的直立控制量
    //MotorOutput();     //所有的控制量相加后，PWM输出
  }
  
}
