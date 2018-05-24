#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include <MK60D10.h>

#include "led.h"
#include "key.h"
#include "delay.h"
#include "pit.h"
#include "pdb.h"
#include "adc0.h"
#include "adc1.h"
#include "usart.h"
#include "pwm.h"
#include "i2c_ee.h"
#include "oled.h"
#include "mpu6050.h"
#include "angle.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "speed.h"
#include "speed_quad.h"
#include "ccd0.h"
#include "ccd1.h"
#include "nrf24l01.h"
#include "ov7725.h"
#include "hc_sr04.h"
#include "ov7725.h"
#include "imu.h"

#include "stdio.h"
#include "stdint.h"
#include "test.h"



void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 800000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}
void delay_short(uint32_t num)
{
    volatile uint32_t i = 0;
    for (i = 0; i < num; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

//测试LED beep ultrasonic
void test_led(void)
{
    LED_Init();
    //BEEP_Init();
    //UltraSonic_Init();
    PRINTF("测试 LED !!!\r\n ");

    while (1)
    {
        delay();
        delay();
        delay();
        delay();
        delay();
        delay();
        delay();
        LEDTog(LED1);

        //BEEPTog();
        //UltraSonicTog();
        
    }
}
//测试按键
void test_key(void)
{
    Key_init();
    Jumper_Init();
    PRINTF("Get_Jumper_value = 0x%02x .\r\n ",Get_Jumper_value());
    Key_Def keyvlaue = KEYNULL;
    PRINTF("测试 按键 !!!\r\n ");
    while(1)
    {
       keyvlaue = KEY_Scan();
       if(keyvlaue!=KEYNULL)
         PRINTF("keyvlaue = %d .\r\n ",keyvlaue);
    }
}
//测试延时systick
void test_delay(void)
{
    LED_Init();
    PRINTF("测试 延时systick !!!\r\n ");
    while (1)
    {
//        delay_ms(1000);
//        delay_us(500000);
//        Delay(8000000); //与下一句比较时间短一些
        delay_short(8000000);  //与上一句比较时间长一些
        LEDTog(LED1);
        LEDTog(LED2);
        LEDTog(LED3);
        LEDTog(LED4);
    }
}

//测试定时器1 配置LED1每隔半秒闪一次
void test_pit(void)
{
    LED_Init();

    PRINTF("测试 定时器1 LED1半秒闪一次!!!\r\n ");
    PitConfig(kPIT_Chnl_1, 500000);
    
    while(1)
    {
      delay();
    }
}


//测试ADC 
void test_adc(void)
{
    uint8_t   advalue;

    PRINTF("测试 2路 ADC!!!\r\n ");

    ADC0_init();
    ADC1_init();
    
    while(1)
    {      
        delay();
       getadc0value(&advalue);
       getadc1value(&advalue);
    }
}
//测试uart
void test_uart(void)
{
    //BEEP_Init();
    UartConfig();
    PRINTF("测试 UART 接收数据后打印出来!!!\r\n ");
    while(1)
    {      
	//BEEPTog();
        printuart();
    }
}
//测试pwm 舵机使用PIT0
void test_pwm(void)
{
    int i = 20, dir = 0;
    PWM_Init(PWM1);
    PWM_Init(PWM2);
    PWM_Init(PWM3);
    PWM_Init(PWM4);
    //PWM_Init(STEERPWM);
    
    PWMSet(PWM1, 20 );
    PWMSet(PWM2, 20 );
    PWMSet(PWM3, 20 );
    PWMSet(PWM4, 20 );
    //PWMSet(STEERPWM, 50 );
    PRINTF("测试 pwm 输出电机 舵机使用PIT0!\r\n ");
    while(1)
    {      
        delay();
        if(dir==0)
        {
            PWMSet(PWM1, i++ );
            PWMSet(PWM2, i++ );
            PWMSet(PWM3, i++ );
            PWMSet(PWM4, i++ );
        }
        else
        {
            PWMSet(PWM1, i-- );
            PWMSet(PWM2, i-- );
            PWMSet(PWM3, i-- );
            PWMSet(PWM4, i-- );
        }
        if(i>80) dir= 1;
        if(i<20) dir = 0;
    }
}
//测试 OLED i2c0 addr:0x3C
void test_oled(void)
{
  uint8_t i;
  LED_Init();
  delay();delay();delay();delay();delay();
  PRINTF("测试 OLED!\r\n ");
  i2c_oled_init();
  
  test_i2c_oled();

    while(1)
    {      
        delay();delay();delay();
        LEDTog(LED1);
        fill_picture(i++);
    }
}


//测试 camera
void test_camera(void)
{
  LED_Init();
  Key_init();
  delay();
  PRINTF("测试 camera\r\n ");
  camera_init();  
    while(1)
    {
      delay();
    }
}

//测试 e2prom i2c0 addr:0x50
void test_e2prom(void)
{
  
  i2c_ee_init();
  PRINTF("测试 24C02!\r\n ");
  eep_self_test(0,256);	
  while(1)
  {
    delay();
  }
}
//测试 mpu6050 i2c0 addr:0x68
void test_mpu6050(void)
{
  LED_Init();
  PRINTF("测试 MPU6050!\r\n ");
  mpu6050_init();
    while(1)
    {      
        delay();
        LEDTog(LED2);
        mpu6050_getdata();
    }
}

//测试 mpu6050 求角度 用卡尔曼滤波 精确计时
void test_mpu6050_angle(void)
{
  char str[64];
  float angle_temp[3]; //由加速度计算的倾斜角度
  float angle_final[3];//最终倾斜角度
  PRINTF("测试 MPU6050 kalman!\r\n ");
  LED_Init();
  angle_init();
    while(1)
    {      
        delay_ms(1);
        LEDTog(LED1);
        /* 一共780us
        I2C 读两次3个数468us 
        Kalman 计算 257us
        拷备6个数65us       
        */
        angle_get(angle_temp, angle_final);
        
        sprintf(str, " %f-%f  %f-%f  %f-%f\r\n",angle_temp[0],angle_final[0], angle_temp[1],angle_final[1],angle_temp[2],angle_final[2]);
        PRINTF(str);
    }
}

//测试 mpu6050 dmp addr:0x68
void test_mpu_dmp(void)
{
  float Pitch;
  short gyro_y, accel_y;
  char str[64];
  LED_Init();
  delay();delay();
  PRINTF("测试 MPU DMP!\r\n ");
  while(mpu_dmp_init())
  {
    PRINTF("MPU6050 DMP error\r\n");
  }
  while(1)
  {      
    if(mpu_dmp_get_data(&Pitch, &gyro_y, &accel_y)==0)   //时间 1.03ms
    {
      sprintf(str, "Pitch = %f   gyro_y=%d , accel_y=%d\r\n", Pitch,  gyro_y, accel_y);
      PRINTF(str);
    }
    else
    {
      PRINTF("NO Pitch, Roll!!!\r\n");
      delay_ms(5);
    }
    delay_ms(5);
    LEDTog(LED1);
  }
}

//测试 mpu6050 dmp addr:0x68
void test_mpu_imu()
{
  float pitch_roll_value[2];
  char str[64];
  LED_Init();
  PRINTF("测试 MPU imu!\r\n ");
  imu_init();
  while(1)
  {      
    imu_getYawPitchRoll(pitch_roll_value);
    sprintf(str, "Pitch = %f Roll = %f  \r\n", pitch_roll_value[0], pitch_roll_value[1]);
    PRINTF(str);
    delay();
    LEDTog(LED1);
  }
}

//测试 speed 得到脉冲宽度 us单位
void test_speed(void)
{
  PRINTF("测试 speed!\r\n ");
  LED_Init();
  speed1_init();
  speed2_init();
 
    while(1)
    {      
        PRINTF("测试 speed1 脉宽 = %d us   ",speed1_get());
        PRINTF("测试 speed2 脉宽= %dus\r\n ",speed2_get());
        delay();
        LEDTog(LED1);
    }
}

//测试 speed 得到脉冲計數
void test_speed_quad(void)
{
  char str[64];
  PRINTF("测试 speed_quad!\r\n ");
  LED_Init();
  i2c_oled_init(); 
  delay();delay();delay();
  fill_picture(0x00);
  speed1_quad_init();
  speed2_quad_init();
 
    while(1)
    {      
        sprintf(str, "%d   ", speed1_quad_get());
        PRINTF("测试 speed1 計數 = %s   ",str);
        OLED_ShowString(1, 20, 8, str);
        sprintf(str, "%d   ", speed2_quad_get());
        PRINTF("测试 speed2 計數 = %s\r\n ",str);
        OLED_ShowString(4, 20, 8, str);
        delay();
        LEDTog(LED1);
    }
}


//测试 ccd 使用adc0
void test_ccd(void)
{
  uint8_t  pixel_pt0[129]={0x00}; //0-127有效，128点无效
  uint8_t  pixel_pt1[129]={0x00}; //0-127有效，128点无效
  float threshold0, threshold1;
  PRINTF("测试 CCD0!\r\n ");
  LED_Init();
  Key_init();
  Jumper_Init();
  i2c_oled_init();  
  ccd0_init();
  //ccd1_init();
  while(1)
  {      
      LEDTog(LED1);
      threshold0 = ImageCapture(pixel_pt0);//一轮11ms
      //hreshold1 = ccd1_ImageCapture(pixel_pt1);//一轮11ms
       
      if( 0x00 == GPIO_ReadPinInput(GPIOE, 11U)) //第3个灯亮
      {
        //串口发送数据
        //SendImageData(pixel_pt0);
        //SendImageData(pixel_pt1);
        
        //OLED显示2路CCD数据
        oled_display_ccd_line (pixel_pt0, threshold0, pixel_pt1, threshold1);
      }
    
  }
}

//测试 nrf24l01 
void test_nrf24l01(void)
{
  PRINTF("测试 nrf24l01!\r\n ");
  rf1start();
}

//测试 hc_sr04 
void test_hc_sr04(void)
{
  char str[256];
  PRINTF("测试 hc_sr04!\r\n ");
  LED_Init();
  i2c_oled_init(); 
  delay();delay();delay();
  fill_picture(0x00);
  hc_sc04_init();
  while(1)
  {      
      delay();delay();delay();
      PRINTF("ultrasonic_width = %d.%d cm  \r\n ", ultrasonic_width_get()/1000,ultrasonic_width_get()%1000);//串口打印出距离
      sprintf(str, "len=%d.%d cm  ", ultrasonic_width_get()/1000,ultrasonic_width_get()%1000);
      OLED_ShowString(2, 0, 8, str);
  }
}