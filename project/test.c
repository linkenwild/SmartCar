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
#include "adc_pdb.h"

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

//����LED beep ultrasonic
void test_led(void)
{
    LED_Init();
    //BEEP_Init();
    //UltraSonic_Init();
    PRINTF("���� LED !!!\r\n ");

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
//���԰���
void test_key(void)
{
    Key_init();
    Jumper_Init();
    PRINTF("Get_Jumper_value = 0x%02x .\r\n ",Get_Jumper_value());
    Key_Def keyvlaue = KEYNULL;
    PRINTF("���� ���� !!!\r\n ");
    while(1)
    {
       keyvlaue = KEY_Scan();
       if(keyvlaue!=KEYNULL)
         PRINTF("keyvlaue = %d .\r\n ",keyvlaue);
    }
}
//������ʱsystick
void test_delay(void)
{
    LED_Init();
    PRINTF("���� ��ʱsystick !!!\r\n ");
    while (1)
    {
//        delay_ms(1000);
//        delay_us(500000);
//        Delay(8000000); //����һ��Ƚ�ʱ���һЩ
        delay_short(8000000);  //����һ��Ƚ�ʱ�䳤һЩ
        LEDTog(LED1);
        LEDTog(LED2);
        LEDTog(LED3);
        LEDTog(LED4);
    }
}

//���Զ�ʱ��1 ����LED1ÿ��������һ��
void test_pit(void)
{
    LED_Init();

    PRINTF("���� ��ʱ��1 LED1������һ��!!!\r\n ");
    PitConfig(kPIT_Chnl_1, 500000);
    
    while(1)
    {
      delay();
    }
}


//����ADC 
void test_adc(void)
{
    uint8_t   advalue;

    PRINTF("���� 2· ADC!!!\r\n ");

    ADC0_init();
    ADC1_init();
    
    while(1)
    {      
        delay();
       getadc0value(&advalue);
       getadc1value(&advalue);
    }
}

//����AIN4, 5
void test_ain(void)
{
    uint32_t   ad0value[4];
    uint32_t   ad1value[4];
    char str[256];

    PRINTF("���� PDB ADC DMA!!!\r\n ");
  Key_init();
  LED_Init();
  i2c_oled_init();   
 OLED_Clear(0);  
    PDB_init();
    AIN0_init();
    AIN1_init();
    PitConfig(kPIT_Chnl_1, 40);//��ʱʱ�䣺40us   > = 40

   
    while(1)
    {
      delay_ms(200);
      get_pdb_adc0_value(ad0value);
      get_pdb_adc1_value(ad1value);
      PRINTF("\r\n");
      {
        for(int i=0; i<4; i++)
        {
          sprintf(str, "%d %d %d %d     ", ad0value[0], ad0value[1], ad1value[0], ad1value[1]);
          OLED_ShowString(0,0,8,str);
          
          sprintf(str, "%d %d %d %d     ", ad0value[2], ad0value[3], ad1value[2], ad1value[3]);
          OLED_ShowString(4,0,8,str);
          
        }
        
      }
      
    }
}


//����uart
void test_uart(void)
{
    //BEEP_Init();
    UartConfig();
    PRINTF("���� UART �������ݺ��ӡ����!!!\r\n ");
    while(1)
    {      
	//BEEPTog();
        printuart();
    }
}
//����pwm ���ʹ��PIT0
void test_pwm(void)
{
    int i = 20, dir = 0;
    char str[64];
    PWM_Init(PWM1);
    PWM_Init(PWM2);
    PWM_Init(PWM3);
    PWM_Init(PWM4);
    PWM_Init(STEERPWM);
    
    PWMSet(PWM1, 20 );
    PWMSet(PWM2, 20 );
    PWMSet(PWM3, 20 );
    PWMSet(PWM4, 20 );
    PWMSet(STEERPWM, 50 );
    i2c_oled_init();
    OLED_Clear(0);
    PRINTF("���� pwm ������ ���ʹ��PIT0!\r\n ");
    while(1)
    {      
        delay();
        if(dir==0)
        {
            PWMSet(PWM1, i++ );
            PWMSet(PWM2, i++ );
            PWMSet(PWM3, i++ );
            PWMSet(PWM4, i++ );
            PWMSet(STEERPWM, i );
        }
        else
        {
            PWMSet(PWM1, i-- );
            PWMSet(PWM2, i-- );
            PWMSet(PWM3, i-- );
            PWMSet(PWM4, i-- );
            PWMSet(STEERPWM, i );
        }
        if(i>80) dir= 1;
        if(i<20) dir = 0;
        sprintf (str, "pwm = %d     ", i);
        OLED_ShowString(2, 20, 8, str);
    }
}
//���� OLED i2c0 addr:0x3C
void test_oled(void)
{
  uint8_t i;
  LED_Init();
  delay();delay();delay();delay();delay();
  PRINTF("���� OLED!\r\n ");
  i2c_oled_init();
  
  test_i2c_oled();

    while(1)
    {      
        delay();delay();delay();
        LEDTog(LED1);
        fill_picture(i++);
    }
}


//���� camera
void test_camera(void)
{
  LED_Init();
  Key_init();
  delay();
  PRINTF("���� camera\r\n ");
  camera_init();  
    while(1)
    {
      delay();
    }
}

//���� e2prom i2c0 addr:0x50
void test_e2prom(void)
{
  
  i2c_ee_init();
  PRINTF("���� 24C02!\r\n ");
  eep_self_test(0,256);	
  while(1)
  {
    delay();
  }
}
//���� mpu6050 i2c0 addr:0x68
void test_mpu6050(void)
{
  int16_t a_data[3];
  int16_t g_data[3];
  int16_t temperature;
  uint8_t str[64];
  
  LED_Init();
  i2c_oled_init();   
  OLED_Clear(0);      
  PRINTF("���� MPU6050!\r\n ");
  mpu6050_init();
 while(1)
    {      
        delay_ms(100);
        LEDTog(LED2);
        mpu6050_getdata(a_data, g_data, &temperature);
        
        sprintf(str, "%d;%d;%d",a_data[0], a_data[1], a_data[2]);
        OLED_ShowString(0,0,8,str);
        
        sprintf(str, "%d;%d;%d",g_data[0], g_data[1], g_data[2]);
        OLED_ShowString(4,0,8,str);
        
        sprintf(str, "t = %d",temperature/100);
        OLED_ShowString(6,64,8,str);     
       
    }
}

//���� mpu6050 ��Ƕ� �ÿ������˲� ��ȷ��ʱ
void test_mpu6050_angle(void)
{
  char str[64];
  float angle_temp[3]; //�ɼ��ٶȼ������б�Ƕ�
  float angle_final[3];//������б�Ƕ�
  i2c_oled_init();   
  OLED_Clear(0); 
  
  
  PRINTF("���� MPU6050 kalman!\r\n ");
  LED_Init();
  angle_init();
    while(1)
    {      
        delay_ms(1);
        LEDTog(LED1);
        /* һ��780us
        I2C ������3����468us 
        Kalman ���� 257us
        ����6����65us       
        */
        angle_get(angle_temp, angle_final);
        
        sprintf(str, " %f-%f  %f-%f  %f-%f\r\n",angle_temp[0],angle_final[0], angle_temp[1],angle_final[1],angle_temp[2],angle_final[2]);
        PRINTF(str);
        sprintf(str, " %.3f  %.3f  %.3f",angle_temp[0], angle_temp[1],angle_temp[2]);
        OLED_ShowString(0,0,8,str);
    }
}

//���� mpu6050 dmp addr:0x68
void test_mpu_dmp(void)
{
  float Pitch;
  short gyro_y, accel_y;
  char str[64];
  LED_Init();
  delay();delay();
  i2c_oled_init();   
  OLED_Clear(0); 
  PRINTF("���� MPU DMP!\r\n ");
  while(mpu_dmp_init())
  {
    PRINTF("MPU6050 DMP error\r\n");
  }
  while(1)
  {      
    if(mpu_dmp_get_data(&Pitch, &gyro_y, &accel_y)==0)   //ʱ�� 1.03ms
    {
      sprintf(str, "Pitch = %f   gyro_y=%d , accel_y=%d\r\n", Pitch,  gyro_y, accel_y);
      PRINTF(str);
      sprintf(str, "Pitch=%.3f  ", Pitch);
      OLED_ShowString(0,0,8,str);
      sprintf(str, "gyro_y=%d", gyro_y);
      OLED_ShowString(2,0,8,str);
      sprintf(str, "accel_y=%d", accel_y);
      OLED_ShowString(4,0,8,str);
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

//���� mpu6050 dmp addr:0x68
void test_mpu_imu()
{
  float pitch_roll_value[2];
  char str[64];
  LED_Init();
  PRINTF("���� MPU imu!\r\n ");
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

//���� speed �õ�������� us��λ
void test_speed(void)
{
  char str[64];
  PRINTF("���� speed!\r\n ");
  LED_Init();
  i2c_oled_init(); 
  speed1_init();
  speed2_init();
  OLED_Clear(0);
 
    while(1)
    {      
        sprintf(str, "%d  us   ",speed1_get());
        PRINTF("���� speed1 ���� = %s    ",str);
        OLED_ShowString(1, 20, 8, str);
        sprintf(str, "%d  us    ",speed2_get());
        PRINTF("���� speed2 ���� = %s    \r\n",str);
        OLED_ShowString(4, 20, 8, str);
        delay();
        LEDTog(LED1);
    }
}

//���� speed �õ�����Ӌ��
void test_speed_quad(void)
{
  char str[64];
  PRINTF("���� speed_quad!\r\n ");
  LED_Init();
  i2c_oled_init(); 
  delay();delay();delay();
  speed1_quad_init();
  speed2_quad_init();
  OLED_Clear(0);
 
    while(1)
    {      
        sprintf(str, "%d   ", speed1_quad_get());
        PRINTF("���� speed1 Ӌ�� = %s   ",str);
        OLED_ShowString(1, 20, 8, str);
        sprintf(str, "%d   ", speed2_quad_get());
        PRINTF("���� speed2 Ӌ�� = %s\r\n ",str);
        OLED_ShowString(4, 20, 8, str);
        delay();
        LEDTog(LED1);
    }
}


//���� ccd ʹ��adc0
void test_ccd(void)
{
  uint8_t  pixel_pt0[129]={0x00}; //0-127��Ч��128����Ч
  uint8_t  pixel_pt1[129]={0x00}; //0-127��Ч��128����Ч
  float threshold0, threshold1;
  PRINTF("���� CCD0!\r\n ");
  LED_Init();
  Key_init();
  Jumper_Init();
  i2c_oled_init();  
  ccd0_init();
  //ccd1_init();
  while(1)
  {      
      LEDTog(LED1);
      threshold0 = ImageCapture(pixel_pt0);//һ��11ms
      //hreshold1 = ccd1_ImageCapture(pixel_ptif( 0x00 == GPIO_ReadPinInput(GPIOE, 11U)) //��3������1);//һ��11ms
       
      
      {
        //���ڷ�������
        //SendImageData(pixel_pt0);
        //SendImageData(pixel_pt1);
        
        //OLED��ʾ2·CCD����
        oled_display_ccd_line (pixel_pt0, threshold0, pixel_pt1, threshold1);
      }
    
  }
}

//���� nrf24l01 
void test_nrf24l01(void)
{
  uint8_t str[64];
  uint8_t rxbuf[64];
  int time;
  
  PRINTF("���� nrf24l01!\r\n ");
  i2c_oled_init(); 
  delay();delay();delay();
  OLED_Clear(0);
  rf1start();
  
  while(1)
  {
    time++;
    if(rf1checkrx(rxbuf)>0)
    {
      sprintf((char*)str, "receive: %s    ",rxbuf);
      OLED_ShowString(2,0,8,str);       
    }
    
    sprintf((char*)str, "send: %d    ", time);
    OLED_ShowString(0,0,8,str);  
    rf1send(str);
    delay_ms(1000);
  }
}

//���� hc_sr04 
void test_hc_sr04(void)
{
  uint32_t dis;
  char str[256];
  PRINTF("���� hc_sr04!\r\n ");
  LED_Init();
  i2c_oled_init(); 
  delay();delay();delay();
  OLED_Clear(0);
  hc_sc04_init();
  while(1)
  {      
      delay();delay();delay();
      dis = ultrasonic_width_get();
      PRINTF("ultrasonic_width = %d.%d cm  \r\n ", dis/1000,dis%1000);//���ڴ�ӡ������
      sprintf(str, "len=%d.%d cm  ", dis/1000,dis%1000);
      OLED_ShowString(2, 0, 8, (uint8_t*)str);
  }
}