#ifndef _TEST_H
#define _TEST_H

void delay(void);
void delay_short(uint32_t num);

//测试LED beep ultrasonic
void test_led(void);

//测试按键
void test_key(void);

//测试延时systick
void test_delay(void);

//测试定时器1 配置LED1每隔半秒闪一次
void test_pit(void);


//测试ADC 
void test_adc(void);

//测试ADC AIN4, AIN5, DMA连续测量
void test_ain(void);


//测试uart
void test_uart(void);

//测试pwm 舵机使用PIT0
void test_pwm(void);

//测试 OLED i2c0 addr:0x3C
void test_oled(void);

//测试 camera
void test_camera(void);

//测试 e2prom i2c0 addr:0x50
void test_e2prom(void);

//测试 mpu6050 i2c0 addr:0x68
void test_mpu6050(void);

//测试 mpu6050 求角度 用卡尔曼滤波 精确计时
void test_mpu6050_angle(void);

//测试 mpu6050 dmp addr:0x68
void test_mpu_dmp(void);

//测试 mpu6050 dmp addr:0x68
void test_mpu_imu();

//测试 speed 得到脉冲宽度 us单位
void test_speed(void);

void test_speed_quad(void);

//测试 nrf24l01 
void test_nrf24l01(void);


//测试 hc_sr04 
void test_hc_sr04(void);
#endif