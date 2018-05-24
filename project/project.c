/*********************2017-2017, NJUT, Edu.************************************ 
FileName: project.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    直立车主程序
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/07/9     1.0     文件创建   
******************************************************************************/ 
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"
#include <MK60D10.h>

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief delay a while.
 */

//测试程序
#include "test.h"
//运行程序
#include "CarSub.h"

/*!
 * @brief Main function
 */
int main(void)
{

    /* Board pin, clock, debug console init */
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
  //控制程序
   // control();
    
  //测试程序
    PRINTF("\r\nInit completed!!!\r\n ");
    PRINTF("  kCLOCK_CoreSysClk  = %d\r\n", CLOCK_GetFreq(kCLOCK_CoreSysClk));
    PRINTF("  kCLOCK_PlatClk  = %d\r\n", CLOCK_GetFreq(kCLOCK_PlatClk));
    PRINTF("  kCLOCK_BusClk  = %d\r\n", CLOCK_GetFreq(kCLOCK_BusClk));
    PRINTF("  kCLOCK_FastPeriphClk  = %d\r\n", CLOCK_GetFreq(kCLOCK_FastPeriphClk));
    PRINTF("  kCLOCK_PllFllSelClk  = %d\r\n", CLOCK_GetFreq(kCLOCK_PllFllSelClk));
    PRINTF("  kCLOCK_McgFixedFreqClk  = %d\r\n", CLOCK_GetFreq(kCLOCK_McgFixedFreqClk));
    PRINTF("  kCLOCK_McgInternalRefClk  = %d\r\n", CLOCK_GetFreq(kCLOCK_McgInternalRefClk));
    PRINTF("  kCLOCK_LpoClk  = %d\r\n", CLOCK_GetFreq(kCLOCK_LpoClk));
//    test_led();
//    test_key();
//    test_delay();
//    test_pit();
//    test_adc();
//    test_uart();
//    test_pwm();
//    test_e2prom();
//    test_oled();
    test_camera();// 80*60扩张成128*60 如果 按下S2键 绿灯亮 OLED 显示赛道 原始数据 最后 4行个填入0
//    test_mpu6050();
//    test_mpu_dmp();
//    test_mpu6050_angle();
    
/*测试 mpu6050 求角度 用卡尔曼滤波 
在文件 Kalman_filter.c 中， Angle_Calcu函數中選擇以下三種濾波方式取一：
  Kalman_Filter_X(Angle_x_temp, Gyro_x);  //卡尔曼滤波计算X倾角
  yijiehubu_P(Angle_x_temp, Gyro_x);  //一阶互补滤波
  Erjielvbo(Angle_x_temp, Gyro_x);   //二阶互补滤波
*/ 

//  test_mpu_imu();//经测试 漂移太大 弃用
//  test_speed(); //脉冲宽度 us单位
//  test_speed_quad();//定时间内 脉冲计数
//  test_nrf24l01();// ST开发板 使用命令： rf1start()   rf1send("123345")
//  test_ccd();//按键S2 OLED 显示赛道
//  test_hc_sr04();//LED4 闪烁变亮距离长 变暗距离短 串口打印输出距离
    
}


