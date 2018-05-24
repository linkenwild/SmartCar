/*********************2017-2017, NJUT, Edu.************************************ 
FileName: project.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.30
Description:    ֱ����������
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/07/9     1.0     �ļ�����   
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

//���Գ���
#include "test.h"
//���г���
#include "CarSub.h"

/*!
 * @brief Main function
 */
int main(void)
{

    /* Board pin, clock, debug console init */
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
  //���Ƴ���
   // control();
    
  //���Գ���
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
    test_camera();// 80*60���ų�128*60 ��� ����S2�� �̵��� OLED ��ʾ���� ԭʼ���� ��� 4�и�����0
//    test_mpu6050();
//    test_mpu_dmp();
//    test_mpu6050_angle();
    
/*���� mpu6050 ��Ƕ� �ÿ������˲� 
���ļ� Kalman_filter.c �У� Angle_Calcu�������x���������N�V����ʽȡһ��
  Kalman_Filter_X(Angle_x_temp, Gyro_x);  //�������˲�����X���
  yijiehubu_P(Angle_x_temp, Gyro_x);  //һ�׻����˲�
  Erjielvbo(Angle_x_temp, Gyro_x);   //���׻����˲�
*/ 

//  test_mpu_imu();//������ Ư��̫�� ����
//  test_speed(); //������ us��λ
//  test_speed_quad();//��ʱ���� �������
//  test_nrf24l01();// ST������ ʹ����� rf1start()   rf1send("123345")
//  test_ccd();//����S2 OLED ��ʾ����
//  test_hc_sr04();//LED4 ��˸�������볤 �䰵����� ���ڴ�ӡ�������
    
}


