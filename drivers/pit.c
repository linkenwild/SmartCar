/*********************2017-2017, NJUT, Edu.********************* 
FileName: pit.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.10
Description:    SmartCar Board pit ����  ����PIT     
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/10     1.0     �ļ�����   
***************************************************************/ 
#include "board.h"
#include "fsl_pit.h"
#include "clock_config.h"
#include "pit.h"
#include "led.h"
	 
////////////////////////////////////////////////////////////////////
/*******************************************************************************
* Function Name  : PIT1_IRQHandler
* Description    : ��ʱ��1 �жϷ�����
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
//void PIT1_IRQHandler(void)
//{
//  /* Clear interrupt flag.*/
//    PIT_ClearStatusFlags(PIT, kPIT_Chnl_1 , kPIT_TimerFlag);
//    /*��������Լ��Ĵ���*/
//    LEDTog(LED1);
//    PDB_DoSoftwareTrigger(PDB0);
//}
/*******************************************************************************
* Function Name  : PIT2_IRQHandler
* Description    : ��ʱ��2 �жϷ�����
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
//void PIT2_IRQHandler(void)
//{
//  /* Clear interrupt flag.*/
//    PIT_ClearStatusFlags(PIT, kPIT_Chnl_2 , kPIT_TimerFlag);
//    /*��������Լ��Ĵ���*/
//    LEDTog(LED2);
//}

/*******************************************************************************
* Function Name  : PIT3_IRQHandler
* Description    : ��ʱ��3 �жϷ�����
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
//void PIT3_IRQHandler(void)
//{
//  /* Clear interrupt flag.*/
//    PIT_ClearStatusFlags(PIT, kPIT_Chnl_3 , kPIT_TimerFlag);
//    /*��������Լ��Ĵ���*/
//    LEDTog(LED3);
//}

/*******************************************************************************
* Function Name  : PitConfig
* Description    : ��ʱ�����ú���,��ʱ���ڣ�us_count
* Input          : us_count ��ʱ���� ��λ us 
*   ͨ���� pit_chanel:  kPIT_Chnl_0 ��kPIT_Chnl_1 ��kPIT_Chnl_2 ��kPIT_Chnl_3 
* Output         : None
* Return         : None
*******************************************************************************/
void PitConfig(pit_chnl_t pit_chanel, uint64_t us_count)
{
  /* Structure of initialize PIT */
  pit_config_t pitConfig;  	

  /*
  * pitConfig.enableRunInDebug = false;
  */
  PIT_GetDefaultConfig(&pitConfig);                      

  /* Init pit module */
  PIT_Init(PIT, &pitConfig);

  /* Set timer period for channel 0 */
  PIT_SetTimerPeriod(PIT, kPIT_Chnl_0 + pit_chanel, USEC_TO_COUNT(us_count,  CLOCK_GetFreq(kCLOCK_BusClk)));

  /* Enable timer interrupts for channel 0 */
  PIT_EnableInterrupts(PIT, kPIT_Chnl_0 + pit_chanel, kPIT_TimerInterruptEnable);

  /* Enable at the NVIC */
  EnableIRQ(PIT0_IRQn + pit_chanel);

  /* Start channel 0 */
  PIT_StartTimer(PIT, kPIT_Chnl_0 + pit_chanel);

}	



