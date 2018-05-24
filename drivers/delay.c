/*********************2017-2017, NJUT, Edu.********************* 
FileName: delay.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.05
Description:    ��systick���о�ȷ��ʱ      
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/05     1.0     �ļ�����   
***************************************************************/   

#include <MK60D10.h>
#include "delay.h"

__IO uint32_t ntime;								    

/*******************************************************************************
* Function Name  : delay_ms
* Description    : ��ʱms
* Input          : nms
* Output         : None
* Return         : None
*******************************************************************************/
void delay_ms(uint16_t nms)
{	 		  	  
	ntime=nms;
	SysTick_Config(SystemCoreClock/1000);
	while(ntime);
	SysTick->CTRL=0x00;			  	    
}   
		    								   
/*******************************************************************************
* Function Name  : delay_us
* Description    : ��ʱus
* Input          : nus
* Output         : None
* Return         : None
*******************************************************************************/
void delay_us(uint32_t nus)
{		
	ntime=nus;
	SysTick_Config(SystemCoreClock/1000000);
	while(ntime);
	SysTick->CTRL=0x00;
}
/*******************************************************************************
* Function Name  : Delay
* Description    : ����ʱ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Delay(uint32_t t)
{ 
	while(t--) 
          __asm("NOP"); /* delay */;	
}

/*******************************************************************************
* Function Name  : SysTick_Handler
* Description    : ϵͳ�δ�ʱ���жϷ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SysTick_Handler(void)
{
	ntime--;
}

/*******************************************************************************
* Function Name  : delay_dmp_ms
* Description    : ϵͳ�����ӕr ��λms
* Input          : t �ӕr�ĕr�g
* Output         : None
* Return         : None
*******************************************************************************/
void delay_dmp_ms(uint32_t ms)
{
  uint32_t i ,j = 0;
  for(j = 0; j<ms; j++)
  {
    for (i = 0; i < 500; i++)
    {
        __NOP();
    }
  }
}