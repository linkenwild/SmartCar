/*********************2017-2017, NJUT, Edu.********************* 
FileName: delay.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.05
Description:    用systick进行精确延时      
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/05     1.0     文件创建   
***************************************************************/   

#include <MK60D10.h>
#include "delay.h"

__IO uint32_t ntime;								    

/*******************************************************************************
* Function Name  : delay_ms
* Description    : 延时ms
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
* Description    : 延时us
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
* Description    : 简单延时函数
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
* Description    : 系统滴答时钟中断服务函数
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
* Description    : 系统簡單延時 單位ms
* Input          : t 延時的時間
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