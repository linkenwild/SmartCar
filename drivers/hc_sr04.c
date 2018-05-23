/******************************2017-2017, NJTECH, Edu.************************** 
FileName: hc-sr04.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    hc-sr04 驱动 使用定時器1 LED4 闪烁变亮距离长 变暗距离短
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     文件创建   
  *          SmartCar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      hc-sr04                |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |        trig                 |        D2                   |
  *          |        echo (led)           |        D5                   |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 
#include "board.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_ftm.h"

#include "stdio.h"
#include "hc_sr04.h"
#include "pit.h"
#include "led.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TRIG_GPIO GPIOD
#define TRIG_PORT PORTD
#define TRIG_IRQ PORTD_IRQn
#define TRIG_GPIO_PIN 2U

#define ECHO_GPIO GPIOD
#define ECHO_PORT PORTD
#define ECHO_IRQ PORTD_IRQn
#define ECHO_GPIO_PIN 5U




/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static  volatile  uint32_t capture1Val;
static  volatile  uint32_t pit1times = 0;  
static  volatile  uint32_t echo_state = 0;  //0为空闲   1为置1态


/*******************************************************************************
 * Code
 ******************************************************************************/
static void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 5000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

/*******************************************************************************
* Function Name  : pit3start
* Description    : 定时器3 启动 定时15us
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
static void pit3restart(uint64_t count)
{
  /* Stop channel 1 */
  PIT_StopTimer(PIT, kPIT_Chnl_3);
  
  /* Set timer period for channel 3 */
  PIT_SetTimerPeriod(PIT, kPIT_Chnl_3, USEC_TO_COUNT(count,  CLOCK_GetFreq(kCLOCK_BusClk)));

  /* Enable timer interrupts for channel 1 */
  PIT_EnableInterrupts(PIT, kPIT_Chnl_3, kPIT_TimerInterruptEnable);
  EnableIRQ(PIT3_IRQn );
  /* Start channel 1 */
  PIT_StartTimer(PIT, kPIT_Chnl_3);

}

static void pit3stop(void)
{
  /* Stop channel 1 */
  PIT_StopTimer(PIT, kPIT_Chnl_3);
}	

/*******************************************************************************
* Function Name  : trigstart
* Description    : 定时器1 启动 定时15us trig置位 产生触发信号
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
static void trigstart(void)
{
            /*trig 置位 产生下一次触发脉冲*/
          GPIO_SetPinsOutput(TRIG_GPIO, 1u << TRIG_GPIO_PIN);
          /*开定时 15us 中断*/
          pit3restart(15);      
}

/*******************************************************************************
* Function Name  : PIT3_IRQHandler
* Description    : 定时器1 中断服务函数
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
void PIT3_IRQHandler(void)
{
    static uint8_t trig_state = 0; //0-触发 1-长延时
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_3 , kPIT_TimerFlag);
    if(echo_state == 0)//定时 2次，一次触发，一次长延时
    {
      if (0 == trig_state)//触发15us 时间到
      {
        trig_state = 1;
        /*关定时定时器*/
        pit3stop();
        /*trig 复位 */
        GPIO_ClearPinsOutput(TRIG_GPIO, 1u << TRIG_GPIO_PIN);
      }
      else if(1 == trig_state)//长延时 时间到
      {
        trig_state = 0;
        trigstart();
      }
    }
    else //计时 
    {
      pit1times++;
    }
}

/*******************************************************************************
* Function Name  : PORTD_IRQHandler
* Description    : 外部D5 中断服务函数
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void PORTD_IRQHandler(void)
{
  /* Clear external interrupt flag. */
	if ((1 << ECHO_GPIO_PIN) == GPIO_GetPinsInterruptFlags(ECHO_GPIO) )
	{
          GPIO_ClearPinsInterruptFlags(ECHO_GPIO, 1U << ECHO_GPIO_PIN);
          if( 1 == (GPIO_ReadPinInput(ECHO_GPIO, ECHO_GPIO_PIN))) //上升沿
          {
                echo_state = 1;
                pit1times = 0;
                LEDOn(LED4);
                pit3restart(1000);
          }
          else if( 0 == (GPIO_ReadPinInput(ECHO_GPIO, ECHO_GPIO_PIN))) //下降沿
          {
                capture1Val =(1000- COUNT_TO_USEC(PIT_GetCurrentTimerCount(PIT, kPIT_Chnl_1),CLOCK_GetFreq(kCLOCK_BusClk))) + 1000*pit1times; //us
                pit3stop();
                echo_state = 0;
                LEDOff(LED4);
                pit3restart(100000);
          }
	}
}

/*******************************************************************************
* Function Name  : hc_sc04_init
* Description    : hc_sc04  初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void hc_sc04_init(void)
{
    gpio_pin_config_t config = 
    {
        kGPIO_DigitalOutput, 0,
    };
    /* Enable  port clock */
    CLOCK_EnableClock(kCLOCK_PortD);
    /* trig output*/
    PORT_SetPinMux(TRIG_PORT, TRIG_GPIO_PIN, kPORT_MuxAsGpio);   
    GPIO_PinInit(TRIG_GPIO, TRIG_GPIO_PIN, &config);
    
    /* echo input*/
    port_pin_config_t port_config = {0};    
    port_config.pullSelect = kPORT_PullUp;
    port_config.mux = kPORT_MuxAsGpio;
    PORT_SetPinConfig(ECHO_PORT, ECHO_GPIO_PIN, &port_config);
    
    config.pinDirection = kGPIO_DigitalInput;
    GPIO_PinInit(ECHO_GPIO, ECHO_GPIO_PIN, &config);
    
    /*设置 echo 上升下降沿中断*/
    PORT_SetPinInterruptConfig(ECHO_PORT, ECHO_GPIO_PIN, kPORT_InterruptEitherEdge);
    EnableIRQ(ECHO_IRQ);//开 echo 中断
    PitConfig(kPIT_Chnl_3, 15);
    /*trig 置位 产生下一次触发脉冲*/
    GPIO_SetPinsOutput(TRIG_GPIO, 1u << TRIG_GPIO_PIN);
    trigstart();

}

/*******************************************************************************
* Function Name  : ultrasonic_width_get
* Description    : 脉冲宽度 换算成距离 单位：mm
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t ultrasonic_width_get(void)
{	
  uint32_t distance;
  distance = capture1Val * 170 / 100;
  
  return distance;
}


