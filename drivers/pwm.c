/*********************2017-2017, NJUT, Edu.********************* 
FileName: pwm.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.10
Description:    pwm驱动  使用FTM0   steerpwm使用PIT0    
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/10     1.0     文件创建   
  *          Smartcar Board Key Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      FunctionPin            |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |      PWM1 (ch0)             |        C1                   |
  *          |      PWM2 (ch1)             |        C2                   |
  *          |      PWM3 (ch2)             |        C3                   |
  *          |      PWM4 (ch3)             |        C4                   |
  *          |      STEERPWM (ch4)         |        D4                   |
  *          +-----------------------------+-----------------------------+
***************************************************************/ 
#include "fsl_ftm.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"

#include "pwm.h"
#include "pit.h"
#include "led.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The Flextimer base address/channel pair used for board */
#define BOARD_FTM_BASEADDR FTM0
/* FTM channel pair 3 works with FTM channels 0 and 1 */
#define BOARD_FTM_CHANNEL kFTM_Chnl_0

/* Interrupt number and interrupt handler for the FTM base address used */
#define FTM_INTERRUPT_NUMBER FTM0_IRQn
#define STEER_PIT_HANDLER PIT0_IRQHandler

/* Interrupt to enable and flag to read */
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl0InterruptEnable


/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

#define STEERMIN 1000
#define STEERMID 1500           //1500us = 1.5ms 中位 
#define STEERMAX 2000
#define STEERWIDTH 20000        //20000us = 20ms周期

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
ftm_pwm_level_select_t pwmLevel = kFTM_LowTrue;

static volatile uint32_t steer_value = 50; //0~100

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
* Function Name  : steerpwmon 内部函数
* Description    : D4置1
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void steerpwmon(void)
{
  GPIO_SetPinsOutput(GPIOD, 1u << 4);
}
/*******************************************************************************
* Function Name  : steerpwmoff 内部函数
* Description    : D4置0
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void steerpwmoff(void)
{
  GPIO_ClearPinsOutput(GPIOD, 1u << 4);
}


/*******************************************************************************
* Function Name  : duty2counter 内部函数
* Description    : duty到 计数值的转换
* Input          : duty 占空比
* Output         : counter 定时器0计数值
* Return         : None
*******************************************************************************/
static uint32_t duty2counter(uint32_t duty_value)
{
  uint32_t counter = 0;
  if( 50 > duty_value)
  {
    counter = STEERMID - (uint32_t)(STEERMID - STEERMIN)/50 * (50-duty_value);
  }
  else if(  50 == duty_value )
  {
    counter = STEERMID;
  }
  else if(( 50 < duty_value )&&(100 > duty_value))
  {
    counter = STEERMID + (uint32_t)(STEERMAX - STEERMID)/50 * duty_value;
  }
  else if(100 <= duty_value)
  {
    counter = STEERMAX;
  }
  return counter;
}

/*******************************************************************************
* Function Name  : STEER_FTM_HANDLER
* Description    : 定时器0 中断服务函数 
*		舵机频率:50Hz, 周期20ms  脉宽1 ms为上限位置，1.5ms为中位,2ms为下限位置 
*		计数 20000us 为一个周期 1000为上限位置，1500 为中位, 2000 为下限位置 
*		STEER_FTM 配置初始值时，为中值 50
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void STEER_PIT_HANDLER(void)
{
  static volatile bool g_highlowflag = true;  
  /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
    /*这里添加自己的代码*/
    g_highlowflag = !g_highlowflag;
    if(true == g_highlowflag)
    {
      PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(duty2counter(steer_value), CLOCK_GetFreq(kCLOCK_BusClk)));
      steerpwmon();
    }
    else
    {
      PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT((STEERWIDTH-duty2counter(steer_value)),  CLOCK_GetFreq(kCLOCK_BusClk)));
      steerpwmoff();
    }
}

/*******************************************************************************
* Function Name  : PWMConfig
* Description    : PWM 初始化
*                  总线频率：24M 频率 PWM频率：24K  周期0.00416ms
* Input          : chanel PWM通道
* Output         : None
* Return         : None
*******************************************************************************/
void PWM_Init(PWM_Def chanel)
{
    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;
    
    gpio_pin_config_t pin_config = 
    {
        kGPIO_DigitalOutput, 0,
    };  
    
    if(STEERPWM == chanel)
    {
      /* Ungate the port clock */
      CLOCK_EnableClock(kCLOCK_PortD);
      /* Affects PORTC_PCR1 register */
      PORT_SetPinMux(PORTD, 4, kPORT_MuxAsGpio);
      /* Init output steer PWM GPIO D4. */
      GPIO_PinInit(GPIOD, 4U, &pin_config);
    
      PitConfig(kPIT_Chnl_0, 200000);  //200ms-50Hz  100ms-100Hz
    }
    else if ((PWM1 == chanel)||(PWM2 == chanel)||(PWM3 == chanel)||(PWM4 == chanel))
    {
      /* Initialize FTM0 pins below */
      /* Ungate the port clock */
      CLOCK_EnableClock(kCLOCK_PortC);
      /* Affects PORTC_PCR1 register */
      PORT_SetPinMux(PORTC, (1U+ chanel), kPORT_MuxAlt4);
      
      /* Declare and initialise for pull up configuration */
      port_pin_config_t config = {0};    
      config.pullSelect = kPORT_PullDisable;//失能内部上拉
      config.mux = kPORT_MuxAlt4;
      config.openDrainEnable = kPORT_OpenDrainEnable;//使能开漏输出
      //PORT_SetPinConfig(PORTC, (1U+ chanel), &config);      
          
      /* Configure ftm params with frequency 24kHZ */
      ftmParam.chnlNumber = BOARD_FTM_CHANNEL +  chanel;
      ftmParam.level = pwmLevel;
      ftmParam.dutyCyclePercent = 10U;
      ftmParam.firstEdgeDelayPercent = 0U;

      FTM_GetDefaultConfig(&ftmInfo);
      /* Initialize FTM module */
      FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);

      FTM_SetupPwm(BOARD_FTM_BASEADDR, &ftmParam, 1U, kFTM_CenterAlignedPwm, 24000U, FTM_SOURCE_CLOCK);

      FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
    }
}

/*******************************************************************************
* Function Name  : PWMSet
* Description    : PWM某一通道设置占空比节
* Input          : chanel PWM通道; value 占空比数据
* Output         : None
* Return         : None
*******************************************************************************/
void PWMSet(PWM_Def chanel, uint32_t updatedDutycycle)
{	
    if((updatedDutycycle>=0)&&(updatedDutycycle<=100))
    {
        if ((PWM1 == chanel)||(PWM2 == chanel)||(PWM3 == chanel)||(PWM4 == chanel))
        {
            /* Disable interrupt to retain current dutycycle for a few seconds */
            FTM_DisableInterrupts(BOARD_FTM_BASEADDR, FTM_CHANNEL_INTERRUPT_ENABLE);	
            
            /* Disable channel output before updating the dutycycle */
            FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, (ftm_chnl_t)chanel, kFTM_NoPwmSignal);
            
            /* Update PWM duty cycle */
            FTM_UpdatePwmDutycycle(BOARD_FTM_BASEADDR, (ftm_chnl_t)chanel, kFTM_CenterAlignedPwm, updatedDutycycle);	
            
            /* Software trigger to update registers */
            FTM_SetSoftwareTrigger(BOARD_FTM_BASEADDR, true);

            /* Start channel output with updated dutycycle */
            FTM_UpdateChnlEdgeLevelSelect(BOARD_FTM_BASEADDR, BOARD_FTM_CHANNEL+chanel, pwmLevel);

        }
        else if(STEERPWM == chanel)
        {
          steer_value = 100 - updatedDutycycle;
        }
    }
}
