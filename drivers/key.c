/*********************2017-2017, NJUT, Edu.********************* 
FileName: key.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    key驱动      
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     文件创建   
  *          SmartCar Board Key Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      Keys                   |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |       SW1                   |        E26                  |
  *          |       SW2                   |        E27                  |
  *          |       SW3 (不可屏蔽中断)    |        A4                   |
  *          |       Jump1                 |        B20                  |
  *          |       Jump2                 |        B21                  |
  *          |       Jump3                 |        B22                  |
  *          |       Jump4                 |        B23                  |
  *          |       Jump5                 |        C0                   |
  *          +-----------------------------+-----------------------------+
***************************************************************************/ 
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "delay.h"
#include "key.h"
#include "led.h"

#define SW1_GPIO GPIOE
#define SW1_PORT PORTE
#define SW1_IRQ PORTE_IRQn
#define SW1_GPIO_PIN 26U

#define SW2_GPIO GPIOE
#define SW2_PORT PORTE
#define SW2_IRQ PORTE_IRQn
#define SW2_GPIO_PIN 27U

Key_Def key_value = KEYNULL;

static void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 5000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

/*******************************************************************************
* Function Name  : PORTE_IRQHandler
* Description    : 外部 中断服务函数
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void PORTE_IRQHandler(void)
{
  delay();  
  /* Clear external interrupt flag. */
	if ((1 << SW1_GPIO_PIN) == GPIO_GetPinsInterruptFlags(SW1_GPIO) )
	{
          GPIO_ClearPinsInterruptFlags(SW1_GPIO, 1U << SW1_GPIO_PIN);
          if( 0 == (GPIO_ReadPinInput(SW1_GPIO, SW1_GPIO_PIN))) 
          {
		/*这里添加自己的代码*/
		LEDTog( LED2 );
		key_value = KEY1;
          }
	}
	else if((1 << SW2_GPIO_PIN) == GPIO_GetPinsInterruptFlags(SW2_GPIO))
	{
		GPIO_ClearPinsInterruptFlags(SW2_GPIO, 1U << SW2_GPIO_PIN);
                if( 0 == (GPIO_ReadPinInput(SW2_GPIO, SW2_GPIO_PIN))) 
                {
                    /*这里添加自己的代码*/
                  LEDTog( LED3 );
                  key_value = KEY2;
                }
	}
}

/*******************************************************************************
* Function Name  : Key_init
* Description    : 按键SW1 SW2 SW3 GPIO初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Key_init(void)
{
    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput, 0,
    };
    /* Enable SW port clock */
    CLOCK_EnableClock(kCLOCK_PortE);
    
    /* Affects PORTA_PCR19 register */
    port_pin_config_t config = {0};    
    config.pullSelect = kPORT_PullUp;
    config.mux = kPORT_MuxAsGpio;
    PORT_SetPinConfig(PORTE, 26U, &config);
    PORT_SetPinConfig(PORTE, 27U, &config);
    
    /* Init input switch GPIO. */
    PORT_SetPinInterruptConfig(SW1_PORT, SW1_GPIO_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(SW1_IRQ);
    GPIO_PinInit(SW1_GPIO, SW1_GPIO_PIN, &sw_config);	
    /* Init input switch GPIO. */
    PORT_SetPinInterruptConfig(SW2_PORT, SW2_GPIO_PIN, kPORT_InterruptFallingEdge);
    EnableIRQ(SW2_IRQ);
    GPIO_PinInit(SW2_GPIO, SW2_GPIO_PIN, &sw_config);	
    
    LED_Init();
}

/*******************************************************************************
* Function Name  : Get_Key，内部函数
* Description    : 获取输入按键是否按下，按下返回1
* Input          : -key:某个按键 
* Output         : None
* Return         : true：输入的按键按下 false：输入的按键没有按下
*******************************************************************************/
bool Get_Key(Key_Def key)
{
	Key_Def key_temp = KEYNULL;
	key_temp = key_value;
	key_value = KEYNULL;
  	if (key_temp != KEYNULL)
	{
		if (key_temp == key)	return true;
		else return false;
	}
        else return false;
}

/*******************************************************************************
* Function Name  : KEY_Scan，内部函数
* Description    : 获取输入按键下的键值
* Input          : None
* Output         : None
* Return         : 按键下的键值
*******************************************************************************/
Key_Def KEY_Scan(void)
{
  	Key_Def key_temp = KEYNULL;
	key_temp = key_value;
	key_value = KEYNULL;
  	return key_temp;
}
/******************************  Jumper ***************************************/
/*******************************************************************************
* Function Name  : Jumper_Init
* Description    : Jumper 初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void Jumper_Init(void)
{
    /* Define the init structure for the input switch pin */
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput, 0,
    };
    
    /* Enable jumper port clock */
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);    
    /* Affects PORTA_PCR19 register */
    port_pin_config_t config = {0};    
    config.pullSelect = kPORT_PullUp;
    config.mux = kPORT_MuxAsGpio;
    PORT_SetPinConfig(PORTB, 20U, &config);
    PORT_SetPinConfig(PORTB, 21U, &config);
    PORT_SetPinConfig(PORTB, 22U, &config);
    PORT_SetPinConfig(PORTB, 23U, &config);
    PORT_SetPinConfig(PORTC, 0U, &config);
    
    /* B20 */
    /* jumper pin mux Configuration */
    PORT_SetPinMux(PORTB, 20U, kPORT_MuxAsGpio); 
    /* Init jumper switch GPIO. */
    GPIO_PinInit(GPIOB, 20U, &sw_config);	

    /* B21 */
    PORT_SetPinMux(PORTB, 21U, kPORT_MuxAsGpio); 
    GPIO_PinInit(GPIOB, 21U, &sw_config);	

    /* B22 */
    PORT_SetPinMux(PORTB, 22U, kPORT_MuxAsGpio); 
    GPIO_PinInit(GPIOB, 22U, &sw_config);	

    /* B23 */
    PORT_SetPinMux(PORTB, 23U, kPORT_MuxAsGpio); 
    GPIO_PinInit(GPIOB, 23U, &sw_config);	

    /* C0 */
    PORT_SetPinMux(PORTC, 0U, kPORT_MuxAsGpio); 
    GPIO_PinInit(GPIOC, 0U, &sw_config);    
}

/*******************************************************************************
* Function Name  : Get_Jumper_value 
* Description    : 获取跳线位置值
* Input          : None 
* Output         : None
* Return         : 路线位置值
*******************************************************************************/
uint32_t Get_Jumper_value(void)
{
  uint32_t value = 0;
  value = GPIO_ReadPinInput(GPIOB, 20U);  
  value |= GPIO_ReadPinInput(GPIOB, 21U) << 1;  
  value |= GPIO_ReadPinInput(GPIOB, 22U) << 2;  
  value |= GPIO_ReadPinInput(GPIOB, 23U) << 3;  
  value |= GPIO_ReadPinInput(GPIOC, 0U) << 4;  
  return value;
}
