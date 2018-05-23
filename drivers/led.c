/*********************2016-2017, NJUT, Edu.********************* 
FileName: led.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    led驱动      
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    16/06/30     1.0     文件创建   
  *          SmartCar Board LED Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      Leds                   |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |      LED1                   |        E6                   |
  *          |      LED2                   |        E7                   |
  *          |      LED3                   |        E11                  |
  *          |      LED4                   |        E12                  |
  *          |      UltraSonic             |        D2                  |
  *          |      BEEP                   |        A6                  |
  *          +-----------------------------+-----------------------------+
***************************************************************/ 
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "led.h"

#define LED_GPIO GPIOE
#define LED_PORT PORTE
#define LED1_GPIO_PIN 6U
#define LED2_GPIO_PIN 7U
#define LED3_GPIO_PIN 11U
#define LED4_GPIO_PIN 12U

#define BEEP_GPIO GPIOA
#define BEEP_PORT PORTA
#define BEEP_GPIO_PIN 6U

#define UltraSonic_GPIO GPIOD
#define UltraSonic_PORT PORTD
#define UltraSonic_GPIO_PIN 2U


/*******************************************************************************
* Function Name  : LED_Init
* Description    : LED GPIO初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void LED_Init(void)
{
    gpio_pin_config_t led_config = 
    {
        kGPIO_DigitalOutput, 0,
    };
    /* Enable LED port clock */
    CLOCK_EnableClock(kCLOCK_PortE);
	
    /* Led pin mux Configuration */
    PORT_SetPinMux(LED_PORT, LED1_GPIO_PIN, kPORT_MuxAsGpio);   
    PORT_SetPinMux(LED_PORT, LED2_GPIO_PIN, kPORT_MuxAsGpio);   
    PORT_SetPinMux(LED_PORT, LED3_GPIO_PIN, kPORT_MuxAsGpio);   
    PORT_SetPinMux(LED_PORT, LED4_GPIO_PIN, kPORT_MuxAsGpio);   
	
    /* Init output LED GPIO. */
    GPIO_PinInit(LED_GPIO, LED1_GPIO_PIN, &led_config);
    GPIO_PinInit(LED_GPIO, LED2_GPIO_PIN, &led_config);
    GPIO_PinInit(LED_GPIO, LED3_GPIO_PIN, &led_config);
    GPIO_PinInit(LED_GPIO, LED4_GPIO_PIN, &led_config);
    
    LEDOff(LEDALL);

}

/*******************************************************************************
* Function Name  : LEDOn
* Description    : LED亮
* Input          : -led:LED值
* Output         : None
* Return         : None
*******************************************************************************/
void LEDOn(Led_Def Led)
{
  	switch( Led )
	{
		case LED1: 
		GPIO_ClearPinsOutput(LED_GPIO, 1u << LED1_GPIO_PIN);
		break;
		case LED2: 
		GPIO_ClearPinsOutput(LED_GPIO, 1u << LED2_GPIO_PIN);
		break;
		case LED3: 
		GPIO_ClearPinsOutput(LED_GPIO, 1u << LED3_GPIO_PIN);
		break;
		case LED4: 
		GPIO_ClearPinsOutput(LED_GPIO, 1u << LED4_GPIO_PIN);
		break;
		case LEDALL: 
		GPIO_ClearPinsOutput(LED_GPIO, 
		(1u << LED1_GPIO_PIN)|
		(1u << LED2_GPIO_PIN)|
		(1u << LED3_GPIO_PIN)|
		(1u << LED4_GPIO_PIN));
		break;
        }
}

/*******************************************************************************
* Function Name  : LEDOff
* Description    : LED灭
* Input          : -led:LED值
* Output         : None
* Return         : None
*******************************************************************************/
void LEDOff(Led_Def Led)
{
  	switch( Led )
	{
		case LED1: 
		GPIO_SetPinsOutput(LED_GPIO, 1u << LED1_GPIO_PIN);
		break;
		case LED2: 
		GPIO_SetPinsOutput(LED_GPIO, 1u << LED2_GPIO_PIN);
		break;
		case LED3: 
		GPIO_SetPinsOutput(LED_GPIO, 1u << LED3_GPIO_PIN);
		break;
		case LED4: 
		GPIO_SetPinsOutput(LED_GPIO, 1u << LED4_GPIO_PIN);
		break;
		case LEDALL: 
		GPIO_SetPinsOutput(LED_GPIO, 
		(1u << LED1_GPIO_PIN)|
		(1u << LED2_GPIO_PIN)|
		(1u << LED3_GPIO_PIN)|
		(1u << LED4_GPIO_PIN));
		break;
        }
}

/*******************************************************************************
* Function Name  : LEDTog
* Description    : LED状态翻转
* Input          : -led:LED值
* Output         : None
* Return         : None
*******************************************************************************/
void LEDTog(Led_Def Led)
{
  	switch( Led )
	{
		case LED1: 
		GPIO_TogglePinsOutput(LED_GPIO, 1u << LED1_GPIO_PIN);
		break;
		case LED2: 
		GPIO_TogglePinsOutput(LED_GPIO, 1u << LED2_GPIO_PIN);
		break;
		case LED3: 
		GPIO_TogglePinsOutput(LED_GPIO, 1u << LED3_GPIO_PIN);
		break;
		case LED4: 
		GPIO_TogglePinsOutput(LED_GPIO, 1u << LED4_GPIO_PIN);
		break;
		case LEDALL: 
		GPIO_TogglePinsOutput(LED_GPIO, 
		(1u << LED1_GPIO_PIN)|
		(1u << LED2_GPIO_PIN)|
		(1u << LED3_GPIO_PIN)|
		(1u << LED4_GPIO_PIN));
		break;
        }
}

/******************************        BEEP     *******************************/
/*******************************************************************************
* Function Name  : BEEP_Init
* Description    : BEEP GPIO初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BEEP_Init(void)
{
    gpio_pin_config_t pin_config = 
    {
        kGPIO_DigitalOutput, 0,
    };
    /* Enable LED port clock */
    CLOCK_EnableClock(kCLOCK_PortA);
	
    /* Led pin mux Configuration */
    PORT_SetPinMux(BEEP_PORT, BEEP_GPIO_PIN, kPORT_MuxAsGpio);   
	
    /* Init output LED GPIO. */
    GPIO_PinInit(BEEP_GPIO, BEEP_GPIO_PIN, &pin_config);
}

/*******************************************************************************
* Function Name  : BEEPOn
* Description    : BEEP亮
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BEEPOn(void)
{
  GPIO_SetPinsOutput(BEEP_GPIO, 1u << BEEP_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : BEEPOff
* Description    : BEEP灭
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BEEPOff(void)
{ 
  GPIO_ClearPinsOutput(BEEP_GPIO, 1u << BEEP_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : BEEPTog
* Description    : BEEP閃
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void BEEPTog(void)
{ 
  GPIO_TogglePinsOutput(BEEP_GPIO, 1u << BEEP_GPIO_PIN);
}

/******************************     UltraSonic  *******************************/
/*******************************************************************************
* Function Name  : UltraSonic_Init
* Description    : UltraSonic GPIO初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UltraSonic_Init(void)
{
    gpio_pin_config_t pin_config = 
    {
        kGPIO_DigitalOutput, 0,
    };
    /* Enable LED port clock */
    CLOCK_EnableClock(kCLOCK_PortD);
	
    /* Led pin mux Configuration */
    PORT_SetPinMux(UltraSonic_PORT, UltraSonic_GPIO_PIN, kPORT_MuxAsGpio);   
	
    /* Init output LED GPIO. */
    GPIO_PinInit(UltraSonic_GPIO, UltraSonic_GPIO_PIN, &pin_config);
}

/*******************************************************************************
* Function Name  : BEEPOn
* Description    : BEEP亮
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UltraSonicOn(void)
{
  GPIO_ClearPinsOutput(UltraSonic_GPIO, 1u << UltraSonic_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : BEEPOff
* Description    : BEEP灭
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UltraSonicOff(void)
{ 
  GPIO_SetPinsOutput(UltraSonic_GPIO, 1u << UltraSonic_GPIO_PIN);
}

/*******************************************************************************
* Function Name  : BEEPTog
* Description    : BEEP閃
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UltraSonicTog(void)
{ 
  GPIO_TogglePinsOutput(UltraSonic_GPIO, 1u << UltraSonic_GPIO_PIN);
}
