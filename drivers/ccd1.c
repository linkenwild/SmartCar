/******************************2017-2017, NJTECH, Edu.************************** 
FileName: ccd1.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    ccd1驱动     使用  adc1
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     文件创建   
  *          SmartCar Board Key Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      name                   |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |       CCD1_AO               |      ADC1_S16               |
  *          |       CCD1_CLK              |      C8                     |
  *          |       CCD1_SI               |      C9                     |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 

#include "fsl_adc16.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include "delay.h"

#include "adc1.h"
#include "ccd1.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CCD1_CLK_PORT PORTC
#define CCD1_CLK_GPIO GPIOC
#define CCD1_CLK_PIN 8U

#define CCD1_SI_PORT PORTC
#define CCD1_SI_GPIO GPIOC
#define CCD1_SI_PIN 9U

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
* Function Name  : ccd1_init
* Description    : ccd1　初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ccd1_init(void)
{
    /*pin config*/
    gpio_pin_config_t config = 
    {
        kGPIO_DigitalOutput, 0,
    };
    /* Enable LED port clock */
    CLOCK_EnableClock(kCLOCK_PortC);
	
    /* Led pin mux Configuration */
    PORT_SetPinMux(CCD1_CLK_PORT, CCD1_CLK_PIN, kPORT_MuxAsGpio);   
    PORT_SetPinMux(CCD1_SI_PORT, CCD1_SI_PIN, kPORT_MuxAsGpio);   
	
    /* Init output LED GPIO. */
    GPIO_PinInit(CCD1_CLK_GPIO, CCD1_CLK_PIN, &config);
    GPIO_PinInit(CCD1_SI_GPIO, CCD1_SI_PIN, &config);  
    
    ADC1_poll_init();
}
/*******************************************************************************
* Function Name  : SamplingDelay
* Description    : CCD延时程序 200ns
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
 static void SamplingDelay(void)
 {
   volatile uint8_t i ;
   for(i=0;i<11;i++) 
   {
    asm("nop");
   }   
}

/*******************************************************************************
* Function Name  : clk si 
* Description    : clk si置位和复位
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void CLK_SetVal(void)
{
  GPIO_SetPinsOutput(CCD1_CLK_GPIO, 1u << CCD1_CLK_PIN);
}
static void CLK_ClrVal(void)
{
  GPIO_ClearPinsOutput(CCD1_CLK_GPIO, 1u << CCD1_CLK_PIN);
}
static void SI_SetVal(void)
{
  GPIO_SetPinsOutput(CCD1_SI_GPIO, 1u << CCD1_SI_PIN);
}
static void SI_ClrVal(void)
{
  GPIO_ClearPinsOutput(CCD1_SI_GPIO, 1u << CCD1_SI_PIN);
}
void CCD1_exposure(void)
{
    uint32_t i;

    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    SamplingDelay();
    CLK_ClrVal();
    
    for(i=0; i<128; i++) 
    {
      SamplingDelay();
      CLK_SetVal();       
      SamplingDelay();
      CLK_ClrVal();
    }
}

float CCD1_ImageCapture_Data(uint8_t * ImageData) 
{
    uint32_t i;
    float threshold;
    uint16_t max ,min ;

    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */   
    getadc1_poll_value(ImageData); //第1点
    max = *ImageData; min = *ImageData;
    ImageData ++ ;
    CLK_ClrVal();           /* CLK = 0 */

    for(i=0; i<128; i++) 
    {
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        getadc1_poll_value(ImageData);//第2-129点 
        if(i < 127)
        {
          if(*ImageData  > max)
          {
              max = *ImageData;
          }
          if(*ImageData < min)
          {
              min = *ImageData;
          }
        }        
        
        ImageData ++ ;
        CLK_ClrVal();       /* CLK = 0 */
    }
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */
    
     /*动态阈值*/
    threshold = 0.5*(max+min); 
    return threshold;  
}

/*******************************************************************************
* Function Name  : ImageCapture
* Description    : CCD采样程序
* Input          : * ImageData   采样数组 getadc0value(ImageData)
* Output         : None
* Return         : 当前128个数的阈值
*******************************************************************************/
float ccd1_ImageCapture(uint8_t * ImageData) 
{
    uint32_t i;
    float threshold;
    uint16_t max ,min ;

    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */
    SamplingDelay();
    CLK_ClrVal();
    
    for(i=0; i<128; i++) 
    {
      SamplingDelay();
      CLK_SetVal();       
      SamplingDelay();
      CLK_ClrVal();
    }
    delay_ms(10);
    
    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */   
    getadc1_poll_value(ImageData); //第1点
    max = *ImageData; min = *ImageData;
    ImageData ++ ;
    CLK_ClrVal();           /* CLK = 0 */

    for(i=0; i<128; i++) 
    {
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        getadc1_poll_value(ImageData);//第2-129点 
        if(i < 127)
        {
          if(*ImageData  > max)
          {
              max = *ImageData;
          }
          if(*ImageData < min)
          {
              min = *ImageData;
          }
        }        
        
        ImageData ++ ;
        CLK_ClrVal();       /* CLK = 0 */
    }
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    CLK_ClrVal();           /* CLK = 0 */
    
     /*动态阈值*/
    threshold = 0.5*(max+min); 
    return threshold;  
    
}

