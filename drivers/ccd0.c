/******************************2017-2017, NJTECH, Edu.************************** 
FileName: ccd0.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    ccd0驱动     使用  adc0
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
  *          |       CCD0_AO               |      ADC0_S16               |
  *          |       CCD0_CLK              |      C6                     |
  *          |       CCD0_SI               |      C7                     |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 

#include "fsl_adc16.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_uart.h"
#include "delay.h"

#include "adc0.h"
#include "ccd0.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CCD0_CLK_PORT PORTC
#define CCD0_CLK_GPIO GPIOC
#define CCD0_CLK_PIN 6U

#define CCD0_SI_PORT PORTC
#define CCD0_SI_GPIO GPIOC
#define CCD0_SI_PIN 7U

/*******************************************************************************
 * Variables
 ******************************************************************************/


/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
* Function Name  : ccd0_init
* Description    : ccd0　初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ccd0_init(void)
{
    /*pin config*/
    gpio_pin_config_t config = 
    {
        kGPIO_DigitalOutput, 0,
    };
    /* Enable LED port clock */
    CLOCK_EnableClock(kCLOCK_PortC);
	
    /* Led pin mux Configuration */
    PORT_SetPinMux(CCD0_CLK_PORT, CCD0_CLK_PIN, kPORT_MuxAsGpio);   
    PORT_SetPinMux(CCD0_SI_PORT, CCD0_SI_PIN, kPORT_MuxAsGpio);   
	
    /* Init output LED GPIO. */
    GPIO_PinInit(CCD0_CLK_GPIO, CCD0_CLK_PIN, &config);
    GPIO_PinInit(CCD0_SI_GPIO, CCD0_SI_PIN, &config);  
    
    ADC0_poll_init();
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
  GPIO_SetPinsOutput(CCD0_CLK_GPIO, 1u << CCD0_CLK_PIN);
}
static void CLK_ClrVal(void)
{
  GPIO_ClearPinsOutput(CCD0_CLK_GPIO, 1u << CCD0_CLK_PIN);
}
static void SI_SetVal(void)
{
  GPIO_SetPinsOutput(CCD0_SI_GPIO, 1u << CCD0_SI_PIN);
}
static void SI_ClrVal(void)
{
  GPIO_ClearPinsOutput(CCD0_SI_GPIO, 1u << CCD0_SI_PIN);
}

void CCD0_exposure(void)
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

float CCD0_ImageCapture_Data(uint8_t * ImageData) 
{
    uint32_t i;
    float threshold;
    uint16_t max ,min ;

    SI_SetVal();            /* SI  = 1 */
    SamplingDelay();
    CLK_SetVal();           /* CLK = 1 */
    SamplingDelay();
    SI_ClrVal();            /* SI  = 0 */   
    getadc0_poll_value(ImageData); //第1点
    max = *ImageData; min = *ImageData;
    ImageData ++ ;
    CLK_ClrVal();           /* CLK = 0 */

    for(i=0; i<128; i++) 
    {
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        getadc0_poll_value(ImageData);//第2-129点 
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
float ImageCapture(uint8_t * ImageData) 
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
    getadc0_poll_value(ImageData); //第1点
    max = *ImageData; min = *ImageData;
    ImageData ++ ;
    CLK_ClrVal();           /* CLK = 0 */

    for(i=0; i<128; i++) 
    {
        SamplingDelay();
        CLK_SetVal();       /* CLK = 1 */
        SamplingDelay();
        getadc0_poll_value(ImageData);//第2-129点 
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
* Function Name  : SendHex
* Description    : 发送16进制数据
* Input          : hex 16进制数
* Output         : None
* Return         : None
*******************************************************************************/
static void UART_PutChar(UART_Type *uartx, int8_t ch)
{
  //等待FIFO准备就绪
  while(!(uartx->S1 & UART_S1_TDRE_MASK));
    //将要发送的1个字节发给UART数据寄存器
  uartx->D = (uint8_t)ch;
}

/*******************************************************************************
* Function Name  : SendImageData
* Description    : 采集发数程序
* Input          : ImageData 要发送数据的地址
* Output         : None
* Return         : None
*******************************************************************************/
void SendImageData(uint8_t * ImageData) 
{
  int i;
  UART_PutChar(UART0, 0xFF);
  for(i=0;i<128;i++)
  {
     if(0xFF == (*ImageData))
     {
        UART_PutChar(UART0, 0xFE);
     }
     else
     {
       UART_PutChar(UART0, * ImageData);      
     }
     ImageData++;
  }
}


