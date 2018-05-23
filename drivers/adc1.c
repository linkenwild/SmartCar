/******************************2017-2017, NJTECH, Edu.************************** 
FileName: adc1.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    adc1驱动     软件触发  
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
  *          |       ADC1_S16              |         无复用              |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 

#include "fsl_adc16.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "adc1.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CCD_ADC16_BASE ADC1
#define CCD_ADC16_CHANNEL_GROUP 0U
#define CCD_ADC16_USER_CHANNEL 16U /*  ADC1_SE16 */

#define CCD_ADC16_IRQn ADC1_IRQn
#define CCD_ADC16_IRQ_HANDLER_FUNC ADC1_IRQHandler

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool g_Adc16ConversionDoneFlag = false;
static volatile uint32_t g_Adc16ConversionValue;
static volatile uint32_t g_Adc16InterruptCounter;

static adc16_channel_config_t adc16ChannelConfigStruct;

/*******************************************************************************
* Function Name  : CCD_ADC16_IRQ_HANDLER_FUNC
* Description    : ADC 中断函数 获取ADC数值
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
void CCD_ADC16_IRQ_HANDLER_FUNC(void)
{
    g_Adc16ConversionDoneFlag = true;

    /* Read conversion result to clear the conversion completed flag. */
    g_Adc16ConversionValue = ADC16_GetChannelConversionValue(CCD_ADC16_BASE, CCD_ADC16_CHANNEL_GROUP);
    g_Adc16InterruptCounter++;

}

/*******************************************************************************
* Function Name  : ADC1_init
* Description    : ADC 采用中断方式　初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC1_init(void)
{
    adc16_config_t adc16ConfigStruct;
    
    EnableIRQ(CCD_ADC16_IRQn);
    
    /*
     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adc16ConfigStruct.enableAsynchronousClock = true;
     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
     * adc16ConfigStruct.enableHighSpeed = false;
     * adc16ConfigStruct.enableLowPower = false;
     * adc16ConfigStruct.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    adc16ConfigStruct.clockSource = kADC16_ClockSourceAlt0;
    adc16ConfigStruct.clockDivider = kADC16_ClockDivider1;
    adc16ConfigStruct.resolution = kADC16_Resolution8or9Bit;
    adc16ConfigStruct.enableHighSpeed = true;
    ADC16_Init(CCD_ADC16_BASE, &adc16ConfigStruct);
    
    /*硬件平均4次*/
    ADC16_SetHardwareAverage(CCD_ADC16_BASE, kADC16_HardwareAverageCount4);
    
    ADC16_EnableHardwareTrigger(CCD_ADC16_BASE, false); /* Make sure the software trigger is used. */
	
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(CCD_ADC16_BASE))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
	
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    adc16ChannelConfigStruct.channelNumber = CCD_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* Enable the interrupt. */
    
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */

        g_Adc16InterruptCounter = 0U;	
	g_Adc16ConversionDoneFlag = false;
	ADC16_SetChannelConfig(CCD_ADC16_BASE, CCD_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);/* Start Convert. */
}

/*******************************************************************************
* Function Name  : getadcvalue
* Description    : 获取当前ADC值 阻塞转换完成
* Input          : None
* Output         : advalue
* Return         : None
*******************************************************************************/
void getadc1value(uint8_t *advalue)
{
	
        g_Adc16ConversionDoneFlag = false;
        ADC16_SetChannelConfig(CCD_ADC16_BASE, CCD_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
        while (!g_Adc16ConversionDoneFlag)
        {
        }
        
          *advalue = g_Adc16ConversionValue;
          PRINTF("ADC1 Value: %d\r\n", g_Adc16ConversionValue);
          //PRINTF("ADC1 Interrupt Count: %d\r\n", g_Adc16InterruptCounter);
        
}
/*-------------------------- 以下轮询方式 ------------------------------------*/
/*******************************************************************************
* Function Name  : ADC1_poll_init
* Description    : ADC 采用中断方式　初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ADC1_poll_init(void)
{
    adc16_config_t adc16ConfigStruct;

    /*
     * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
     * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
     * adc16ConfigStruct.enableAsynchronousClock = true;
     * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
     * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
     * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
     * adc16ConfigStruct.enableHighSpeed = false;
     * adc16ConfigStruct.enableLowPower = false;
     * adc16ConfigStruct.enableContinuousConversion = false;
     */
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    adc16ConfigStruct.clockSource = kADC16_ClockSourceAlt0;
    adc16ConfigStruct.clockDivider = kADC16_ClockDivider1;
    adc16ConfigStruct.resolution = kADC16_Resolution8or9Bit;
    adc16ConfigStruct.enableHighSpeed = true;
    ADC16_Init(CCD_ADC16_BASE, &adc16ConfigStruct);
    
    /*硬件平均4次*/
    ADC16_SetHardwareAverage(CCD_ADC16_BASE, kADC16_HardwareAverageCount4);
    
    ADC16_EnableHardwareTrigger(CCD_ADC16_BASE, false); /* Make sure the software trigger is used. */
    
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(CCD_ADC16_BASE))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */
    adc16ChannelConfigStruct.channelNumber = CCD_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
    ADC16_SetChannelConfig(CCD_ADC16_BASE, CCD_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
}

/*******************************************************************************
* Function Name  : getadc1_poll_value
* Description    : 获取当前ADC值 查询标志位完成
* Input          : None
* Output         : advalue
* Return         : None
*******************************************************************************/
void getadc1_poll_value(uint8_t *advalue)
{
	
        ADC16_SetChannelConfig(CCD_ADC16_BASE, CCD_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
        while (0U == (kADC16_ChannelConversionDoneFlag &
                      ADC16_GetChannelStatusFlags(CCD_ADC16_BASE, CCD_ADC16_CHANNEL_GROUP)))
        {
        }
        
          *advalue = CCD_ADC16_BASE->R[0];
          PRINTF("ADC1 Value: %d\r\n", g_Adc16ConversionValue);
          //PRINTF("ADC0 Interrupt Count: %d\r\n", g_Adc16InterruptCounter);
        
}

