/******************************2018-2018, NJTECH, Edu.************************** 
FileName: adc_dma.c 
Author:  孙冬梅       Version :  1.0        Date: 2018.05.30
Description:    模拟量高速采集,  使用
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    18/05/30     1.0     文件创建  
  *1. 采用PIT2 定时器触发采样时间 9.604 us 触发ADC
  *2. ADC单次转换后，触发DMA0-ch1 ,将通道数据 搬到 ADCx_SC1A 
  *3. DMA0-ch1 ,将通道数据 搬到 ADCx_SC1A ,初始化后立刻执行，后地址加1
  *4. 或者将第3步修改为 产生中断，在中断服务中切换通道地址
  *          SmartCar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |         AD Input            |     Port   &  Chanel        |
  *          +-----------------------------+-----------------------------+
  *          |          ADC_IN0            |     ADC0_DP1 ADC0-ch1       |
  *          |          ADC_IN1            |     ADC0_DM1 ADC0-ch20      |
  *          |          ADC_IN2            |     ADC1_DP1 ADC1-ch1       |
  *          |          ADC_IN3            |     ADC1_DM1 ADC2-ch20      |
  *          |          ADC_IN4            |     PGA0_DP  ADC0-ch0       |
  *          |          ADC_IN5            |     PGA0_DM  ADC0-ch19      |
  *          |          ADC_IN6            |     PGA1_DP  ADC1-ch0       |
  *          |          ADC_IN7            |     PGA1_DM  ADC1-ch19      |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_adc16.h"
#include "fsl_pdb.h"

#include "adc_pdb.h"

/*******************************************************************************
 * Definitions  ADC0
 ******************************************************************************/
#define AIN_ADC16_BASE ADC0
#define AIN_ADC16_CHANNEL_GROUP 0U
#define AIN_ADC16_USER_CHANNEL0 1U   /*ADC0_DP1 */
#define AIN_ADC16_USER_CHANNEL1 20U /* ADC0_DM1 */
#define AIN_ADC16_USER_CHANNEL4 0U   /*ADC0_DP */
#define AIN_ADC16_USER_CHANNEL5 19U /* ADC0_DM */

#define AIN_ADC16_IRQn ADC0_IRQn
#define AIN_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler

#define AIN_PDB_BASE PDB0
#define AIN_PDB_IRQ_ID PDB0_IRQn
#define AIN_PDB_IRQ_HANDLER PDB0_IRQHandler

#define AIN_PDB_ADC_TRIGGER_CHANNEL 0U    /* For ADC0. */
#define AIN_PDB_ADC_PRETRIGGER_CHANNEL 0U /* For ADC0_SC1[0]. */


/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool g_Adc16ConversionDoneFlag = false;
static volatile uint32_t g_Adc16ConversionValue[4];
static volatile uint32_t g_Adc16InterruptCounter;
static adc16_channel_config_t adc16ChannelConfigStruct;
static uint32_t adc0_chanel[4] = {AIN_ADC16_USER_CHANNEL0, AIN_ADC16_USER_CHANNEL1, 
                      AIN_ADC16_USER_CHANNEL4, AIN_ADC16_USER_CHANNEL5};
static uint32_t  AIN_chanel = 0;

static volatile uint32_t g_PdbDelayInterruptCounter;
static volatile bool g_PdbDelayInterruptFlag;

/*******************************************************************************
* Function Name  : AIN_ADC16_IRQ_HANDLER_FUNC
* Description    : ADC 中断函数 获取ADC数值
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
void AIN_ADC16_IRQ_HANDLER_FUNC(void)
{
  g_Adc16ConversionDoneFlag = true;
  
  /* Read conversion result to clear the conversion completed flag. */
  g_Adc16ConversionValue[AIN_chanel] = ADC16_GetChannelConversionValue(AIN_ADC16_BASE, AIN_ADC16_CHANNEL_GROUP);
  g_Adc16InterruptCounter++;
  
  AIN_chanel++;
  if(AIN_chanel >= 4 ) AIN_chanel = 0;
  
  adc16ChannelConfigStruct.channelNumber = adc0_chanel[AIN_chanel];
  ADC16_SetChannelConfig(AIN_ADC16_BASE, AIN_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
}

/*******************************************************************************
* Function Name  : AIN0_init
* Description    : ADC 采用中断方式　初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AIN0_init(void)
{
    adc16_config_t adc16ConfigStruct;
    
    EnableIRQ(AIN_ADC16_IRQn);
    
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
    adc16ConfigStruct.resolution = kADC16_Resolution12or13Bit;
    adc16ConfigStruct.enableHighSpeed = true;
    ADC16_Init(AIN_ADC16_BASE, &adc16ConfigStruct);
    
    /*硬件平均4次*/
    ADC16_SetHardwareAverage(AIN_ADC16_BASE, kADC16_HardwareAverageCount4);
    
    ADC16_EnableHardwareTrigger(AIN_ADC16_BASE, true); /* 硬件触发 */
	
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(AIN_ADC16_BASE))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
	
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    adc16ChannelConfigStruct.channelNumber = adc0_chanel[AIN_chanel];
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = true; /* 使能中断 */
    
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
    ADC16_SetChannelConfig(AIN_ADC16_BASE, AIN_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
}

void PDB_init(void)
{
    pdb_config_t pdbConfigStruct;
    pdb_adc_pretrigger_config_t pdbAdcPreTriggerConfigStruct;

    EnableIRQ(AIN_PDB_IRQ_ID);

    /* Configure the PDB counter. */
    /*
     * pdbConfigStruct.loadValueMode = kPDB_LoadValueImmediately;
     * pdbConfigStruct.prescalerDivider = kPDB_PrescalerDivider1;
     * pdbConfigStruct.dividerMultiplicationFactor = kPDB_DividerMultiplicationFactor1;
     * pdbConfigStruct.triggerInputSource = kPDB_TriggerSoftware;
     * pdbConfigStruct.enableContinuousMode = false;
     */
    PDB_GetDefaultConfig(&pdbConfigStruct);
    PDB_Init(AIN_PDB_BASE, &pdbConfigStruct);

    /* Configure the delay interrupt. */
    PDB_SetModulusValue(AIN_PDB_BASE, 1000U);// 1 周期 20us

    /* The available delay value is less than or equal to the modulus value. */
    PDB_SetCounterDelayValue(AIN_PDB_BASE, 1000U);
    PDB_EnableInterrupts(AIN_PDB_BASE, kPDB_DelayInterruptEnable);

    /* Configure the ADC0 Pre-Trigger. */
    pdbAdcPreTriggerConfigStruct.enablePreTriggerMask = 1U << AIN_PDB_ADC_PRETRIGGER_CHANNEL;
    pdbAdcPreTriggerConfigStruct.enableOutputMask = 1U << AIN_PDB_ADC_PRETRIGGER_CHANNEL;
    pdbAdcPreTriggerConfigStruct.enableBackToBackOperationMask = 0U;
    PDB_SetADCPreTriggerConfig(AIN_PDB_BASE, AIN_PDB_ADC_TRIGGER_CHANNEL, &pdbAdcPreTriggerConfigStruct);//ADC0
    PDB_SetADCPreTriggerDelayValue(AIN_PDB_BASE, AIN_PDB_ADC_TRIGGER_CHANNEL, AIN_PDB_ADC_PRETRIGGER_CHANNEL, 200U);
    /* The available Pre-Trigger delay value is less than or equal to the modulus value. */

    /* Configure the ADC1 Pre-Trigger. */
    pdbAdcPreTriggerConfigStruct.enablePreTriggerMask = 1U << AIN_PDB_ADC_PRETRIGGER_CHANNEL;
    pdbAdcPreTriggerConfigStruct.enableOutputMask = 1U << AIN_PDB_ADC_PRETRIGGER_CHANNEL;
    pdbAdcPreTriggerConfigStruct.enableBackToBackOperationMask = 0U;
    PDB_SetADCPreTriggerConfig(AIN_PDB_BASE, AIN_PDB_ADC_TRIGGER_CHANNEL + 1, &pdbAdcPreTriggerConfigStruct);//ADC1
    PDB_SetADCPreTriggerDelayValue(AIN_PDB_BASE, AIN_PDB_ADC_TRIGGER_CHANNEL + 1, AIN_PDB_ADC_PRETRIGGER_CHANNEL, 200U);//ADC1
    /* The available Pre-Trigger delay value is less than or equal to the modulus value. */

    
    PDB_DoLoadValues(AIN_PDB_BASE);
 }

/*!
 * @brief ISR for PDB interrupt function
 */
void AIN_PDB_IRQ_HANDLER(void)
{
    PDB_ClearStatusFlags(AIN_PDB_BASE, kPDB_DelayEventFlag);
    g_PdbDelayInterruptCounter++;
    g_PdbDelayInterruptFlag = true;
}

#include "fsl_pit.h"
/*******************************************************************************
* Function Name  : PIT1_IRQHandler
* Description    : 定时器1 中断服务函数
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
void PIT1_IRQHandler(void)
{
  /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_1 , kPIT_TimerFlag);
    /*PDB软件触发 */
    PDB_DoSoftwareTrigger(PDB0);
}

/*------------------------------------ADC1------------------------------------*/
/*******************************************************************************
 * Definitions  ADC1
 ******************************************************************************/

#define AIN_ADC16_BASE1 ADC1
#define AIN_ADC16_USER_CHANNEL2 1U   /*ADC1_DP1 */
#define AIN_ADC16_USER_CHANNEL3 20U /* ADC1_DM1 */
#define AIN_ADC16_USER_CHANNEL6 0U   /*ADC1_DP */
#define AIN_ADC16_USER_CHANNEL7 19U /* ADC1_DM */

#define AIN_ADC16_IRQn1 ADC1_IRQn
#define AIN_ADC16_IRQ_HANDLER_FUNC1 ADC1_IRQHandler

#define AIN_PDB_ADC_TRIGGER_CHANNEL1 1U    /* For ADC0. */
#define AIN_PDB_ADC_PRETRIGGER_CHANNEL1 0U /* For ADC0_SC1[0]. */


/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool g_Adc16ConversionDoneFlag1 = false;
static volatile uint32_t g_Adc16ConversionValue1[4];
static volatile uint32_t g_Adc16InterruptCounter1;
static adc16_channel_config_t adc16ChannelConfigStruct1;
static uint32_t adc1_chanel[4] = {AIN_ADC16_USER_CHANNEL2, AIN_ADC16_USER_CHANNEL3, 
                      AIN_ADC16_USER_CHANNEL6, AIN_ADC16_USER_CHANNEL7};
static uint32_t  AIN_chanel1 = 0;

/*******************************************************************************
* Function Name  : AIN_ADC16_IRQ_HANDLER_FUNC1
* Description    : ADC1 中断函数 获取ADC数值
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
void AIN_ADC16_IRQ_HANDLER_FUNC1(void)
{
  g_Adc16ConversionDoneFlag1 = true;
  
  /* Read conversion result to clear the conversion completed flag. */
  g_Adc16ConversionValue1[AIN_chanel1] = ADC16_GetChannelConversionValue(AIN_ADC16_BASE1, AIN_ADC16_CHANNEL_GROUP);
  g_Adc16InterruptCounter1++;
  
  AIN_chanel1++;
  if(AIN_chanel1 >= 4 ) AIN_chanel1 = 0;

  adc16ChannelConfigStruct1.channelNumber = adc1_chanel[AIN_chanel1];
  ADC16_SetChannelConfig(AIN_ADC16_BASE1, AIN_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct1);
}

/*******************************************************************************
* Function Name  : AIN1_init
* Description    : ADC 采用中断方式　初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void AIN1_init(void)
{
    adc16_config_t adc16ConfigStruct;
    
    EnableIRQ(AIN_ADC16_IRQn1);
    
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
    adc16ConfigStruct.resolution = kADC16_Resolution12or13Bit;
    adc16ConfigStruct.enableHighSpeed = true;
    ADC16_Init(AIN_ADC16_BASE1, &adc16ConfigStruct);
    
    /*硬件平均4次*/
    ADC16_SetHardwareAverage(AIN_ADC16_BASE1, kADC16_HardwareAverageCount4);
    
    ADC16_EnableHardwareTrigger(AIN_ADC16_BASE1, true); /* 硬件触发 */
	
#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
    if (kStatus_Success == ADC16_DoAutoCalibration(AIN_ADC16_BASE1))
    {
        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
    }
    else
    {
        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
    }
	
#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

    adc16ChannelConfigStruct1.channelNumber = adc1_chanel[AIN_chanel1];
    adc16ChannelConfigStruct1.enableInterruptOnConversionCompleted = true; /* 使能中断 */
    
#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
    adc16ChannelConfigStruct1.enableDifferentialConversion = false;
#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
    ADC16_SetChannelConfig(AIN_ADC16_BASE1, AIN_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct1);
}


/*-----------------------------------------------------------------------------*/
void get_pdb_adc0_value(uint32_t* value)//ADC 由PDB触发 后进入中断 更換通道
{
  int ainch[4] = {0, 1, 4, 5};
  for(int i =0 ;i<4; i++)
  {
    PRINTF("AIN%d(ADC0-ch%d):%d ", ainch[i], adc0_chanel[i], g_Adc16ConversionValue[i]);
    *value++ = g_Adc16ConversionValue[i];
  }
  PRINTF("\r\n");
}

void get_pdb_adc1_value(uint32_t* value)//ADC 由PDB触发 后进入中断 更換通道
{
  int ainch1[4] = {2, 3, 6, 7 };
  for(int i =0 ;i<4; i++)
  {
    PRINTF("AIN%d(ADC1-ch%d):%d ", ainch1[i], adc1_chanel[i], g_Adc16ConversionValue1[i]);
    *value++ = g_Adc16ConversionValue1[i];
  }
  PRINTF("\r\n");
}









