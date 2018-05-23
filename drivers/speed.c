/*********************2017-2017, NJUT, Edu.********************* 
FileName: speed.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.10
Description:    ftm capture驱动  使用FTM1chanel0 FTM2chanel0  双边上升下降沿捕获 测脉冲宽度 
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/10     1.0     文件创建   
  *          Smartcar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      FunctionPin            |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |      FTM1 (PHA)             |        B0                   |
  *          |      FTM1 (PHB)None         |        B1                   |
  *          |      FTM2 (PHA)             |        B18                  |
  *          |      FTM2 (PHB)None         |        B19                  |
  *          +-----------------------------+-----------------------------+
***************************************************************/ 
#include "fsl_ftm.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

#include "speed.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*speed2----------------------------------------------------------------------*/
/* The Flextimer instance/channel used for board */
#define SPEED2_FTM_BASEADDR FTM2

/* FTM channel pair used for the dual-edge capture, channel pair 0 uses channels 0 and 1 */
#define SPEED2_FTM_INPUT_CAPTURE_CHANNEL_PAIR kFTM_Chnl_0

/* Interrupt number and interrupt handler for the FTM instance used */
#define FTM2_INTERRUPT_NUMBER FTM2_IRQn
#define FTM2_INPUT_CAPTURE_HANDLER FTM2_IRQHandler

/* Interrupt to enable and flag to read; depends on the FTM channel used for dual-edge capture */
#define FTM2_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl1InterruptEnable
#define FTM2_CHANNEL_FLAG kFTM_Chnl1Flag

/* Get source clock for FTM driver */
#define FTM2_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

/*speed1------------------------------------------------------------------------*/
/* The Flextimer instance/channel used for board */
#define SPEED1_FTM_BASEADDR FTM1

/* FTM channel pair used for the dual-edge capture, channel pair 0 uses channels 0 and 1 */
#define SPEED1_FTM_INPUT_CAPTURE_CHANNEL_PAIR kFTM_Chnl_0

/* Interrupt number and interrupt handler for the FTM instance used */
#define FTM1_INTERRUPT_NUMBER FTM1_IRQn
#define FTM1_INPUT_CAPTURE_HANDLER FTM1_IRQHandler

/* Interrupt to enable and flag to read; depends on the FTM channel used for dual-edge capture */
#define FTM1_CHANNEL_INTERRUPT_ENABLE kFTM_Chnl1InterruptEnable
#define FTM1_CHANNEL_FLAG kFTM_Chnl1Flag

/* Get source clock for FTM driver */
#define FTM1_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static volatile bool ftmIsrFlag2 = false;
static volatile bool ftmIsrFlag1 = false;

static volatile  uint32_t pulseWidth2;
static volatile  uint32_t pulseWidth1;


/*******************************************************************************
 * Code
 ******************************************************************************/

/*---------------------------------SPEED2-------------------------------------*/

/*******************************************************************************
* Function Name  : FTM2_INPUT_CAPTURE_HANDLER
* Description    : FTM2 中断服务函数 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FTM2_INPUT_CAPTURE_HANDLER(void)
{
    uint32_t capture1Val;
    uint32_t capture2Val;  
    if ((FTM_GetStatusFlags(SPEED2_FTM_BASEADDR) & FTM2_CHANNEL_FLAG) == FTM2_CHANNEL_FLAG)
    {
        /* Clear interrupt flag.*/
        FTM_ClearStatusFlags(SPEED2_FTM_BASEADDR, FTM2_CHANNEL_FLAG);
    }
    ftmIsrFlag2 = true;
    capture1Val = SPEED2_FTM_BASEADDR->CONTROLS[SPEED2_FTM_INPUT_CAPTURE_CHANNEL_PAIR * 2].CnV;
    capture2Val = SPEED2_FTM_BASEADDR->CONTROLS[(SPEED2_FTM_INPUT_CAPTURE_CHANNEL_PAIR * 2) + 1].CnV;
    
    /* FTM clock source is not prescaled and is
     * divided by 1000000 as the output is printed in microseconds
     */
    pulseWidth2 = ((capture2Val - capture1Val) + 1) / (FTM2_SOURCE_CLOCK / 1000000);
    if(pulseWidth2 > 20000000) pulseWidth2 = 0;
    SPEED2_FTM_BASEADDR->CNT = 0;
}

/*******************************************************************************
* Function Name  : speed2_init
* Description    : FTM2 初始化 测量通道0 脉冲宽度
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void speed2_init(void)
{
    ftm_config_t ftmInfo;
    ftm_dual_edge_capture_param_t edgeParam;

    /* Initialize FTM2 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Affects PORTB_PCR18 register */
    PORT_SetPinMux(PORTB, 18U, kPORT_MuxAlt3);

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(SPEED2_FTM_BASEADDR, &ftmInfo);

    edgeParam.mode = kFTM_Continuous;
    /* Set capture edges to calculate the pulse width of input signal */
    edgeParam.currChanEdgeMode = kFTM_RisingEdge;
    edgeParam.nextChanEdgeMode = kFTM_FallingEdge;

    /* Setup dual-edge capture on a FTM channel pair */
    FTM_SetupDualEdgeCapture(SPEED2_FTM_BASEADDR, SPEED2_FTM_INPUT_CAPTURE_CHANNEL_PAIR, &edgeParam, 0);

    /* Set the timer to be in free-running mode */
    SPEED2_FTM_BASEADDR->MOD = 0xFFFF;

    /* Enable channel interrupt when the second edge is detected */
    FTM_EnableInterrupts(SPEED2_FTM_BASEADDR, FTM2_CHANNEL_INTERRUPT_ENABLE);

    /* Enable at the NVIC */
    EnableIRQ(FTM2_INTERRUPT_NUMBER);

    /*啟動測速*/
    FTM_StartTimer(SPEED2_FTM_BASEADDR, kFTM_SystemClock);    
    
}
/*******************************************************************************
* Function Name  : speed2_get
* Description    : FTM2 脉冲宽度
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t speed2_get(void)
{	
    uint32_t times = 0;  
    ftmIsrFlag2 = false;
    pulseWidth2 = 0;
    while( (ftmIsrFlag2 != true) &&(times < 20000))//等待大约5ms
    {     
      times++;
    }
    return pulseWidth2 ;
}

/*----------------------------SPEED1------------------------------------------*/
/*******************************************************************************
* Function Name  : FTM1_INPUT_CAPTURE_HANDLER
* Description    : FTM2 中断服务函数 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void FTM1_INPUT_CAPTURE_HANDLER(void)
{
    uint32_t capture1Val;
    uint32_t capture2Val;  
    if ((FTM_GetStatusFlags(SPEED1_FTM_BASEADDR) & FTM1_CHANNEL_FLAG) == FTM1_CHANNEL_FLAG)
    {
        /* Clear interrupt flag.*/
        FTM_ClearStatusFlags(SPEED1_FTM_BASEADDR, FTM1_CHANNEL_FLAG);
    }
    ftmIsrFlag1 = true;
    capture1Val = SPEED1_FTM_BASEADDR->CONTROLS[SPEED1_FTM_INPUT_CAPTURE_CHANNEL_PAIR * 2].CnV;
    capture2Val = SPEED1_FTM_BASEADDR->CONTROLS[(SPEED1_FTM_INPUT_CAPTURE_CHANNEL_PAIR * 2) + 1].CnV;
    
    /* FTM clock source is not prescaled and is
     * divided by 1000000 as the output is printed in microseconds
     */
    pulseWidth1 = ((capture2Val - capture1Val) + 1) / (FTM1_SOURCE_CLOCK / 1000000);
    if(pulseWidth1 > 20000000) pulseWidth1 = 0;
    SPEED1_FTM_BASEADDR->CNT = 0;
}

/*******************************************************************************
* Function Name  : speed1_init
* Description    : FTM2 初始化 测量通道0 脉冲宽度
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void speed1_init(void)
{
    ftm_config_t ftmInfo;
    ftm_dual_edge_capture_param_t edgeParam;

    /* Initialize FTM1 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Affects PORTB_PCR18 register */
    PORT_SetPinMux(PORTB, 0U, kPORT_MuxAlt3);

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(SPEED1_FTM_BASEADDR, &ftmInfo);

    edgeParam.mode = kFTM_Continuous;
    /* Set capture edges to calculate the pulse width of input signal */
    edgeParam.currChanEdgeMode = kFTM_RisingEdge;
    edgeParam.nextChanEdgeMode = kFTM_FallingEdge;

    /* Setup dual-edge capture on a FTM channel pair */
    FTM_SetupDualEdgeCapture(SPEED1_FTM_BASEADDR, SPEED1_FTM_INPUT_CAPTURE_CHANNEL_PAIR, &edgeParam, 0);

    /* Set the timer to be in free-running mode */
    SPEED1_FTM_BASEADDR->MOD = 0xFFFF;

    /* Enable channel interrupt when the second edge is detected */
    FTM_EnableInterrupts(SPEED1_FTM_BASEADDR, FTM1_CHANNEL_INTERRUPT_ENABLE);

    /* Enable at the NVIC */
    EnableIRQ(FTM1_INTERRUPT_NUMBER);

    /*啟動測速*/
    FTM_StartTimer(SPEED1_FTM_BASEADDR, kFTM_SystemClock);
    
}
/*******************************************************************************
* Function Name  : speed1_get
* Description    : FTM1 脉冲宽度 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
uint32_t speed1_get(void)
{	
  uint32_t times = 0;  
  ftmIsrFlag1 = false;
  pulseWidth1 = 0;
    while( (ftmIsrFlag1 != true) &&(times < 20000))//等待大约5ms
    {     
      times++;
    }
    return pulseWidth1 ;
}
