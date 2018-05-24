/*********************2017-2017, NJUT, Edu.********************* 
FileName: speed.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.10
Description:    ftm ��������ģʽ����   ʹ��pit2��ʱʱ�����������  
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/10     1.0     �ļ�����   
  *          Smartcar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      FunctionPin            |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |      FTM1 (PHA)             |        B0                   |
  *          |      FTM1 (PHB)             |        B1                   |
  *          |      FTM2 (PHA)             |        B18                  |
  *          |      FTM2 (PHB)             |        B19                  |
  *          +-----------------------------+-----------------------------+
***************************************************************/ 
#include "fsl_ftm.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"

#include "speed_quad.h"
#include "pit.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* ��ʱ�� ��ʱʱ��ѡ������ 
���٣�3.9m/s
�����巽ʽ�� 500�߱����� ��Լ 9K Hz   90/ms 9000/1ms
�������뷽ʽ�� 500�߱����� ��Լ 36K Hz 36/ms 3600/1ms

���٣�0.414m/s
�����巽ʽ�� 500�߱����� , 1K Hz  10/ms 1000/1ms
�������뷽ʽ�� 500�߱����� ��Լ 4K Hz 40/ms 4000/1ms
*/
#define TEST_TIME 1000   //�������ʱ���� ��λ us

/*���� speed2 ���Сֵ*/
#define SPEED2_MAX      14000
#define SPEED2_MIN      0

/*���� speed1 ���Сֵ*/
#define SPEED1_MAX      14000
#define SPEED1_MIN      0


/*speed2-----------------------------------------------------------------------*/
/* The Flextimer instance/channel used for board */
#define SPEED2_FTM_BASEADDR FTM2

/* FTM channel pair used for the quad capture, uses channels 0 and 1 */
#define SPEED2_FTM_INPUT_CAPTURE_CHANNEL_QUADA kFTM_Chnl_0
#define SPEED2_FTM_INPUT_CAPTURE_CHANNEL_QUADB kFTM_Chnl_1

/* Get source clock for FTM driver */
#define FTM2_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

/*speed1-----------------------------------------------------------------------*/
/* The Flextimer instance/channel used for board */
#define SPEED1_FTM_BASEADDR FTM1

/* FTM channel pair used for the quad capture, uses channels 0 and 1 */
#define SPEED1_FTM_INPUT_CAPTURE_CHANNEL_QUADA kFTM_Chnl_0
#define SPEED1_FTM_INPUT_CAPTURE_CHANNEL_QUADB kFTM_Chnl_1

/* Get source clock for FTM driver */
#define FTM1_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static int32_t capture2Val;
static int32_t capture1Val;


/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
* Function Name  : PIT2_IRQHandler
* Description    : ��ʱ��2 �жϷ�����
* Input          : None
* Output         : None
* Return         : None
 ******************************************************************************/
void PIT2_IRQHandler(void)
{
  /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_2 , kPIT_TimerFlag);

    /*��ȡ����ֵ*/
    capture2Val = SPEED2_FTM_BASEADDR->CNT;     
    capture1Val = SPEED1_FTM_BASEADDR->CNT;     
    
    /*��ȡ����*/
    if((FTM_QDCTRL_QUADIR_MASK != ((SPEED2_FTM_BASEADDR->QDCTRL) & FTM_QDCTRL_QUADIR_MASK) ) &&(capture2Val != 0))
    {
      capture2Val = capture2Val - SPEED2_MAX;       
    }
    if((FTM_QDCTRL_QUADIR_MASK != ((SPEED1_FTM_BASEADDR->QDCTRL) & FTM_QDCTRL_QUADIR_MASK) ) &&(capture1Val != 0))
    {
      capture1Val = capture1Val - SPEED1_MAX;       
    }
        
    /*��������*/
    SPEED2_FTM_BASEADDR->CNT = 0;
    SPEED1_FTM_BASEADDR->CNT = 0;
}

/*---------------------------------SPEED2-------------------------------------*/

/*******************************************************************************
* Function Name  : speed2_quad_init
* Description    : FTM2 ��ʼ�� ����ͨ��0 ��1 �������� �������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void speed2_quad_init(void)
{
    ftm_config_t ftmInfo;
    ftm_phase_params_t phaseParam;

    /* Initialize FTM2 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Affects PORTB_PCR18 register */
    PORT_SetPinMux(PORTB, 18U, kPORT_MuxAlt6);
    PORT_SetPinMux(PORTB, 19U, kPORT_MuxAlt6);

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(SPEED2_FTM_BASEADDR, &ftmInfo);
    
    SPEED2_FTM_BASEADDR->COMBINE = 0;
    SPEED2_FTM_BASEADDR->DEADTIME = 0;
    SPEED2_FTM_BASEADDR->SYNCONF = 0;
    SPEED2_FTM_BASEADDR->SYNC = 0;

    /* Setup rise edge capture on a FTM1 channe0 ,1 */
    FTM_SetupInputCapture(SPEED2_FTM_BASEADDR, SPEED2_FTM_INPUT_CAPTURE_CHANNEL_QUADA, kFTM_RisingEdge, 0);
    FTM_SetupInputCapture(SPEED2_FTM_BASEADDR, SPEED2_FTM_INPUT_CAPTURE_CHANNEL_QUADB, kFTM_RisingEdge, 0);

    /* Set the max timer  */
    SPEED2_FTM_BASEADDR->MOD  = SPEED2_MAX;
    
    /* Set the min timer  */
    SPEED2_FTM_BASEADDR->CNTIN = SPEED2_MIN;
    
    SPEED2_FTM_BASEADDR->CNT = 0;    
    
    phaseParam.enablePhaseFilter = false;
    phaseParam.phaseFilterVal = 10;
    phaseParam.phasePolarity = kFTM_QuadPhaseNormal;
    
    /* Setup Quad Decode on a FTM channel  */
    /*kFTM_QuadPhaseEncode:�� A ���� B ����ģʽ  kFTM_QuadCountAndDir:�����ͷ������ģʽ*/
    FTM_SetupQuadDecode(SPEED2_FTM_BASEADDR, &phaseParam, &phaseParam, kFTM_QuadPhaseEncode);

    /*���Ӝy��*/
    FTM_StartTimer(SPEED2_FTM_BASEADDR, kFTM_SystemClock);

    /*��ʱ��2��ʱ TEST_TIME */
    PitConfig(kPIT_Chnl_2, TEST_TIME);
}

/*******************************************************************************
* Function Name  : speed2_quad_get
* Description    : FTM2 �������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int32_t speed2_quad_get(void)
{       
    return capture2Val;
}

/*----------------------------SPEED1------------------------------------------*/

/*******************************************************************************
* Function Name  : speed1_init
* Description    : FTM1 ��ʼ�� ����ͨ��0 ��1 �������� �������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void speed1_quad_init(void)
{
    ftm_config_t ftmInfo;
    ftm_phase_params_t phaseParam;

    /* Initialize FTM2 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Affects PORTB_PCR18 register */
    PORT_SetPinMux(PORTB, 0U, kPORT_MuxAlt6);
    PORT_SetPinMux(PORTB, 1U, kPORT_MuxAlt6);

    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(SPEED1_FTM_BASEADDR, &ftmInfo);
    
    SPEED1_FTM_BASEADDR->COMBINE = 0;
    SPEED1_FTM_BASEADDR->DEADTIME = 0;
    SPEED1_FTM_BASEADDR->SYNCONF = 0;
    SPEED1_FTM_BASEADDR->SYNC = 0;

    /* Setup rise edge capture on a FTM1 channe0 ,1 */
    FTM_SetupInputCapture(SPEED1_FTM_BASEADDR, SPEED1_FTM_INPUT_CAPTURE_CHANNEL_QUADA, kFTM_RisingEdge, 0);
    FTM_SetupInputCapture(SPEED1_FTM_BASEADDR, SPEED1_FTM_INPUT_CAPTURE_CHANNEL_QUADB, kFTM_RisingEdge, 0);

    /* Set the max timer  */
    SPEED1_FTM_BASEADDR->MOD  = SPEED1_MAX;
    
    /* Set the min timer  */
    SPEED1_FTM_BASEADDR->CNTIN = SPEED1_MIN;
    
    SPEED1_FTM_BASEADDR->CNT = 0;    
    
    phaseParam.enablePhaseFilter = false;
    phaseParam.phaseFilterVal = 10;
    phaseParam.phasePolarity = kFTM_QuadPhaseNormal;
    
    /* Setup Quad Decode on a FTM channel  */
    /*kFTM_QuadPhaseEncode:�� A ���� B ����ģʽ  kFTM_QuadCountAndDir:�����ͷ������ģʽ*/
    FTM_SetupQuadDecode(SPEED1_FTM_BASEADDR, &phaseParam, &phaseParam, kFTM_QuadPhaseEncode);

    /*���Ӝy��*/
    FTM_StartTimer(SPEED1_FTM_BASEADDR, kFTM_SystemClock);

    /*��ʱ��2��ʱ TEST_TIME */
    //PitConfig(kPIT_Chnl_2, TEST_TIME);    
}

/*******************************************************************************
* Function Name  : speed1_get
* Description    : FTM1 �������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
int32_t speed1_quad_get(void)
{
    return capture1Val ;
}

/*******************************************************************************
* Function Name  : speed_quad_get
* Description    : FTM1 FTM2 �������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void speed_quad_get(uint32_t *speed1, uint32_t *speed2)
{
    /*��ȡ����ֵ*/
    capture2Val = SPEED2_FTM_BASEADDR->CNT;     
    capture1Val = SPEED1_FTM_BASEADDR->CNT;     
    
    /*��ȡ����*/
    if((FTM_QDCTRL_QUADIR_MASK != ((SPEED2_FTM_BASEADDR->QDCTRL) & FTM_QDCTRL_QUADIR_MASK) ) &&(capture2Val != 0))
    {
      capture2Val = capture2Val - SPEED2_MAX;       
    }
    if((FTM_QDCTRL_QUADIR_MASK != ((SPEED1_FTM_BASEADDR->QDCTRL) & FTM_QDCTRL_QUADIR_MASK) ) &&(capture1Val != 0))
    {
      capture1Val = capture1Val - SPEED1_MAX;       
    }
        
    /*��������*/
    SPEED2_FTM_BASEADDR->CNT = 0;
    SPEED1_FTM_BASEADDR->CNT = 0;
    
    *speed1 = capture1Val;
    *speed2 = capture2Val;    
}
