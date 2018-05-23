/*********************2017-2017, NJUT, Edu.********************* 
FileName: pdb.c 
Author:  孙冬梅       Version :  1.0        Date: 2013.04.05
Description:    1. 用pdb进行延时      
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    13/04/05     1.0     文件创建   
***************************************************************/   

#include "board.h"
#include "fsl_pdb.h"
#include "fsl_debug_console.h"

#include "pdb.h"
#include "led.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_PDB_BASE PDB0
#define BOARD_PDB_IRQ_ID PDB0_IRQn
#define BOARD_PDB_IRQ_HANDLER PDB0_IRQHandler

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_PdbDelayInterruptCounter;
volatile bool g_PdbDelayInterruptFlag;

/*******************************************************************************
* Function Name  : delay_ms
* Description    : 延时ms
* Input          : nms
* Output         : None
* Return         : None
 ******************************************************************************/
void BOARD_PDB_IRQ_HANDLER(void)
{
    PDB_ClearStatusFlags(BOARD_PDB_BASE, kPDB_DelayEventFlag);
    g_PdbDelayInterruptCounter++;
    g_PdbDelayInterruptFlag = true;
}
/*******************************************************************************
* Function Name  : delay_ms
* Description    : 延时ms
* Input          : nms
* Output         : None
* Return         : None
*******************************************************************************/
void pdb_delay_us(uint32_t nus)
{	 		  	  
    pdb_config_t pdbConfigStruct;

    EnableIRQ(BOARD_PDB_IRQ_ID);

    /* Configure the PDB counter. */
    /*
     * pdbConfigStruct.loadValueMode = kPDB_LoadValueImmediately;
     * pdbConfigStruct.prescalerDivider = kPDB_PrescalerDivider1;
     * pdbConfigStruct.dividerMultiplicationFactor = kPDB_DividerMultiplicationFactor1;
     * pdbConfigStruct.triggerInputSource = kPDB_TriggerSoftware;
     * pdbConfigStruct.enableContinuousMode = false;
     */
    PDB_GetDefaultConfig(&pdbConfigStruct);
    PDB_Init(BOARD_PDB_BASE, &pdbConfigStruct);

    /* Configure the delay interrupt. */
    PDB_SetModulusValue(BOARD_PDB_BASE, nus); //1us

    /* The available delay value is less than or equal to the modulus value. */
    PDB_SetCounterDelayValue(BOARD_PDB_BASE, nus);
    PDB_EnableInterrupts(BOARD_PDB_BASE, kPDB_DelayInterruptEnable);
    PDB_DoLoadValues(BOARD_PDB_BASE);
	
    g_PdbDelayInterruptFlag = false;
    PDB_DoSoftwareTrigger(BOARD_PDB_BASE);    
    while (!g_PdbDelayInterruptFlag)
    {
    }
    PRINTF("PDB Delay Interrupt Counter: %d\r\n", g_PdbDelayInterruptCounter);
}   
		    								   
