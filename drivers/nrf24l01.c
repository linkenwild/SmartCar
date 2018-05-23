/******************************2017-2017, NJTECH, Edu.************************** 
FileName: nrf24l01.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.30
Description:    NRF24L01����ʹ��Ӳ��spi
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     �ļ�����   
  *          SmartCar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      nrf24l01               |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |        CE                   |        E0                  |
  *          |        MOSI                 |        E1                  |
  *          |        SCK                  |        E2                  |
  *          |        MISO                 |        E3                  |
  *          |        PCS0                 |        E4                  |
  *          |        IRQ                  |        E5                  |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "board.h"
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "stdio.h"

#include "nrf24l01.h"
#include "delay.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RF_DSPI_MASTER_BASEADDR SPI1
#define DSPI_MASTER_CLK_SRC DSPI1_CLK_SRC
#define RF_DSPI_MASTER_PCS_FOR_INIT kDSPI_Pcs0
#define RF_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
#define EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER kDSPI_MasterPcs0
    
#define TRANSFER_SIZE 256U        /*! Transfer dataSize */
#define TRANSFER_BAUDRATE 500000U /*! Transfer baudrate - 500k */
////////////////////////////////////////////////////////////////////////////////////////

//NRF24L01�Ĵ�����������
#define SPI_READ_REG    0x00  //�����üĴ���,��5λΪ�Ĵ�����ַ
#define SPI_WRITE_REG   0x20  //д���üĴ���,��5λΪ�Ĵ�����ַ
#define RD_RX_PLOAD     0x61  //��RX��Ч����,1~32�ֽ�
#define WR_TX_PLOAD     0xA0  //дTX��Ч����,1~32�ֽ�
#define FLUSH_TX        0xE1  //���TX FIFO�Ĵ���.����ģʽ����
#define FLUSH_RX        0xE2  //���RX FIFO�Ĵ���.����ģʽ����
#define REUSE_TX_PL     0xE3  //����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
#define NOP             0xFF  //�ղ���,����������״̬�Ĵ���	 
//SPI(NRF24L01)�Ĵ�����ַ
#define CONFIG          0x00  //���üĴ�����ַ;bit0:1����ģʽ,0����ģʽ;bit1:��ѡ��;bit2:CRCģʽ;bit3:CRCʹ��;
                              //bit4:�ж�MAX_RT(�ﵽ����ط������ж�)ʹ��;bit5:�ж�TX_DSʹ��;bit6:�ж�RX_DRʹ��
#define EN_AA           0x01  //ʹ���Զ�Ӧ����  bit0~5,��Ӧͨ��0~5
#define EN_RXADDR       0x02  //���յ�ַ����,bit0~5,��Ӧͨ��0~5
#define SETUP_AW        0x03  //���õ�ַ���(��������ͨ��):bit1,0:00,3�ֽ�;01,4�ֽ�;02,5�ֽ�;
#define SETUP_RETR      0x04  //�����Զ��ط�;bit3:0,�Զ��ط�������;bit7:4,�Զ��ط���ʱ 250*x+86us
#define RF_CH           0x05  //RFͨ��,bit6:0,����ͨ��Ƶ��;
#define RF_SETUP        0x06  //RF�Ĵ���;bit3:��������(0:1Mbps,1:2Mbps);bit2:1,���书��;bit0:�������Ŵ�������
#define STATUS          0x07  //״̬�Ĵ���;bit0:TX FIFO����־;bit3:1,��������ͨ����(���:6);bit4,�ﵽ�����ط�
                              //bit5:���ݷ�������ж�;bit6:���������ж�;
#define MAX_TX  	    0x10  //�ﵽ����ʹ����ж�
#define TX_OK       	0x20  //TX��������ж�
#define RX_OK   	    0x40  //���յ������ж�

#define OBSERVE_TX      0x08  //���ͼ��Ĵ���,bit7:4,���ݰ���ʧ������;bit3:0,�ط�������
#define CD              0x09  //�ز����Ĵ���,bit0,�ز����;
#define RX_ADDR_P0      0x0A  //����ͨ��0���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P1      0x0B  //����ͨ��1���յ�ַ,��󳤶�5���ֽ�,���ֽ���ǰ
#define RX_ADDR_P2      0x0C  //����ͨ��2���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P3      0x0D  //����ͨ��3���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P4      0x0E  //����ͨ��4���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define RX_ADDR_P5      0x0F  //����ͨ��5���յ�ַ,����ֽڿ�����,���ֽ�,����ͬRX_ADDR_P1[39:8]���;
#define TX_ADDR         0x10  //���͵�ַ(���ֽ���ǰ),ShockBurstTMģʽ��,RX_ADDR_P0��˵�ַ���
#define RX_PW_P0        0x11  //��������ͨ��0��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P1        0x12  //��������ͨ��1��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P2        0x13  //��������ͨ��2��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P3        0x14  //��������ͨ��3��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P4        0x15  //��������ͨ��4��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define RX_PW_P5        0x16  //��������ͨ��5��Ч���ݿ��(1~32�ֽ�),����Ϊ0��Ƿ�
#define FIFO_STATUS     0x17  //FIFO״̬�Ĵ���;bit0,RX FIFO�Ĵ����ձ�־;bit1,RX FIFO����־;bit2,3,����
                              //bit4,TX FIFO�ձ�־;bit5,TX FIFO����־;bit6,1,ѭ��������һ���ݰ�.0,��ѭ��;
/**********************************************************************************************************/
//NRF2401Ƭѡ�ź�
#define Clr_NRF24L01_CE      {GPIO_ClearPinsOutput(GPIOE, 1 << 0);}
#define Set_NRF24L01_CE      {GPIO_SetPinsOutput(GPIOE, 1 << 0);}

//SPIƬѡ�ź�	
#define Clr_NRF24L01_CSN     {GPIO_ClearPinsOutput(GPIOE, 1 << 4);}
#define Set_NRF24L01_CSN     {GPIO_SetPinsOutput(GPIOE, 1 << 4);}
    
//NRF2401_IRQ��������
#define READ_NRF24L01_IRQ       (GPIO_ReadPinInput(GPIOE, 5U))

//NRF24L01���ͽ������ݿ�ȶ���
#define TX_ADR_WIDTH    5                               //5�ֽڵĵ�ַ���
#define RX_ADR_WIDTH    5                               //5�ֽڵĵ�ַ���
#define TX_PLOAD_WIDTH  32                              //20�ֽڵ��û����ݿ��
#define RX_PLOAD_WIDTH  32                              //20�ֽڵ��û����ݿ��

const uint8_t static  TX_ADDRESS[TX_ADR_WIDTH]={0x73,0x75,0x6E,0x64,0x6D}; //���͵�ַ
const uint8_t static  RX_ADDRESS[RX_ADR_WIDTH]={0x73,0x75,0x6E,0x64,0x6D}; //���͵�ַ	
/////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* DSPI user callback */
void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData);
/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t masterRxData[TRANSFER_SIZE] = {0U};
uint8_t masterTxData[TRANSFER_SIZE] = {0U};
dspi_master_handle_t g_m_handle;

volatile bool isTransferCompleted = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
* Function Name  : spi1_pin_init
* Description    : spi1 �ܽų�ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void spi1_pin_init(void)
{
    gpio_pin_config_t config = 
    {
      kGPIO_DigitalOutput  , 0,
    };
    
    CLOCK_EnableClock(kCLOCK_PortE);
    /* SPI1 */
    PORT_SetPinMux(PORTE, 1U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTE, 2U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTE, 3U, kPORT_MuxAlt2);
    //PORT_SetPinMux(PORTE, 4U, kPORT_MuxAlt2);
    
    /*CE ��� */
    PORT_SetPinMux(PORTE, 0U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOE, 0U, &config);
    /*�����*/
    port_pin_config_t prot_config = {0};    
    prot_config.pullSelect = kPORT_PullDisable;//ʧ���ڲ�����
    prot_config.mux = kPORT_MuxAsGpio;
    prot_config.openDrainEnable = kPORT_OpenDrainDisable;//ʧ�ܿ�©���
    PORT_SetPinConfig(PORTE, 4, &prot_config); 
    
    PORT_SetPinMux(PORTE, 4U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOE, 4U, &config);
    /*IRQ*/
    PORT_SetPinMux(PORTE, 5U, kPORT_MuxAsGpio);
    config.pinDirection = kGPIO_DigitalInput;
    GPIO_PinInit(GPIOE, 5U, &config);
}
/*******************************************************************************
* Function Name  : spi1_init
* Description    : spi1 ��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void spi1_init(void)
{

    uint32_t srcClock_Hz;
    dspi_master_config_t masterConfig;

    /* Master config */
    masterConfig.whichCtar = kDSPI_Ctar0;
    masterConfig.ctarConfig.baudRate = TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.bitsPerFrame = 8U;
    masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
    masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
    masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
    masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;
    masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / TRANSFER_BAUDRATE;

    masterConfig.whichPcs = RF_DSPI_MASTER_PCS_FOR_INIT;
    masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;

    masterConfig.enableContinuousSCK = false;
    masterConfig.enableRxFifoOverWrite = false;
    masterConfig.enableModifiedTimingFormat = false;
    masterConfig.samplePoint = kDSPI_SckToSin0Clock;

    srcClock_Hz = CLOCK_GetFreq(DSPI_MASTER_CLK_SRC);
    DSPI_MasterInit(RF_DSPI_MASTER_BASEADDR, &masterConfig, srcClock_Hz);

  /*�ж�����*/    

}

void DSPI_MasterUserCallback(SPI_Type *base, dspi_master_handle_t *handle, status_t status, void *userData)
{
    if (status == kStatus_Success)
    {
        isTransferCompleted = true;
    }
}

/*******************************************************************************
* Function Name  : SPI1_ReadWriteByte
* Description    : spi1 ���߶�дһ���ֽ�
* Input          : TxData Ҫд�����
* Output         : None
* Return         : ��������
*******************************************************************************/
static uint8_t SPI1_ReadWriteByte(uint8_t TxData)
{
  dspi_transfer_t masterXfer;
  uint8_t txdata;
  txdata = TxData;
  /* Set up master transfer for interrupt*/
  DSPI_MasterTransferCreateHandle(RF_DSPI_MASTER_BASEADDR, &g_m_handle, DSPI_MasterUserCallback, NULL);

  /*Start master transfer*/
  masterXfer.txData = &txdata;
  masterXfer.rxData = masterRxData;
  masterXfer.dataSize = 1;
  masterXfer.configFlags = kDSPI_MasterCtar0 | EXAMPLE_DSPI_MASTER_PCS_FOR_TRANSFER | kDSPI_MasterPcsContinuous;
  
  isTransferCompleted = false;
  /* Start master transfer for nonblock*/
//  DSPI_MasterTransferNonBlocking(RF_DSPI_MASTER_BASEADDR, &g_m_handle, &masterXfer);
//  while(!isTransferCompleted);
  /* Start master transfer for block*/
  DSPI_MasterTransferBlocking(RF_DSPI_MASTER_BASEADDR, &masterXfer);

  return masterRxData[0];
}

/*******************************************************************************
* Function Name  : NRF24L01_Check
* Description    : �ϵ���NRF24L01�Ƿ���λ
* Input          : None
* Output         : None
* Return         : true:��ʾ��λ;false����ʾ����λ
*******************************************************************************/
bool NRF24L01_Check(void)
{
  uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
  uint8_t buf1[5];
  uint8_t i;   

  NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//д��5���ֽڵĵ�ַ.	
  NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //����д��ĵ�ַ  	
  for(i=0;i<5;i++)if(buf1[i]!=0XA5)break;					   
  if(i!=5)return false;                               //NRF24L01����λ	
  return true;		                                //NRF24L01��λ
}	 	 

/*******************************************************************************
* Function Name  : nrf24l01_init
* Description    : nrf24l01 ��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NRF24L01_Init(void)
{
  spi1_pin_init();

  Set_NRF24L01_CE;      //��ʼ��ʱ������
  Set_NRF24L01_CSN;     //��ʼ��ʱ������
  
  spi1_init();
  
  Clr_NRF24L01_CE;       //ʹ��24L01
  Set_NRF24L01_CSN;      //SPIƬѡȡ��  
}

/*******************************************************************************
* Function Name  : NRF24L01_Write_Reg
* Description    : ͨ��SPIд�Ĵ���
* Input          : regaddr:Ҫд�ļĴ��� data:Ҫд������
* Output         : None
* Return         : status ,״ֵ̬
*******************************************************************************/
static uint8_t NRF24L01_Write_Reg(uint8_t regaddr,uint8_t data)
{
  uint8_t status;	
  Clr_NRF24L01_CSN;                    //ʹ��SPI����
  status =SPI1_ReadWriteByte(regaddr); //���ͼĴ����� 
  SPI1_ReadWriteByte(data);            //д��Ĵ�����ֵ
  Set_NRF24L01_CSN;                    //��ֹSPI����	   
  return(status);       		//����״ֵ̬
}

/*******************************************************************************
* Function Name  : NRF24L01_Read_Reg
* Description    : ͨ��SPI���Ĵ���
* Input          : regaddr:Ҫ���ļĴ���
* Output         : None
* Return         : reg_val ,����������
*******************************************************************************/
static uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
  uint8_t reg_val;	    
  Clr_NRF24L01_CSN;                //ʹ��SPI����		
  SPI1_ReadWriteByte(regaddr);     //���ͼĴ�����
  reg_val=SPI1_ReadWriteByte(0XFF);//��ȡ�Ĵ�������
  Set_NRF24L01_CSN;                //��ֹSPI����		    
  return(reg_val);                 //����״ֵ̬
}

/*******************************************************************************
* Function Name  : NRF24L01_Read_Buf
* Description    : ��ָ��λ�ö���ָ�����ȵ�����
* Input          : regaddr:Ҫ���ļĴ���  datalen:ָ������
* Output         : *pBuf:��������ָ��
* Return         : status ,������״ֵ̬
*******************************************************************************/
static uint8_t NRF24L01_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{
  uint8_t status,uint8_t_ctr;	       
  Clr_NRF24L01_CSN;                     //ʹ��SPI����
  status=SPI1_ReadWriteByte(regaddr);   //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬   	   
  for(uint8_t_ctr=0;uint8_t_ctr<datalen;uint8_t_ctr++)pBuf[uint8_t_ctr]=SPI1_ReadWriteByte(0XFF);//��������
  Set_NRF24L01_CSN;                     //�ر�SPI����
  return status;                        //���ض�����״ֵ̬
}

/*******************************************************************************
* Function Name  : NRF24L01_Write_Buf
* Description    : ��ָ��λ��дָ�����ȵ�����
* Input          : regaddr:Ҫд�ļĴ���  datalen:ָ������
* Output         : *pBuf:Ҫд�������ָ��
* Return         : status ,������״ֵ̬
*******************************************************************************/
static uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
  uint8_t status,uint8_t_ctr;	    
  Clr_NRF24L01_CSN;                                    //ʹ��SPI����
  status = SPI1_ReadWriteByte(regaddr);                //���ͼĴ���ֵ(λ��),����ȡ״ֵ̬
  for(uint8_t_ctr=0; uint8_t_ctr<datalen; uint8_t_ctr++)SPI1_ReadWriteByte(*pBuf++); //д������	 
  Set_NRF24L01_CSN;                                    //�ر�SPI����
  return status;                                       //���ض�����״ֵ̬
}				   

/*******************************************************************************
* Function Name  : NRF24L01_TxPacket
* Description    : ����NRF24L01����һ������
* Input          : txbuf:�����������׵�ַ
* Output         : None
* Return         : �������״��
*******************************************************************************/
static bool NRF24L01_TxPacket(uint8_t *txbuf)
{
  uint8_t state;   
  Clr_NRF24L01_CE;
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//д���ݵ�TX BUF  32���ֽ�
  Set_NRF24L01_CE;                                     //��������	   
  while(READ_NRF24L01_IRQ!=0);                              //�ȴ��������
  state=NRF24L01_Read_Reg(STATUS);                     //��ȡ״̬�Ĵ�����ֵ	   
  NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //���TX_DS��MAX_RT�жϱ�־
  if(state&MAX_TX)                                     //�ﵽ����ط�����
  {
          NRF24L01_Write_Reg(FLUSH_TX,0xff);               //���TX FIFO�Ĵ��� 
          return false; 
  }
  if(state&TX_OK)                                      //�������
  {
          return true;
  }
  return false;                                         //����ԭ����ʧ��
}

/*******************************************************************************
* Function Name  : NRF24L01_RxPacket
* Description    : ����NRF24L01����һ������
* Input          : rxbuf:���������׵�ַ
* Output         : None
* Return         :  �������״��
*******************************************************************************/
static bool NRF24L01_RxPacket(uint8_t *rxbuf)
{
  uint8_t state;		    							      
  state=NRF24L01_Read_Reg(STATUS);                //��ȡ״̬�Ĵ�����ֵ    	 
  NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //���TX_DS��MAX_RT�жϱ�־
  if(state&RX_OK)                                 //���յ�����
  {
          NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//��ȡ����
          NRF24L01_Write_Reg(FLUSH_RX,0xff);          //���RX FIFO�Ĵ��� 
          return true; 
  }	   
  return false;                                      //û�յ��κ�����
}

/*******************************************************************************
* Function Name  : RX_Mode
* Description    : �ú�����ʼ��NRF24L01��RXģʽ
*  ����RX��ַ,дRX���ݿ��,ѡ��RFƵ��,�����ʺ�LNA HCURR
*  ��CE��ߺ�,������RXģʽ,�����Խ�������
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void RX_Mode(void)
{
	Clr_NRF24L01_CE;	  
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//дRX�ڵ��ַ
	  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);    //ʹ��ͨ��0���Զ�Ӧ��    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//ʹ��ͨ��0�Ľ��յ�ַ  	 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,40);	     //����RFͨ��Ƶ��		  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//ѡ��ͨ��0����Ч���ݿ�� 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); //����TX�������,0db����,2Mbps,���������濪��   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG, 0x0f);  //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
  	Set_NRF24L01_CE;                                //CEΪ��,�������ģʽ 
}						 

/*******************************************************************************
* Function Name  : TX_Mode
* Description    : �ú�����ʼ��NRF24L01��TXģʽ
*  ����TX��ַ,дTX���ݿ��,����RX�Զ�Ӧ��ĵ�ַ,���TX��������,ѡ��RFƵ��,
*  �����ʺ�LNA HCURR PWR_UP,CRCʹ��,��CE��ߺ�,������TXģʽ,��������
*  CEΪ�ߴ���10us,����������.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void TX_Mode(void)
{														 
  Clr_NRF24L01_CE;	    
  NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    //дTX�ڵ��ַ 
  NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK	  

  NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);     //ʹ��ͨ��0���Զ�Ӧ��    
  NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01); //ʹ��ͨ��0�Ľ��յ�ַ  
  NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);//�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
  NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,40);       //����RFͨ��Ϊ40
  NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);  //����TX�������,0db����,2Mbps,���������濪��   
  NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG,0x0e);    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
  Set_NRF24L01_CE;                                  //CEΪ��,10us����������
}		  

/******************************���� Ӧ�ú���*****************************/
 void rf1start(void)
{
  static uint32_t times;
  uint8_t disstr[TX_ADR_WIDTH];
  uint8_t buf[TX_ADR_WIDTH] = {0x00};

  NRF24L01_Init();  
  
  if(NRF24L01_Check()==true)
  {
    PRINTF("RF ģ���ʼ���ɹ���\r\n");
  }
  else  
  {
    PRINTF("RF ģ�鲻���ڣ�\r\n");
    return;
  }
  RX_Mode();
  while (1)
  {  
    if(NRF24L01_RxPacket(buf) == true)
    {
       PRINTF("\r\n RF ���յ����ݣ�\r\n"); 
        {
          PRINTF((char const*)buf);
        }
    }
    times++;
    
    delay_ms(10);  
    
    if((times%100)==0)
    {
      sprintf(disstr,"time= %d ��",times/100);
      rf1send(disstr);
    }
  }
}


void rf1send(uint8_t * str)
{ 
      PRINTF("\r\n \r\n"); 
      TX_Mode();
      PRINTF("\r\n RF1 �������ݣ�%s \r\n", str); 
      NRF24L01_TxPacket(str);
      RX_Mode();
}









