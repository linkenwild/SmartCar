/******************************2017-2017, NJTECH, Edu.************************** 
FileName: nrf24l01.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    NRF24L01驱动使用硬件spi
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     文件创建   
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

//NRF24L01寄存器操作命令
#define SPI_READ_REG    0x00  //读配置寄存器,低5位为寄存器地址
#define SPI_WRITE_REG   0x20  //写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD     0x61  //读RX有效数据,1~32字节
#define WR_TX_PLOAD     0xA0  //写TX有效数据,1~32字节
#define FLUSH_TX        0xE1  //清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX        0xE2  //清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL     0xE3  //重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP             0xFF  //空操作,可以用来读状态寄存器	 
//SPI(NRF24L01)寄存器地址
#define CONFIG          0x00  //配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                              //bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA           0x01  //使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR       0x02  //接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW        0x03  //设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR      0x04  //建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH           0x05  //RF通道,bit6:0,工作通道频率;
#define RF_SETUP        0x06  //RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS          0x07  //状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                              //bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX  	    0x10  //达到最大发送次数中断
#define TX_OK       	0x20  //TX发送完成中断
#define RX_OK   	    0x40  //接收到数据中断

#define OBSERVE_TX      0x08  //发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD              0x09  //载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0      0x0A  //数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1      0x0B  //数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2      0x0C  //数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3      0x0D  //数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4      0x0E  //数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5      0x0F  //数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR         0x10  //发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0        0x11  //接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1        0x12  //接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2        0x13  //接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3        0x14  //接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4        0x15  //接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5        0x16  //接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define FIFO_STATUS     0x17  //FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                              //bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
/**********************************************************************************************************/
//NRF2401片选信号
#define Clr_NRF24L01_CE      {GPIO_ClearPinsOutput(GPIOE, 1 << 0);}
#define Set_NRF24L01_CE      {GPIO_SetPinsOutput(GPIOE, 1 << 0);}

//SPI片选信号	
#define Clr_NRF24L01_CSN     {GPIO_ClearPinsOutput(GPIOE, 1 << 4);}
#define Set_NRF24L01_CSN     {GPIO_SetPinsOutput(GPIOE, 1 << 4);}
    
//NRF2401_IRQ数据输入
#define READ_NRF24L01_IRQ       (GPIO_ReadPinInput(GPIOE, 5U))

//NRF24L01发送接收数据宽度定义
#define TX_ADR_WIDTH    5                               //5字节的地址宽度
#define RX_ADR_WIDTH    5                               //5字节的地址宽度
#define TX_PLOAD_WIDTH  32                              //20字节的用户数据宽度
#define RX_PLOAD_WIDTH  32                              //20字节的用户数据宽度

const uint8_t static  TX_ADDRESS[TX_ADR_WIDTH]={0x73,0x75,0x6E,0x64,0x6D}; //发送地址
const uint8_t static  RX_ADDRESS[RX_ADR_WIDTH]={0x73,0x75,0x6E,0x64,0x6D}; //发送地址	
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
* Description    : spi1 管脚初始化
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
    
    /*CE 输出 */
    PORT_SetPinMux(PORTE, 0U, kPORT_MuxAsGpio);
    GPIO_PinInit(GPIOE, 0U, &config);
    /*推挽出*/
    port_pin_config_t prot_config = {0};    
    prot_config.pullSelect = kPORT_PullDisable;//失能内部上拉
    prot_config.mux = kPORT_MuxAsGpio;
    prot_config.openDrainEnable = kPORT_OpenDrainDisable;//失能开漏输出
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
* Description    : spi1 初始化
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

  /*中断配置*/    

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
* Description    : spi1 总线读写一个字节
* Input          : TxData 要写入的字
* Output         : None
* Return         : 读出的字
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
* Description    : 上电检测NRF24L01是否在位
* Input          : None
* Output         : None
* Return         : true:表示在位;false，表示不在位
*******************************************************************************/
bool NRF24L01_Check(void)
{
  uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
  uint8_t buf1[5];
  uint8_t i;   

  NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,buf,5);//写入5个字节的地址.	
  NRF24L01_Read_Buf(TX_ADDR,buf1,5);              //读出写入的地址  	
  for(i=0;i<5;i++)if(buf1[i]!=0XA5)break;					   
  if(i!=5)return false;                               //NRF24L01不在位	
  return true;		                                //NRF24L01在位
}	 	 

/*******************************************************************************
* Function Name  : nrf24l01_init
* Description    : nrf24l01 初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void NRF24L01_Init(void)
{
  spi1_pin_init();

  Set_NRF24L01_CE;      //初始化时先拉高
  Set_NRF24L01_CSN;     //初始化时先拉高
  
  spi1_init();
  
  Clr_NRF24L01_CE;       //使能24L01
  Set_NRF24L01_CSN;      //SPI片选取消  
}

/*******************************************************************************
* Function Name  : NRF24L01_Write_Reg
* Description    : 通过SPI写寄存器
* Input          : regaddr:要写的寄存器 data:要写的数据
* Output         : None
* Return         : status ,状态值
*******************************************************************************/
static uint8_t NRF24L01_Write_Reg(uint8_t regaddr,uint8_t data)
{
  uint8_t status;	
  Clr_NRF24L01_CSN;                    //使能SPI传输
  status =SPI1_ReadWriteByte(regaddr); //发送寄存器号 
  SPI1_ReadWriteByte(data);            //写入寄存器的值
  Set_NRF24L01_CSN;                    //禁止SPI传输	   
  return(status);       		//返回状态值
}

/*******************************************************************************
* Function Name  : NRF24L01_Read_Reg
* Description    : 通过SPI读寄存器
* Input          : regaddr:要读的寄存器
* Output         : None
* Return         : reg_val ,读出的数据
*******************************************************************************/
static uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
  uint8_t reg_val;	    
  Clr_NRF24L01_CSN;                //使能SPI传输		
  SPI1_ReadWriteByte(regaddr);     //发送寄存器号
  reg_val=SPI1_ReadWriteByte(0XFF);//读取寄存器内容
  Set_NRF24L01_CSN;                //禁止SPI传输		    
  return(reg_val);                 //返回状态值
}

/*******************************************************************************
* Function Name  : NRF24L01_Read_Buf
* Description    : 在指定位置读出指定长度的数据
* Input          : regaddr:要读的寄存器  datalen:指定长度
* Output         : *pBuf:读出数据指针
* Return         : status ,读到的状态值
*******************************************************************************/
static uint8_t NRF24L01_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{
  uint8_t status,uint8_t_ctr;	       
  Clr_NRF24L01_CSN;                     //使能SPI传输
  status=SPI1_ReadWriteByte(regaddr);   //发送寄存器值(位置),并读取状态值   	   
  for(uint8_t_ctr=0;uint8_t_ctr<datalen;uint8_t_ctr++)pBuf[uint8_t_ctr]=SPI1_ReadWriteByte(0XFF);//读出数据
  Set_NRF24L01_CSN;                     //关闭SPI传输
  return status;                        //返回读到的状态值
}

/*******************************************************************************
* Function Name  : NRF24L01_Write_Buf
* Description    : 在指定位置写指定长度的数据
* Input          : regaddr:要写的寄存器  datalen:指定长度
* Output         : *pBuf:要写入的数据指针
* Return         : status ,读到的状态值
*******************************************************************************/
static uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
  uint8_t status,uint8_t_ctr;	    
  Clr_NRF24L01_CSN;                                    //使能SPI传输
  status = SPI1_ReadWriteByte(regaddr);                //发送寄存器值(位置),并读取状态值
  for(uint8_t_ctr=0; uint8_t_ctr<datalen; uint8_t_ctr++)SPI1_ReadWriteByte(*pBuf++); //写入数据	 
  Set_NRF24L01_CSN;                                    //关闭SPI传输
  return status;                                       //返回读到的状态值
}				   

/*******************************************************************************
* Function Name  : NRF24L01_TxPacket
* Description    : 启动NRF24L01发送一次数据
* Input          : txbuf:待发送数据首地址
* Output         : None
* Return         : 发送完成状况
*******************************************************************************/
static bool NRF24L01_TxPacket(uint8_t *txbuf)
{
  uint8_t state;   
  Clr_NRF24L01_CE;
  NRF24L01_Write_Buf(WR_TX_PLOAD,txbuf,TX_PLOAD_WIDTH);//写数据到TX BUF  32个字节
  Set_NRF24L01_CE;                                     //启动发送	   
  while(READ_NRF24L01_IRQ!=0);                              //等待发送完成
  state=NRF24L01_Read_Reg(STATUS);                     //读取状态寄存器的值	   
  NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state);      //清除TX_DS或MAX_RT中断标志
  if(state&MAX_TX)                                     //达到最大重发次数
  {
          NRF24L01_Write_Reg(FLUSH_TX,0xff);               //清除TX FIFO寄存器 
          return false; 
  }
  if(state&TX_OK)                                      //发送完成
  {
          return true;
  }
  return false;                                         //其他原因发送失败
}

/*******************************************************************************
* Function Name  : NRF24L01_RxPacket
* Description    : 启动NRF24L01接收一次数据
* Input          : rxbuf:接收数据首地址
* Output         : None
* Return         :  接收完成状况
*******************************************************************************/
static bool NRF24L01_RxPacket(uint8_t *rxbuf)
{
  uint8_t state;		    							      
  state=NRF24L01_Read_Reg(STATUS);                //读取状态寄存器的值    	 
  NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS,state); //清除TX_DS或MAX_RT中断标志
  if(state&RX_OK)                                 //接收到数据
  {
          NRF24L01_Read_Buf(RD_RX_PLOAD,rxbuf,RX_PLOAD_WIDTH);//读取数据
          NRF24L01_Write_Reg(FLUSH_RX,0xff);          //清除RX FIFO寄存器 
          return true; 
  }	   
  return false;                                      //没收到任何数据
}

/*******************************************************************************
* Function Name  : RX_Mode
* Description    : 该函数初始化NRF24L01到RX模式
*  设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
*  当CE变高后,即进入RX模式,并可以接收数据
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void RX_Mode(void)
{
	Clr_NRF24L01_CE;	  
  	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);//写RX节点地址
	  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);    //使能通道0的自动应答    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);//使能通道0的接收地址  	 
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,40);	     //设置RF通信频率		  
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);//选择通道0的有效数据宽度 	    
  	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  	NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG, 0x0f);  //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
  	Set_NRF24L01_CE;                                //CE为高,进入接收模式 
}						 

/*******************************************************************************
* Function Name  : TX_Mode
* Description    : 该函数初始化NRF24L01到TX模式
*  设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,
*  波特率和LNA HCURR PWR_UP,CRC使能,当CE变高后,即进入TX模式,发送数据
*  CE为高大于10us,则启动发送.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void TX_Mode(void)
{														 
  Clr_NRF24L01_CE;	    
  NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH);    //写TX节点地址 
  NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH); //设置TX节点地址,主要为了使能ACK	  

  NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01);     //使能通道0的自动应答    
  NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01); //使能通道0的接收地址  
  NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);//设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
  NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,40);       //设置RF通道为40
  NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f);  //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
  NRF24L01_Write_Reg(SPI_WRITE_REG+CONFIG,0x0e);    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
  Set_NRF24L01_CE;                                  //CE为高,10us后启动发送
}		  

/******************************以下 应用函数*****************************/
 void rf1start(void)
{
  static uint32_t times;
  uint8_t disstr[TX_ADR_WIDTH];
  uint8_t buf[TX_ADR_WIDTH] = {0x00};

  NRF24L01_Init();  
  
  if(NRF24L01_Check()==true)
  {
    PRINTF("RF 模块初始化成功！\r\n");
  }
  else  
  {
    PRINTF("RF 模块不存在！\r\n");
    return;
  }
  RX_Mode();
  while (1)
  {  
    if(NRF24L01_RxPacket(buf) == true)
    {
       PRINTF("\r\n RF 接收到数据：\r\n"); 
        {
          PRINTF((char const*)buf);
        }
    }
    times++;
    
    delay_ms(10);  
    
    if((times%100)==0)
    {
      sprintf(disstr,"time= %d 秒",times/100);
      rf1send(disstr);
    }
  }
}


void rf1send(uint8_t * str)
{ 
      PRINTF("\r\n \r\n"); 
      TX_Mode();
      PRINTF("\r\n RF1 发送数据：%s \r\n", str); 
      NRF24L01_TxPacket(str);
      RX_Mode();
}









