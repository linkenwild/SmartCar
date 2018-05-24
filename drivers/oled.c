/******************************2017-2017, NJTECH, Edu.************************** 
FileName: oled.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.30
Description:    i2c oled ����ʹ��Ӳ��i2c, oled ��ַ��0x3C
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     �ļ�����   
  *          SmartCar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      I2C Pin                |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |        SCL                  |        B2                   |
  *          |        SDA                  |        B3                   |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 

#include "fsl_i2c.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"
#include "fsl_debug_console.h"

#include "oled_font.h"
#include "oled.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* I2C source clock */
#define I2C_CLK_SRC I2C0_CLK_SRC

#define I2C_RELEASE_SDA_PORT PORTB
#define I2C_RELEASE_SCL_PORT PORTB
#define I2C_RELEASE_SDA_GPIO GPIOB
#define I2C_RELEASE_SDA_PIN 3U
#define I2C_RELEASE_SCL_GPIO GPIOB
#define I2C_RELEASE_SCL_PIN 2U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 100000U

#define I2C_OLED_SLAVE_ADDR_7BIT 0x3CU

/*******************************************************************************s
 * Variables
 ******************************************************************************/
/*  i2c_oled */
static i2c_master_handle_t g_m_handle;

static volatile bool completionFlag = false;
static volatile bool nakFlag = false;


const unsigned char  show[]=
{
/*------------------------------------------------------------------------------
;  ���������룬������ģ��ʽ���ã�ע��ѡ����ȷ��ȡģ������ֽ�λ˳��
;  Դ�ļ� / ���� : �й�����
;  ����ߣ����أ�: 128��64
;  ��ģ��ʽ/��С : ��ɫ����Һ����ģ������ȡģ���ֽڵ���/1024�ֽ�
;  ����ת������  : 2016/8/21 10:13:25
------------------------------------------------------------------------------*/
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0xE0,0x30,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
0x18,0xFF,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0xF0,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0x01,0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0xF8,0xF8,
0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x00,0x01,0xFF,0x00,0x00,0x00,0x00,0x00,
0x00,0xC0,0x80,0x01,0x03,0x06,0x04,0x08,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0xFF,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x02,
0x02,0x82,0x82,0x82,0x82,0x9F,0x82,0xE2,0x82,0x82,0x82,0x82,0x82,0x82,0x82,0x82,
0x9F,0x82,0x82,0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0xFF,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x08,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0xFF,0xFF,
0x0C,0x0C,0x2C,0x6C,0x8C,0x0C,0x0C,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x01,0x03,0x02,0x04,0x0C,0x80,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x80,0xE1,0x19,0x01,0x01,0x01,0xC1,0x7F,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0x01,0xFF,0xFF,0x00,0x00,0x78,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
0x01,0xFF,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0xFF,0x80,0x90,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x9F,0x9F,
0x98,0x98,0x98,0x98,0x99,0x9B,0x98,0x98,0x80,0x80,0xFF,0x00,0x00,0x00,0x00,0x00,
0x80,0xC0,0x60,0x30,0x18,0x0C,0x03,0x81,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
0xFF,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x00,0x00,0x00,0x04,0x87,
0x81,0x40,0x60,0x30,0x18,0x06,0x03,0x00,0x00,0x00,0x00,0xC0,0x80,0x80,0x80,0x80,
0x80,0x80,0x7F,0x03,0x00,0x00,0x00,0x03,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x01,
0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

};

/*��ʼ��oled ������*/
static uint8_t initial_oled_data[28]={0xAE,0x20,0x10,0xb0,0xc8,0x00,0x10,0x40,0x81,0xdf,0xa1,0xa6,
0xa8,0x3F,0xa4,0xd3,0x00,0xd5,0xf0,0xd9,0x22,0xda,0x12,0xdb,0x20,0x8d,0x14,0xaf};



/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
* Function Name  : I2C_ReadWhoAmI �ڲ�����
* Description    : i2c���߳�ʼ�� ���������ַ
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static bool I2C_ReadWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint32_t sourceClock = 0;

    i2c_master_config_t masterConfig;

    /*
     * masterConfig.baudRate_Bps = 100000U;
     * masterConfig.enableHighDrive = false;
     * masterConfig.enableStopHold = false;
     * masterConfig.glitchFilterWidth = 0U;
     * masterConfig.enableMaster = true;
     */
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    sourceClock = CLOCK_GetFreq(I2C_CLK_SRC);
    I2C_MasterInit(BOARD_I2C_BASEADDR, &masterConfig, sourceClock);
    
    return true;    
}

/*******************************************************************************
* Function Name  : i2c_release_bus_delay �ڲ�����
* Description    : ����ʱ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

/*******************************************************************************
* Function Name  : i2c_bus_delay �ڲ�����
* Description    : ����ʱ����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void i2c_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < 0x200000; i++)
    {
        __NOP();
    }
}


/*******************************************************************************
* Function Name  : BOARD_I2C_ReleaseBus �ڲ�����
* Description    : i2c �����ͷ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortB);

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA low */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

/*******************************************************************************
* Function Name  : BOARD_I2C_ConfigurePins �ڲ�����
* Description    : i2c pin��ʼ������ 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void BOARD_I2C_ConfigurePins(void)
{
    port_pin_config_t pinConfig = {0};
    pinConfig.pullSelect = kPORT_PullUp;
    pinConfig.slewRate = kPORT_FastSlewRate;
    pinConfig.passiveFilterEnable = kPORT_PassiveFilterDisable;
    pinConfig.openDrainEnable = kPORT_OpenDrainEnable;
    pinConfig.driveStrength = kPORT_LowDriveStrength;
    pinConfig.mux = kPORT_MuxAlt2;
    CLOCK_EnableClock(kCLOCK_PortB);

    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &pinConfig);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &pinConfig);
}

/*******************************************************************************
* Function Name  : i2c_master_callback �ڲ�����
* Description    : i2c ���ݴ����жϻص�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if (status == kStatus_I2C_Nak)
    {
        nakFlag = true;
    }
}

/*******************************************************************************
* Function Name  : IIC_oled_Buffer_Write
* Description    : ��oled��дbuffer��������һ�����ݵ�д������
* Input          : data_addr Ҫд�����ݵĵ�ַ lenҪд�����ݵĳ��� mode Ϊ0ʱ CMD Ϊ1ʱ DATA
* Output         : None
* Return         : None 
*******************************************************************************/
static bool IIC_oled_Buffer_Write( uint8_t* data_addr, uint16_t len, uint8_t mode)
{
  i2c_master_transfer_t masterXfer;
  memset(&masterXfer, 0, sizeof(masterXfer));

  uint8_t WriteAddr = 0;
  if(mode==0)
  {
    WriteAddr = 0x00;  
  }
  else if(mode ==1)
  {
    WriteAddr = 0x40;  
  }   
  
  masterXfer.slaveAddress = I2C_OLED_SLAVE_ADDR_7BIT;
  masterXfer.direction = kI2C_Write;
  masterXfer.subaddress = WriteAddr;
  masterXfer.subaddressSize = 1;
  masterXfer.data = data_addr;
  masterXfer.dataSize = len;
  masterXfer.flags = kI2C_TransferDefaultFlag;

  /*  direction=write : start+device_write;cmdbuff;xBuff; */
  /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

  //I2C_MasterTransferNonBlocking(BOARD_I2C_BASEADDR, &g_m_handle, &masterXfer);

  /*  wait for transfer completed. 
  while ((!nakFlag) && (!completionFlag))
  {
  }
  nakFlag = false; 

  if (completionFlag == true)
  {
      completionFlag = false;
      return true;
  }
  else
  {
      return false;
  }
  */
  I2C_MasterTransferBlocking(BOARD_I2C_BASEADDR, &masterXfer);
}

/*******************************************************************************
* Function Name  : i2c_oled_init
* Description    : oled��i2c ��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void i2c_oled_init(void)
{
    
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    
    I2C_MasterTransferCreateHandle(BOARD_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
    I2C_ReadWhoAmI();   
    
  /*�ó�ʼ������ ��ʼ�� oled*/
    IIC_oled_Buffer_Write( initial_oled_data,28,0);     
}

/*******************************************************************************
* Function Name  : fill_picture
* Description    : ��oled�Ļ������
* Input          : fill_Data Ҫ�������� 
* Output         : None
* Return         : None 
*******************************************************************************/
void fill_picture(uint8_t fill_Data)
{
  uint8_t cmd_buf[3] = {0xb0,0x00,0x10};
  uint8_t data_buf[128*8] = {0x00};
  
  /*��Ҫ�������� ��������*/
  for(int i=0;i<128*8;i++)
  {
    data_buf[i] = fill_Data;
  }
  
  IIC_oled_Buffer_Write(cmd_buf, 3, 0);
  IIC_oled_Buffer_Write(data_buf, 128*8, 1);   
}

/*******************************************************************************
* Function Name  : show_picture
* Description    : ��ͼ������ʾ ������show�е�ͼ��
* Input          : None
* Output         : None
* Return         : None 
*******************************************************************************/
void show_picture(void)
{
  uint8_t cmd_buf[3] = {0xb0,0x00,0x10};

  IIC_oled_Buffer_Write(cmd_buf, 3, 0);
  IIC_oled_Buffer_Write((uint8_t *)&show, 128*8, 1);	
}

/*******************************************************************************
* Function Name  : OLED_Set_Pos
* Description    : ���õ�ǰ��ʾλ��
* Input          : x-��λ��  y-ҳλ��
* Output         : None
* Return         : None
*******************************************************************************/
static void OLED_Set_Pos(uint8_t x, uint8_t y)
{
 
  uint8_t cmd_buf[3] = {0xb0,0x00,0x10};

  cmd_buf[0] = 0xb0+y;
  cmd_buf[1] = (x&0x0f)|0x01;
  cmd_buf[2] = ((x&0xf0)>>4)|0x10;
  IIC_oled_Buffer_Write(cmd_buf, 3, 0);
}

/*******************************************************************************
* Function Name  : test_i2c_oled
* Description    : ��ʾ�ڰ���䣨4�У����ƣ�����ʾ���֡��й����ա�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void test_i2c_oled (void)
{
  /*��ʾ���� 1Ϊ��ɫ 0 Ϊ��ɫ ��λ����*/
  fill_picture(0xF1);//��һ�а���
  i2c_bus_delay();
  /*��ʾһ��ͼ�� ���й����ա�*/
  show_picture(); 

}
/*******************************************************************************
* Function Name  : oled_display_ccd_image
* Description    : ��ʾCCDͼ�� һ��  
* Input          : data-���ݵ�ַ threshold-��ֵ
                    datalenght-���ݵ���  128
                    startline-��ʼ�� 8
                    height-�������� 48
* Output         : None
* Return         : None 
*******************************************************************************/
void oled_display_ccd_image (uint8_t* data, float threshold, uint16_t datalenght,  uint16_t startline,  uint16_t heigth)
{
  uint8_t data_buf[128*64] = {0xFF};//�׵�
  uint16_t page, seg, datalen, startpage, pagelen ;
  datalen = datalenght;         if(datalen>128) datalen = 128; 
  startpage = startline/8;          if(startpage>=8)startpage = 7; //ȱʡ��1ҳ
  pagelen = heigth/8;           if(pagelen>8)   pagelen = 8;   //ȱʡ2ҳ
  
  for(int i=0;i<128*64;i++)
  {
    data_buf[i] = 0xff;
  }
  
  for(seg = 0; seg<128; seg++)
  {
     if(*data > threshold)
     {
      for(page = startpage; page<(startpage+pagelen); page++)
      {
        data_buf[128*page+seg] = 0xFF;  //��ɫ
      }
     }
     else
     {
      for(page = startpage; page<(startpage+pagelen); page++)
      {
       data_buf[128*page+seg] = 0x00; //��ɫ
      }
     }
     data++ ;
  }
   IIC_oled_Buffer_Write(data_buf, 1024, 1);
}

/*******************************************************************************
* Function Name  : oled_display_ccd_line
* Description    : ��ʾCCDͼ�� ��������  
* Input          : ��һ· data0-���ݵ�ַ threshold0-��ֵ
                   �ڶ�· data1-���ݵ�ַ threshold1-��ֵ
* Output         : None
* Return         : None 
*******************************************************************************/
void oled_display_ccd_line (uint8_t* data0, float threshold0, uint8_t* data1, float threshold1)
{
  uint8_t data_buf[128*64] = {0xFF};//�׵�
  uint16_t page, seg, datalen, startpage ;
  datalen = 128;         
  startpage = 0;         
  
  for(int i=0;i<128*64;i++)
  {
    data_buf[i] = 0xff;
  }
  
  for(seg = 0; seg<128; seg++)
  {
     if(*data0 > threshold0)
     {
      for(page = 0; page < 4; page++)
      {
        data_buf[128*page+seg] = 0xFF;  //��ɫ
        if(page == 0) data_buf[128*page+seg] = data_buf[128*page+seg] | 0x0F;
        if(page == 3) data_buf[128*page+seg] = data_buf[128*page+seg] | 0xF0;
      }
     }
     else
     {
      for(page = 0; page < 4; page++)
      {
        data_buf[128*page+seg] = 0x00; //��ɫ
        if(page == 0) data_buf[128*page+seg] = data_buf[128*page+seg] | 0x0F;
        if(page == 3) data_buf[128*page+seg] = data_buf[128*page+seg] | 0xF0;
      }
     }
     data0++ ;
  }
  
  for(seg = 0; seg<128; seg++)
  {
     if(*data1 > threshold1)
     {
      for(page = 4; page < 8; page++)
      {
        data_buf[128*page+seg] = 0xFF;  //��ɫ
        if(page == 4) data_buf[128*page+seg] = data_buf[128*page+seg] | 0x0F;
        if(page == 7) data_buf[128*page+seg] = data_buf[128*page+seg] | 0xF0;
      }
     }
     else
     {
      for(page = 4; page < 8; page++)
      {
        data_buf[128*page+seg] = 0x00; //��ɫ
        if(page == 4) data_buf[128*page+seg] = data_buf[128*page+seg] | 0x0F;
        if(page == 7) data_buf[128*page+seg] = data_buf[128*page+seg] | 0xF0;
      }
     }
     data1++ ;
  }
  
  IIC_oled_Buffer_Write(data_buf, 1024, 1);
}

/*----------------------------------------��ʾ�ַ�----------------------------*/
//������ʾλ��
static uint8_t OLED_SetXY(uint8_t x,uint8_t y)
{
  uint8_t cmd_buf[3] = {0x00};  
  cmd_buf[0] = 0xb0|y;  //y
  cmd_buf[1] = ((0xf0&x)>>4)|0x10; //����x��4λ
  cmd_buf[2] = ((0x0f&x)>>4)|0x01;//����x��4λ

  IIC_oled_Buffer_Write(cmd_buf, 3, 0);
  return 0;
}

//�������ƣ�void OLED_Clear(uint8_t Fill)
//�������ܣ�OLED���/����
//����������Fill,1-��� ���� 0-���
void OLED_Clear(uint8_t Fill)
{
  uint8_t cmd_buf[6] = {0x00}; 
  cmd_buf[0] = 0x22 ; //Page Address
  cmd_buf[1] = 0x00; //Page Start Address
  cmd_buf[2] = 0x07 ; //Page End Address
  cmd_buf[3] = 0x21 ; //Column Address
  cmd_buf[4] = 0x00 ; //Column Start Address
  cmd_buf[5] = 0x7F ; //Column Start Address
  IIC_oled_Buffer_Write(cmd_buf, 6, 0);  
	
  if(Fill)
  {	fill_picture(0xFF);
  }
  else
  {	fill_picture(0x00);
  }
                  
}
//�������ƣ�void OLED_ShowChar(u8 uPage_sta,u8 uColumn_sta,u8 uSize,u8 uChar)
//�������ܣ���ָ��Page_sta,Column_sta�������СΪSize���ַ�Char
//����������uPage_sta:ҳ����uColumn_sta:������uSize:�����С��uChar��������ַ�
//ע�⣺�������������������ַ����뺯���������Զ����л�ҳ���ܣ�������������������������
void OLED_ShowChar(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uSize,uint8_t uChar)
{										  											
#define MAX_CHAR_X	127						//Column����
#define MAX_CHAR_Y	7						//Page����

 
  uint8_t data_buf[128*8] = {0x00};
  uint8_t uChar_Buf[16];
	uint8_t uCount;
	uint8_t uPage_end,uColumn_end;				 
	uint8_t i;
  uint8_t j = 0;
  {
	if(uSize==6)							//�����8x6����
	{	uPage_end=uPage_sta;				//��ֵPage_end
		uColumn_end=uColumn_sta+uSize;  	//��ֵCloumn_end
		uCount=6;							//��ֵuCount
	}
	else if(uSize==8)						//�����16x8����
	{	uPage_end=uPage_sta+1;				//��ֵPage_end
		uColumn_end=uColumn_sta+uSize;  	//��ֵCloumn_end
		uCount=16;							//��ֵuCount
	}
	else
	{	return;								//uSize����
	}
/////////////////////////////////�Զ����л�ҳ//////////////////////////////////////////////////////////////////////////////////////////////////////
	if(uColumn_end>MAX_CHAR_X)				//���Cloumn>127
	{
		uColumn_sta=0;						//Column_sta=0
		uColumn_end=uColumn_sta+uSize;		
		if(uSize==6)
		{	uPage_sta=uPage_sta+1;			//Page_sta+1
		 	uPage_end=uPage_sta;
		}
		else
		{	uPage_sta=uPage_sta+2;			//Page_sta+2
		 	uPage_end=uPage_sta+1;	
		}	
	}
	if(uPage_end>MAX_CHAR_Y)				//���Page>7
	{
		uPage_sta=0;						//Page_sta=0
		if(uSize==6)
		{	uPage_end=uPage_sta;	
		}
		else
		{	uPage_end=uPage_sta+1;
		}
	}
/////////////////////////////////�Զ����л�ҳ//////////////////////////////////////////////////////////////////////////////////////////////////////
  uint8_t cmd_buf[6] = {0x00}; 
  cmd_buf[0] = 0x22 ;                           //Page Address
  cmd_buf[1] = 0x00+uPage_sta;    		//Start Page Address
  cmd_buf[2] = 0x00+uPage_end;			//End Page Address
  cmd_buf[3] = 0x21 ;                           //Column Address
  cmd_buf[4] = 0x00+uColumn_sta;	 	//Start Column Address
  cmd_buf[5] = 0x00+uColumn_end;		//End Column Address
  IIC_oled_Buffer_Write(cmd_buf, 6, 0);  
	
  
	uChar=uChar-' ';				 		//���ƫ����
	for(i=0;i<uCount;i++)			 		//�����ַ�
	{
		if(uSize==8)
		{	uChar_Buf[j] = ASCII_1608[uChar][i];j++;
		}
		else 
		{	uChar_Buf[j] = ASCII_0806[uChar][i];j++;
		}
	}
  if(uSize==8)
  {	
    for(i=0;i<uCount/2;i++)	
    {
      data_buf[i+8] = uChar_Buf[i*2+1];
      data_buf[i] = uChar_Buf[i*2];
    }
  }	
  IIC_oled_Buffer_Write(data_buf, j+1, 1);   
  }
}


//�������ƣ�void OLED_ShowString
//�������ܣ���ָ��Page_sta,Column_sta�������СΪSize���ַ���*p
//����������uPage_sta:ҳ����uColumn_sta:������uSize:�����С��*p��������ַ���ָ��
//ע�⣺���������ַ������뺯���������Զ����л�ҳ���ܣ�������������������������
void OLED_ShowString(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uSize,uint8_t *p)
{
  {
  while(*p!='\0')
    {  
/////////////////////////////////�Զ����л�ҳ////////////////////////////////////////////////////////////////////////////////////////////////////// 
		if((uColumn_sta+uSize)>MAX_CHAR_X)
		{	uColumn_sta=0;
			if(uSize==6)
			{	uPage_sta=uPage_sta+1;			//Page_sta+1
			}
			else
			{	uPage_sta=uPage_sta+2;			//Page_sta+2	
			}	
		}
		if(uSize==6)
 	    {	if(uPage_sta>MAX_CHAR_Y)
			{	uPage_sta=0;
			}
		}
		else
		{	if((uPage_sta+1)>MAX_CHAR_Y)
			{	uPage_sta=0;
			}
		}
/////////////////////////////////�Զ����л�ҳ//////////////////////////////////////////////////////////////////////////////////////////////////////		  
        OLED_ShowChar(uPage_sta,uColumn_sta,uSize,*p);
        uColumn_sta+=uSize;
        p++;
    }
  }
}


//m^n����
uint32_t mypow(uint8_t m,uint8_t n)
{
	uint32_t result=1;	 
	while(n--)result*=m;    
	return result;
}
//���ܣ�������������!!!����0����Ϊ1!!!
uint8_t LongCheck(uint32_t num)
{
	uint8_t len=0;
	if(num==0)	   //����0����ҲΪ1
	{	return 1;
	}
	while(num)
	{	num/=10;
		len++;
	}
	return len;
}
//�������ƣ�void OLED_ShowNum(u8 uPage_sta,u8 uColumn_sta,u8 uSize,u32 uNum)
//�������ܣ���ָ��Page_sta,Column_sta�������СΪSize������uNum,��Χ:(0~4294967295)
//����������uPage_sta:ҳ����uColumn_sta:������uSize:�����С��uNum�����������	 
void OLED_ShowNum(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uSize,uint32_t uNum)
{         	
	uint8_t i,temp;
	uint8_t len;
        {
          len=LongCheck(uNum);			//�������			   
          for(i=0;i<len;i++)
          {
                  temp=(uNum/mypow(10,len-i-1))%10;
                  OLED_ShowChar(uPage_sta,uColumn_sta,uSize,temp+'0');
                  uColumn_sta+=uSize; 
          }
        }
}


//�������ƣ�void OLED_Fill(u8 uPage_sta,u8 uColumn_sta,u8 uColumn_end,u8 Fill)
//�������ܣ���Page_sta,��uColumn_sta��uColumn_end��������
//����������uPage_sta��ҳ����uColumn_sta���п�ʼ��uColumn_end���н�����Fill:���/���
void OLED_Fill(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uColumn_end,uint8_t Fill)
{
	uint8_t i,j=0;
	uint8_t ulen;        
  uint8_t cmd_buf[6] = {0x00}; 
  uint8_t data_buf[] = {0x00};
  cmd_buf[0] = 0x22 ;                           //Page Address
  cmd_buf[1] = 0x00+uPage_sta;    		//Start Page Address
  cmd_buf[2] = 0x00+uPage_sta;			//End Page Address
  cmd_buf[3] = 0x21 ;                           //Column Address
  cmd_buf[4] = 0x00+uColumn_sta;	 	//Start Column Address
  cmd_buf[5] = 0x00+uColumn_end;		//End Column Address
  IIC_oled_Buffer_Write(cmd_buf, 6, 0);          
        
	
	ulen=uColumn_end-uColumn_sta+1;
	for(i=0;i<=ulen;i++)
	{	
		if(Fill)
		{	data_buf[j] = 0xff;		    //���
		}
		else
		{	data_buf[j] = 0x00;		    //���
		}
	}	
        IIC_oled_Buffer_Write(data_buf, j+1, 1);   
        
        
}



/*******************************************************************************
* Function Name  : OledImge_Extend
* Description    : �ֽ����ţ�1�ֽ�8λ����λ��ʾ 
* Input          : data-ת��ǰ������  xSize-һ�ж��ٸ����� ySize-��������8�ı���
* Output         : buf-ת���������
* Return         : None
*******************************************************************************/
void OledImge_Extend(uint8_t *buf,uint8_t *data, int xSize, int ySize)  
{
  uint8_t colour[2]={0xff,0x00};
  uint8_t t;
  int x,y;
  data++;
    for(y = 0; y < ySize; y++)
    {
      for(x = 1; x < (xSize/8)+1; x++)  
      {
        t=*data++;
        *buf++=colour[(t>>7)&0x01];
        *buf++=colour[(t>>6)&0x01];
        *buf++=colour[(t>>5)&0x01]; 
        *buf++=colour[(t>>4)&0x01]; 
        *buf++=colour[(t>>3)&0x01];
        *buf++=colour[(t>>2)&0x01];
        *buf++=colour[(t>>1)&0x01];
        *buf++=colour[(t>>0)&0x01];
      }
    }
}

/*******************************************************************************
* Function Name  : oled_data120160_convert
* Description    : ͼ����ת���� 120*160 
* sizd = 0:   60*80    -->  160*120  -->  120*60 ����ÿ��3��ȥ��1�㣬 ������һ��
* sizd = 1:   120*160  -->  120*60 ����ÿ��3��ȥ��1�㣬 ������һ��
* sizd = 2:   180*240  -->  120*60 ����ÿ��2��ȥ��1�㣬������Ϊ1/3
* Input          : dataorigin-ԭ������ 
* Output         : buf-ת��������
* Return         : None
*******************************************************************************/
bool oled_data120160_convert (uint8_t* buf, uint8_t* dataorigin, uint8_t size)
{
  uint32_t i,j;
  bool result = false;
  uint8_t data120160[120][160];
  uint8_t data8060[60][80];

          if(size == 0)    //ͼ��60*80����Ϊ120*160
          {       
              for(i=0; i<120; i++)
              {
                    for(j=0; j<160; j++) {  data120160[i][j] = *(dataorigin+(i/2)*80+(j/2));}
              }
              result = true;
          }  
          else if(size == 1)    //ͼ��120*160
          {                    
              for(i=0; i<120; i++)
              {
                    for(j=0; j<160; j++){   data120160[i][j] = *(dataorigin+i*160+j);   }
              }  
              result = true;
          } 
          else if(size == 2)    //ͼ��180*240��ѹ��Ϊ60*80������Ϊ120*160
          {
              for(i=0; i<60; i++)
              {
                    for(j=0; j<80; j++)   {   data8060[i][j] = *(dataorigin+(i*3)*80+(j*3)) ; }
              }             
                    
              for(i=0; i<120; i++)
              {
                    for(j=0; j<160; j++)    {   data120160[i][j] = data8060[i/2][j/2] ; }
              }  
              result = true;
          }  
  
  for(i=0; i<120; i++)
  {
        for(j=0; j<160; j++) {  *buf = data120160[i][j]; buf++; }
  }  
  
  return result;
}



/*******************************************************************************
* Function Name  : oled_display_image
* Description    : ͼ�� 120*160 ���� 128*64 ���4����0
* Input          : dataorigin-ԭ������ 
* Output         : buf-ת��������
* Return         : None
*******************************************************************************/
bool oled_data120160_data12864_convert (uint8_t* buf, uint8_t* dataorigin)
{
  uint32_t i,j,m,n;
  uint8_t data[64][128] = {0U}; //���ͼ��

      m = 0;
      n = 0;
      
      for(i=0; i<120; i++)
      {
        if(0 == i%2) //ֻ��ʾż����
        {
          for(j=0; j<160; j++) //ÿ���� ÿ5����ȥ��1���� 160-->128
          {  
            if(4 != j%5)
            {
              data[m][n] = *( dataorigin + (i*160+ j)); 
              n++;
            }
          }
          m++; n = 0;
        }
      } 
      
      for(i=0; i<60; i++)
      {
            for(j=0; j<128; j++) {  *buf = data[i][j] ; buf++;};
      } 
      for(i=60; i<64; i++) //���4�� Ϊ ��
      {
            for(j=0; j<128; j++) {  *buf = 0xFF ; buf++;};
      }
      
    return true;
}

/*******************************************************************************
* Function Name  : oled_display_image
* Description    : ��ʾ·��һ֡ͼ��  datalenght-���ݵ��� width-һ�ж��ٵ� height-��������
*   ����: 80*64 ͼ��   datalenght-80*64 width-80 height-64        
*         128*64 ͼ��   datalenght-128*64  width-128 height-64  
          height-64   һ��Ϊ8�ı��������ܳ���64
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void oled_display_image (uint8_t* data, uint16_t datalenght, uint16_t width, uint16_t heigth)
{
  uint16_t page, seg;
  uint16_t col = 0;
  uint8_t data_buf[128*8] = {0x00};
  
  uint16_t i,bit_pos;

  for (i = 0; i<datalenght; i++)
  {
    seg = i % width;                      //��ƫ��
    page = i/(width*8);                 //ҳƫ��
    bit_pos = 8- (i / width) %(8*page);        //λƫ��
       
    if(0x00 == *data)
      data_buf[col+seg] &= ~(1<<(8-bit_pos));//��0
    else if(0xff == *data) //��ɫ
      data_buf[col+seg] |= 1<<(8-bit_pos);//��1
    
    if((heigth*(width/8)-1) == (i%(heigth*(width/8))))//���� heigth*(width/8) ����
    {
      col = col+128;
    }
    data++;
  }
   IIC_oled_Buffer_Write(data_buf, 1024, 1);
}



