/******************************2017-2017, NJTECH, Edu.************************** 
FileName: i2c_ee.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    i2c 24c02 使用 硬件i2c 
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     文件创建   
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

#include "i2c_ee.h"
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
#define I2C_BAUDRATE 50000U

#define I2C_EE_SLAVE_ADDR_7BIT 0x50U
#define I2C_EE_PAGESIZE 8

/*******************************************************************************s
 * Variables
 ******************************************************************************/
/*  i2c_ee */
static i2c_master_handle_t g_m_handle;

static volatile bool completionFlag = false;
static volatile bool nakFlag = false;



/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
* Function Name  : I2C_ReadWhoAmI 内部函数
* Description    : i2c总线初始化 查找自身地址
* Input          : None
* Output         : None
* Return         : 是否成功
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
* Function Name  : i2c_release_bus_delay 内部函数
* Description    : 短延时函数
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
* Function Name  : i2c_bus_delay 内部函数
* Description    : 长延时函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void i2c_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < 0x20000; i++)
    {
        __NOP();
    }
}

/*******************************************************************************
* Function Name  : BOARD_I2C_ReleaseBus 内部函数
* Description    : i2c 总线释放
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
* Function Name  : BOARD_I2C_ConfigurePins 内部函数
* Description    : i2c pin初始化函数 
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
* Function Name  : i2c_master_callback 内部函数
* Description    : i2c 数据传输中断回调函数
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
* Function Name  : i2c_ee_init
* Description    : e2prom　i2c 初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void i2c_ee_init(void)
{
    i2c_master_transfer_t masterXfer;
    
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    
    I2C_MasterTransferCreateHandle(BOARD_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
    I2C_ReadWhoAmI();
    
}

/*******************************************************************************
* Function Name  : i2c_ee_write_onepage 
* Description    : 向E2PROM写一页，不能跨页
* Input          : pBuffer-要写入的数据  WriteAddr-写入地址 
*                 NumByteToWrite-写入数据的长度
* Output         : None
* Return         : 是否成功
*******************************************************************************/
bool i2c_ee_write_onepage(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
  i2c_master_transfer_t masterXfer;
  memset(&masterXfer, 0, sizeof(masterXfer));

  masterXfer.slaveAddress = I2C_EE_SLAVE_ADDR_7BIT;
  masterXfer.direction = kI2C_Write;
  masterXfer.subaddress = WriteAddr%256;
  masterXfer.subaddressSize = 1;
  masterXfer.data = pBuffer;
  masterXfer.dataSize = NumByteToWrite;
  masterXfer.flags = kI2C_TransferDefaultFlag;

  /*  direction=write : start+device_write;cmdbuff;xBuff; */
  /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

  I2C_MasterTransferNonBlocking(BOARD_I2C_BASEADDR, &g_m_handle, &masterXfer);

  /*  wait for transfer completed. */
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
}


/*******************************************************************************
* Function Name  : i2c_ee_buffer_read 
* Description    : 从E2PROM读一页，不能跨页
* Input          :  ReadAddr-读出数据起始地址 
*                 NumByteToRead-读出数据的长度
* Output         : pBuffer-读出数据要存放的地址
* Return         : 是否成功
*******************************************************************************/
bool i2c_ee_buffer_read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)
{
  i2c_master_transfer_t masterXfer;
  memset(&masterXfer, 0, sizeof(masterXfer));
  
  masterXfer.slaveAddress = I2C_EE_SLAVE_ADDR_7BIT;
  masterXfer.direction = kI2C_Read;
  masterXfer.subaddress = ReadAddr%256;
  masterXfer.subaddressSize = 1;
  masterXfer.data = pBuffer;
  masterXfer.dataSize = NumByteToRead;
  masterXfer.flags = kI2C_TransferDefaultFlag;

  /*  direction=write : start+device_write;cmdbuff;xBuff; */
  /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

  I2C_MasterTransferNonBlocking(BOARD_I2C_BASEADDR, &g_m_handle, &masterXfer);

  /*  wait for transfer completed. */
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
}

/*******************************************************************************
* Function Name  : eep_self_test e2prom测试函数
* Description    : 向eeprom写入一段数据，然后读出，最后比较。
* Input          :  begin-测试数据起始地址 
*                   end-测试数据的结束地址
* Output         : None
* Return         : 是否成功
*******************************************************************************/
bool eep_self_test(uint32_t begin, uint32_t end)
{
    int i,j;
    uint8_t buf[I2C_EE_PAGESIZE];

    for(i=0; i<(end - begin)/I2C_EE_PAGESIZE; i++)
    {
        for(j=0; j<sizeof(buf);j++)
        {
            buf[j] = j&0xFF;
        }
        PRINTF("eep write 0x%X...", i*I2C_EE_PAGESIZE);
        i2c_ee_write_onepage(buf, i*I2C_EE_PAGESIZE,  sizeof(buf));
        i2c_bus_delay();
        memset(buf, 0, sizeof(buf));
        i2c_ee_buffer_read(buf, i*I2C_EE_PAGESIZE,  sizeof(buf));
        i2c_bus_delay();
        PRINTF("varify...");
        for(j=0; j<sizeof(buf); j++)
        {
            if(buf[j] != j%0xFF)
            {
                return false;
            }
        }
        PRINTF("ok!\r\n");
    }
    return true;
}


