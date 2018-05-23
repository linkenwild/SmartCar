/******************************2017-2017, NJTECH, Edu.************************** 
FileName: dmp_i2c.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    i2c mpu6050 dmp库 驱动使用硬件i2c, mpu6050 地址：0x68 
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

//#include "i2c.h"
#include "dmp_i2c.h"
#include "stdio.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* I2C source clock */
#define ACCEL_I2C_CLK_SRC I2C0_CLK_SRC
#define BOARD_ACCEL_I2C_BASEADDR I2C0

#define I2C_RELEASE_SDA_PORT PORTB
#define I2C_RELEASE_SCL_PORT PORTB
#define I2C_RELEASE_SDA_GPIO GPIOB
#define I2C_RELEASE_SDA_PIN 3U
#define I2C_RELEASE_SCL_GPIO GPIOB
#define I2C_RELEASE_SCL_PIN 2U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 500000U

#define MPU6050_WHOAMI 0x68U
#define MPU6050_WHOAMI_REG 0x75U
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*  */

/*******************************************************************************
 * Code
 ******************************************************************************/
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
* Function Name  : MPU_Write_Len
* Description    : 向MPU写入一定长度的数据 
* Input          : addr-器件地址 reg-寄存器地址 len-数据长度 buf-数据区
* Output         : None
* Return         : 0-正常 
*******************************************************************************/
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
//return (I2C_BurstWrite(1, addr, reg, 1, buf , len));
  status_t state;
  i2c_master_transfer_t masterXfer;
  memset(&masterXfer, 0, sizeof(masterXfer));

  masterXfer.slaveAddress = addr;
  masterXfer.direction = kI2C_Write;
  masterXfer.subaddress = reg;
  masterXfer.subaddressSize = 1;
  masterXfer.data = buf;
  masterXfer.dataSize = len;
  masterXfer.flags = kI2C_TransferNoStopFlag;

  /*  direction=write : start+device_write;cmdbuff;txBuff; */
  /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;rxBuff; */
  state = I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer);
  return state;
}

/*******************************************************************************
* Function Name  : MPU_Read_Len
* Description    : 从MPU读取一定长度的数据
* Input          : addr-器件地址 reg-寄存器地址 len-数据长度
* Output         : buf-读出的数据存储区
* Return         : 0-正常 
*******************************************************************************/
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
  //return (I2C_BurstRead(1 , addr, reg, 1, buf, len));
  status_t state;
  i2c_master_transfer_t masterXfer;
  memset(&masterXfer, 0, sizeof(masterXfer));

  masterXfer.slaveAddress = addr;
  masterXfer.direction = kI2C_Read;
  masterXfer.subaddress = reg;
  masterXfer.subaddressSize = 1;
  masterXfer.data = buf;
  masterXfer.dataSize = len;
  masterXfer.flags = kI2C_TransferRepeatedStartFlag;

  state = (I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer));
  return state;
}

/*******************************************************************************
* Function Name  : I2C_ReadWhoAmI 内部函数
* Description    : i2c总线初始化 查找自身地址
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static bool I2C_ReadAccelWhoAmI(void)
{
    /*
    How to read the device who_am_I value ?
    Start + Device_address_Write , who_am_I_register;
    Repeart_Start + Device_address_Read , who_am_I_value.
    */
    uint8_t who_am_i_reg = MPU6050_WHOAMI_REG;
    uint8_t who_am_i_value = 0x00;
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

    sourceClock = CLOCK_GetFreq(ACCEL_I2C_CLK_SRC);

    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, sourceClock);

    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = MPU6050_WHOAMI;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &who_am_i_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferNoStopFlag;

    I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer);

    masterXfer.slaveAddress = MPU6050_WHOAMI;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &who_am_i_value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferRepeatedStartFlag;

    I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR,  &masterXfer);



    if (who_am_i_value == MPU6050_WHOAMI)
    {
        PRINTF("Found a MPU6050 on board , the device address is 0x%x . \r\n", who_am_i_value);
        return true;
    }
    else
    {
        PRINTF("Do not found a device.\r\n ");
        return false;
    }

}

/*******************************************************************************
* Function Name  : dmp_i2c_init
* Description    : mpu6050 dmp　i2c 初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void dmp_i2c_init()
{
   //I2C_QuickInit();
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    I2C_ReadAccelWhoAmI();
    uint8_t data = 0x80;
    MPU_Write_Len(0x68, 0x6B, 1, &data);

}

