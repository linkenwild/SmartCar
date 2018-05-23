/******************************2017-2017, NJTECH, Edu.************************** 
FileName: i2c.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    软件i2c,从渡鸭开发板移植，主要实现SCCB协议 IIC模块的底层功能函数
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     文件创建   
  *          SmartCar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      i2c                    |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |        SCL                  |        B2                   |
  *          |        SDA                  |        B3                   |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/
#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"

#include "i2c.h"

#define I2C_RELEASE_SDA_PORT PORTB
#define I2C_RELEASE_SCL_PORT PORTB
#define I2C_RELEASE_SDA_GPIO GPIOB
#define I2C_RELEASE_SDA_PIN     1U
#define I2C_RELEASE_SCL_GPIO GPIOB
#define I2C_RELEASE_SCL_PIN     0U


static void SDA_DDR_OUT(void)
{
     gpio_pin_config_t config = 
    {
        kGPIO_DigitalOutput, 0,
    };
    
    /* Init output SDA GPIO. */
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &config);    
}

static void SDA_DDR_IN(void)
{
     gpio_pin_config_t config = {
        kGPIO_DigitalInput, 0,
    };
    /* Init input SDA GPIO. */
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &config);    
    
}

static void SDA_H(void)
{
  GPIO_SetPinsOutput(I2C_RELEASE_SDA_GPIO, 1u << I2C_RELEASE_SDA_PIN);
}

static void SDA_L(void)
{
  GPIO_ClearPinsOutput(I2C_RELEASE_SDA_GPIO, 1u << I2C_RELEASE_SDA_PIN);
}

static void SCL_H(void)
{
  GPIO_SetPinsOutput(I2C_RELEASE_SCL_GPIO, 1u << I2C_RELEASE_SCL_PIN);
}

static void SCL_L(void)
{
  GPIO_ClearPinsOutput(I2C_RELEASE_SCL_GPIO, 1u << I2C_RELEASE_SCL_PIN);
}

static void I2C_DELAY(void)
{
    uint32_t i = 0;
    for (i = 0; i < 100; i++)
    {
        __NOP();
    }
}
/**
 * @brief  I2C 初始化函数
 */
uint8_t I2C_QuickInit(void)
{
    gpio_pin_config_t config = 
    {
        kGPIO_DigitalOutput, 0,
    };
    /* Enable i2c port clock */
    CLOCK_EnableClock(kCLOCK_PortB);
    
    /* sda pin mux Configuration */  
    PORT_SetPinMux(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, kPORT_MuxAsGpio); 
    /* Init output SDA GPIO. */
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &config);
    
    /* sda pin mux Configuration */  
    PORT_SetPinMux(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, kPORT_MuxAsGpio); 
    /* Init output SDA GPIO. */
    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &config);
    
    return true;
}

/**
 * \brief I2C 初始化(待定义)
 * \param[in] I2C_InitStruct 指向I2C结构体的指针 
 */
void I2C_Init(I2C_InitTypeDef* I2C_InitStruct)
{
    
}

/**
 * \brief 读取I2C上SDA数据，Internal function
 * \return SDA上的数据(1 bit)
 */
static inline uint8_t SDA_IN(void)
{
    return GPIO_ReadPinInput(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN); ;
}

/**
 * \brief I2C Start，Internal function
 * \retval true
 */
static bool I2C_Start(void)
{
    SDA_DDR_OUT();
    SDA_H();
    SCL_H();
    I2C_DELAY();
    SDA_L();
    I2C_DELAY();
    SCL_L();
    return true;
}

/**
 * \brief I2C Stop，Internal function
 * \retval None
 */
static void I2C_Stop(void)
{
    SCL_L();
    SDA_L();
    I2C_DELAY();
    SCL_H();
    SDA_H();
    I2C_DELAY();
}

/**
 * \brief I2C Ack，Internal function
 * \retval None
 */
static void I2C_Ack(void)
{
    SCL_L();
    SDA_L();
    I2C_DELAY();
    SCL_H();
    I2C_DELAY();
    SCL_L();
    I2C_DELAY();
}

/**
 * \brief I2C Not Ack，Internal function
 * \retval None
 */
static void I2C_NAck(void)
{
    SCL_L();
    I2C_DELAY();
    SDA_H();
    I2C_DELAY();
    SCL_H();
    I2C_DELAY();
    SCL_L();
    I2C_DELAY();
}

/**
 * \brief I2C Wait Ack，Internal function
 * \return 应答信号
 */
static bool I2C_WaitAck(void)
{
    uint8_t ack;
    SDA_DDR_IN();
    SCL_L();
    
    I2C_DELAY();
    SCL_H();
    I2C_DELAY();
    ack = SDA_IN();
    SCL_L();
    SDA_DDR_OUT();
    
    return ack;
}

/**
 * \brief I2C 发送一个字节数据，Internal function
 * \param[in] data 待发送的数据(字节)
 * \retval None
 */
static void I2C_SendByte(uint8_t data)
{
    volatile uint8_t i;
    
    i = 8;
    while(i--)
    {
        if(data & 0x80) SDA_H();
        else SDA_L();
        data <<= 1;
        I2C_DELAY();
        SCL_H();
        I2C_DELAY();
        SCL_L();
    }

}

/**
 * \brief I2C 接收一个字节数据，Internal function
 * \return 待接收的数据(字节)
 */
static uint8_t I2C_GetByte(void)
{
    uint8_t i,byte;
    
    i = 8;
    byte = 0;

    SDA_DDR_IN();
    while(i--)
    {
        SCL_L();
        I2C_DELAY();
        SCL_H();
        I2C_DELAY();
        byte = (byte<<1)|(SDA_IN() & 1);
    }
    SCL_L();
    SDA_DDR_OUT();
    return byte;
}

/**
 * @brief  I2C write mutiple data
 * @param[in]  instance instance of i2c moudle
 * \param[in]  chipAddr    i2c slave addr
 * \param[in]  addr        i2c slave register offset
 * \param[in]  addrLen     len of slave register addr(in byte)
 * \param[in]  buf         data buf
 * \param[in]  len         data length
 * \retval 0 success
 * \retval 1 failure
 */
int I2C_BurstWrite(uint32_t instance ,uint8_t chipAddr, uint32_t addr, uint32_t addrLen, uint8_t *buf, uint32_t len)
{
    uint8_t *p;
    uint8_t err;
    
    p = (uint8_t*)&addr;
    err = 0;
    chipAddr <<= 1;
    
    I2C_Start();
    I2C_SendByte(chipAddr);
    err += I2C_WaitAck();

    while(addrLen--)
    {
        I2C_SendByte(*p++);
        err += I2C_WaitAck();
    }
    
    while(len--)
    {
        I2C_SendByte(*buf++);
        err += I2C_WaitAck();  
    }

    I2C_Stop();
    return err;
}

/**
 * @brief  write single register value
 * \param[in]  instance    instance of i2c module
 * \param[in]  chipAddr    i2c slave addr
 * \param[in]  addr        i2c slave register offset
 * \param[in]  data        data to write 
 * @note   usually used on i2c sensor devices
 * \retval 0 success
 * \retval 1 failure
 */
int I2C_WriteSingleRegister(uint32_t instance, uint8_t chipAddr, uint8_t addr, uint8_t data)
{
    return I2C_BurstWrite(instance, chipAddr, addr, 1, &data, 1);
}

/**
 * @brief  I2C read mutiple data
 * \param[in]  instance    instance of i2c moudle
 * \param[in]  chipAddr    i2c slave addr
 * \param[in]  addr        i2c slave register offset
 * \param[in]  addrLen     len of slave register addr(in byte)
 * \param[out] buf         data buf
 * \param[in]  len         data length
 * \retval 0 success
 * \retval 1 failure
 */
int I2C_BurstRead(uint32_t instance ,uint8_t chipAddr, uint32_t addr, uint32_t addrLen, uint8_t *buf, uint32_t len)
{
    uint8_t *p;
    uint8_t err;
    
    p = (uint8_t*)&addr;
    err = 0;
    chipAddr <<= 1;
    
    I2C_Start();
    I2C_SendByte(chipAddr);
    err += I2C_WaitAck();
    
    while(addrLen--)
    {
        I2C_SendByte(*p++);
        err += I2C_WaitAck();
    }
    
    I2C_Start();
    I2C_SendByte(chipAddr+1);
    err += I2C_WaitAck();
    
    while(len--)
    {
        *buf++ = I2C_GetByte();
        if(len)
        {
            I2C_Ack();
        }
    }
    
    I2C_NAck();
    I2C_Stop();
    
    return err;
}

/**
 * @brief  proble i2c bus
 * @param[in]  instance instance of i2c moudle
 * \param[in]  chipAddr i2c slave addr
 * @note   see if it's available i2c slave on the bus
 * \retval 0 success
 * \retval 1 failure
 */
int I2C_Probe(uint32_t instance, uint8_t chipAddr)
{
    uint8_t err;
    
    err = 0;
    chipAddr <<= 1;
    
    I2C_Start();
    I2C_SendByte(chipAddr);
    err = I2C_WaitAck();
    I2C_Stop();
    return err;
}

/**
 * @brief  read single register value
 * \param[in]  instance   instance of i2c moudle
 * \param[in]  chipAddr   i2c slave addr
 * \param[in]  addr       i2c slave register offset
 * \param[out] data       data pointer
 * @note   usually used on i2c sensor devices
 * \retval 0 success
 * \retval 1 failure
 */
int I2C_ReadSingleRegister(uint32_t instance, uint8_t chipAddr, uint8_t addr, uint8_t *data)
{
    return I2C_BurstRead(instance, chipAddr, addr, 1, data, 1);
}

/**
 * @brief  SCCB(protocol,the same as i2c) read single register value
 * \param[in]  instance   instance of i2c moudle
 * \param[in]  chipAddr   i2c slave addr
 * \param[in]  addr       i2c slave register offset
 * \param[out] data       data pointer
 * @note   usually used on i2c sensor devices
 * \retval 0 success
 * \retval 1 failure
 */
int SCCB_ReadSingleRegister(uint32_t instance, uint8_t chipAddr, uint8_t addr, uint8_t* data)
{
    uint8_t err;
    uint8_t retry;
    
    retry = 10;
    chipAddr <<= 1;
    
    while(retry--)
    {
        err = 0;
        I2C_Start();
        I2C_SendByte(chipAddr);
        err += I2C_WaitAck();
        
        I2C_SendByte(addr);
        err += I2C_WaitAck();
        
        I2C_Stop();
        I2C_Start();
        I2C_SendByte(chipAddr+1);
        err += I2C_WaitAck();
        
        *data = I2C_GetByte();
       // err += I2C_WaitAck();
        
        I2C_NAck();
        I2C_Stop();
        if(!err)
        {
            break;
        }
    }

    return err;
}

/**
 * @brief  SCCB(protocol,the same as i2c) write single register value
 * \param[in]  instance    instance of i2c module
 * \param[in]  chipAddr    i2c slave addr
 * \param[in]  addr        i2c slave register offset
 * \param[in]  data        data to write 
 * @note   usually used on i2c sensor devices
 * \retval 0 success
 * \retval 1 failure
 */
int SCCB_WriteSingleRegister(uint32_t instance, uint8_t chipAddr, uint8_t addr, uint8_t data)
{
    uint8_t err;
    uint8_t retry;
    
    retry = 10;
    
    while(retry--)
    {
        err = I2C_WriteSingleRegister(instance, chipAddr, addr, data);
        if(!err)
        {
            break;
        }
    }
    return err;
}


/**
 * \brief i2c bus scan test
 * @param[in] MAP  I2C引脚配置缩略图,详见i2c.h
 * \retval None
 */
void I2C_Scan(uint32_t MAP)
{
    uint8_t i;
    uint8_t ret;
    uint32_t instance;
    instance = I2C_QuickInit();
    for(i = 1; i < 127; i++)
    {
        ret = I2C_Probe(instance , i);
        if(!ret)
        {
            PRINTF("ADDR:0x%2X(7BIT) | 0x%2X(8BIT) found!\r\n", i, i<<1);
        }
    }
}
