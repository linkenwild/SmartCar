/******************************2017-2017, NJTECH, Edu.************************** 
FileName: mpu6050.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.30
Description:    i2c mpu6050 ����ʹ��Ӳ��i2c, mpu6050 ��ַ��0x68 
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

#include "mpu6050.h"
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
#define I2C_BAUDRATE 400000U

#define FOXS8700_WHOAMI 0xC7U
#define MMA8451_WHOAMI 0x1AU
#define MPU6050_WHOAMI 0x68U
#define ACCEL_STATUS 0x00U
#define ACCEL_XYZ_DATA_CFG 0x0EU
#define ACCEL_CTRL_REG1 0x2AU
/* MPU6050 who_am_i register address. */
#define ACCEL_WHOAMI_REG 0x75U
#define ACCEL_READ_TIMES 10U
/**********The register define of the MPU-6050***********************************/
/********Register Name**********Register Addr**************/
/*******|				|******|    	|*************/
#define	 AUX_VDDIO			0x01		//1	����I2C��Դѡ��Ĵ���			
#define	 SELF_TEST_X			0x0D		//13 	X���Լ�Ĵ���
#define	 SELF_TEST_Y			0x0E		//14 	Y���Լ�Ĵ���		
#define	 SELF_TEST_Z			0x0F		//15 	Z���Լ�Ĵ���
#define	 SELF_TEST_A			0x10		//16	���ٶȼ��Լ�
#define	 SMPLRT_DIV			0x19		//19	����Ƶ�ʷ�Ƶ�Ĵ���
#define	 CONFIG				0x1A		//26	���üĴ���		
#define  GYRO_CONFIG			0x1B		//27	���������üĴ���	
#define  ACCEL_CONFIG 			0x1C		//28	���ټ���üĴ����				
#define  FF_THR	 			0x1D		//29	����������ֵ�Ĵ���				
#define  FF_DUR				0x1E		//30	�����������ʱ��Ĵ���			
#define  MOT_THR			0x1F		//31	�˶�̽����ֵ�Ĵ���			
#define  MOT_DUR			0x20		//32	�˶�̽�����ʱ��Ĵ���				
#define  ZRMOT_THR			0x21		//33	���˶���ֵ���Ĵ���				
#define  ZRMOT_DUR			0x22		//34	���˶�����ʱ��Ĵ���		
#define  FIFO_EN			0x23		//35	FIFOʹ�ܼĴ���				
#define  I2C_MST_CTRL			0x24		//36	I2C�������ƼĴ���				
#define  I2C_SLV0_ADDR			0x25		//37	I2C�ӻ�0��ַ�Ĵ���				
#define  I2C_SLV0_REG			0x26		//38	I2C�ӻ�0�Ĵ���			
#define  I2C_SLV0_CTRL			0x27		//39	I2C�ӻ�0���ƼĴ���		
#define  I2C_SLV1_ADDR			0x28		//40	I2C�ӻ�1��ַ�Ĵ���			
#define  I2C_SLV1_REG			0x29		//41	I2C�ӻ�1�Ĵ���			
#define  I2C_SLV1_CTRL			0x2A		//42	I2C�ӻ�1���ƼĴ���			
#define  I2C_SLV2_ADDR			0x2B		//43	I2C�ӻ�2��ַ�Ĵ���				
#define  I2C_SLV2_REG			0x2C		//44	I2C�ӻ�2�Ĵ���				
#define  I2C_SLV2_CTRL			0x2D		//45	I2C�ӻ�2���ƼĴ���			
#define  I2C_SLV3_ADDR			0x2E		//46	I2C�ӻ�3��ַ�Ĵ���			
#define  I2C_SLV3_REG 			0x2F		//47	I2C�ӻ�3�Ĵ���			
#define  I2C_SLV3_CTRL			0x30		//48	I2C�ӻ�3���ƼĴ���		
#define  I2C_SLV4_ADDR	 		0x31		//49	I2C�ӻ�4��ַ�Ĵ���				
#define  I2C_SLV4_REG			0x32		//50	I2C�ӻ�4�Ĵ���			
#define  I2C_SLV4_DO			0x33		//51	I2C�ӻ�4ֱ������Ĵ�����Direct Output��			
#define  I2C_SLV4_CTRL			0x34		//52	I2C�ӻ�4���ƼĴ���			
#define  I2C_SLV2_DI			0x35		//53	I2C�ӻ�4ֱ������Ĵ�����Direct Iutput��				
#define  I2C_MST_STATUS			0x36		//54	I2C����״̬�Ĵ���				
#define  INT_PIN_CFG			0x37		//55	�ж�����/��·ʹ�����üĴ���			
#define  INT_ENABLE			0x38		//56	�ж�ʹ�ܼĴ���				
#define  INT_STATUS			0x3A		//58	�ж�״̬�Ĵ���				
#define  ACCEL_XOUT_H			0x3B		//59	���ټƲ���ֵ�Ĵ���		X��߰�λ		
#define  ACCEL_XOUT_L			0x3C		//60	���ټƲ���ֵ�Ĵ���		X��Ͱ�λ
#define  ACCEL_YOUT_H			0x3D		//61	���ټƲ���ֵ�Ĵ���		Y��߰�λ		
#define  ACCEL_YOUT_L			0x3E		//62	���ټƲ���ֵ�Ĵ���		Y��Ͱ�λ		
#define  ACCEL_ZOUT_H			0x3F		//63	���ټƲ���ֵ�Ĵ���		Z��߰�λ		
#define  ACCEL_ZOUT_L			0x40		//64	���ټƲ���ֵ�Ĵ���		Z��Ͱ�λ		
#define  TEMP_OUT_H			0x41		//65	�¶Ȳ���ֵ�Ĵ���		�߰�λ			
#define  TEMP_OUT_L			0x42		//66	�¶Ȳ���ֵ�Ĵ���		�Ͱ�λ			
#define  GYRO_XOUT_H			0x43		//67	�����ǲ���ֵ�Ĵ���		X��߰�λ		
#define  GYRO_XOUT_L			0x44		//68	�����ǲ���ֵ�Ĵ���		X��Ͱ�λ		
#define  GYRO_YOUT_H			0x45		//69	�����ǲ���ֵ�Ĵ���		Y��߰�λ		
#define  GYRO_YOUT_L			0x46		//70	�����ǲ���ֵ�Ĵ���		Y��Ͱ�λ		
#define  GYRO_ZOUT_H			0x47		//71	�����ǲ���ֵ�Ĵ���		Z��߰�λ		
#define  GYRO_ZOUT_L			0x48		//72	�����ǲ���ֵ�Ĵ���		Z��Ͱ�λ		
#define  EXT_SENS_DATA_00		0x49		//73	��Ӵ��������ݼĴ���0	���������ã�			
#define  EXT_SENS_DATA_01		0x4A		//74	��Ӵ��������ݼĴ���1	���������ã�			
#define  EXT_SENS_DATA_02		0x4B		//75	��Ӵ��������ݼĴ���2	���������ã�			
#define  EXT_SENS_DATA_03		0x4C		//76	��Ӵ��������ݼĴ���3	���������ã�			
#define  EXT_SENS_DATA_04		0x4D		//77	��Ӵ��������ݼĴ���4	���������ã�			
#define  EXT_SENS_DATA_05		0x4E		//78	��Ӵ��������ݼĴ���5	���������ã�			
#define  EXT_SENS_DATA_06		0x4F		//79	��Ӵ��������ݼĴ���6	���������ã�			
#define  EXT_SENS_DATA_07		0x50		//80	��Ӵ��������ݼĴ���7	���������ã�			
#define  EXT_SENS_DATA_08		0x51		//81	��Ӵ��������ݼĴ���8	���������ã�			
#define  EXT_SENS_DATA_09		0x52		//82	��Ӵ��������ݼĴ���9	���������ã�			
#define  EXT_SENS_DATA_10		0x53		//83	��Ӵ��������ݼĴ���10	���������ã�			
#define  EXT_SENS_DATA_11		0x54		//84	��Ӵ��������ݼĴ���11	���������ã�			
#define  EXT_SENS_DATA_12		0x55		//85	��Ӵ��������ݼĴ���12	���������ã�			
#define  EXT_SENS_DATA_13		0x56		//86	��Ӵ��������ݼĴ���13	���������ã�			
#define  EXT_SENS_DATA_14		0x57		//87	��Ӵ��������ݼĴ���14	���������ã�			
#define  EXT_SENS_DATA_15		0x58		//88	��Ӵ��������ݼĴ���15	���������ã�			
#define  EXT_SENS_DATA_16		0x59		//89	��Ӵ��������ݼĴ���16	���������ã�			
#define  EXT_SENS_DATA_17		0x5A		//90	��Ӵ��������ݼĴ���17	���������ã�			
#define  EXT_SENS_DATA_18		0x5B		//91	��Ӵ��������ݼĴ���18	���������ã�			
#define  EXT_SENS_DATA_19		0x5C		//92	��Ӵ��������ݼĴ���19	���������ã�			
#define  EXT_SENS_DATA_20		0x5D		//93	��Ӵ��������ݼĴ���20	���������ã�			
#define  EXT_SENS_DATA_21		0x5E		//94	��Ӵ��������ݼĴ���21	���������ã�			
#define  EXT_SENS_DATA_22		0x5F		//95	��Ӵ��������ݼĴ���22	���������ã�			
#define  EXT_SENS_DATA_23		0x60		//96	��Ӵ��������ݼĴ���23	���������ã�			
#define  MOT_DETECT_STATUS		0x61		//97	�˶�̽��״̬�Ĵ���				
#define  I2C_SLV0_D0			0x63		//99	I2C0ģʽ��������Ĵ���				
#define  I2C_SLV1_D0			0x64		//100	I2C1ģʽ��������Ĵ���			
#define  I2C_SLV2_D0			0x65		//101	I2C2ģʽ��������Ĵ���			
#define  I2C_SLV3_D0			0x66		//102	I2C3ģʽ��������Ĵ���			
#define  I2C_MST_DELAY_CTRL		0x67		//103	I2C����ģʽ��ʱ���ƼĴ���				
#define  SINGLE_PATH_RESET		0x68		//104	�����ź�·����λ�Ĵ��������ģ��������ź�·����			
#define  MOT_DETECT_CTRL		0x69		//105	�˶�̽����ƼĴ���			
#define  USER_CTRL			0x6A		//106	�û����ƼĴ���			
#define  PWR_MGMT_1			0x6B		//107	��Դ����Ĵ���1			
#define  PWR_MGMT_2			0x6C		//108	��Դ����Ĵ���2		
#define  FIFO_COUNTH			0x72		//		FIFO�������Ĵ����߰�λ		
#define  FIFO_COUNTL			0x73		//		FIFO�������Ĵ����Ͱ�λ		
#define  FIFO_R_W			0x74		//		FIFO��д�Ĵ���		
#define  WHO_AM_I			0x75		//		�����֤�Ĵ���		
/***********************************************************************************************************/

/* PWR_MGMT_1	 Bit Fields */
#define MPU_PWR_MGMT_1_DEVICE_RESET_MASK	0x80u				
#define MPU_PWR_MGMT_1_DEVICE_RESET_SHIFT	7				
#define MPU_PWR_MGMT_1_SLEEP_MASK			0x40u			
#define MPU_PWR_MGMT_1_SLEEP_RESET_SHIFT	6				
#define MPU_PWR_MGMT_1_CYCLE_MASK			0x20u			
#define MPU_PWR_MGMT_1_CYCLE_RESET_SHIFT	5				
#define MPU_PWR_MGMT_1_TEMP_DIS_MASK		0x8u			
#define MPU_PWR_MGMT_1_TEMP_DIS_SHIFT		3			
#define MPU_PWR_MGMT_1_CLKSEL_MASK			0x3u		
#define MPU_PWR_MGMT_1_CLKSEL_SHIFT			0
#define MPU_PWR_MGMT_1_CLKSEL_DATA(x)		(((uint8_t)(((uint8_t)(x))<<MPU_PWR_MGMT_1_CLKSEL_SHIFT))&MPU_PWR_MGMT_1_CLKSEL_MASK)
/* CONFIG	 Bit Fields */
#define MPU_CONFIG_EXT_SYNC_SET_MASK		0x38u
#define MPU_CONFIG_EXT_SYNC_SET_SHIFT		3
#define MPU_CONFIG_EXT_SYNC_SET_DATA(x)		(((uint8_t)(((uint8_t)(x))<<MPU_CONFIG_EXT_SYNC_SET_SHIFT))&MPU_CONFIG_EXT_SYNC_SET_MASK)
#define MPU_CONFIG_DLPF_CFG_MASK			0x3u
#define MPU_CONFIG_DLPF_CFG_SHIFT			0
#define MPU_CONFIG_DLPF_CFG_DATA(x)			(((uint8_t)(((uint8_t)(x))<<MPU_CONFIG_DLPF_CFG_SHIFT))&MPU_CONFIG_DLPF_CFG_MASK)
/* SMPLRT_DIV	 Bit Fields */
#define MPU_SMPLRT_DIV_DATA_MASK			0xFFu
#define MPU_SMPLRT_DIV_DATA_SHIFT			0
#define MPU_SMPLRT_DIV_DATA(x)		 		(((uint8_t)(((uint8_t)(x))<<MPU_SMPLRT_DIV_DATA_SHIFT))&MPU_SMPLRT_DIV_DATA_MASK)
/* GYRO_CONFIG	 Bit Fields */
#define MPU_GYRO_CONFIG_XG_ST_MASK			0x80u
#define MPU_GYRO_CONFIG_XG_ST_SHIFT			7
#define MPU_GYRO_CONFIG_YG_ST_MASK			0x40u
#define MPU_GYRO_CONFIG_YG_ST_SHIFT			6
#define MPU_GYRO_CONFIG_ZG_ST_MASK			0x20u
#define MPU_GYRO_CONFIG_ZG_ST_SHIFT			5
#define MPU_GYRO_CONFIG_FS_SEL_MASK			0x18u
#define MPU_GYRO_CONFIG_FS_SEL_SHIFT		3
#define MPU_GYRO_CONFIG_FS_SEL_DATA(x)		(((uint8_t)(((uint8_t)(x))<<MPU_GYRO_CONFIG_FS_SEL_SHIFT))&MPU_GYRO_CONFIG_FS_SEL_MASK)
/* ACCEL_CONFIG	 Bit Fields */
#define MPU_ACCEL_CONFIG_XA_ST_MASK			0x80u
#define MPU_ACCEL_CONFIG_XA_ST_SHIFT			7
#define MPU_ACCEL_CONFIG_YA_ST_MASK			0x40u
#define MPU_ACCEL_CONFIG_YA_ST_SHIFT			6
#define MPU_ACCEL_CONFIG_ZA_ST_MASK			0x20u
#define MPU_ACCEL_CONFIG_ZA_ST_SHIFT			5
#define MPU_ACCEL_CONFIG_AFS_SEL_MASK		0x18u
#define MPU_ACCEL_CONFIG_AFS_SEL_SHIFT			3
#define MPU_ACCEL_CONFIG_AFS_SEL_DATA(x)	(((uint8_t)(((uint8_t)(x))<<MPU_ACCEL_CONFIG_AFS_SEL_SHIFT))&MPU_ACCEL_CONFIG_AFS_SEL_MASK)	
/***********Device base address*************/
/*******************************************************************************
 * Prototypes
 ******************************************************************************/


static bool I2C_ReadAccelWhoAmI(void);
static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*  mpu6050 device address */
static const uint8_t g_accel_address[] = {0x68U, 0x69};

static i2c_master_handle_t g_m_handle;

uint8_t g_accel_addr_found = 0x00;

static volatile bool completionFlag = false;
static volatile bool nakFlag = false;
static bool isThereAccel = false;
/*******************************************************************************
 * Code
 ******************************************************************************/

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
* Function Name  : I2C_ReadWhoAmI �ڲ�����
* Description    : i2c���߳�ʼ�� ���������ַ
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
    uint8_t who_am_i_reg = ACCEL_WHOAMI_REG;
    uint8_t who_am_i_value = 0x00;
    uint8_t accel_addr_array_size = 0x00;
    bool find_device = false;
    uint8_t i = 0;
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

    masterXfer.slaveAddress = g_accel_address[0];
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0;
    masterXfer.subaddressSize = 0;
    masterXfer.data = &who_am_i_reg;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferNoStopFlag;

    accel_addr_array_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);

    for (i = 0; i < accel_addr_array_size; i++)
    {
        masterXfer.slaveAddress = g_accel_address[i];

        I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            find_device = true;
            g_accel_addr_found = masterXfer.slaveAddress;
            break;
        }
    }

    if (find_device == true)
    {
        masterXfer.direction = kI2C_Read;
        masterXfer.subaddress = 0;
        masterXfer.subaddressSize = 0;
        masterXfer.data = &who_am_i_value;
        masterXfer.dataSize = 1;
        masterXfer.flags = kI2C_TransferRepeatedStartFlag;

        I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

        /*  wait for transfer completed. */
        while ((!nakFlag) && (!completionFlag))
        {
        }

        nakFlag = false;

        if (completionFlag == true)
        {
            completionFlag = false;
            if (who_am_i_value == FOXS8700_WHOAMI)
            {
                PRINTF("Found a FOXS8700 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else if (who_am_i_value == MMA8451_WHOAMI)
            {
                PRINTF("Found a MMA8451 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else if (who_am_i_value == MPU6050_WHOAMI)
            {
                PRINTF("Found a MPU6050 on board , the device address is 0x%x . \r\n", masterXfer.slaveAddress);
                return true;
            }
            else
            {
                PRINTF("Found a device, the WhoAmI value is 0x%x\r\n", who_am_i_value);
                PRINTF("It's not MMA8451 or FXOS8700. \r\n");
                PRINTF("The device address is 0x%x. \r\n", masterXfer.slaveAddress);
                return false;
            }
        }
        else
        {
            PRINTF("Not a successful i2c communication \r\n");
            return false;
        }
    }
    else
    {
        PRINTF("\r\n Do not find an accelerometer device ! \r\n");
        return false;
    }
}

/*******************************************************************************
* Function Name  : I2C_WriteAccelReg �ڲ�����
* Description    : д��mpu6050�ڲ��Ĵ�����ֵ
* Input          : base-i2c���� device_addr-�豸��ַ reg_addr�Ĵ�����ַ 
*                      value - Ҫд�������
* Output         : None
* Return         : �Ƿ�ɹ�
*******************************************************************************/
static bool I2C_WriteAccelReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

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
* Function Name  : I2C_WriteAccelRegs �ڲ�����
* Description    : д��mpu6050�ڲ��Ĵ�����ֵ
* Input          : base-i2c���� device_addr-�豸��ַ reg_addr�Ĵ�����ַ 
*                   len-д�����ݵĳ���   buf - Ҫд������� 
* Output         : None
* Return         : �Ƿ�ɹ�
*******************************************************************************/
static bool I2C_WriteAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr,uint8_t len,  uint8_t* buf)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = buf;
    masterXfer.dataSize = len;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

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
* Function Name  : I2C_ReadAccelRegs �ڲ�����
* Description    : ��ȡmpu6050�ڲ��Ĵ�����ֵ
* Input          : base-i2c���� device_addr-�豸��ַ reg_addr�Ĵ�����ַ 
*                      rxSize-�������ݵĳ���
* Output         : rxBuff - Ҫ����������
* Return         : �Ƿ�ɹ�
*******************************************************************************/
static bool I2C_ReadAccelRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);

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
* Function Name  : I2C_ReadAccelRegs �ڲ�����
* Description    : ��ȡmpu6050�ڲ��Ĵ�����ֵ
* Input          : base-i2c���� device_addr-�豸��ַ reg_addr�Ĵ�����ַ 
*                      rxSize-�������ݵĳ���
* Output         : rxBuff - Ҫ����������
* Return         : �Ƿ�ɹ�
*******************************************************************************/
static bool I2C_ReadAccelRegs_Poll(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));
    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /*  direction=write : start+device_write;cmdbuff;xBuff; */
    /*  direction=recive : start+device_write;cmdbuff;repeatStart+device_read;xBuff; */

    //I2C_MasterTransferNonBlocking(BOARD_ACCEL_I2C_BASEADDR, &g_m_handle, &masterXfer);
    I2C_MasterTransferBlocking(BOARD_ACCEL_I2C_BASEADDR, &masterXfer);
    /*  wait for while */

        return true;

}

/*******************************************************************************
* Function Name  : mpu6050_read_temperature �ڲ�����
* Description    : ��ȡmpu6050 �¶�ֵ
* Input          : None
* Output         : None
* Return         : �¶�ֵ(������100��)
*******************************************************************************/
uint16_t mpu6050_read_temperature(void)
{
  uint8_t err;  
  uint8_t readBuff[2]; 
  uint16_t raw;
  float temp;
  uint16_t temperature;
  err = I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, TEMP_OUT_H, readBuff, 2);  
  raw = ((uint16_t)readBuff[0]<<8)|readBuff[1];  
  temp = 36.53 +(float) (((int16_t)raw)/340);
  temperature = (uint16_t) (temp*100);
  return temperature;
}

/*******************************************************************************
* Function Name  : mpu6050_read_accel �ڲ�����
* Description    : ��ȡmpu6050 ���ٶ�ֵ
* Input          : None
* Output         : adata - ����������
* Return         : �Ƿ�ɹ�
*******************************************************************************/
int mpu6050_read_accel(int16_t* adata)
{
    uint8_t err;
    uint8_t readBuff[6];
    
    //err = I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_XOUT_H, readBuff, 6);    
    err = I2C_ReadAccelRegs_Poll(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, ACCEL_XOUT_H, readBuff, 6);    
    adata[0] = ((int16_t)(((readBuff[0] * 256U) | readBuff[1])));  	    
    adata[1] = ((int16_t)(((readBuff[2] * 256U) | readBuff[3])));  	    
    adata[2] = ((int16_t)(((readBuff[4] * 256U) | readBuff[5]))); 
    
    return err;    
}

/*******************************************************************************
* Function Name  : mpu6050_read_gyro �ڲ�����
* Description    : ��ȡmpu6050 ���ٶ�ֵ
* Input          : None
* Output         : gdata - ����������
* Return         : �Ƿ�ɹ�
*******************************************************************************/
int mpu6050_read_gyro(int16_t *gdata)
{
    uint8_t err;
    uint8_t readBuff[6];
    
    //err = I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, GYRO_XOUT_H, readBuff, 6);    
    err = I2C_ReadAccelRegs_Poll(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, GYRO_XOUT_H, readBuff, 6);    
    gdata[0] = ((int16_t)(((readBuff[0] * 256U) | readBuff[1]))); 	    
    gdata[1] = ((int16_t)(((readBuff[2] * 256U) | readBuff[3]))); 	    
    gdata[2] = ((int16_t)(((readBuff[4] * 256U) | readBuff[5])));
    
    return err;    
}

/*******************************************************************************
* Function Name  : mpu6050_init
* Description    : mpu6050��i2c ��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void mpu6050_init(void)
{
    i2c_master_transfer_t masterXfer;
    
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    
    I2C_MasterTransferCreateHandle(BOARD_I2C_BASEADDR, &g_m_handle, i2c_master_callback, NULL);
    isThereAccel = I2C_ReadAccelWhoAmI();  
    
    /*  read the accel xyz value if there is accel device on board */
    if(true == isThereAccel)
    {

        /*  please refer to the "example MPU6050 Driver Code" in MPU6050 datasheet. */
        /*  MPU6050��ʼ������*/
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,PWR_MGMT_1, 0x00);//�������ٶ�
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,SMPLRT_DIV, 0x0A);//
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,CONFIG, 0x00);
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,AUX_VDDIO,0x80);
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,GYRO_CONFIG, 0x08);
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,ACCEL_CONFIG, 0x00);
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,I2C_MST_CTRL, 0x00);
        I2C_WriteAccelReg(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found,INT_PIN_CFG, 0x02);
    }
    
    uint8_t status0_value = 0;        
    while ( 0x01 != (status0_value&0x01))
            {
                I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, INT_STATUS, &status0_value, 1);
            }

}
/*******************************************************************************
* Function Name  : mpu6050_getdata
* Description    : ��ȡ��ǰmpu6050ֵ �������
* Input          : None
* Output         : advalue
* Return         : None
*******************************************************************************/
void mpu6050_getdata(int16_t* a_data, int16_t* g_data, int16_t* temperature)
{
        uint8_t status0_value = 0;
        int16_t adata[3];
        int16_t gdata[3];
        int16_t temp;
        char str[64];
        while ( 0x01 != (status0_value&0x01))//�ȴ�����׼����
        {
            I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, g_accel_addr_found, INT_STATUS, &status0_value, 1);
        }
        mpu6050_read_accel(adata);
        mpu6050_read_gyro(gdata);
        sprintf(str, "adata x= %d y= %d z= %d ;", adata[0], adata[1], adata[2]);
        PRINTF(str);
        sprintf(str, "gdata x= %d y= %d z= %d ;", gdata[0], gdata[1], gdata[2]);
        PRINTF(str);
        temp = mpu6050_read_temperature();
        sprintf(str, "temperature = %d.%d \r\n", temp/100, temp%100);
        PRINTF(str);
        i2c_bus_delay();    
        
        *a_data++ = adata[0];*a_data++ = adata[1];*a_data++ = adata[2];
        *g_data++ = gdata[0];*g_data++ = gdata[1];*g_data++ = gdata[2];
        *temperature = temp;
}

/*DMP���ú���*/
/*******************************************************************************
* Function Name  : MPU_Write_Len 
* Description    : д��mpu6050�ڲ��Ĵ�����ֵ
* Input          : addr-MPU6050��ַ reg�Ĵ�����ַ 
*                   len-д�����ݵĳ���   buf - Ҫд������� 
* Output         : None
* Return         : �Ƿ�ɹ�
*******************************************************************************/
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
  if(true == I2C_WriteAccelRegs(BOARD_ACCEL_I2C_BASEADDR, addr, reg,len,  buf))
    return 0;
  else
    return 1;
}

/*******************************************************************************
* Function Name  : MPU_Read_Len 
* Description    : ��mpu6050�ڲ��Ĵ�����������
* Input          : addr-�豸��ַ addr�Ĵ�����ַ 
*                   len-Ҫ�������ݵĳ���   buf - ���������ݴ�ŵ�ַ
* Output         : None
* Return         : �Ƿ�ɹ�
*******************************************************************************/
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
  if(true == I2C_ReadAccelRegs(BOARD_ACCEL_I2C_BASEADDR, addr, reg, buf, len))
    return 0;
  else
    return 1;
}
