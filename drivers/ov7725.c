/******************************2017-2017, NJTECH, Edu.************************** 
FileName: ov7725.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:    ov7725 驱动使用軟件SCCB协议,  ov7725 地址：0x21
Version:         1.0 
History:         
      <author>  <time>   <version >   <desc> 
      Sundm    17/06/30     1.0     文件创建   
  *          SmartCar Board Pin assignment
  *          =============================
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+-----------------------------+
  *          |      Camera                 |     Port & Pin              |
  *          +-----------------------------+-----------------------------+
  *          |        SCL                  |        B2                   |
  *          |        SDA                  |        B3                   |
  *          |        D0                   |        A8                   |
  *          |        D1                   |        A9                   |
  *          |        D2                   |        A10                  |
  *          |        D3                   |        A11                  |
  *          |        D4                   |        A12                  |
  *          |        D5                   |        A13                  |
  *          |        D6                   |        A14                  |
  *          |        D7                   |        A15                  |
  *          |        PCLK                 |        A7                   |
  *          |        VSYNC                |        A16                  |
  *          |        HREF                 |        A17                  |
  *          +-----------------------------+-----------------------------+
*******************************************************************************/ 
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_i2c.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "camera_oled.h"

#include "ov7725.h"
#include "i2c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define OV7725_DEBUG		1
#if ( OV7725_DEBUG == 1 )
#include <stdio.h>
#define OV7725_TRACE	PRINTF
#else
#define OV7725_TRACE(...)
#endif

// 改变图像大小
//0: 80x60
//1: 160x120
//2: 240x180
#define IMAGE_SIZE  0

#if (IMAGE_SIZE  ==  0)
#define OV7620_W    (80)
#define OV7620_H    (60)

#elif (IMAGE_SIZE == 1)
#define OV7620_W    (160)
#define OV7620_H    (120)

#elif (IMAGE_SIZE == 2)
#define OV7620_W    (240)
#define OV7620_H    (180)

#else
#error "Image Size Not Support!"
#endif

#define BOARD_OV7620_DATA_OFFSET    (8) 

/* I2C source clock */
#define CAMERA_I2C_CLK_SRC I2C0_CLK_SRC
#define BOARD_CAMERA_I2C_BASEADDR I2C0

#define I2C_RELEASE_SDA_PORT PORTB
#define I2C_RELEASE_SCL_PORT PORTB
#define I2C_RELEASE_SDA_GPIO GPIOB
#define I2C_RELEASE_SDA_PIN 1U
#define I2C_RELEASE_SCL_GPIO GPIOB
#define I2C_RELEASE_SCL_PIN 0U
#define I2C_RELEASE_BUS_COUNT 100U
#define I2C_BAUDRATE 50000U

/*寄存器定義*/ 
#define OV7725_GAIN      0x00
#define OV7725_BLUE      0x01
#define OV7725_RED       0x02
#define OV7725_GREEN     0x03
#define OV7725_BAVG      0x05
#define OV7725_GAVG      0x06
#define OV7725_RAVG      0x07
#define OV7725_AECH      0x08
#define OV7725_COM2      0x09
#define OV7725_PID       0x0A
#define OV7725_VER       0x0B
#define OV7725_COM3      0x0C
#define OV7725_COM4      0x0D//
#define OV7725_COM5      0x0E
#define OV7725_COM6      0x0F
#define OV7725_AEC       0x10
#define OV7725_CLKRC     0x11//
#define OV7725_COM7      0x12
#define OV7725_COM8      0x13
#define OV7725_COM9      0x14
#define OV7725_COM10     0x15
#define OV7725_REG16     0x16
#define OV7725_HSTART    0x17
#define OV7725_HSIZE     0x18
#define OV7725_VSTRT     0x19
#define OV7725_VSIZE     0x1A
#define OV7725_PSHFT     0x1B
#define OV7725_MIDH      0x1C
#define OV7725_MIDL      0x1D
#define OV7725_LAEC      0x1F
#define OV7725_COM11     0x20
#define OV7725_BDBase    0x22
#define OV7725_BDMStep   0x23
#define OV7725_AEW       0x24
#define OV7725_AEB       0x25
#define OV7725_VPT       0x26
#define OV7725_REG28     0x28
#define OV7725_HOutSize  0x29
#define OV7725_EXHCH     0x2A
#define OV7725_EXHCL     0x2B
#define OV7725_VOutSize  0x2C
#define OV7725_ADVFL     0x2D
#define OV7725_ADVFH     0x2E
#define OV7725_YAVE      0x2F
#define OV7725_LumHTh    0x30
#define OV7725_LumLTh    0x31
#define OV7725_HREF      0x32
#define OV7725_DM_LNL    0x33
#define OV7725_DM_LNH    0x34
#define OV7725_ADoff_B   0x35
#define OV7725_ADoff_R   0x36
#define OV7725_ADoff_Gb  0x37
#define OV7725_ADoff_Gr  0x38
#define OV7725_Off_B     0x39
#define OV7725_Off_R     0x3A
#define OV7725_Off_Gb    0x3B
#define OV7725_Off_Gr    0x3C
#define OV7725_COM12     0x3D
#define OV7725_COM13     0x3E
#define OV7725_COM14     0x3F
#define OV7725_COM16     0x41
#define OV7725_TGT_B     0x42
#define OV7725_TGT_R     0x43
#define OV7725_TGT_Gb    0x44
#define OV7725_TGT_Gr    0x45
#define OV7725_LC_CTR    0x46
#define OV7725_LC_XC     0x47
#define OV7725_LC_YC     0x48
#define OV7725_LC_COEF   0x49
#define OV7725_LC_RADI   0x4A
#define OV7725_LC_COEFB  0x4B
#define OV7725_LC_COEFR  0x4C
#define OV7725_FixGain   0x4D
#define OV7725_AREF1     0x4F
#define OV7725_AREF6     0x54
#define OV7725_UFix      0x60
#define OV7725_VFix      0x61
#define OV7725_AWBb_blk  0x62
#define OV7725_AWB_Ctrl0 0x63
#define OV7725_DSP_Ctrl1 0x64
#define OV7725_DSP_Ctrl2 0x65
#define OV7725_DSP_Ctrl3 0x66
#define OV7725_DSP_Ctrl4 0x67
#define OV7725_AWB_bias  0x68
#define OV7725_AWBCtrl1  0x69
#define OV7725_AWBCtrl2  0x6A
#define OV7725_AWBCtrl3  0x6B
#define OV7725_AWBCtrl4  0x6C
#define OV7725_AWBCtrl5  0x6D
#define OV7725_AWBCtrl6  0x6E
#define OV7725_AWBCtrl7  0x6F
#define OV7725_AWBCtrl8  0x70
#define OV7725_AWBCtrl9  0x71
#define OV7725_AWBCtrl10 0x72
#define OV7725_AWBCtrl11 0x73
#define OV7725_AWBCtrl12 0x74
#define OV7725_AWBCtrl13 0x75
#define OV7725_AWBCtrl14 0x76
#define OV7725_AWBCtrl15 0x77
#define OV7725_AWBCtrl16 0x78
#define OV7725_AWBCtrl17 0x79
#define OV7725_AWBCtrl18 0x7A
#define OV7725_AWBCtrl19 0x7B
#define OV7725_AWBCtrl20 0x7C
#define OV7725_AWBCtrl21 0x7D
#define OV7725_GAM1      0x7E
#define OV7725_GAM2      0x7F
#define OV7725_GAM3      0x80
#define OV7725_GAM4      0x81
#define OV7725_GAM5      0x82
#define OV7725_GAM6      0x83
#define OV7725_GAM7      0x84
#define OV7725_GAM8      0x85
#define OV7725_GAM9      0x86
#define OV7725_GAM10     0x87
#define OV7725_GAM11     0x88
#define OV7725_GAM12     0x89
#define OV7725_GAM13     0x8A
#define OV7725_GAM14     0x8B
#define OV7725_GAM15     0x8C
#define OV7725_SLOP      0x8D
#define OV7725_DNSTh     0x8E
#define OV7725_EDGE0     0x8F
#define OV7725_EDGE1     0x90
#define OV7725_DNSOff    0x91
#define OV7725_EDGE2     0x92
#define OV7725_EDGE3     0x93
#define OV7725_MTX1      0x94
#define OV7725_MTX2      0x95
#define OV7725_MTX3      0x96
#define OV7725_MTX4      0x97
#define OV7725_MTX5      0x98
#define OV7725_MTX6      0x99
#define OV7725_MTX_Ctrl  0x9A
#define OV7725_BRIGHT    0x9B
#define OV7725_CNST      0x9C
#define OV7725_UVADJ0    0x9E
#define OV7725_UVADJ1    0x9F
#define OV7725_SCAL0     0xA0
#define OV7725_SCAL1     0xA1
#define OV7725_SCAL2     0xA2
#define OV7725_SDE       0xA6
#define OV7725_USAT      0xA7
#define OV7725_VSAT      0xA8
#define OV7725_HUECOS    0xA9
#define OV7725_HUESIN    0xAA
#define OV7725_SIGN      0xAB
#define OV7725_DSPAuto   0xAC

/*******************************************************************************
 * Variables 图像数据
 ******************************************************************************/

uint8_t  image_exd_data[180*240]={0x00};
uint8_t  image_data[]=
{
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00,
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, //20
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, //40
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 
0x55,0x00,0x06,0x00,0x00,0x00,0x00, 0x00, 0x00, 0x00, 0x00, 


};
   
struct ov7725_reg
{
    uint8_t addr;
    uint8_t val;
};

struct 
{
    uint32_t i2c_instance;
    uint8_t  addr;
    uint32_t h_size;
    uint32_t v_size;
}h_ov7725;

typedef enum
{
    TRANSFER_IN_PROCESS,
    NEXT_FRAME,
}OV7620_Status;

edma_transfer_config_t transferConfig;
edma_handle_t g_EDMA_Handle;
volatile bool g_Transfer_Done = false;

static const uint8_t ov7725_addr[] = {0x21};
/*寄存器初始化数值*/
static const struct ov7725_reg reg_tbl[] =
{
    {OV7725_COM4         , 0X40},  // , 0x81},
    {OV7725_CLKRC        , 0X00},  // , 0x01},
    {OV7725_COM2         , 0x03},  
    {OV7725_COM3         , 0xD0},  
    {OV7725_COM7         , 0x40},  
    {OV7725_HSTART       , 0x3F},  
    {OV7725_HSIZE        , 0x50},  
    {OV7725_VSTRT        , 0x03},  
    {OV7725_VSIZE        , 0x78},  
    {OV7725_HREF         , 0x00},  
    {OV7725_SCAL0        , 0x0A},  
    {OV7725_AWB_Ctrl0    , 0xE0},  
    {OV7725_DSPAuto      , 0xff},  
    {OV7725_DSP_Ctrl2    , 0x0C},  
    {OV7725_DSP_Ctrl3    , 0x00},  
    {OV7725_DSP_Ctrl4    , 0x00},  
    {OV7725_EXHCH        , 0x00},  
    {OV7725_GAM1         , 0x0c},  
    {OV7725_GAM2         , 0x16},  
    {OV7725_GAM3         , 0x2a},  
    {OV7725_GAM4         , 0x4e},  
    {OV7725_GAM5         , 0x61},  
    {OV7725_GAM6         , 0x6f},  
    {OV7725_GAM7         , 0x7b},  
    {OV7725_GAM8         , 0x86},  
    {OV7725_GAM9         , 0x8e},  
    {OV7725_GAM10        , 0x97},  
    {OV7725_GAM11        , 0xa4},  
    {OV7725_GAM12        , 0xaf},  
    {OV7725_GAM13        , 0xc5},  
    {OV7725_GAM14        , 0xd7},  
    {OV7725_GAM15        , 0xe8},  
    {OV7725_SLOP         , 0x20},  
    {OV7725_LC_RADI      , 0x00},  
    {OV7725_LC_COEF      , 0x13},  
    {OV7725_LC_XC        , 0x08},  
    {OV7725_LC_COEFB     , 0x14},  
    {OV7725_LC_COEFR     , 0x17},  
    {OV7725_LC_CTR       , 0x05},  
    {OV7725_BDBase       , 0x99},  
    {OV7725_BDMStep      , 0x03},  
    {OV7725_SDE          , 0x04},  
    {OV7725_BRIGHT       , 0x00},  
    {OV7725_CNST         , 70},    
    {OV7725_SIGN         , 0x06},  
    {OV7725_UVADJ0       , 0x11},  
    {OV7725_UVADJ1       , 0x02},  
};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* 图像内存池 */
uint8_t gCCD_RAM[(OV7620_H)*((OV7620_W/8)+1)];
uint8_t * gpHREF[OV7620_H+1];

/*******************************************************************************
 * Code
 ******************************************************************************/
/*******************************************************************************
* Function Name  : delay 内部函数  内部函数
* Description    : 短延时函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < 0x10000; i++)
    {
        __NOP();
    }
}

/*******************************************************************************
* Function Name  : ov7725_probe  内部函数
* Description    : clk si置位和复位
* Input          : None  移植SCCB时保留 i2c_instance-随便填写 
* Output         : None
* Return         : 0-成功 1-失败
*******************************************************************************/
static int ov7725_probe(uint8_t i2c_instance)
{
    int i,j;
    int r;
    uint8_t dummy;
    for(i = 0; i < ARRAY_SIZE(ov7725_addr); i++)
    {
        if(!SCCB_ReadSingleRegister(i2c_instance, ov7725_addr[i], OV7725_VER, &dummy))
        {
            /* found device */
            OV7725_TRACE("device found addr:0x%X\r\n", ov7725_addr[i]);
            /* reset */
            SCCB_WriteSingleRegister(i2c_instance, ov7725_addr[i], OV7725_COM7, 0x80);
            /* inject default register value */
            for(j = 0; j < ARRAY_SIZE(reg_tbl); j++)
            {
                delay();
                r = SCCB_WriteSingleRegister(i2c_instance, ov7725_addr[i], reg_tbl[j].addr, reg_tbl[j].val);
                if(r)
                {
                    OV7725_TRACE("device[addr:0x%X]regiser[addr:0x%X] write error!\r\n", ov7725_addr[i], reg_tbl[j].addr);
                }
            }
            h_ov7725.addr = ov7725_addr[i];
            h_ov7725.i2c_instance = i2c_instance;
            h_ov7725.h_size = 80;
            h_ov7725.v_size = 60;
            return 0;
        }
    }
    OV7725_TRACE("no sccb device found!\r\n");
    return 1;
}

/*******************************************************************************
* Function Name  : ov7725_set_image_size  内部函数
* Description    : 设置图像大小 ov7725
* Input          : size ： H_80_W_60,   H_120_W_160,  H_180_W_240, H_240_W_320,
* Output         : None
* Return         : 0-成功 1-失败
*******************************************************************************/
static int ov7725_set_image_size(ov7725_size size)
{
    switch(size)
    {
        case H_80_W_60:
            h_ov7725.h_size = 80;
            h_ov7725.v_size = 60;
            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_HOutSize, 0x14);
            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_VOutSize, 0x1E);
            break;
        case H_120_W_160:
            h_ov7725.h_size = 160;
            h_ov7725.v_size = 120;
            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_HOutSize, 0x28);
            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_VOutSize, 0x3C);  
            break;
        case H_180_W_240:
            h_ov7725.h_size = 240;
            h_ov7725.v_size = 180;
            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_HOutSize, 0x3C);
            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_VOutSize, 0x5A);  
            break;
        case H_240_W_320:
            h_ov7725.h_size = 320;
            h_ov7725.v_size = 240;
            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_HOutSize, 0x50);
            SCCB_WriteSingleRegister(h_ov7725.i2c_instance, h_ov7725.addr, OV7725_VOutSize, 0x78);  
            break;
        default:
            OV7725_TRACE("wrong param in func:ov7725_set_image_size\r\n");
            break;
    }
    return 0;
}

/*******************************************************************************
* Function Name  : camera_pin_init  内部函数
* Description    : camera pin 初始化 数据口浮空输入 pclk vsync href下拉输入
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
 static void camera_pin_init(void)
{
    /* Define the init structure for the input switch pin */
    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalInput, 0,
    };
    /* Enableport clock */
    CLOCK_EnableClock(kCLOCK_PortA);
    
    /* Affects PORTA_PCR register */
    port_pin_config_t port_config = {0};    
    port_config.pullSelect = kPORT_PullDisable;
    port_config.mux = kPORT_MuxAsGpio;
    
    PORT_SetPinConfig(PORTA, 8 , &port_config);
    PORT_SetPinConfig(PORTA, 9 , &port_config);
    PORT_SetPinConfig(PORTA, 10, &port_config);
    PORT_SetPinConfig(PORTA, 11, &port_config);
    PORT_SetPinConfig(PORTA, 12, &port_config);
    PORT_SetPinConfig(PORTA, 13, &port_config);
    PORT_SetPinConfig(PORTA, 14, &port_config);
    PORT_SetPinConfig(PORTA, 15, &port_config);

    /* 数据线A8-A15  输入*/
    GPIO_PinInit(GPIOA, 8 , &gpio_config);	
    GPIO_PinInit(GPIOA, 9 , &gpio_config);	
    GPIO_PinInit(GPIOA, 10, &gpio_config);	
    GPIO_PinInit(GPIOA, 11, &gpio_config);	
    GPIO_PinInit(GPIOA, 12, &gpio_config);	
    GPIO_PinInit(GPIOA, 13, &gpio_config);	
    GPIO_PinInit(GPIOA, 14, &gpio_config);	
    GPIO_PinInit(GPIOA, 15, &gpio_config);
    
    port_config.pullSelect = kPORT_PullDown;
    PORT_SetPinConfig(PORTA, 7 , &port_config);
    PORT_SetPinConfig(PORTA, 16, &port_config);
    PORT_SetPinConfig(PORTA, 17, &port_config);
    
    /* Init input  GPIO. 行HREF  下降 中断 在場中斷中打開*/
    GPIO_PinInit(GPIOA, 17U, &gpio_config);
    //PORT_SetPinInterruptConfig(PORTA, 17U, kPORT_InterruptFallingEdge);
    
    /* Init input  GPIO. 开 场vsync下降 中断 */
    GPIO_PinInit(GPIOA, 16U, &gpio_config);	
    PORT_SetPinInterruptConfig(PORTA, 16U, kPORT_InterruptFallingEdge);

    /*开 pclk 上升沿 DMA 传输 */
    GPIO_PinInit(GPIOA, 7U, &gpio_config);	
    PORT_SetPinInterruptConfig(PORTA, 7U, kPORT_DMARisingEdge);
    EnableIRQ(PORTA_IRQn);
}

/*******************************************************************************
* Function Name  : camera_dma_init  内部函数
* Description    : camera DMA传输配置
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
static void camera_dma_init(void)
{
    edma_config_t userConfig;

    /* Configure DMAMUX */
    DMAMUX_Init(DMAMUX0);
    DMAMUX_SetSource(DMAMUX0, 0, kDmaRequestMux0PortA);
    DMAMUX_EnableChannel(DMAMUX0, 0);
    /* Configure EDMA one shot transfer */
    /*
     * userConfig.enableRoundRobinArbitration = false;
     * userConfig.enableHaltOnError = true;
     * userConfig.enableContinuousLinkMode = false;
     * userConfig.enableDebugMode = false;
     */
    EDMA_GetDefaultConfig(&userConfig);
    EDMA_Init(DMA0, &userConfig);
    EDMA_CreateHandle(&g_EDMA_Handle, DMA0, 0);
    EDMA_PrepareTransfer(&transferConfig, (void*)(0x400ff011), 1, (void*)gpHREF[0], 1,
                         1, ((OV7620_W/8)+1), kEDMA_PeripheralToMemory);
    EDMA_SubmitTransfer(&g_EDMA_Handle, &transferConfig);
}

/*******************************************************************************
* Function Name  : DMA_SetMajorLoopCounter   内部函数
* Description    : 设置DMA主传输次数
* Input          : ch-通道 val-传输次数  
* Output         : None
* Return         : None
*******************************************************************************/
static void DMA_SetMajorLoopCounter(uint8_t chl, uint32_t val)
{
    DMA0->TCD[chl].CITER_ELINKNO &= ~DMA_CITER_ELINKNO_CITER_MASK;
    DMA0->TCD[chl].CITER_ELINKNO |= DMA_CITER_ELINKNO_CITER(val);
}

/*******************************************************************************
* Function Name  : DMA_SetDestAddress  内部函数
* Description    : 设置DMA传输时目的地址
* Input          : ch-通道 address-目的地址 
* Output         : None
* Return         : None
*******************************************************************************/
static void DMA_SetDestAddress(uint8_t ch, uint32_t address)
{
    DMA0->TCD[ch].DADDR = address;
}

/*******************************************************************************
* Function Name  : show_image  内部函数
* Description    : OLED显示图像
* Input          : None 
* Output         : None
* Return         : None
*******************************************************************************/
static void show_image(void)
{
        OledImge_Extend(image_exd_data, gCCD_RAM,OV7620_W,OV7620_H);// 原始数据 80*60        
        oled_data120160_convert (image_exd_data, image_exd_data ,0);
        oled_data120160_data12864_convert (image_exd_data, image_exd_data);
        oled_display_image (image_exd_data, 128*64, 128, 64);
  
}

/*******************************************************************************
* Function Name  : SerialDispImage  
* Description    : camera 图像data串口打印
* Input          : ppData图像地址, xSize-图像宽,  ySize-图像高
* Output         : None
* Return         : None
*******************************************************************************/
static void SerialDispImage(int xSize, int ySize, uint8_t** ppData)
{
    int x,y;
    
    for(y = 0; y < ySize; y++)
    {
        for(x = 1; x < (xSize/8)+1; x++)
        {
            PRINTF("%d",(ppData[y][x]>>7) & 0x01);
            PRINTF("%d",(ppData[y][x]>>6) & 0x01);
            PRINTF("%d",(ppData[y][x]>>5) & 0x01);
            PRINTF("%d",(ppData[y][x]>>4) & 0x01);
            PRINTF("%d",(ppData[y][x]>>3) & 0x01);
            PRINTF("%d",(ppData[y][x]>>2) & 0x01);
            PRINTF("%d",(ppData[y][x]>>1) & 0x01);
            PRINTF("%d",(ppData[y][x]>>0) & 0x01);
            if(x == xSize/8)
                printf("\r\n");   
        }
        if(y==ySize -1)
        {
            printf("                                                                                ");
            printf("\r\n");  
        }				
    }
}

/*******************************************************************************
* Function Name  : PORTA_IRQHandler
* Description    : 外部 中断服务函数 A16 A17
* Input          : None
* Output         : None
* Return         : None
******************************************************************************/
void PORTA_IRQHandler(void)
{
    static uint8_t status = TRANSFER_IN_PROCESS;
    static uint32_t h_counter;
    /* A17 行中断 */
    if ((1 << 17) == (GPIO_GetPinsInterruptFlags(GPIOA)&(1 << 17)) )
    {
        /* Clear external interrupt A17 flag. */
        GPIO_ClearPinsInterruptFlags(GPIOA, 1U << 17);
        
        /*打开DMA传输*/
        DMA_SetDestAddress(0, (uint32_t)gpHREF[h_counter++]);
        DMA_SetMajorLoopCounter(0, (OV7620_W/8)+1);
        EDMA_StartTransfer(&g_EDMA_Handle);
        return;
    }
    /* A16 场中断 */
     if ((1 << 16) == (GPIO_GetPinsInterruptFlags(GPIOA)&(1 << 16)) )
    {
        h_counter = 0;
        /* Clear external interrupt A16 flag. */
        GPIO_ClearPinsInterruptFlags(GPIOA, 1U << 16);
        /*关闭A16行中断 */
        PORT_SetPinInterruptConfig(PORTA, 16U, kPORT_InterruptOrDMADisabled);
        /*关闭A17场中断 */
        PORT_SetPinInterruptConfig(PORTA, 17U, kPORT_InterruptOrDMADisabled);
        /*隔场显示*/
        switch(status)
        {
            case TRANSFER_IN_PROCESS: 
              /*如果 按下S2键 绿灯亮 OLED 显示赛道 原始数据 最后 4行个填入0 */
              if( 0x00 == GPIO_ReadPinInput(GPIOE, 11U)) //第3个灯亮
              {
                SerialDispImage(OV7620_W, OV7620_H, gpHREF);
                //show_image();
              }
              status = NEXT_FRAME;
              break;
            case NEXT_FRAME: // waiting for next transfer 
              status =  TRANSFER_IN_PROCESS;
              break;
            default:
              break;
        }
        /*打开A17行中断 */
        PORT_SetPinInterruptConfig(PORTA, 17U, kPORT_InterruptFallingEdge );
        /*打开A16场中断 */
        PORT_SetPinInterruptConfig(PORTA, 16U, kPORT_InterruptFallingEdge);
        PORTA->ISFR = 0xFFFFFFFF;
        return;
    }
}

/*******************************************************************************
* Function Name  : SCCB_Init  内部函数
* Description    : 基本SCCB 协议的 O7725 初始化
* Input          : None  移植SCCB时保留 I2C_MAP-随便填写 
* Output         : None
* Return         : 0-成功 1-失败
*******************************************************************************/
static int SCCB_Init(uint32_t I2C_MAP)
{
    int r;
    uint32_t instance;
    instance = I2C_QuickInit();
    r = ov7725_probe(instance);
    if(r)
    {
        return 1;
    }
    r = ov7725_set_image_size(H_80_W_60);
    if(r)
    {
        printf("OV7725 set image error\r\n");
        return 1;
    }
    return 0;
}

/*******************************************************************************
* Function Name  : camera_init 
* Description    : i2c总线 camera 初始化 查找自身地址 配置数据DMA
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
 void camera_init(void)
{
    /*INIT OV7725*/
    if(SCCB_Init(I2C0_SCL_PB02_SDA_PB03))
    {
        PRINTF("no ov7725device found!\r\n");
        while(1);
    }
    PRINTF("OV7725 setup complete\r\n");    

    /*RAM 分配 */ 
    for(int i=0; i<OV7620_H+1; i++)
    {
        gpHREF[i] = (uint8_t*)&gCCD_RAM[i*(OV7620_W/8)];
    }
    //camera_i2c_oled_init();
    //ra_fill_picture(0xFF);
    
    /*OV7725管脚初始化*/
    camera_pin_init();
    /*OV7725 DMA传输初始化*/
    camera_dma_init();    
}

