/******************************2017-2017, NJTECH, Edu.************************** 
FileName: camera_oled.c 
Author:  孙冬梅       Version :  1.0        Date: 2017.06.30
Description:   由于与camera 协议不同，不能用硬件i2c  使用软件i2c, oled 地址：0x3C
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
#include "board.h"

#include "i2c.h"
#include "camera_oled.h"

#define I2C_OLED_SLAVE_ADDR_7BIT 0x3CU

/*初始化oled 的数组*/
static uint8_t initial_oled_data[28]={0xAE,0x20,0x10,0xb0,0xc8,0x00,0x10,0x40,0x81,0xdf,0xa1,0xa6,
0xa8,0x3F,0xa4,0xd3,0x00,0xd5,0xf0,0xd9,0x22,0xda,0x12,0xdb,0x20,0x8d,0x14,0xaf};


/*******************************************************************************
* Function Name  : IIC_oled_Buffer_Write
* Description    : 对oled的写buffer操作，是一段数据的写操作。
* Input          : data_addr 要写入数据的地址 len要写入数据的长度 mode 为0时 CMD 为1时 DATA
* Output         : None
* Return         : None 
*******************************************************************************/
static void IIC_oled_Buffer_Write( uint8_t* data_addr, uint16_t len, uint8_t mode)
{
  uint8_t WriteAddr = 0;
  if(mode==0)
  {
    WriteAddr = 0x00;  
  }
  else if(mode ==1)
  {
    WriteAddr = 0x40;  
  }   
  
  I2C_BurstWrite(1, I2C_OLED_SLAVE_ADDR_7BIT, WriteAddr, 1, data_addr , len);
}


/*******************************************************************************
* Function Name  : OLED_Set_Pos
* Description    : 设置当前显示位置
* Input          : x-段位置  y-页位置
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
* Function Name  : camera_i2c_oled_init
* Description    : camera oled　i2c 初始化
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void camera_i2c_oled_init(void)
{
    
  /*用初始化数组 初始化 oled*/
    IIC_oled_Buffer_Write( initial_oled_data,28,0);     
}

/*******************************************************************************
* Function Name  : camera_fill_picture
* Description    : 对oled的画面填充
* Input          : fill_Data 要填充的数据 
* Output         : None
* Return         : None 
*******************************************************************************/
void camera_fill_picture(uint8_t fill_Data)
{
  uint8_t cmd_buf[3] = {0xb0,0x00,0x10};
  uint8_t data_buf[128*8] = {0x00};
  
  /*将要填充的数据 填满数组*/
  for(int i=0;i<128*8;i++)
  {
    data_buf[i] = fill_Data;
  }
  
  IIC_oled_Buffer_Write(cmd_buf, 3, 0);
  IIC_oled_Buffer_Write(data_buf, 128*8, 1);   
}

/*******************************************************************************
* Function Name  : OledImge_Extend
* Description    : 字节扩张，1字节8位，以位显示 
* Input          : data-转换前的数据  xSize-一行多少个数据 ySize-共多少行8的倍数
* Output         : buf-转换后的数据
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
* Description    : 图像先转换成 120*160 
* sizd = 0:   60*80    -->  160*120  -->  120*60 行上每隔3点去年1点， 行数缩一半
* sizd = 1:   120*160  -->  120*60 行上每隔3点去年1点， 行数缩一半
* sizd = 2:   180*240  -->  120*60 行上每隔2点去年1点，行数缩为1/3
* Input          : dataorigin-原来数据 
* Output         : buf-转换后数据
* Return         : None
*******************************************************************************/
bool oled_data120160_convert (uint8_t* buf, uint8_t* dataorigin, uint8_t size)
{
  uint32_t i,j;
  bool result = false;
  uint8_t data120160[120][160];
  uint8_t data8060[60][80];

          if(size == 0)    //图像60*80扩张为120*160
          {       
              for(i=0; i<120; i++)
              {
                    for(j=0; j<160; j++) {  data120160[i][j] = *(dataorigin+(i/2)*80+(j/2));}
              }
              result = true;
          }  
          else if(size == 1)    //图像120*160
          {                    
              for(i=0; i<120; i++)
              {
                    for(j=0; j<160; j++){   data120160[i][j] = *(dataorigin+i*160+j);   }
              }  
              result = true;
          } 
          else if(size == 2)    //图像180*240先压缩为60*80再扩张为120*160
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
* Description    : 图像 120*160 换成 128*64 最后4行填0
* Input          : dataorigin-原来数据 
* Output         : buf-转换后数据
* Return         : None
*******************************************************************************/
bool oled_data120160_data12864_convert (uint8_t* buf, uint8_t* dataorigin)
{
  uint32_t i,j,m,n;
  uint8_t data[64][128] = {0U}; //最大图像

      m = 0;
      n = 0;
      
      for(i=0; i<120; i++)
      {
        if(0 == i%2) //只显示偶数行
        {
          for(j=0; j<160; j++) //每行中 每5个点去掉1个点 160-->128
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
      for(i=60; i<64; i++) //最后4行 为 白
      {
            for(j=0; j<128; j++) {  *buf = 0xFF ; buf++;};
      }
      
    return true;
}

/*******************************************************************************
* Function Name  : oled_display_image
* Description    : 显示路径一帧图像  datalenght-数据点数 width-一行多少点 height-共多少行
*   例如: 80*64 图像   datalenght-80*64 width-80 height-64        
*         128*64 图像   datalenght-128*64  width-128 height-64  
          height-64   一定为8的倍数，不能超过64
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
    seg = i % width;                      //段偏移
    page = i/(width*8);                 //页偏移
    bit_pos = 8- (i / width) %(8*page);        //位偏移
       
    if(0x00 == *data)
      data_buf[col+seg] &= ~(1<<(8-bit_pos));//置0
    else if(0xff == *data) //白色
      data_buf[col+seg] |= 1<<(8-bit_pos);//置1
    
    if((heigth*(width/8)-1) == (i%(heigth*(width/8))))//传完 heigth*(width/8) 个点
    {
      col = col+128;
    }
    data++;
  }
   IIC_oled_Buffer_Write(data_buf, 1024, 1);
}

