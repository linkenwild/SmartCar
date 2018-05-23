/******************************2017-2017, NJTECH, Edu.************************** 
FileName: camera_oled.c 
Author:  �ﶬ÷       Version :  1.0        Date: 2017.06.30
Description:   ������camera Э�鲻ͬ��������Ӳ��i2c  ʹ�����i2c, oled ��ַ��0x3C
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
#include "board.h"

#include "i2c.h"
#include "camera_oled.h"

#define I2C_OLED_SLAVE_ADDR_7BIT 0x3CU

/*��ʼ��oled ������*/
static uint8_t initial_oled_data[28]={0xAE,0x20,0x10,0xb0,0xc8,0x00,0x10,0x40,0x81,0xdf,0xa1,0xa6,
0xa8,0x3F,0xa4,0xd3,0x00,0xd5,0xf0,0xd9,0x22,0xda,0x12,0xdb,0x20,0x8d,0x14,0xaf};


/*******************************************************************************
* Function Name  : IIC_oled_Buffer_Write
* Description    : ��oled��дbuffer��������һ�����ݵ�д������
* Input          : data_addr Ҫд�����ݵĵ�ַ lenҪд�����ݵĳ��� mode Ϊ0ʱ CMD Ϊ1ʱ DATA
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
* Function Name  : camera_i2c_oled_init
* Description    : camera oled��i2c ��ʼ��
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void camera_i2c_oled_init(void)
{
    
  /*�ó�ʼ������ ��ʼ�� oled*/
    IIC_oled_Buffer_Write( initial_oled_data,28,0);     
}

/*******************************************************************************
* Function Name  : camera_fill_picture
* Description    : ��oled�Ļ������
* Input          : fill_Data Ҫ�������� 
* Output         : None
* Return         : None 
*******************************************************************************/
void camera_fill_picture(uint8_t fill_Data)
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

