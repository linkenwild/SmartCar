#ifndef __I2C__OLED_H
#define __I2C__OLED_H


void i2c_oled_init(void);
void test_i2c_oled(void);
void fill_picture(uint8_t fill_Data);

void OLED_Clear(uint8_t Fill);           								//OLED清屏/清屏
void OLED_ShowChar(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uSize,uint8_t uChar);  //在指定Page_sta,Column_sta和Size输入字符Char
void OLED_ShowString(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uSize,uint8_t *p);	//在指定Page_sta,Column_sta处输入大小为Size的字符串*p
void OLED_ShowNum(uint8_t uPage,uint8_t uColumn_sta,uint8_t uSize,uint32_t uNum);		//在指定Page_sta,Column_sta处输入大小为Size的数字uNum,范围:(0~4294967295)
void OLED_Fill(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uColumn_end,uint8_t Fill); //在Page_sta,从uColumn_sta到uColumn_end填充或清屏

void oled_display_ccd_image (uint8_t* data, float threshold, uint16_t datalenght,  uint16_t startline,  uint16_t heigth);
void oled_display_ccd_line (uint8_t* data0, float threshold0, uint8_t* data1, float threshold1);
#endif
