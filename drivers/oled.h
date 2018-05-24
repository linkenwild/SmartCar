#ifndef __I2C__OLED_H
#define __I2C__OLED_H


void i2c_oled_init(void);
void test_i2c_oled(void);
void fill_picture(uint8_t fill_Data);

void OLED_Clear(uint8_t Fill);           								//OLED����/����
void OLED_ShowChar(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uSize,uint8_t uChar);  //��ָ��Page_sta,Column_sta��Size�����ַ�Char
void OLED_ShowString(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uSize,uint8_t *p);	//��ָ��Page_sta,Column_sta�������СΪSize���ַ���*p
void OLED_ShowNum(uint8_t uPage,uint8_t uColumn_sta,uint8_t uSize,uint32_t uNum);		//��ָ��Page_sta,Column_sta�������СΪSize������uNum,��Χ:(0~4294967295)
void OLED_Fill(uint8_t uPage_sta,uint8_t uColumn_sta,uint8_t uColumn_end,uint8_t Fill); //��Page_sta,��uColumn_sta��uColumn_end��������

void oled_display_ccd_image (uint8_t* data, float threshold, uint16_t datalenght,  uint16_t startline,  uint16_t heigth);
void oled_display_ccd_line (uint8_t* data0, float threshold0, uint8_t* data1, float threshold1);

void OledImge_Extend(uint8_t *buf,uint8_t *data, int xSize, int ySize) ;
void oled_display_image (uint8_t* data, uint16_t datalenght, uint16_t width, uint16_t heigth);
bool oled_data120160_convert (uint8_t* buf, uint8_t* dataorigin, uint8_t size);
bool oled_data120160_data12864_convert (uint8_t* buf, uint8_t* dataorigin);
#endif
