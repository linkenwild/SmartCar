#ifndef __CAMERA_I2C__OLED_H
#define __CAMERA_I2C__OLED_H


void camera_i2c_oled_init(void);
void camera_fill_picture(uint8_t fill_Data);

void OledImge_Extend(uint8_t *buf,uint8_t *data, int xSize, int ySize) ;
void oled_display_image (uint8_t* data, uint16_t datalenght, uint16_t width, uint16_t heigth);
bool oled_data120160_convert (uint8_t* buf, uint8_t* dataorigin, uint8_t size);
bool oled_data120160_data12864_convert (uint8_t* buf, uint8_t* dataorigin);
#endif
