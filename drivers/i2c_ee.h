#ifndef __I2C__EE_H
#define __I2C__EE_H

void i2c_ee_init(void);
bool i2c_ee_write_onepage(uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite);
bool i2c_ee_buffer_read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead);
bool eep_self_test(uint32_t begin, uint32_t end);
#endif
