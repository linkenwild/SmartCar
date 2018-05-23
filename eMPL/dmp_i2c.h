#ifndef __DMP_I2C_H
#define __DMP_I2C_H

void dmp_i2c_init(); 	
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);

#endif
