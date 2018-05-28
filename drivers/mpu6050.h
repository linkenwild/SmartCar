#ifndef __I2C__MPU6050_H
#define __I2C__MPU6050_H

void mpu6050_init(void);
int mpu6050_read_accel(int16_t* adata);
int mpu6050_read_gyro(int16_t *gdata);

void mpu6050_getdata(int16_t* a_data, int16_t* g_data, int16_t* temperature);

uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);

#endif
