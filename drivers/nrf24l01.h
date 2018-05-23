
#ifndef _NRF24L01_H
#define _NRF24L01_H

void NRF24L01_Init(void);                                //NRF24l01初始化
void RX_Mode(void);                                      //配置为接收模式
void TX_Mode(void);                                      //配置为发送模式
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen); //写数据区
uint8_t NRF24L01_Read_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);  //读数据区		  
uint8_t NRF24L01_Read_Reg(uint8_t regaddr);		                 //读寄存器
uint8_t NRF24L01_Write_Reg(uint8_t regaddr, uint8_t data);              //写寄存器
bool NRF24L01_Check(void);                                 //检查NRF24L01是否在位
bool NRF24L01_TxPacket(uint8_t *txbuf);                         //发送一个包的数据
bool NRF24L01_RxPacket(uint8_t *rxbuf);                         //接收一个包的数据

void rf1send(uint8_t * str);
 void rf1start(void);

#endif
