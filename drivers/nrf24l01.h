
#ifndef _NRF24L01_H
#define _NRF24L01_H

void NRF24L01_Init(void);                                //NRF24l01��ʼ��
void RX_Mode(void);                                      //����Ϊ����ģʽ
void TX_Mode(void);                                      //����Ϊ����ģʽ
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen); //д������
uint8_t NRF24L01_Read_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen);  //��������		  
uint8_t NRF24L01_Read_Reg(uint8_t regaddr);		                 //���Ĵ���
uint8_t NRF24L01_Write_Reg(uint8_t regaddr, uint8_t data);              //д�Ĵ���
bool NRF24L01_Check(void);                                 //���NRF24L01�Ƿ���λ
bool NRF24L01_TxPacket(uint8_t *txbuf);                         //����һ����������
bool NRF24L01_RxPacket(uint8_t *rxbuf);                         //����һ����������

void rf1send(uint8_t * str);
 void rf1start(void);

#endif
