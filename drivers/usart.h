#ifndef __USART_H
#define __USART_H


void UartConfig(void);
void PutChar(uint8_t ch);
uint8_t GetChar(void);
void PutString(uint8_t *str);
void UartPrintf(const char *format, ...);

void printuart(void);
#endif
