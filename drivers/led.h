#ifndef __LED_H
#define __LED_H

typedef enum 
{
  LED1 = 1,
  LED2 = 2,
  LED3 = 3,
  LED4 = 4,
  LEDALL = 0,
} Led_Def;


void LED_Init(void);
void LEDOn(Led_Def Led);
void LEDOff(Led_Def Led);
void LEDTog(Led_Def Led);

void BEEP_Init(void);
void BEEPOn(void);
void BEEPOff(void);
void BEEPTog(void);

void UltraSonic_Init(void);
void UltraSonicOn(void);
void UltraSonicOff(void);
void UltraSonicTog(void);

#endif 

