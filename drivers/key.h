#ifndef __KEY_H
#define __KEY_H

typedef enum 
{  
  KEY1 = 1,
  KEY2 = 2,
  KEYNULL = -1,
} Key_Def;


void Key_init(void);
Key_Def KEY_Scan(void);
bool Get_Key(Key_Def key);

void Jumper_Init(void);
uint32_t Get_Jumper_value(void);
#endif

