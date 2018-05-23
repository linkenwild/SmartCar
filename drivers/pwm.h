#ifndef __PWM_H
#define __PWM_H

typedef enum 
{
  PWM1 = 0,
  PWM2 = 1,
  PWM3 = 2,
  PWM4 = 3,
  STEERPWM = 4,
} PWM_Def;


void PWM_Init(PWM_Def chanel);
void PWMSet(PWM_Def chanel, uint32_t updatedDutycycle);
void SteerPWMSet( uint32_t updatedDutycycle);
#endif
