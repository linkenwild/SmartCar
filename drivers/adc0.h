#ifndef __adc0_H
#define __adc0_H

void ADC0_init(void);
void getadc0value(uint8_t *advalue);
void ADC0_poll_init(void);
void getadc0_poll_value(uint8_t *advalue);

#endif
