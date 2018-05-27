#ifndef __ADC_PDB_H
#define __ADC_PDB_H

void PDB_init();
void AIN0_init(void);
void AIN1_init(void);
void get_pdb_adc0_value(uint32_t* value);
void get_pdb_adc1_value(uint32_t* value);

#endif

