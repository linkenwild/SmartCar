#ifndef PIT_H
#define PIT_H 
#include "fsl_pit.h"

void PitConfig(pit_chnl_t pit_chanel, uint64_t us_count);

#endif
