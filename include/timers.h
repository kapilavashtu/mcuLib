#ifndef TIMERS_H
#define TIMERS_H

#include "defines.h"

#define IWDG_PRESCALER                      (uint32_t) 0x07
#define IWDG_RLRVAL                         (uint32_t) 0x4FF

void TIMER6_Init(void);


#endif