#ifndef PTCAN_TIM_H_
#define PTCAN_TIM_H_

#include "stm32f4xx_hal.h"

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void PtCan_Tim_SetState(uint8_t instance_timer, uint8_t on_off);
uint16_t PtCan_Tim_GetCounter(void);
void PtCan_Tim_SetCounter(uint16_t value);

#endif /* PTCAN_TIM_H_ */
