#ifndef PTCAN_TIM_H_
#define PTCAN_TIM_H_


// Timer
static volatile uint32_t counter = 0;		// Zaehler fuer CAN Uebertragung

void MX_TIM2_Init(void);
void MX_TIM3_Init(void);

#endif /* PTCAN_TIM_H_ */
