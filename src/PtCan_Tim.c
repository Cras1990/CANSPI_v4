/*
 * PtCan_Tim.c
 *
 *  Created on: 14.11.2016
 *      Author: SamirAlexis
 */
#include "PtCan_Tim.h"
#include "PtCan_Cfg.h"
#include "stm32f4xx_hal.h"
#include "led_button.h"
#include "PtCan_ErrHandling.h"
#include "Std_Types.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

// Timer
static volatile uint32_t counter = 0;		// Zaehler fuer CAN Uebertragung

/** TIM2 init function
 **/
void MX_TIM2_Init(void) {

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 84000;                    // Timer eingestellt bei 1ms
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

}

/* TIM2 init function */
void MX_TIM3_Init(void) {

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16800;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10000;                    // Timer eingestellt bei 2s
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim3);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {

		if (counter < 0xFFFFFFFF) {

			counter++;
		} else
			counter = 0;

//	} else if (htim->Instance == TIM3 && program_start == 1) {
	} else if (htim->Instance == TIM3) {

		__HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
		HAL_NVIC_EnableIRQ(KEY_BUTTON_EXTI_IRQn);

		if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK) { // Stop timer interrupt nach 2 Sekunden
			/* Counter Enable Error */
			Error_Handler();
		}

	}
}

void PtCan_Tim_SetState(uint8_t instance_timer, uint8_t on_off) {
	if (on_off == STD_ON) {
		if (instance_timer == INST_TIM2) {
			if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) { // Initialisierung Timer3 zur Entprellung
				/* Counter Enable Error */
				Error_Handler();
			}
		} else {
			if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) { // Initialisierung Timer3 zur Entprellung
				/* Counter Enable Error */
				Error_Handler();
			}
		}
	} else {
		if (instance_timer == INST_TIM2) {
					if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK) { // Stop timer interrupt nach 2 Sekunden
						/* Counter Enable Error */
						Error_Handler();
					}
		} else {
					if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK) { // Stop timer interrupt nach 2 Sekunden
						/* Counter Enable Error */
						Error_Handler();
					}
		}
	}
}

uint16_t PtCan_Tim_GetCounter()
{
	return counter;
}

void PtCan_Tim_SetCounter(uint16_t value)
{
	counter = value;
}
