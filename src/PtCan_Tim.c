/*
 * PtCan_Tim.c
 *
 *  Created on: 14.11.2016
 *      Author: SamirAlexis
 *
 *  -----------------------------------------------------------------------------------------------------------------
 *  FILE DESCRIPTION
 *  -----------------------------------------------------------------------------------------------------------------
 *     \file  PtCan_Tim.c
 *        \brief  File to manage the timer handling object routines
 *
 *      \details  Dependencies: PtCan_Tim.h
 *      												PtCan_ErrHandling.h
 *      												stm32f4xx_hal.h
 *      												PtCan_Cfg.h
 *      												Std_Types.h
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/

#include "PtCan_Tim.h"
#include "PtCan_Cfg.h"
#include "stm32f4xx_hal.h"
#include "led_button.h"
#include "PtCan_ErrHandling.h"
#include "Std_Types.h"

/**********************************************************************************************************************
 *  LOCAL DATA PROTOTYPES
 **********************************************************************************************************************/
// timer objects
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

// Counter
static volatile uint32_t counter = 0;		// Zaehler fuer CAN Uebertragung


/**********************************************************************************************************************
 *  GLOBAL FUNCTIONS
 **********************************************************************************************************************/

/**
 * @brief  Initialization functions for Timer 2 object.
 * @param  None
 * @retval None
 */
void MX_TIM2_Init(void) {

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 84000;                    // Timer eingestellt bei 1ms
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

}

/**
 * @brief  Initialization functions for Timer 3 object.
 * @param  None
 * @retval None
 */
void MX_TIM3_Init(void) {

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16800;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10000;                    // Timer eingestellt bei 2s
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim3);

}

/**
 * @brief  Callback handling. TIM2 counts until 1ms to increment variable before being restarted.
 * 				 TIM3 counts until 2s to activate external interrupt for button usage, before beginning counting process.
 * @param  htim: timer handling object
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM2) {

		if (counter < 0xFFFFFFFF) {

			counter++;
		} else
			counter = 0;

	} else if (htim->Instance == TIM3) {

		__HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
		HAL_NVIC_EnableIRQ(KEY_BUTTON_EXTI_IRQn);

//		if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK) { // Stop timer interrupt nach 2 Sekunden
//			/* Counter Enable Error */
//			Error_Handler();
//		}
		PtCan_Tim_SetState(INST_TIM3, STD_OFF);

	}
}

/**
 * @brief  Initialize or stop timer interrupt handling process
 * @param  instance_timer: timer handling macro
 * 												- INST_TIM2
 * 												- INST_TIM3
 * 				 on_off: 	- STD_ON: activate timer interrupt handling process
 * 				 					- STD_OFF: deactivate timer interrupt handling process
 * @retval None
 */
void PtCan_Tim_SetState(uint8_t instance_timer, uint8_t on_off) {
	if (on_off == STD_ON) {
		if (instance_timer == INST_TIM2) {
			if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) { // (Erneute) Initialisierung Timer2
				/* Counter Enable Error */
				Error_Handler();
			}
		} else {
			if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) { // (Erneute) Initialisierung Timer3
				/* Counter Enable Error */
				Error_Handler();
			}
		}
	} else {
		// Stop timer interrupt
		if (instance_timer == INST_TIM2) {
					if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK) {
						/* Counter Enable Error */
						Error_Handler();
					}
		} else {
					if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK) {
						/* Counter Enable Error */
						Error_Handler();
					}
		}
	}
}

/**
 * @brief  Get actual counter number incresed thought timer2 interrupt handling routine
 * @retval counter
 */
uint16_t PtCan_Tim_GetCounter()
{
	return counter;
}

/**
 * @brief  Reset  counter number incresed thought timer2 interrupt handling routine
 * @param  None
 * @retval None
 */
void PtCan_Tim_ResetCounter()
{
	counter = 0;
}
