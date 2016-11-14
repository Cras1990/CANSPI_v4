/*
 * PtCan_Tim.c
 *
 *  Created on: 14.11.2016
 *      Author: SamirAlexis
 */
#include "PtCan_Tim.h"
#include "PtCan_SdStorage.h"
#include "stm32f4xx_hal.h"
#include "led_button.h"


TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/** TIM2 init function
 **/
void MX_TIM2_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 84000;                    // Timer eingestellt bei 1ms
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM2 init function */
void MX_TIM3_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16800;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10000;                    // Timer eingestellt bei 2s
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim3);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {

		if (counter < 0xFFFFFFFF) {

			counter++;
		} else
			counter = 0;

	} else if (htim->Instance == TIM3 && program_start == 1) {

		__HAL_GPIO_EXTI_CLEAR_IT(KEY_BUTTON_PIN);
		HAL_NVIC_EnableIRQ(KEY_BUTTON_EXTI_IRQn);

		if (HAL_TIM_Base_Stop_IT(&htim3) != HAL_OK) { // Stop timer interrupt nach 2 Sekunden
			/* Counter Enable Error */
			Error_Handler();
		}

	}
}
