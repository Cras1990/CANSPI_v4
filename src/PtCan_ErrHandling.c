/*
 * PtCan_ErrHandling.c
 *
 *  Created on: 14.09.2016
 *      Author: SamirAlexis
 */

#include "PtCan_ErrHandling.h"
#include "PtCan_SdStorage.h"
#include "led_button.h"


/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler_CANT(void) {
//	f_mount(NULL, "SD:", 1);
//
//	while (1) {
//		BSP_LED_Toggle(LED6);
//		HAL_Delay(1000);
//	}
	messung = CAN2_AUS;
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler_CANR(void) {
	f_mount(NULL, "SD:", 1);

	while (1) {
		BSP_LED_Toggle(LED6);
		HAL_Delay(1000);
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler_fats(void) {
	f_mount(NULL, "SD:", 1);

	while (1) {
		BSP_LED_Toggle(LED6);
		HAL_Delay(1000);
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	f_mount(NULL, "SD:", 1);
	while (1) {
		BSP_LED_Toggle(LED6);
		HAL_Delay(1000);
	}
}
