/*
 * PtCan_ErrHandling.c
 *
 *  Created on: 14.09.2016
 *      Author: SamirAlexis
 *
 *  -----------------------------------------------------------------------------------------------------------------
 *  FILE DESCRIPTION
 *  -----------------------------------------------------------------------------------------------------------------
 *     \file  PtCan_ErrHandling.c
 *        \brief  File to manage the timer handling object routines
 *
 *      \details  Dependencies: PtCan_ErrHandling.h
 *      												PtCan_SdStorage.h
 *      												led_button.h
 *      												tm_stm32_fatfs.h
 *
 *********************************************************************************************************************/

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/

#include "PtCan_ErrHandling.h"
#include "PtCan_SdStorage.h"
#include "led_button.h"
#include "tm_stm32_fatfs.h"


/**********************************************************************************************************************
 *  GLOBAL FUNCTIONS
 **********************************************************************************************************************/

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler_CANT(void) {
	f_mount(NULL, "SD:", 1);

	while (1) {
		BSP_LED_Toggle(LED6);
		HAL_Delay(100);
	}
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
		HAL_Delay(100);
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
		HAL_Delay(100);
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
		HAL_Delay(100);
	}
}
