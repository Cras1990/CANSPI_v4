/*
 * led_button.c
 *
 *  Created on: 07.03.2016
 *      Author: revelo
 */

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "led_button.h"
#include "PtCan_Tim.h"
#include "PtCan_Cfg.h"
#include "Std_Types.h"

/**********************************************************************************************************************
 *  LOCAL CONSTANT MACROS
 **********************************************************************************************************************/

GPIO_TypeDef* GPIO_PORT[LEDn] = { LED4_GPIO_PORT, LED3_GPIO_PORT,
LED5_GPIO_PORT, LED6_GPIO_PORT };
const uint16_t GPIO_PIN[LEDn] = { LED4_PIN, LED3_PIN, LED5_PIN, LED6_PIN };

GPIO_TypeDef* BUTTON_PORT[BUTTONn] = { KEY_BUTTON_GPIO_PORT };
const uint16_t BUTTON_PIN[BUTTONn] = { KEY_BUTTON_PIN };
const uint8_t BUTTON_IRQn[BUTTONn] = { KEY_BUTTON_EXTI_IRQn };

/**********************************************************************************************************************
 *  LOCAL DATA PROTOTYPES
 **********************************************************************************************************************/
// STD_ON signalizes button has been pressed
// and some functionality is in active state
static volatile uint8_t button = STD_OFF;


/**********************************************************************************************************************
 *  LOCAL FUNCTIONS
 **********************************************************************************************************************/

/**
 * @brief  Configures LED GPIO.
 * @param  Led: Specifies the Led to be configured.
 *   This parameter can be one of following parameters:
 *     @arg LED4
 *     @arg LED3
 *     @arg LED5
 *     @arg LED6
 * @retval None
 */
void BSP_LED_Init(Led_TypeDef Led) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the GPIO_LED Clock */
	LEDx_GPIO_CLK_ENABLE(Led);

	/* Configure the GPIO_LED pin */
	GPIO_InitStruct.Pin = GPIO_PIN[Led];
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIO_PORT[Led], &GPIO_InitStruct);

	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
 * @brief  Toggles the selected LED.
 * @param  Led: Specifies the Led to be toggled.
 *   This parameter can be one of following parameters:
 *     @arg LED4
 *     @arg LED3
 *     @arg LED5
 *     @arg LED6
 * @retval None
 */
void BSP_LED_Toggle(Led_TypeDef Led) {
	HAL_GPIO_TogglePin(GPIO_PORT[Led], GPIO_PIN[Led]);
}

/**
 * @brief  Turns selected LED On.
 * @param  Led: Specifies the Led to be set on.
 *   This parameter can be one of following parameters:
 *     @arg LED4
 *     @arg LED3
 *     @arg LED5
 *     @arg LED6
 * @retval None
 */
void BSP_LED_On(Led_TypeDef Led) {
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_SET);
}

/**
 * @brief  Turns selected LED Off.
 * @param  Led: Specifies the Led to be set off.
 *   This parameter can be one of following parameters:
 *     @arg LED4
 *     @arg LED3
 *     @arg LED5
 *     @arg LED6
 * @retval None
 */
void BSP_LED_Off(Led_TypeDef Led) {
	HAL_GPIO_WritePin(GPIO_PORT[Led], GPIO_PIN[Led], GPIO_PIN_RESET);
}

/**
 * @brief  Configures Button GPIO and EXTI Line.
 * @param  Button: Specifies the Button to be configured.
 *   This parameter should be: BUTTON_KEY
 * @param  Mode: Specifies Button mode.
 *   This parameter can be one of following parameters:
 *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO
 *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
 *                            generation capability
 * @retval None
 */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef Mode) {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Enable the BUTTON Clock */
	BUTTONx_GPIO_CLK_ENABLE(Button);

	if (Mode == BUTTON_MODE_GPIO) {
		/* Configure Button pin as input */
		GPIO_InitStruct.Pin = BUTTON_PIN[Button];
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;

		HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);
	}

	if (Mode == BUTTON_MODE_EXTI) {
		/* Configure Button pin as input with External interrupt */
		GPIO_InitStruct.Pin = BUTTON_PIN[Button];
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
		HAL_GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStruct);

		/* Enable and set Button EXTI Interrupt to the lowest priority */
		HAL_NVIC_SetPriority((IRQn_Type) (BUTTON_IRQn[Button]), 14, 0);
		HAL_NVIC_EnableIRQ((IRQn_Type) (BUTTON_IRQn[Button]));
	}
}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == KEY_BUTTON_PIN) {

		// Wurde Knopf zur Datenspeicherung gedrueckt?
		if (get_button_state() == STD_OFF) {
			// Dann setze ensprechendes Signalisierungsbit
			set_button_state();
		} else {
			// Sonst, signalisiere, dass Knopf erneut gedrueckt wurde, um Messung zu stoppen
			reset_button_state();

		}

		// Startet Timer, welcher 2 Sekunde lang laeuft
		PtCan_Tim_SetState(INST_TIM3, STD_ON);

		// Deaktivierung des Interrupts fuer den Button 2 Sekunde lang
		HAL_NVIC_DisableIRQ(KEY_BUTTON_EXTI_IRQn);

	}

}

/**
 * @brief  Set new button state. Button has been pushed. The call of this
 * 				 function should signalize that some
 * 				 functionality has been activated
 * @param  None
 * @retval None
 */
void set_button_state(void) {

	button = STD_ON;

}

/**
 * @brief  Reset new button state. Button has been pushed. The call of this
 * 				 function should signalize that some
 * 				 functionality has been deactivated
 * @param  None
 * @retval None
 */
void reset_button_state(void) {

	button = STD_OFF;

}

/**
 * @brief  Get button state
 * @param  None
 * @retval None
 */
uint8_t get_button_state() {

	return button;

}

