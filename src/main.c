/**
 ******************************************************************************
 * File Name          : main.c

 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */



/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "PtCan_Can.h"
#include "PtCan_SdStorage.h"
#include "led_button.h"
#include "PtCan_Cfg.h"
#include "PtCan_ErrHandling.h"
#include "PtCan_Tim.h"
#include "Std_Types.h"

/**********************************************************************************************************************
 *  LOCAL CONSTANT MACROS
 **********************************************************************************************************************/

/**********************************************************************************************************************
 *  LOCAL FUNCTION PROTOTYPES
 **********************************************************************************************************************/
static void main_InitMcu(void);

static void main_StartStorage(void);

static void main_StopStorage(void);

static void SystemClock_Config(void);

/**********************************************************************************************************************
 *  LOCAL DATA PROTOTYPES
 **********************************************************************************************************************/
// Globale Variable zur Signalisierung, dass das ganze Programm ab der Messung gestartet ist
// Sein Wert wird nie mehr geaendert
static volatile uint8_t program_start = STD_OFF;


/**********************************************************************************************************************
 *  LOCAL FUNCTIONS
 **********************************************************************************************************************/
int main(void) {

	main_InitMcu();

	while (1) {

		// Warte bis Knopf gedruckt wird
		while (get_button_state() == STD_ON) {

			// Wurde die Datenuebertragung richtig eingestellt und Messung noch nicht gestartet?
			if (PtCan_SdStorage_getStorageState() == STD_OFF) {
				// Starte Speicherung
				main_StartStorage();
			}
#if defined (CAN2_EIN)
			PtCan_Can2_Transmit();
#endif
			// Sende Rohdaten an SD-Karte, wenn Buffer voll sind
			PtCan_SdStorage_storeSD();
		}

		// wurde Knopf erneut gedruckt und Datenspeicherung laeuft?
		if (PtCan_SdStorage_getStorageState() != STD_OFF && get_button_state() == STD_OFF) {
			//Stope Speicherung
			main_StopStorage();
		}

	}
}

/**
 * @brief  System Clock Configuration.
 * @param  None
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	__PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
 * @brief  Pinout Configuration.
 * @param  None
 * @retval None
 */
void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__GPIOD_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;

}

/**
 * @brief  Initialization routine for all peripheral interfaces and internal HAL-Functions.
 * @param  None
 * @retval None
 */
void main_InitMcu() {

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

	PtCan_initMemory();

	/* Configure LED1, LED2, LED3 and LED4 */
	BSP_LED_Init(LED4);
	BSP_LED_Init(LED3);
	BSP_LED_Init(LED5);
	BSP_LED_Init(LED6);

	/*##-1- Configure the CAN peripheral #######################################*/
	// CAN Controller-Empfaenger
	MX_CAN1_Init();
#if defined (CAN2_EIN)
		// CAN Controller-Sender
		MX_CAN2_Init();
#endif
	MX_TIM2_Init();
	MX_TIM3_Init();

	/* Configure Key Button */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	reset_button_state();

}

/**
 * @brief  Routine for initialization of CAN-Communication and SPI-Communication with SD-Card.
 * @param  None
 * @retval None
 */
void main_StartStorage() {


//	if ( PtCan_SdStorage_SDMount(STD_ON) != E_OK) {
//		Error_Handler_fats();
//	}
	PtCan_SdStorage_SDMount(STD_ON);

	BSP_LED_On(LED5);
#if defined (CAN2_EIN)
	// Diese LED gibt an, dass CAN2 nicht dabei ist
	BSP_LED_On(LED4);
#endif

	// Setze Initialisierungsbit der Speicherung
	PtCan_SdStorage_setStorageState(STD_ON);

	if (program_start == STD_OFF) {

		PtCan_Can1ActivReceiveIT();
	} else {
		// Sonst erwecke den CAN-Controller
		PtCan_Can1WU();
	}

	PtCan_Tim_SetState(INST_TIM2, STD_ON);

	// Setze Initialisierungsbit ab Start der Speicherung.
	// Gibt an, dass Button das erste Mal zum Abspeichern gedruckt wurde.
	// Sein Wert wird nie mehr geaendert
	program_start = STD_ON;
}

/**
 * @brief  Routine for stopping CAN-Communication and SPI-Communication with SD-Card.
 * @param  None
 * @retval None
 */
void main_StopStorage() {

	/* Unmount SDCARD */
//	if( PtCan_SdStorage_SDMount(STD_OFF) != E_OK) {
//		Error_Handler_fats();
//	}

	PtCan_SdStorage_SDMount(STD_OFF);

	PtCan_Can1Sleep();

	PtCan_Tim_SetState(INST_TIM2, STD_OFF);

	BSP_LED_Off(LED5);
#if defined (CAN2_EIN)
	// Diese LED gibt an, dass CAN2 nicht dabei ist
	BSP_LED_Off(LED4);
#endif

	// Die CAN-Uebertragung kann wieder neu gestartet werden, nachdem dieses Bit zurueckgesetzt wurde
	PtCan_SdStorage_setStorageState(STD_OFF);
	// Reset Timer
	PtCan_Tim_ResetCounter();
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
