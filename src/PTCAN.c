/*
 * PTCAN.c
 *
 *  Created on: 07.03.2016
 *      Author: revelo
 */

#include "stm32fxxx_hal.h"
#include "tm_stm32_fatfs.h"
#include "led_button.h"
#include "PTCAN.h"
#include "PtCan_ErrHandling.h"
#include "PtCan_SdStorage.h"
#include "PtCan_Can.h"
#include "PtCan_Tim.h"
#include "PtCan_Cfg.h"


volatile uint8_t cantrans_sdstor_init = 0; // cantrans_sdstor_init = 0 -> Datafile closed, storage has still not begun or has been finished. Otherwise the storage is active
volatile uint8_t program_start = 0;	// Globale Variable zur Signalisierung, dass das ganze Programm ab der Messung gestartet ist

//// Timer
extern volatile uint32_t counter;		// Zaehler fuer CAN Uebertragung
extern volatile uint8_t messung;

void SystemClock_Config(void);

/** System Clock Configuration
 *
 **/
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



/** Pinout Configuration
 */
void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__GPIOD_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;

}



void init_config_MCU() {

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
	MX_CAN1_Init();			// CAN Controller-Empfaenger
	if (messung == CAN2_EIN) {	// Nur CAN2 Einschalten, wenn gewuenscht!
		MX_CAN2_Init();			// CAN Controller-Sender
	}
	MX_TIM2_Init();
	MX_TIM3_Init();

	/* Configure Key Button */
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

	reset_button_state();

}

void start_storage() {


	if ( PtCan_SdStorage_SetStateSD(STD_ON) != FR_OK) {
		Error_Handler_fats();
	}

	BSP_LED_On(LED5);
	BSP_LED_On(LED4);	// Diese LED gibt an, ob CAN2 verwendet wird oder nicht

	set_storage_state(STD_ON);			// Setze Initialisierungsbit der Speicherung

	if (program_start == 0) {

		PtCan_Can1ActivReceiveIT();
	} else {								// Sonst erwecke den CAN-Controller

		PtCan_Can1WU();
	}

	PtCan_Tim_SetState(INST_TIM2, STD_ON);

	program_start = 1; // Setze Initialisierungsbit ab der Speicherung und wird nie mehr geaendert
}

void stop_storage() {

	/* Unmount SDCARD */
	PtCan_SdStorage_SetStateSD(STD_OFF);

	PtCan_Can1Sleep();

	PtCan_Tim_SetState(INST_TIM2, STD_OFF);

	BSP_LED_Off(LED5);
	BSP_LED_Off(LED4); // Diese LED wird ausgemacht, um anzugeben, dass CAN2 nicht dabei ist

	set_storage_state(STD_OFF); // Die CAN-Uebertragung kann wieder neu gestartet werden, nachdem dieses Bit zurueckgesetzt wurde
	counter = 0;
}

void store_incomming_data() { // von hier aus werden die Daten an die SD gesendet, von der isr werden die Daten in die Puffer gespeichert

	PtCan_SdStorage_storeSD();

}
