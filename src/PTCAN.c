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

// Ein- oder Ausblenden von CAN2

#define CAN2_AUS     0x00
#define CAN2_EIN     0x01

// Zur Speicherung benutzte Defines
#define TORQUE_1        0xA8
#define TORQUE_3        0xAA
#define GESCHWINDIGKEIT_RAD  0xCE
#define STAT_KOMBI      0x1B4

#define TORQUE_1_MASK                   0x01
#define TORQUE_3_MASK                   0x02
#define GESCHWINDIGKEIT_RAD_MASK        0x04
#define STAT_KOMBI_MASK                 0x08

// Variablen zum Ansteuern der Datenspeicherung und Senden der CAN-Signale zur SD-Karte

volatile uint8_t messung = CAN2_EIN;
uint8_t can_deinit = 0;

volatile uint16_t vehicle_speed; // velocity of the car. It can be negative
volatile int16_t trq_eng_in1;  		//      torque actual value can be negative
volatile uint16_t trq_eng_in3;  			//      rpm_engine
volatile int16_t left_wheel_speed;     // velocity of left wheel can be negative

uint32_t wbytes;                // Zaehlvariable bezueglich fwrite

volatile uint8_t cantrans_sdstor_init = 0; // cantrans_sdstor_init = 0 -> Datafile closed, storage has still not begun or has been finished. Otherwise the storage is active
volatile uint8_t save_files = 0; // Gibt an, wann ein Datenpaket an die SD-Karte bezueglich eines Messsignals geschickt werden soll
// 1.Signal
volatile uint16_t co_arr_save_TR1 = 0;
volatile uint8_t co_arr_full_TR1 = 0;
// 2.Signal
volatile uint16_t co_arr_save_TR3 = 0;
volatile uint8_t co_arr_full_TR3 = 2;
// 3.Signal
volatile uint16_t co_arr_save_GR_L = 0;
volatile uint8_t co_arr_full_GR_L = 4;
// 4.Signal
volatile uint16_t co_arr_save_STKO = 0;
volatile uint8_t co_arr_full_STKO = 6;

volatile uint8_t signal_arrvoll_vAuto = 0;
volatile uint8_t signal_arrvoll_v_lwheel = 0;
volatile uint8_t signal_arrvoll_torque1 = 0;
volatile uint8_t signal_arrvoll_torque3 = 0;

// Buffer-Variablen
char buffer[100];
uint8_t arr2SD[8][512];

// Fats-Variablen
volatile uint32_t FileSize = 0;
FATFS FS;
FIL fil_gesch_wheel;
FIL fil_torque1;
FIL fil_torque3;
FIL fil_gesch_car;
FRESULT fres;
TM_FATFS_Size_t CardSize;

// Timer
static volatile unsigned int counter = 0;		// Zaehler fuer CAN Uebertragung

static uint8_t program_start = 0;// Globale Variable zur Signalisierung, dass das ganze Programm ab der Messung gestartet ist

// Einstellungsvariablen fuer Peripherie-Controller
static CanTxMsgTypeDef TxMessage;
static CanRxMsgTypeDef RxMessage;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void Error_Handler_CANT(void);
void Error_Handler_CANR(void);
void Error_Handler_fats(void);
void Error_Handler(void);
void set_storage_state(uint8_t neu);

extern void reset_button_state(void);
extern void set_button_state(void);

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

/** Pinout Configuration
 */
void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__GPIOD_CLK_ENABLE()
	;
	__GPIOB_CLK_ENABLE()
	;

}

/* CAN1 init function
 * @brief  Funktion, zur Einstellung der CAN-Schnittstelle zum Empfangen von CAN-Signalen.
 * 		  Die Baudrate wird mit den unten angegebenen Groessen konfiguriert
 * 		  und betraegt dabei 500 Kbps.
 *
 * @param  None
 *
 * @retval None
 */
void MX_CAN1_Init(void) {
	CAN_FilterConfTypeDef sFilterConfig;

	/*##-1- Configure the CAN peripheral #######################################*/
	hcan1.Instance = CAN1;
	hcan1.pRxMsg = &RxMessage;
	hcan1.Init.Prescaler = 12;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_3TQ;
	hcan1.Init.BS1 = CAN_BS1_4TQ;
	hcan1.Init.BS2 = CAN_BS2_2TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;

	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-3- Configure the CAN Filter ###########################################*/
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
	sFilterConfig.FilterIdHigh = TORQUE_1 << 5;
	sFilterConfig.FilterIdLow = TORQUE_3 << 5;
//    sFilterConfig.FilterIdHigh = 0xFF << 5;
//	sFilterConfig.FilterIdLow = 0xFF << 5;
	sFilterConfig.FilterMaskIdHigh = GESCHWINDIGKEIT_RAD << 5;
//	sFilterConfig.FilterMaskIdHigh = 0xFF << 5;
	sFilterConfig.FilterMaskIdLow = STAT_KOMBI << 5;
//  sFilterConfig.FilterMaskIdLow = 0xFF << 5;
	sFilterConfig.FilterFIFOAssignment = 0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 14;

	if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
		/* Filter configuration Error */
		Error_Handler();
	}
}

/* CAN2 init function
 * @brief  Funktion, zur Einstellung der CAN-Schnittstelle zum Senden von CAN-Signalen.
 * 		  Diese Methode wird beim Auslesen von CAN-Signalen nicht verwenden.
 * 		  Die Baudrate wird mit den unten angegebenen Groessen konfiguriert
 * 		  und betraegt dabei 500 Kbps.
 *
 * @param  None
 *
 * @retval None
 */
void MX_CAN2_Init(void) {

	hcan2.pTxMsg = &TxMessage;
	hcan2.Instance = CAN2;
	hcan2.Init.Prescaler = 12;
	hcan2.Init.Mode = CAN_MODE_NORMAL;
	hcan2.Init.SJW = CAN_SJW_3TQ;
	hcan2.Init.BS1 = CAN_BS1_4TQ;
	hcan2.Init.BS2 = CAN_BS2_2TQ;
	hcan2.Init.TTCM = DISABLE;
	hcan2.Init.ABOM = DISABLE;
	hcan2.Init.AWUM = DISABLE;
	hcan2.Init.NART = DISABLE;
	hcan2.Init.RFLM = DISABLE;
	hcan2.Init.TXFP = DISABLE;

	if (HAL_CAN_Init(&hcan2) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}

	/*##-3- Configure Transmission process #####################################*/
//	hcan2.pTxMsg->StdId = STAT_KOMBI;
	hcan2.pTxMsg->StdId = TORQUE_1;
	hcan2.pTxMsg->ExtId = 0x01;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->DLC = 8;

	/* Set the data to be transmitted */
	hcan2.pTxMsg->Data[0] = 72;
	hcan2.pTxMsg->Data[1] = 13;
	hcan2.pTxMsg->Data[2] = 53;
	hcan2.pTxMsg->Data[3] = 61;
	hcan2.pTxMsg->Data[4] = 7;
	hcan2.pTxMsg->Data[5] = 82;
	hcan2.pTxMsg->Data[6] = 99;
	hcan2.pTxMsg->Data[7] = 100;

}

/**
 * @brief  Transmission complete callback in non blocking mode.
 * 		   In dieser Methode ist die Datenspeicherung in ein Buffer programmiert.
 * @param  CanHandle: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @retval None
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle) {

	uint8_t help_array = 0;

//	/* Receive. This function must be called after each data reception process */
//	if (HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0) != HAL_OK) {
//		/* Reception Error */
//		Error_Handler_CANR();
//	}

	switch (CanHandle->pRxMsg->StdId) {
	case (TORQUE_1):     // Drehmoment

		// Drehmoment abspeichern
		if (CanHandle->pRxMsg->Data[2] & 0x80)       // Falls Zahl negative
			trq_eng_in1 = 0xF0 << 8;
		else
			trq_eng_in1 = 0;
		trq_eng_in1 += ((CanHandle->pRxMsg->Data[2]) << 4)
				+ ((CanHandle->pRxMsg->Data[1]) >> 4);
		trq_eng_in1 = trq_eng_in1 >> 1;

		help_array = sprintf(buffer, "%u;%d\n", counter, trq_eng_in1); // Fuelle Array zum ueberpruefen, ob 512 Bytes schon voll sind

		if (512 <= (co_arr_save_TR1 + help_array)) { // Falls zu fuellendes Array keinen Platz mehr hat, dann...
			uint16_t anfang_2fuell = co_arr_save_TR1;
			signal_arrvoll_torque1 = 1;     // Array ist voll

			for (uint8_t i = 0; i < help_array; i++) {

				if (i < 512 - anfang_2fuell) { // Fuelle Array bis 512...
					arr2SD[co_arr_full_TR1][co_arr_save_TR1++] = buffer[i];
				} else {         // den Rest im naechsten Array
					if (co_arr_full_TR1 == 0) { // Falls 1. array voll
						arr2SD[co_arr_full_TR1 + 1][co_arr_save_TR1++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_TR1+1, da ich nicht immer wieder co_arr_full_TR1 erhoehen kann
					} else
						// Falls 2. array voll
						arr2SD[co_arr_full_TR1 - 1][co_arr_save_TR1++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_TR1+1, da ich nicht immer wieder co_arr_full_TR1 erhoehen kann
				}
				if (co_arr_save_TR1 == 512) // Ist beliebiger Array voll
					co_arr_save_TR1 = 0;    //initialisiere erneut Arrayfuellung
			}

		} else { // Falls zu fuellendes Array noch Platz hat, dann...

			for (uint8_t i = 0; i < help_array; i++)
				arr2SD[co_arr_full_TR1][co_arr_save_TR1++] = buffer[i];

		}

		if (signal_arrvoll_torque1 && ((save_files & TORQUE_1_MASK) == 0)) { // Falls buffer voll und Datenspeicherung gerade nicht stattfindet...
			save_files = save_files | TORQUE_1_MASK; // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			if (co_arr_full_TR1 == 0) // nachdem am Anfang das 1. array gefuellt wurde
				co_arr_full_TR1++;
			else
				// nachdem das 2. array gefuellt wurde, wird das erste wieder gefuellt
				co_arr_full_TR1--;
		}
		if (messung == CAN2_EIN)
			hcan2.pTxMsg->StdId = TORQUE_3;

		break;

	case (TORQUE_3):     // Speed of the shaft
		trq_eng_in3 = (((CanHandle->pRxMsg->Data[5]) << 8)
				| CanHandle->pRxMsg->Data[4]) >> 2;
		help_array = sprintf(buffer, "%u;%hu\n", counter, trq_eng_in3); // Fuelle Array zum ueberpruefen, ob 512 Bytes schon voll sind

		if (512 <= (co_arr_save_TR3 + help_array)) { // Falls zu fuellendes Array keinen Platz mehr hat, dann...
			uint16_t anfang_2fuell = co_arr_save_TR3;
			signal_arrvoll_torque3 = 1;     // Array ist voll

			for (uint8_t i = 0; i < help_array; i++) {

				if (i < 512 - anfang_2fuell) { // Fuelle Array bis 512...
					arr2SD[co_arr_full_TR3][co_arr_save_TR3++] = buffer[i];
				} else {         // den Rest im naechsten Array
					if (co_arr_full_TR3 == 2) { // Falls 1. array voll
						arr2SD[co_arr_full_TR3 + 1][co_arr_save_TR3++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_TR3+1, da ich nicht immer wieder co_arr_full_TR3 erhoehen kann
					} else
						// Falls 2. array voll
						arr2SD[co_arr_full_TR3 - 1][co_arr_save_TR3++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_TR3+1, da ich nicht immer wieder co_arr_full_TR3 erhoehen kann
				}
				if (co_arr_save_TR3 == 512) // Ist beliebiger Array voll
					co_arr_save_TR3 = 0;    //initialisiere erneut Arrayfuellung
			}

		} else { // Falls zu fuellendes Array noch Platz hat, dann...

			for (uint8_t i = 0; i < help_array; i++)
				arr2SD[co_arr_full_TR3][co_arr_save_TR3++] = buffer[i];

		}

		if (signal_arrvoll_torque3 && ((save_files & TORQUE_3_MASK) == 0)) { // Falls buffer voll und Datenspeicherung gerade nicht stattfindet...
			save_files = save_files | TORQUE_3_MASK; // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			if (co_arr_full_TR3 == 2) // nachdem am Anfang das 1. array gefuellt wurde
				co_arr_full_TR3++;
			else
				// nachdem das 2. array gefuellt wurde, wird das erste wieder gefuellt
				co_arr_full_TR3--;
		}
		if (messung == CAN2_EIN)
			hcan2.pTxMsg->StdId = GESCHWINDIGKEIT_RAD;

		break;

	case (GESCHWINDIGKEIT_RAD):

		left_wheel_speed = (((CanHandle->pRxMsg->Data[5]) << 8)
				| CanHandle->pRxMsg->Data[4]) >> 4;
		help_array = sprintf(buffer, "%u;%d\n", counter, left_wheel_speed); // Fuelle Array zum ueberpruefen, ob 512 Bytes schon voll sind

		if (512 <= (co_arr_save_GR_L + help_array)) { // Falls zu fuellendes Array keinen Platz mehr hat, dann...
			uint16_t anfang_2fuell = co_arr_save_GR_L;
			signal_arrvoll_v_lwheel = 1;     // Array ist voll

			for (uint8_t i = 0; i < help_array; i++) {

				if (i < 512 - anfang_2fuell) { // Fuelle Array bis 512...
					arr2SD[co_arr_full_GR_L][co_arr_save_GR_L++] = buffer[i];
				} else {         // den Rest im naechsten Array
					if (co_arr_full_GR_L == 4) { // Falls 1. array voll
						arr2SD[co_arr_full_GR_L + 1][co_arr_save_GR_L++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_TR3+1, da ich nicht immer wieder co_arr_full_TR3 erhoehen kann
					} else
						// Falls 2. array voll
						arr2SD[co_arr_full_GR_L - 1][co_arr_save_GR_L++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_TR3+1, da ich nicht immer wieder co_arr_full_TR3 erhoehen kann
				}
				if (co_arr_save_GR_L == 512) // Ist beliebiger Array voll
					co_arr_save_GR_L = 0;   //initialisiere erneut Arrayfuellung
			}

		} else { // Falls zu fuellendes Array noch Platz hat, dann...

			for (uint8_t i = 0; i < help_array; i++)
				arr2SD[co_arr_full_GR_L][co_arr_save_GR_L++] = buffer[i];

		}

		if (signal_arrvoll_v_lwheel
				&& ((save_files & GESCHWINDIGKEIT_RAD_MASK) == 0)) { // Falls buffer voll und Datenspeicherung gerade nicht stattfindet...
			save_files = save_files | GESCHWINDIGKEIT_RAD_MASK; // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			if (co_arr_full_GR_L == 4) // nachdem am Anfang das 1. array gefuellt wurde
				co_arr_full_GR_L++;
			else
				// nachdem das 2. array gefuellt wurde, wird das erste wieder gefuellt
				co_arr_full_GR_L--;
		}
		if (messung == CAN2_EIN)
			hcan2.pTxMsg->StdId = STAT_KOMBI;

		break;

	case (STAT_KOMBI):

		// Geschwindigkeit abspeichern
		vehicle_speed = ((CanHandle->pRxMsg->Data[1] & 0xF) << 8)
				| CanHandle->pRxMsg->Data[0];
		vehicle_speed /= 10;
		help_array = sprintf(buffer, "%u;%hu\n", counter, vehicle_speed); // Fuelle Array zum ueberpruefen, ob 512 Bytes schon voll sind

		if (512 <= (co_arr_save_STKO + help_array)) { // Falls zu fuellendes Array keinen Platz mehr hat, dann...
			uint16_t anfang_2fuell = co_arr_save_STKO;
			signal_arrvoll_vAuto = 1;     // Array ist voll
			for (uint8_t i = 0; i < help_array; i++) {

				if (i < 512 - anfang_2fuell) // Fuelle Array bis 512...
					arr2SD[co_arr_full_STKO][co_arr_save_STKO++] = buffer[i];
				else {         // den Rest im naechsten Array
					if (co_arr_full_STKO == 6) { // Falls 1. array voll
						arr2SD[co_arr_full_STKO + 1][co_arr_save_STKO++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_STKO+1, da ich nicht immer wieder co_arr_full_STKO erhoehen kann
					} else
						// Falls 2. array voll
						arr2SD[co_arr_full_STKO - 1][co_arr_save_STKO++] =
								buffer[i]; // ich verwende diese Anweisung co_arr_full_STKO+1, da ich nicht immer wieder co_arr_full_STKO erhoehen kann
				}
				if (co_arr_save_STKO == 512) // Ist beliebiger Array voll
					co_arr_save_STKO = 0;   //initialisiere erneut Arrayfuellung
			}
		} else { // Falls zu fuellendes Array noch Platz hat, dann...

			for (uint8_t i = 0; i < help_array; i++)
				arr2SD[co_arr_full_STKO][co_arr_save_STKO++] = buffer[i];

		}

		if (signal_arrvoll_vAuto && ((save_files & STAT_KOMBI_MASK) == 0)) { // Falls buffer voll und Datenspeicherung gerade nicht stattfindet...
			save_files = save_files | STAT_KOMBI_MASK; // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			//co_arr_save_STKO = 0;            // setze counter fuer pointer des arrays 512->0
			if (co_arr_full_STKO == 6) // nachdem am Anfang das 1. array gefuellt wurde
				co_arr_full_STKO++;
			else
				// nachdem das 2. array gefuellt wurde, wird das erste wieder gefuellt
				co_arr_full_STKO--;
		}
		if (messung == CAN2_EIN)
			hcan2.pTxMsg->StdId = TORQUE_1;

		break;

	default:
		break;
	}

	/* Receive. This function must be called after each data reception process */
	if (HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0) != HAL_OK) {
		/* Reception Error */
		Error_Handler_CANR();
	}

}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == KEY_BUTTON_PIN) {

		if (get_button_state() == 0) { // Wurde Knopf zur Datenspeicherung gedrueckt?
			set_button_state();	// Dann setze ensprechendes Signalisierungsbit
		} else {
			reset_button_state();// Sonst, signalisiere, dass Knopf erneut gedrueckt wurde, um Messung zu stoppen

		}

		if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) { // Initialisierung Timer3 zur Entprellung
			/* Counter Enable Error */
			Error_Handler();
		}

		HAL_NVIC_DisableIRQ(KEY_BUTTON_EXTI_IRQn); // Deaktivierung des Interrupts fuer den Button 2 Sekunde lang

	}

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

void init_config_MCU() {

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();

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

	if (f_mount(&FS, "SD:", 1) != FR_OK) {
		Error_Handler_fats();
	}
	BSP_LED_On(LED5);
	BSP_LED_On(LED4);	// Diese LED gibt an, ob CAN2 verwendet wird oder nicht

	set_storage_state(1);			// Setze Initialisierungsbit der Speicherung

	if (program_start == 0) {

		if (HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0) != HAL_OK) { // Wurde zum ersten Mal das STM-Board angemacht, dann initialisiere CAN-Empfangsuebertragung
			/* Reception Error */
			Error_Handler_CANR();
		}
	} else {								// Sonst erwecke den CAN-Controller
		if (HAL_CAN_WakeUp(&hcan1) != HAL_OK) {
			/* Reception Error */
			Error_Handler_CANR();
		}
	}

	if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) { // Initialisierung Timer fuer CAN-Daten
		/* Counter Enable Error */
		Error_Handler();
	}

	program_start = 1; // Setze Initialisierungsbit ab der Speicherung und wird nie mehr geaendert
}

void stop_storage() {

	/* Unmount SDCARD */
	f_mount(NULL, "SD:", 1);

	if (HAL_CAN_Sleep(&hcan1) != HAL_OK) {
		/* Reception Error */
		Error_Handler_CANR();
	}

	if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK) { // Initialisierung Timer fuer CAN-Daten
		/* Counter Enable Error */
		Error_Handler();
	}

	BSP_LED_Off(LED5);
	BSP_LED_Off(LED4); // Diese LED wird ausgemacht, um anzugeben, dass CAN2 nicht dabei ist

	set_storage_state(0); // Die CAN-Uebertragung kann wieder neu gestartet werden, nachdem dieses Bit zurueckgesetzt wurde
	counter = 0;
}

void can_transmit() {
	if (messung == CAN2_EIN) {	// Nur CAN2 Einschalten, wenn gewuenscht!
		if (HAL_CAN_Transmit(&hcan2, 10) != HAL_OK) {
			/* Transmition Error */
			Error_Handler_CANT();

		}
	} else {
		if (can_deinit == 0) {   // wurde CAN2 deinitialisiert?
			HAL_CAN_MspDeInit(&hcan2);			// CAN Controller-Sender
			can_deinit = 1;
		}
		BSP_LED_Off(LED4);// Diese LED wird ausgemacht, um anzugeben, dass CAN2 nicht dabei ist
	}
}

uint8_t get_storage_state() {

	return cantrans_sdstor_init;

}

void set_storage_state(uint8_t neu) {

	cantrans_sdstor_init = neu;

}

void store_incomming_data() { // von hier aus werden die Daten an die SD gesendet, von der isr werden die Daten in die Puffer gespeichert

	if (save_files & TORQUE_1_MASK) {

//		if (get_storage_state() == 1) { 	//Loesche vorhandene Datei
//			FileSize = 0;
//			fres = f_unlink("SD:torque1_zus.csv");
//			set_storage_state(2); // Dadurch wird der Zustand gespeichert, in dem angegeben wird, dass Datenspeicherung erst angefangen hat. 1 funktioniert nicht, da sore_incomm... nur dann aufgerufen wird, nachdem cantrans_sdstor_init=1 gesetzt wurde
//		} else {
//			FileSize = f_size(&fil_torque1);
//		}

		if ((fres = f_open(&fil_torque1, "SD:torque1_zus.csv",
		FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) == FR_OK) {	// Oeffne neue Datei

			FileSize = f_size(&fil_torque1);
			/* Move to offset of FileSize from top of the file */
			fres = f_lseek(&fil_torque1, FileSize);

			/* Write 512 bytes to file */

			if (co_arr_full_TR1 == 1) { // nachdem am Anfang das 1. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==1
				if (f_write(&fil_torque1, arr2SD[co_arr_full_TR1 - 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			} else { // nachdem das 2. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==0
				if (f_write(&fil_torque1, arr2SD[co_arr_full_TR1 + 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			}

			f_close(&fil_torque1);

			// data transfer finished, eventuell muss ich hier die CAN-ISR deaktivieren
			save_files = save_files & (~TORQUE_1_MASK); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			wbytes = 0;
			signal_arrvoll_torque1 = 0;     // Array gesendet
		}
	}
	if (save_files & TORQUE_3_MASK) {

//		if (get_storage_state() == 1) { 	//Loesche vorhandene Datei
//			FileSize = 0;
//			fres = f_unlink("SD:torque3_zus.csv");
//			set_storage_state(2); // Dadurch wird der Zustand gespeichert, in dem angegeben wird, dass Datenspeicherung erst angefangen hat. 1 funktioniert nicht, da store_incomm... nur dann aufgerufen wird, nachdem cantrans_sdstor_init=1 gesetzt wurde
//		} else {
//			FileSize = f_size(&fil_torque3);
//		}

		if ((fres = f_open(&fil_torque3, "SD:torque3_zus.csv",
		FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) == FR_OK) {

			FileSize = f_size(&fil_torque3);

			/* Move to offset of FileSize from top of the file */
			fres = f_lseek(&fil_torque3, FileSize);

			/* Write 512 bytes to file */

			if (co_arr_full_TR3 == 3) { // nachdem am Anfang das 1. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==1
				if (f_write(&fil_torque3, arr2SD[co_arr_full_TR3 - 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			} else { // nachdem das 2. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==0
				if (f_write(&fil_torque3, arr2SD[co_arr_full_TR3 + 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			}

			f_close(&fil_torque3);

// data transfer finished, eventuell muss ich hier die CAN-ISR deaktivieren
			save_files = save_files & (~TORQUE_3_MASK); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			wbytes = 0;
			signal_arrvoll_torque3 = 0;     // Array gesendet
		}
	}
	if (save_files & GESCHWINDIGKEIT_RAD_MASK) {

//		if (get_storage_state() == 1) { 	//Loesche vorhandene Datei
//			FileSize = 0;
//			fres = f_unlink("SD:geschwindigkeit_rad_zus.csv");
//			set_storage_state(2); // Dadurch wird der Zustand gespeichert, in dem angegeben wird, dass Datenspeicherung erst angefangen hat. 1 funktioniert nicht, da sore_incomm... nur dann aufgerufen wird, nachdem cantrans_sdstor_init=1 gesetzt wurde
//		} else {
//			FileSize = f_size(&fil_gesch_wheel);
//		}

		if ((fres = f_open(&fil_gesch_wheel, "SD:geschwindigkeit_rad_zus.csv",
		FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) == FR_OK) {

			FileSize = f_size(&fil_gesch_wheel);

			/* Move to offset of FileSize from top of the file */
			fres = f_lseek(&fil_gesch_wheel, FileSize);

			/* Write 512 bytes to file */

			if (co_arr_full_GR_L == 5) { // nachdem am Anfang das 1. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==1
				if (f_write(&fil_gesch_wheel, arr2SD[co_arr_full_GR_L - 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			} else { // nachdem das 2. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==0
				if (f_write(&fil_gesch_wheel, arr2SD[co_arr_full_GR_L + 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			}

			f_close(&fil_gesch_wheel);

// data transfer finished, eventuell muss ich hier die CAN-ISR deaktivieren
			save_files = save_files & (~GESCHWINDIGKEIT_RAD_MASK); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
			wbytes = 0;
			signal_arrvoll_v_lwheel = 0;
		}
	}
	if (save_files & STAT_KOMBI_MASK) {

//		if (get_storage_state() == 1) { 	//Loesche vorhandene Datei
//			FileSize = 0;
//			fres = f_unlink("SD:geschwindigkeit_auto_zus.csv");
//			set_storage_state(2); // Dadurch wird der Zustand gespeichert, in dem angegeben wird, dass Datenspeicherung erst angefangen hat. 1 funktioniert nicht, da sore_incomm... nur dann aufgerufen wird, nachdem cantrans_sdstor_init=1 gesetzt wurde
//		} else {
//			FileSize = f_size(&fil_gesch_car);
//		}

		if ((fres = f_open(&fil_gesch_car, "SD:geschwindigkeit_auto_zus.csv",
		FA_OPEN_ALWAYS | FA_READ | FA_WRITE)) == FR_OK) {

			FileSize = f_size(&fil_gesch_car);

			/* Move to offset of FileSize from top of the file */
			fres = f_lseek(&fil_gesch_car, FileSize);

			/* Write 512 bytes to file */

			if (co_arr_full_STKO == 7) { // nachdem am Anfang das 1. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==1
				if (f_write(&fil_gesch_car, arr2SD[co_arr_full_STKO - 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			} else { // nachdem das 2. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==0
				if (f_write(&fil_gesch_car, arr2SD[co_arr_full_STKO + 1], 512,
						(void *) &wbytes) != FR_OK) {
					Error_Handler_fats();
				}
			}

			f_close(&fil_gesch_car);

			// data transfer finished, eventuell muss ich hier die CAN-ISR deaktivieren
			save_files = save_files & (~STAT_KOMBI_MASK); // ruecksetze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. gesendet worden ist
			wbytes = 0;
			signal_arrvoll_vAuto = 0;     // Array gesendet
		}
	}

}

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

