/*
 * PtCan_Can.c
 *
 *  Created on: 14.11.2016
 *      Author: revelo
 */

#include "PtCan_Can.h"
#include "PtCan_SdStorage.h"
#include "stm32f4xx_hal.h"
#include "PtCan_Cfg.h"

// Einstellungsvariablen fuer Peripherie-Controller
static CanTxMsgTypeDef TxMessage;
static CanRxMsgTypeDef RxMessage;

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

		PtCan_SdStorage_storeRAM(0, &co_arr_full_TR1, &co_arr_save_TR1,
				&signal_arrvoll_torque1, counter, trq_eng_in1);

		if (messung == CAN2_EIN)
			hcan2.pTxMsg->StdId = TORQUE_3;

		break;

	case (TORQUE_3):     // Speed of the shaft
		trq_eng_in3 = (((CanHandle->pRxMsg->Data[5]) << 8)
				| CanHandle->pRxMsg->Data[4]) >> 2;

		PtCan_SdStorage_storeRAM(1, &co_arr_full_TR3, &co_arr_save_TR3,
				&signal_arrvoll_torque3, counter, trq_eng_in3);

		if (messung == CAN2_EIN)
			hcan2.pTxMsg->StdId = GESCHWINDIGKEIT_RAD;

		break;

	case (GESCHWINDIGKEIT_RAD):

		left_wheel_speed = (((CanHandle->pRxMsg->Data[5]) << 8)
				| CanHandle->pRxMsg->Data[4]) >> 4;

		PtCan_SdStorage_storeRAM(2, &co_arr_full_GR_L, &co_arr_save_GR_L,
				&signal_arrvoll_v_lwheel, counter, left_wheel_speed);

		if (messung == CAN2_EIN)
			hcan2.pTxMsg->StdId = STAT_KOMBI;

		break;

	case (STAT_KOMBI):

		// Geschwindigkeit abspeichern
		vehicle_speed = ((CanHandle->pRxMsg->Data[1] & 0xF) << 8)
				| CanHandle->pRxMsg->Data[0];
		vehicle_speed /= 10;

		PtCan_SdStorage_storeRAM(3, &co_arr_full_STKO, &co_arr_save_STKO,
				&signal_arrvoll_vAuto, counter, vehicle_speed);

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
