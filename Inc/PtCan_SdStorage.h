/*
 * PtCan_SdStorage.h
 *
 *  Created on: 14.09.2016
 *      Author: SamirAlexis
 */

#ifndef PTCAN_SDSTORAGE_H_
#define PTCAN_SDSTORAGE_H_

#include "stm32f4xx_hal.h"

#define DATA1_OPENSD 0x00
#define DATA2_OPENSD 0x01
#define DATA3_OPENSD 0x02
#define DATA4_OPENSD 0x03

// Ein- oder Ausblenden von CAN2

#define CAN2_AUS     0x00
#define CAN2_EIN     0x01

/* Global variable Declarations */
// Variablen zum Ansteuern der Datenspeicherung und Senden der CAN-Signale zur SD-Karte

volatile uint8_t messung = CAN2_EIN;
uint8_t program_start = 0;	// Globale Variable zur Signalisierung, dass das ganze Programm ab der Messung gestartet ist
uint8_t can_deinit = 0;

/* Global Function Declarations */
void PtCan_initMemory(void);
void PtCan_SdStorage_storeSD(void);
void PtCan_SdStorage_storeRAM(void);


#endif /* PTCAN_SDSTORAGE_H_ */
