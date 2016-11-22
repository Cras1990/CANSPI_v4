/*
 * PtCan_SdStorage.h
 *
 *  Created on: 14.09.2016
 *      Author: SamirAlexis
 */

#ifndef PTCAN_SDSTORAGE_H_
#define PTCAN_SDSTORAGE_H_

#include "stm32f4xx_hal.h"
#include "Std_Types.h"

/* Global variable Declarations */
// Variablen zum Ansteuern der Datenspeicherung und Senden der CAN-Signale zur SD-Karte

/* Global Function Declarations */
void PtCan_initMemory(void);
void PtCan_SdStorage_storeSD(void);
void PtCan_SdStorage_storeRAM(uint8_t dataIndex, volatile uint32_t time, volatile uint16_t value);
StatusType PtCan_SdStorage_SetStateSD(uint8_t on_off);
uint8_t get_storage_state(void);
void set_storage_state(uint8_t neu);


#endif /* PTCAN_SDSTORAGE_H_ */
