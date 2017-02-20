/*
 * PtCan_SdStorage.h
 *
 *  Created on: 14.09.2016
 *      Author: SamirAlexis
 */

#ifndef PTCAN_SDSTORAGE_H_
#define PTCAN_SDSTORAGE_H_

/**********************************************************************************************************************
 *  INCLUDES
 *********************************************************************************************************************/
#include "stm32f4xx_hal.h"
#include "Std_Types.h"

/**********************************************************************************************************************
 *  GLOBAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/
void PtCan_initMemory(void);
void PtCan_SdStorage_storeSD(void);
void PtCan_SdStorage_storeRAM(uint8_t dataIndex, volatile uint32_t time, volatile int16_t value);
void PtCan_SdStorage_SDMount(uint8_t on_off);
void PtCan_SdStorage_setStorageState(uint8_t neu);
uint8_t PtCan_SdStorage_getStorageState(void);


#endif /* PTCAN_SDSTORAGE_H_ */
