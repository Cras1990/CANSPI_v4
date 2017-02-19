/*
 * PtCan_Can.h
 *
 *  Created on: 14.11.2016
 *      Author: revelo
 */

#ifndef PTCAN_CAN_H_
#define PTCAN_CAN_H_

/**********************************************************************************************************************
 *  GLOBAL FUNCTION PROTOTYPES
 *********************************************************************************************************************/

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);
void PtCan_Can2_Transmit(void);
void PtCan_Can1ActivReceiveIT(void);
void PtCan_Can1Sleep(void);
void PtCan_Can1WU(void);


#endif /* PTCAN_CAN_H_ */
