/*
 * PTCAN.h
 *
 *  Created on: 07.03.2016
 *      Author: revelo
 */

#ifndef PTCAN_H_
#define PTCAN_H_

// Diese Methoden werden auch im Hauptprogramm zur Verfuegung gestellt

void store_incomming_data(void);

void init_config_MCU(void);

void start_storage(void);

void stop_storage(void);

void can_transmit(void);

uint8_t get_storage_state(void);



#endif /* PTCAN_H_ */
