/*
 * PtCan_SdStorage.c
 *
 *  Created on: 14.09.2016
 *      Author: SamirAlexis
 */

#include "PtCan_SdStorage.h"
#include "PtCan_ErrHandling.h"
#include "ff.h"
#include "Std_Types.h"

#define NUM_STRINGS_DATA 4
#define TORQUE_1_MASK                   0x01
#define TORQUE_3_MASK                   0x02
#define GESCHWINDIGKEIT_RAD_MASK        0x04
#define STAT_KOMBI_MASK                 0x08

static FIL fil_gesch_wheel;
static FIL fil_torque1;
static FIL fil_torque3;
static FIL fil_gesch_car;

static FIL *file_data[NUM_STRINGS_DATA];
static char *strs_data[NUM_STRINGS_DATA];
static uint8_t arr2SD[8][512];

static volatile uint16_t vehicle_speed; // velocity of the car. It can be negative
static volatile int16_t trq_eng_in1; //      torque actual value can be negative
static volatile uint16_t trq_eng_in3;  			//      rpm_engine
static volatile int16_t left_wheel_speed; // velocity of left wheel can be negative

static uint32_t wbytes;                // Zaehlvariable bezueglich fwrite

static volatile uint8_t cantrans_sdstor_init = 0; // cantrans_sdstor_init = 0 -> Datafile closed, storage has still not begun or has been finished. Otherwise the storage is active
static volatile uint8_t save_files = 0; // Gibt an, wann ein Datenpaket an die SD-Karte bezueglich eines Messsignals geschickt werden soll
// 1.Signal
static volatile uint16_t co_arr_save_TR1 = 0;
static volatile uint8_t co_arr_full_TR1 = 0;
// 2.Signal
static volatile uint16_t co_arr_save_TR3 = 0;
static volatile uint8_t co_arr_full_TR3 = 2;
// 3.Signal
static volatile uint16_t co_arr_save_GR_L = 0;
static volatile uint8_t co_arr_full_GR_L = 4;
// 4.Signal
static volatile uint16_t co_arr_save_STKO = 0;
static volatile uint8_t co_arr_full_STKO = 6;

static volatile uint8_t signal_arrvoll_vAuto = 0;
static volatile uint8_t signal_arrvoll_v_lwheel = 0;
static volatile uint8_t signal_arrvoll_torque1 = 0;
static volatile uint8_t signal_arrvoll_torque3 = 0;

/* Local Function Declarations */
static StatusType PtCan_SdStorage_openSd(uint8_t dataIndex);
static StatusType PtCan_SdStorage_closeSd(uint8_t dataIndex);
static void PtCan_SdStorage_sendData(uint8_t dataIndex, volatile uint8_t co_arr_full);



/* Local Function Definitions */

static StatusType PtCan_SdStorage_openSd(uint8_t dataIndex) {
	StatusType statusOpening = f_open(file_data[dataIndex], strs_data[dataIndex],
	FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	return statusOpening;
}

static StatusType PtCan_SdStorage_closeSd(uint8_t dataIndex) {
	StatusType statusClosing = f_close(file_data[dataIndex]);
	return statusClosing;
}

static void PtCan_SdStorage_sendData(uint8_t dataIndex, volatile uint8_t *co_arr_full) {

	if (PtCan_SdStorage_openSd(dataIndex) == E_OK) {
		uint32_t FileSize = f_size(file_data[dataIndex]);
		uint8_t wbytes;
		/* Move to offset of FileSize from top of the file */
		f_lseek(file_data[dataIndex], FileSize);

		/* Write 512 bytes to file */

		if (*co_arr_full == 1) { // nachdem am Anfang das 1. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==1
			if (f_write(file_data[dataIndex], arr2SD[(*co_arr_full) - 1], 512,
					(void *) &wbytes) != FR_OK) {
				Error_Handler_fats();
			}
		} else { // nachdem das 2. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==0
			if (f_write(file_data[dataIndex], arr2SD[(*co_arr_full) + 1], 512,
					(void *) &wbytes) != FR_OK) {
				Error_Handler_fats();
			}
		}
		PtCan_SdStorage_closeSd(dataIndex);
	} else {
		//Error ausgeben
	}
}



/* Global Function Declarations */

void PtCan_SdStorage_storeSD() { // von hier aus werden die Daten an die SD gesendet, von der isr werden die Daten in die Puffer gespeichert

	if (save_files & TORQUE_1_MASK) {

		PtCan_SdStorage_sendData(0, &co_arr_full_TR1);
		// data transfer finished
		save_files = save_files & (~TORQUE_1_MASK); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
		signal_arrvoll_torque1 = 0;
	}

	if (save_files & TORQUE_3_MASK) {

		PtCan_SdStorage_sendData(1, &co_arr_full_TR3);

// data transfer finished
		save_files = save_files & (~TORQUE_3_MASK); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
		signal_arrvoll_torque3 = 0;

	}
	if (save_files & GESCHWINDIGKEIT_RAD_MASK) {

		PtCan_SdStorage_sendData(2, &co_arr_full_GR_L);
// data transfer finished
		save_files = save_files & (~GESCHWINDIGKEIT_RAD_MASK); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
		signal_arrvoll_v_lwheel = 0;

	}
	if (save_files & STAT_KOMBI_MASK) {

		PtCan_SdStorage_sendData(3, &co_arr_full_STKO);
// data transfer finished
		save_files = save_files & (~STAT_KOMBI_MASK); // ruecksetze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. gesendet worden ist
		signal_arrvoll_vAuto = 0; // Array gesendet
	}
}

void PtCan_SdStorage_storeRAM(volatile uint8_t *co_arr_full, volatile uint16_t *co_arr_save, volatile uint8_t *signal_arrvoll, volatile uint32_t time, volatile uint16_t value)
{
	uint8_t help_array = 0;
	char buffer_tmp[50];
	help_array = sprintf(buffer_tmp, "%u;%d\n", time, value); // Fuelle Array zum ueberpruefen, ob 512 Bytes schon voll sind

	if (512 <= (*co_arr_save + help_array)) { // Falls zu fuellendes Array keinen Platz mehr hat, dann...
		uint16_t anfang_2fuell = *co_arr_save;
		*signal_arrvoll = 1;     // Array ist voll

		for (uint8_t i = 0; i < help_array; i++) {

			if (i < 512 - anfang_2fuell) { // Fuelle Array bis 512...
				arr2SD[*co_arr_full][(*co_arr_save)++] = buffer_tmp[i];
			} else {         // den Rest im naechsten Array
				if (*co_arr_full == 0) { // Falls 1. array voll
					arr2SD[*co_arr_full + 1][(*co_arr_save)++] = buffer_tmp[i]; // ich verwende diese Anweisung co_arr_full_TR1+1, da ich nicht immer wieder co_arr_full_TR1 erhoehen kann
				} else
					// Falls 2. array voll
					arr2SD[*co_arr_full - 1][(*co_arr_save)++] = buffer_tmp[i]; // ich verwende diese Anweisung co_arr_full_TR1+1, da ich nicht immer wieder co_arr_full_TR1 erhoehen kann
			}
			if (*co_arr_save == 512) // Ist beliebiger Array voll
				*co_arr_save = 0;    //initialisiere erneut Arrayfuellung
		}

	} else { // Falls zu fuellendes Array noch Platz hat, dann...

		for (uint8_t i = 0; i < help_array; i++)
			arr2SD[*co_arr_full][(*co_arr_save)++] = buffer_tmp[i];
	}
}

void PtCan_initMemory() {
	strs_data[0] = "SD:torque1_zus.csv";
	strs_data[1] = "SD:torque3_zus.csv";
	strs_data[2] = "SD:geschwindigkeit_rad_zus.csv";
	strs_data[3] = "SD:geschwindigkeit_auto_zus.csv";

	file_data[0] = &fil_torque1;
	file_data[1] = &fil_torque3;
	file_data[2] = &fil_gesch_wheel;
	file_data[3] = &fil_gesch_car;

}



