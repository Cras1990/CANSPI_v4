/*
 * PtCan_SdStorage.c
 *
 *  Created on: 14.09.2016
 *      Author: SamirAlexis
 */

#include "PtCan_SdStorage.h"
#include "PtCan_ErrHandling.h"
#include "tm_stm32_fatfs.h"
#include "PtCan_Cfg.h"

typedef struct {
	volatile uint8_t savingMasks;
	volatile uint16_t co_arr_save;
	volatile uint8_t co_arr_full;
	volatile uint8_t signal_arrvoll;
	FIL *file_data;
	char *strs_data;
} FileDescriptorType;

static FIL fil_gesch_wheel;
static FIL fil_torque1;
static FIL fil_torque3;
static FIL fil_gesch_car;

FATFS FS;
static uint8_t arr2SD[8][512];

static volatile uint8_t cantrans_sdstor_init = 0; // cantrans_sdstor_init = 0 -> Datafile closed, storage has still not begun or has been finished. Otherwise the storage is active
static volatile uint8_t save_files = 0; // Gibt an, wann ein Datenpaket an die SD-Karte bezueglich eines Messsignals geschickt werden soll

FileDescriptorType FileDescriptorMap[4];


/* Local Function Declarations */
static StatusType PtCan_SdStorage_openFile(uint8_t dataIndex);
static StatusType PtCan_SdStorage_closeFile(uint8_t dataIndex);
static void PtCan_SdStorage_sendData(uint8_t dataIndex);

/* Local Function Definitions */

static StatusType PtCan_SdStorage_openFile(uint8_t dataIndex) {
	StatusType statusOpening = f_open(FileDescriptorMap[dataIndex].file_data,
			FileDescriptorMap[dataIndex].strs_data,
			FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	return statusOpening;
}

static StatusType PtCan_SdStorage_closeFile(uint8_t dataIndex) {
	StatusType statusClosing = f_close(FileDescriptorMap[dataIndex].file_data);
	return statusClosing;
}

static void PtCan_SdStorage_sendData(uint8_t dataIndex) {

	if (PtCan_SdStorage_openFile(dataIndex) == E_OK) {
		uint32_t FileSize = f_size(FileDescriptorMap[dataIndex].file_data);
		uint8_t wbytes;
		/* Move to offset of FileSize from top of the file */
		f_lseek(FileDescriptorMap[dataIndex].file_data, FileSize);

		/* Write 512 bytes to file */
		if (FileDescriptorMap[dataIndex].co_arr_full == ((dataIndex<<1) + 1)) { // nachdem am Anfang das 1. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==1
			if (f_write(FileDescriptorMap[dataIndex].file_data,
					arr2SD[FileDescriptorMap[dataIndex].co_arr_full - 1], 512, (void *) &wbytes) != FR_OK) {
				Error_Handler_fats();
			}
		} else { // nachdem das 2. array gefuellt wurde, wird dieses an die SD geschickt. Hierbei ist co_arr_full_GR==0
			if (f_write(FileDescriptorMap[dataIndex].file_data,
					arr2SD[FileDescriptorMap[dataIndex].co_arr_full + 1], 512, (void *) &wbytes) != FR_OK) {
				Error_Handler_fats();
			}
		}
		PtCan_SdStorage_closeFile(dataIndex);
	} else {
		//Error ausgeben
	}
}

/* Global Function Declarations */

void PtCan_SdStorage_storeSD() { // von hier aus werden die Daten an die SD gesendet, von der isr werden die Daten in die Puffer gespeichert

	if (save_files & FileDescriptorMap[0].savingMasks) {

		PtCan_SdStorage_sendData(0);
		// data transfer finished
		save_files = save_files & (~FileDescriptorMap[0].savingMasks); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
		FileDescriptorMap[0].signal_arrvoll = 0;
	}

	if (save_files & FileDescriptorMap[1].savingMasks) {

		PtCan_SdStorage_sendData(1);

// data transfer finished
		save_files = save_files & (~FileDescriptorMap[1].savingMasks); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
		FileDescriptorMap[1].signal_arrvoll = 0;

	}

	if (save_files & FileDescriptorMap[2].savingMasks) {

		PtCan_SdStorage_sendData(2);
// data transfer finished
		save_files = save_files & (~FileDescriptorMap[2].savingMasks); // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
		FileDescriptorMap[2].signal_arrvoll = 0;

	}

	if (save_files & FileDescriptorMap[3].savingMasks) {

		PtCan_SdStorage_sendData(3);
// data transfer finished
		save_files = save_files & (~FileDescriptorMap[3].savingMasks); // ruecksetze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. gesendet worden ist
		FileDescriptorMap[3].signal_arrvoll = 0; // Array gesendet
	}
}

void PtCan_SdStorage_storeRAM(uint8_t dataIndex, volatile uint32_t time,
		volatile uint16_t value) {
	uint8_t help_array = 0;
	char buffer_tmp[50];
	help_array = sprintf(buffer_tmp, "%u;%d\n", time, value); // Fuelle Array zum ueberpruefen, ob 512 Bytes schon voll sind

	if (512 <= (FileDescriptorMap[dataIndex].co_arr_save + help_array)) { // Falls zu fuellendes Array keinen Platz mehr hat, dann...
		uint16_t anfang_2fuell = FileDescriptorMap[dataIndex].co_arr_save;
		FileDescriptorMap[dataIndex].signal_arrvoll = 1;     // Array ist voll

		for (uint8_t i = 0; i < help_array; i++) {

			if (i < 512 - anfang_2fuell) { // Fuelle Array bis 512...
				arr2SD[FileDescriptorMap[dataIndex].co_arr_full][FileDescriptorMap[dataIndex].co_arr_save++] =	buffer_tmp[i];
			} else {         // den Rest im naechsten Array
				if (FileDescriptorMap[dataIndex].co_arr_full == (dataIndex<<1)) { // Falls 1. array voll
					arr2SD[FileDescriptorMap[dataIndex].co_arr_full + 1][FileDescriptorMap[dataIndex].co_arr_save++] =	buffer_tmp[i]; // ich verwende diese Anweisung co_arr_full_TR1+1, da ich nicht immer wieder co_arr_full_TR1 erhoehen kann
				} else
					// Falls 2. array voll
					arr2SD[FileDescriptorMap[dataIndex].co_arr_full - 1][FileDescriptorMap[dataIndex].co_arr_save++] =	buffer_tmp[i]; // ich verwende diese Anweisung co_arr_full_TR1+1, da ich nicht immer wieder co_arr_full_TR1 erhoehen kann
			}
			if (FileDescriptorMap[dataIndex].co_arr_save == 512) // Ist beliebiger Array voll
				FileDescriptorMap[dataIndex].co_arr_save = 0; //initialisiere erneut Arrayfuellung
		}

	} else { // Falls zu fuellendes Array noch Platz hat, dann...

		for (uint8_t i = 0; i < help_array; i++)
			arr2SD[FileDescriptorMap[dataIndex].co_arr_full][FileDescriptorMap[dataIndex].co_arr_save++] =	buffer_tmp[i];
	}

	if (FileDescriptorMap[dataIndex].signal_arrvoll	&& ((save_files & FileDescriptorMap[dataIndex].savingMasks) == 0)) { // Falls buffer voll und Datenspeicherung gerade nicht stattfindet...
		save_files = save_files | FileDescriptorMap[dataIndex].savingMasks; // setze entsprechendes bit von Gesch.rad bei save_files, damit man signal., dass der erster Buffer von Geschw. voll ist
		if (FileDescriptorMap[dataIndex].co_arr_full == (dataIndex<<1)) // nachdem am Anfang das 1. array gefuellt wurde
			FileDescriptorMap[dataIndex].co_arr_full++;
		else
			// nachdem das 2. array gefuellt wurde, wird das erste wieder gefuellt
			FileDescriptorMap[dataIndex].co_arr_full--;
	}
}

void PtCan_initMemory() {

	FileDescriptorMap[0].savingMasks = 0x01;
	FileDescriptorMap[0].co_arr_full = 0;
	FileDescriptorMap[0].co_arr_save = 0;
	FileDescriptorMap[0].signal_arrvoll = 0;
	FileDescriptorMap[0].file_data = &fil_torque1;
	FileDescriptorMap[0].strs_data = "SD:torque1_zus.csv";

	FileDescriptorMap[1].savingMasks = 0x02;
	FileDescriptorMap[1].co_arr_full = 2;
	FileDescriptorMap[1].co_arr_save = 0;
	FileDescriptorMap[1].signal_arrvoll = 0;
	FileDescriptorMap[1].file_data = &fil_torque3;
	FileDescriptorMap[1].strs_data = "SD:torque3_zus.csv";

	FileDescriptorMap[2].savingMasks = 0x04;
	FileDescriptorMap[2].co_arr_full = 4;
	FileDescriptorMap[2].co_arr_save = 0;
	FileDescriptorMap[2].signal_arrvoll = 0;
	FileDescriptorMap[2].file_data = &fil_gesch_wheel;
	FileDescriptorMap[2].strs_data = "SD:geschwindigkeit_rad_zus.csv";

	FileDescriptorMap[3].savingMasks = 0x08;
	FileDescriptorMap[3].co_arr_full = 6;
	FileDescriptorMap[3].co_arr_save = 0;
	FileDescriptorMap[3].signal_arrvoll = 0;
	FileDescriptorMap[3].file_data = &fil_gesch_car;
	FileDescriptorMap[3].strs_data = "SD:geschwindigkeit_auto_zus.csv";

}

StatusType PtCan_SdStorage_SDMount(uint8_t on_off) {
	if (on_off == STD_ON) {
		return f_mount(&FS, "SD:", 1);
	} else {
		/* Unmount SDCARD */
		return f_mount(NULL, "SD:", 1);
	}
}

uint8_t PtCan_SdStorage_getStorageState() {

	return cantrans_sdstor_init;

}

void PtCan_SdStorage_setStorageState(uint8_t neu) {

	cantrans_sdstor_init = neu;

}

