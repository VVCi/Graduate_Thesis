#include "../ak/fsm.h"
#include "../ak/port.h"
#include "../ak/message.h"
#include "../ak/timer.h"

#include <stdio.h>
#include <math.h>

#include "../sys/sys_io.h"
#include "../sys/sys_dbg.h"
#include "../common/utils.h"
#include "../sys/sys_ctrl.h"

#include "app.h"
#include "app_dbg.h"

#include "task_dbg.h"
#include "task_list.h"
#include "stm32f4xx.h"

#include "../driver/sdcard/tm_stm32f4_fatfs.h"

//Fatfs object
FATFS FatFs;
//File object
FIL fil;
//free_1 and total space
uint32_t total, free_1;

uint16_t spi1_data_send = 0;
uint16_t spi1_datarx = 0;

void task_dbg(ak_msg_t* msg) {
	switch (msg->sig) {
	case AC_DBG_TEST_1: {
		APP_DBG("AC_DBG_TEST_1\n");

		static uint32_t count = 0;
		count++;
		spi1_data_send = count;
		SS_EN;
		while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		SPI_I2S_SendData(SPI1, spi1_data_send);
		SS_DIS;
	}
		break;

	case AC_DBG_TEST_2: {
		APP_DBG("AC_DBG_TEST_2\n");
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		spi1_datarx = (uint8_t)SPI_I2S_ReceiveData(SPI1);
		xprintf("spi1 rx: %d\n", spi1_datarx);
	}
		break;

	case 7: {
		xprintf("fatfs sig\n");
		//Mount drive
		if (f_mount(&FatFs, "", 1) == FR_OK) {
			//Mounted OK, turn on RED LED
			xprintf("mount ok\n");

			//Try to open file
			if (f_open(&fil, "fanning.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE) == FR_OK) {
				//File opened, turn off RED and turn on GREEN led
				xprintf("open file ok\n");

				//If we put more than 0 characters (everything OK)
				if (f_puts("why does it rain ?\n", &fil) > 0) {
					if (TM_FATFS_DriveSize(&total, &free_1) == FR_OK) {
						xprintf("write data\n");
					}
					//Turn on both leds
					xprintf("write data ok\n");
				}

				//Close file, don't forget this!
				f_close(&fil);
				xprintf("close file\n");
			}

			//Unmount drive, don't forget this!
			f_mount(0, "", 1);
			xprintf("unmount drive\n");
		}
	}

	default:
		break;
	}
}
