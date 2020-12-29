/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   05/09/2016
 ******************************************************************************
**/
#include "sys_dbg.h"
#include "sys_ctrl.h"

#include "../ak/ak.h"

#include "../driver/eeprom/eeprom.h"
#include "../driver/flash/flash.h"

#include "../app/app_eeprom.h"
#include "../app/app_flash.h"

#include "../common/utils.h"
#include "../common/log_queue.h"

#include "sys_irq.h"
#include "sys_svc.h"

#if defined(STM32L_PLATFORM)
#include "../common/xprintf.h"
#include "../platform/stm32l/io_cfg.h"
#include "../platform/stm32l/sys_cfg.h"
#elif defined(STM32F10X_PLATFORM)
#include "../common/xprintf.h"
#include "../platform/stm32f10x/io_cfg.h"
#include "../platform/stm32f10x/sys_cfg.h"
#elif defined(STM32F40XX)
#include "../common/xprintf.h"
#include "../platform/stm32f4/io_cfg.h"
#include "../platform/stm32f4/sys_cfg.h"
#else
#error Please choose platform for kernel.
#endif

#define DUMP_RAM_UNIT_SIZE			256

static fatal_log_t fatal_log;

void sys_dbg_fatal(const int8_t* s, uint8_t c) {
	unsigned char rev_c = 0;
	task_t*		ptemp_current_task;
	ak_msg_t*	ptemp_current_active_object;

#if defined(TIVA_PLATFORM)
	UARTprintf("%s\t%x\n", s, c);
#endif

#if defined(STM32L_PLATFORM) || defined(STM32F10X_PLATFORM) ||defined(STM32F40XX)
	xprintf("%s\t%x\n", s, c);
#endif

#if defined(NON_CLEAR_RAM_FATAL_LOG)
	non_clear_ram_fatal_log.fatal_times ++;

	/* set fatal string */
	mem_set(non_clear_ram_fatal_log.string, 0, 10);
	str_cpy(non_clear_ram_fatal_log.string, s);

	/* set fatal code */
	non_clear_ram_fatal_log.code = c;

	/* get task fatal */
	ptemp_current_task = get_current_task_info();
	ptemp_current_task->id = get_current_task_id();

	/* get active object fatal */
	ptemp_current_active_object = get_current_active_object();

	/* get core register */
	non_clear_ram_fatal_log.m3_core_reg.ipsr		= __get_IPSR();
	non_clear_ram_fatal_log.m3_core_reg.primask	= __get_PRIMASK();
	non_clear_ram_fatal_log.m3_core_reg.faultmask	= __get_FAULTMASK();
	non_clear_ram_fatal_log.m3_core_reg.basepri	= __get_BASEPRI();
	non_clear_ram_fatal_log.m3_core_reg.control	= __get_CONTROL();

	mem_cpy(&non_clear_ram_fatal_log.current_task, ptemp_current_task, sizeof(task_t));
	mem_cpy(&non_clear_ram_fatal_log.current_active_object, ptemp_current_active_object, sizeof(ak_msg_t));
#else
	task_exit_interrupt();

	/* read fatal data from flash */
	flash_read(APP_FLASH_AK_DBG_FATAL_LOG_SECTOR, (uint8_t*)&fatal_log, sizeof(fatal_log_t));

	/* increase fatal time */
	fatal_log.fatal_times ++;

	/* set fatal string */
	mem_set(fatal_log.string, 0, 10);
	str_cpy(fatal_log.string, s);

	/* set fatal code */
	fatal_log.code = c;

	/* get task fatal */
	ptemp_current_task = get_current_task_info();

	/* get active object fatal */
	ptemp_current_active_object = get_current_active_object();

	mem_cpy(&fatal_log.current_task, ptemp_current_task, sizeof(task_t));
	mem_cpy(&fatal_log.current_active_object, ptemp_current_active_object, sizeof(ak_msg_t));

	/* flash_write fatal data to epprom */
	flash_erase_sector(APP_FLASH_AK_DBG_FATAL_LOG_SECTOR);
	flash_write(APP_FLASH_AK_DBG_FATAL_LOG_SECTOR, (uint8_t*)&fatal_log, sizeof(fatal_log_t));

	/************************
	 *  trace irq info    *
	 ************************/
#if defined(AK_IRQ_OBJ_LOG_ENABLE)
	static exception_info_t t_exception_info;
	uint32_t flash_irq_log_address = APP_FLASH_AK_DBG_IRQ_LOG_SECTOR;
	SYS_PRINT("start write irq info\n");
	flash_erase_sector(flash_irq_log_address);
	while(log_queue_len(&log_irq_queue)) {
		log_queue_get(&log_irq_queue, &t_exception_info);
		flash_write(flash_irq_log_address, (uint8_t*)&t_exception_info, sizeof(exception_info_t));
		flash_irq_log_address += sizeof(exception_info_t);
	}
#endif


	/************************
	 *  trace fatal message *
	 ************************/
#if defined(AK_TASK_OBJ_LOG_ENABLE)
	uint32_t flash_sys_log_address = APP_FLASH_AK_DBG_MSG_SECTOR_1;
	SYS_PRINT("start write message log to flash\n");
	flash_erase_sector(flash_sys_log_address);
	while(log_queue_len(&log_task_dbg_object_queue)) {
		log_queue_get(&log_task_dbg_object_queue, &t_msg);
		flash_write(flash_sys_log_address, (uint8_t*)&t_msg, sizeof(ak_msg_t));
		flash_sys_log_address += sizeof(ak_msg_t);
	}
#endif

#endif

	sys_ctrl_delay_us(1000);

#if defined(RELEASE)
	sys_ctrl_reset();
#endif

	while(1) {
		/* reset watchdog */
		sys_ctrl_independent_watchdog_reset();
		sys_ctrl_soft_watchdog_reset();

		/* FATAL debug option */
		rev_c = sys_ctrl_shell_get_char();
		if (rev_c) {
			switch (rev_c) {
			case 'r':
				sys_ctrl_reset();
				break;

			default:
				break;
			}
		}

		/* led notify FATAL */
		led_life_on();
		sys_ctrl_delay_us(200000);
		led_life_off();
		sys_ctrl_delay_us(200000);
	}
}

void sys_dbg_func_stack_dump(uint32_t* args) {
	/**
	Stack frame contains:
	r0, r1, r2, r3, r12, r14, the return address and xPSR
	- Stacked R0	<-> args[0]
	- Stacked R1	<-> args[1]
	- Stacked R2	<-> args[2]
	- Stacked R3	<-> args[3]
	- Stacked R12	<-> args[4]
	- Stacked LR	<-> args[5]
	- Stacked PC	<-> args[6]
	- Stacked xPSR	<-> args[7]
	*/
	(void)args;
	SYS_DBG("[st]R0\t0x%08X\n",		args[0]);
	SYS_DBG("[st]R1\t0x%08X\n",		args[1]);
	SYS_DBG("[st]R2\t0x%08X\n",		args[2]);
	SYS_DBG("[st]R3\t0x%08X\n",		args[3]);
	SYS_DBG("[st]R12\t0x%08X\n",	args[4]);
	SYS_DBG("[st]LR\t0x%08X\n",		args[5]);
	SYS_DBG("[st]PC\t0x%08X\n",		args[6]);
	SYS_DBG("[st]PSR\t0x%08X\n",	args[7]);
}

void sys_dbg_cpu_dump() {
	SYS_PRINT("[cr]IPSR\t%d\n", __get_IPSR());
	SYS_PRINT("[cr]PRIMASK\t0x%08X\n", __get_PRIMASK());
	SYS_PRINT("[cr]FAULTMASK\t0x%08X\n", __get_FAULTMASK());
	SYS_PRINT("[cr]BASEPRI\t0x%08X\n", __get_BASEPRI());
	SYS_PRINT("[cr]CONTROL\t0x%08X\n", __get_CONTROL());
}

void sys_dbg_stack_space_dump() {
	extern uint32_t _estack;
	uint32_t* start_addr = (uint32_t*)((uint32_t)&_estack) - sizeof(uint32_t);
	uint32_t* end_addr = (uint32_t*)((uint32_t)__get_MSP());
	SYS_PRINT("--- sys_dbg_stack_space_dump ---\n");
	for (uint32_t* i = start_addr; i > end_addr ; i--) {
		SYS_PRINT("[%08X] %08X\n", i, *i);
	}
}
