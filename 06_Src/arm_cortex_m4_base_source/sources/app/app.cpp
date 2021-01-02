/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   13/08/2016
 ******************************************************************************
**/

/* kernel include */
#include "../ak/ak.h"
#include "../ak/message.h"
#include "../ak/timer.h"
#include "../ak/fsm.h"

/* driver include */
#include "../driver/led/led.h"
#include "../driver/button/button.h"
#include "../driver/flash/flash.h"
#include "../driver/hs1101/hs1101.h"

/* app include */
#include "app.h"
#include "app_dbg.h"
#include "app_bsp.h"
#include "app_flash.h"
#include "app_non_clear_ram.h"

#include "task_list.h"
#include "task_shell.h"
#include "task_life.h"
#include "task_if.h"
#include "task_rf24_if.h"

/* sys include */
#include "../sys/sys_boot.h"
#include "../sys/sys_irq.h"
#include "../sys/sys_io.h"
#include "../sys/sys_ctrl.h"
#include "../sys/sys_dbg.h"
#include "../sys/sys_arduino.h"

/* common include */
#include "../common/utils.h"
#include "../common/screen_manager.h"

#include "../platform/stm32f4/io_cfg.h"
#include "../platform/stm32f4/stm32f4.h"

const char* app_version = APP_VER;

static boot_app_share_data_t boot_app_share_data;

static void app_start_timer();
static void app_init_state_machine();
static void app_task_init();

/*****************************************************************************/
/* app main function.
 */
/*****************************************************************************/
int main_app() {
	APP_PRINT("app version: %s\n", app_version);

	/******************************************************************************
	* init active kernel
	*******************************************************************************/
	ENTRY_CRITICAL();
	task_init();
	task_create(app_task_table);
	EXIT_CRITICAL();

	/******************************************************************************
	* init applications
	*******************************************************************************/
	/*********************
	* hardware configure *
	**********************/
	/* init watch dog timer */

	/*********************
	* software configure *
	**********************/

	//uart2_config();

	/* life led init */
	led_init(&led_life, led_life_init, led_life_on, led_life_off);

	/* flash io init */
//	flash_io_ctrl_init();
//	windboneFlash_spi_ctrl_init();

	/* MPU Init */
	MPU6050_I2C_Init();
	//MPU6050_Initialize();

	/* timer4 init */
	timer4_config();

	/* gpio for task encoder init */
	ALL_GPIO_INIT();

	/* pwm init */
	ALL_PWM_INIT();

	/* encoder init */
	ALL_ENCODER_INIT();

	/* uart for transfer data init */
	io_uart_interface_cfg();

	EXIT_CRITICAL();

	/* start timer for application */
	app_init_state_machine();
	app_start_timer();
	/******************************************************************************
	* app task initial
	*******************************************************************************/
	app_task_init();

	/******************************************************************************
	* run applications
	*******************************************************************************/
	return task_run();
}

/*****************************************************************************/
/* app initial function.
 */
/*****************************************************************************/

/* start software timer for application
 * used for app tasks
 */
void app_start_timer() {
	/* start timer to toggle life led */
	timer_set(AC_TASK_LIFE_ID, AC_LIFE_SYSTEM_CHECK, 10000, TIMER_PERIODIC);
	timer_set(AC_TASK_ENCODER_PID_ID, SL_DELTA_ROBOT_AUTO_RUN_REQ, 500, TIMER_PERIODIC);
}

/* init state machine for tasks
 * used for app tasks
 */
void app_init_state_machine() {
}

/* send first message to trigger start tasks
 * used for app tasks
 */
void app_task_init() {
}

/*****************************************************************************/
/* app common function
 */
/*****************************************************************************/

/* hardware timer interrupt 10ms
 * used for led, button polling
 */
void sys_irq_timer_10ms() {
}

/* hardware timer interrupt 50ms
 * used for encode and decode ir
 */
void sys_irq_timer_50us() {
}

/* hardware rtc interrupt alarm
 * used for internal rtc alarm
 */
void sys_irq_timer_hs1101() {
}

/* hardware io interrupt at rev ir pin
 * used for decode ir
 */
void sys_irq_ir_io_rev() {
}

void* app_get_boot_share_data() {
	return (void*)&boot_app_share_data;
}
