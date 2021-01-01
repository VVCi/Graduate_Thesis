/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   05/09/2016
 ******************************************************************************
**/
#include <stdint.h>
#include <stdbool.h>

#include "sys_cfg.h"
#include "system.h"
#include "stm32f4.h"

#include "system_stm32f4xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"

#include "../common/xprintf.h"

#include "../../sys/sys_dbg.h"
#include "../../sys/sys_ctrl.h"

#include "../../sys/sys_dbg.h"
#include "../../sys/sys_ctrl.h"
#include "../../sys/sys_irq.h"
#include "../../sys/sys_boot.h"
#include "../../sys/sys_svc.h"

#include "../../ak/ak.h"
#include "../../ak/message.h"
#include "../../ak/timer.h"
#include "../../ak/port.h"

#include "../../app/app.h"

#include "../app/task_list.h"
#include "io_cfg.h"
#include "task_encoder_pid.h"

/*****************************************************************************/
/* linker variable                                                           */
/*****************************************************************************/
extern uint32_t _ldata;
extern uint32_t _data;
extern uint32_t _edata;
extern uint32_t _bss;
extern uint32_t _ebss;
extern uint32_t _estack;

extern void (*__preinit_array_start[])();
extern void (*__preinit_array_end[])();
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();
extern void _init();

/*****************************************************************************/
/* static function prototype                                                 */
/*****************************************************************************/
/*****************************/
/* system interrupt function */
/*****************************/
void default_handler();
void reset_handler();

/*****************************/
/* user interrupt function   */
/*****************************/
void exti_line1_irq();
void exti_line15_irq();
void timer6_irq();
void timer4_irq();
void timer7_irq();
void uart2_irq(void);
void uart3_irq(void);
void spi2_irq(void);

/* cortex-M processor fault exceptions */
void nmi_handler()          __attribute__ ((weak));
void hard_fault_handler()   __attribute__ ((weak));
void mem_manage_handler()   __attribute__ ((weak));
void bus_fault_handler()    __attribute__ ((weak));
void usage_fault_handler()  __attribute__ ((weak));

/* cortex-M processor non-fault exceptions */
void dg_monitor_handler()   __attribute__ ((weak, alias("default_handler")));
void pendsv_handler();
void svc_handler();
void systick_handler();

/* external interrupts */
void shell_handler();
void svc_exe(uint32_t* svc_args);



/*****************************************************************************/
/* system variable                                                           */
/*****************************************************************************/
system_info_t system_info;

/*****************************************************************************/
/* interrupt vector table                                                    */
/*****************************************************************************/
__attribute__((section(".isr_vector")))
void (* const isr_vector[])() = {
		((void (*)())(uint32_t)&_estack),		//	The initial stack pointer
		reset_handler              ,// Reset Handler
		nmi_handler                ,// NMI Handler
		hard_fault_handler          ,// Hard Fault Handler
		mem_manage_handler          ,// MPU Fault Handler
		bus_fault_handler           ,// Bus Fault Handler
		usage_fault_handler         ,// Usage Fault Handler
		0                          ,// Reserved
		0                          ,// Reserved
		0                          ,// Reserved
		0                          ,// Reserved
		svc_handler                ,// SVCall Handler
		dg_monitor_handler           ,// Debug Monitor Handler
		0                          ,// Reserved
		pendsv_handler             ,// PendSV Handler
		systick_handler            ,// SysTick Handler
		default_handler                   ,// Window WatchDog
		default_handler                    ,// PVD through EXTI Line detection
		default_handler             ,// Tamper and TimeStamps through the EXTI line
		default_handler               ,// RTC Wakeup through the EXTI line
		default_handler                  ,// FLASH
		default_handler                    ,// RCC
		default_handler                  ,// EXTI Line0
		default_handler                  ,// EXTI Line1
		default_handler  ,// EXTI Line2
		default_handler  ,// EXTI Line3
		default_handler  ,// EXTI Line4
		default_handler  ,// DMA1 Stream 0
		default_handler  ,// DMA1 Stream 1
		default_handler  ,// DMA1 Stream 2
		default_handler  ,// DMA1 Stream 3
		default_handler  ,// DMA1 Stream 4
		default_handler  ,// DMA1 Stream 5
		default_handler  ,// DMA1 Stream 6
		default_handler  ,// ADC1, ADC2 and ADC3s
		default_handler  ,// CAN1 TX
		default_handler  ,// CAN1 RX0
		default_handler  ,// CAN1 RX1
		default_handler  ,// CAN1 SCE
		default_handler  ,// External Line[9:5]s
		default_handler  ,// TIM1 Break and TIM9
		default_handler  ,// TIM1 Update and TIM10
		default_handler  ,// TIM1 Trigger and Commutation and TIM11
		default_handler  ,// TIM1 Capture Compare
		default_handler  ,// TIM2
		default_handler  ,// TIM3
		timer4_irq		 ,// TIM4
		default_handler  ,// I2C1 Event
		default_handler  ,// I2C1 Error
		default_handler  ,// I2C2 Event
		default_handler  ,// I2C2 Error
		default_handler  ,// SPI1
		default_handler  ,// SPI2
		shell_handler    ,// USART1
		uart2_irq		 ,// USART2
		default_handler  ,// USART3
		default_handler  ,// External Line[15:10]s
		default_handler  ,// RTC Alarm (A and B) through EXTI Line
		default_handler  ,// USB OTG FS Wakeup through EXTI line
		default_handler  ,// TIM8 Break and TIM12
		default_handler  ,// TIM8 Update and TIM13
		default_handler  ,// TIM8 Trigger and Commutation and TIM14
		default_handler  ,// TIM8 Capture Compare
		default_handler  ,// DMA1 Stream7
		default_handler  ,// FSMC
		default_handler  ,// SDIO
		default_handler  ,// TIM5
		default_handler  ,// SPI3
		default_handler  ,// UART4
		default_handler  ,// UART5
		default_handler  ,// TIM6 and DAC1&2 underrun errors
		default_handler  ,// TIM7
		default_handler  ,// DMA2 Stream 0
		default_handler  ,// DMA2 Stream 1
		default_handler  ,// DMA2 Stream 2
		default_handler  ,// DMA2 Stream 3
		default_handler  ,// DMA2 Stream 4
		default_handler  ,// Ethernet
		default_handler  ,// Ethernet Wakeup through EXTI line
		default_handler  ,// CAN2 TX
		default_handler  ,// CAN2 RX0
		default_handler  ,// CAN2 RX1
		default_handler  ,// CAN2 SCE
		default_handler  ,// USB OTG FS
		default_handler  ,// DMA2 Stream 5
		default_handler  ,// DMA2 Stream 6
		default_handler  ,// DMA2 Stream 7
		default_handler	 ,// USART6
		default_handler  ,// I2C3 event
		default_handler  ,// I2C3 error
		default_handler  ,// USB OTG HS End Point 1 Out
		default_handler  ,// USB OTG HS End Point 1 In
		default_handler  ,// USB OTG HS Wakeup through EXTI
		default_handler  ,// USB OTG HS
		default_handler  ,// DCMI
		default_handler  ,// CRYP crypto
		default_handler  ,// Hash and Rng
		default_handler   // FPU
		};

void __attribute__((naked))
sys_ctrl_delay(volatile uint32_t count)
{
	(void)count;
	__asm("    subs    r0, #1\n"
	"    bne     sys_ctrl_delay\n"
	"    bx      lr");
}

static uint32_t millis_current  = 0;

uint32_t sys_ctrl_millis() {
	volatile uint32_t ret;
	ENTRY_CRITICAL();
	ret = millis_current;
	EXIT_CRITICAL();
	return ret;
}

void _init() {
	/* dummy */
}

/*****************************************************************************/
/* static function defination                                                */
/*****************************************************************************/
void default_handler() {
	FATAL("SY", 0xEE);
}

void reset_handler() {
	/* MUST BE disable interrupt */
	__disable_irq();
	uint32_t *pSrc	= &_ldata;
	uint32_t *pDest	= &_data;
	volatile unsigned i, cnt;

	/* init system */
	SystemInit();

	/* copy init data from FLASH to SRAM */
	while(pDest < &_edata) {
		*pDest++ = *pSrc++;
	}

	/* zero bss */
	for (pDest = &_bss; pDest < &_ebss; pDest++) {
		*pDest = 0UL;
	}

	ENTRY_CRITICAL();

	sys_cfg_clock(); /* init system clock */
	sys_cfg_svc(); /* setting svc exception priority */
	sys_cfg_pendsv(); /* setting psv exception priority */
	sys_cfg_tick(); /* system tick 1ms */
	sys_cfg_console(); /* system console */

	/* invoke all static constructors */
	cnt = __preinit_array_end - __preinit_array_start;
	for (i = 0; i < cnt; i++)
		__preinit_array_start[i]();

	_init();

	cnt = __init_array_end - __init_array_start;
	for (i = 0; i < cnt; i++)
		__init_array_start[i]();

	/* wait configuration stable */
	sys_ctrl_delay(100);  /* wait 300 cycles clock */

	/* update system information */
	sys_cfg_update_info();

	/* entry app function */
	main_app();
}

/***************************************/
/* cortex-M processor fault exceptions */
/***************************************/
void nmi_handler() {
	FATAL("SY", 0x01);
}

void hard_fault_handler() {
	FATAL("SY", 0x02);
}

void mem_manage_handler() {
	FATAL("SY", 0x03);
}

void bus_fault_handler() {
	FATAL("SY", 0x04);
}

void usage_fault_handler() {
	FATAL("SY", 0x05);
}

/*******************************************/
/* cortex-M processor non-fault exceptions */
/*******************************************/
static uint32_t div_counter = 0;

void systick_handler() {
	task_entry_interrupt();

	/* increasing millis counter */
	millis_current++;
	timer_tick(1);

	if (div_counter == 10) {
		div_counter = 0;
	}

	switch(div_counter) {
	case 0:
		/* trigger heart beat of system */
		sys_irq_timer_10ms();
		break;

	case 1:
		break;

	default:
		break;
	}
	div_counter++;

	task_exit_interrupt();
}

void svc_exe(uint32_t* svc_args) {
	uint8_t svc_number;

	sys_dbg_func_stack_dump(svc_args);
	sys_dbg_cpu_dump();

	svc_number = ((uint8_t*)svc_args[6])[-2];

	switch (svc_number) {
	/* start application */
	case 0x01: {
	}
		break;

		/* switch to unprivileged state*/
	case 0x02: {
	}
		break;

	default:
		break;
	}
}

void __attribute__ ((naked)) svc_handler() {
	__asm volatile (
				"tst lr, #4\n"
				"ite eq\n"
				"mrseq r0, msp\n"
				"mrsne r0, psp\n"
				"b svc_exe\n"
				);
}

volatile uint32_t stack_restart_app[8];
volatile uint32_t *sp_restart_app = &stack_restart_app[8];

void __attribute__ ((naked)) pendsv_handler() {
	__disable_irq();

	/* fabricate Cortex-M ISR stack frame for restart application */
	*(--sp_restart_app) = (1U << 24);  /* xPSR */
	*(--sp_restart_app) = (uint32_t)&sys_cfg_restore_app; /* PC */
	*(--sp_restart_app) = 0x00000000U; /* LR  */
	*(--sp_restart_app) = 0x00000000U; /* R12 */
	*(--sp_restart_app) = 0x00000000U; /* R3  */
	*(--sp_restart_app) = 0x00000000U; /* R2  */
	*(--sp_restart_app) = 0x00000000U; /* R1  */
	*(--sp_restart_app) = 0x00000000U; /* R0  */

	__asm volatile ("MSR msp, %0\n" : : "r" (sp_restart_app) : "sp");
	__asm volatile ("bx lr");
}

/************************/
/* external interrupts  */
/************************/
void shell_handler() {
	task_entry_interrupt();

	if (USART_GetITStatus(USARTx, USART_IT_RXNE) == SET) {
		/* DO NOT clear pending interrupt right here ! */
		sys_irq_shell();
	}

	task_exit_interrupt();
}

void exti_line1_irq() {
	task_entry_interrupt();

	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		sys_irq_nrf24l01();
		EXTI_ClearITPendingBit(EXTI_Line1);
	}

	task_exit_interrupt();
}

void exti_line15_irq() {
	task_entry_interrupt();

	if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
		sys_irq_ir_io_rev();
		EXTI_ClearITPendingBit(EXTI_Line15);
	}

	task_exit_interrupt();
}

void timer6_irq() {
	task_entry_interrupt();

	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET) {
		sys_irq_timer_50us();
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}

	task_exit_interrupt();
}

void timer7_irq() {
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET) {
		sys_ctrl_soft_watchdog_increase_counter();
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}
}


float ploi, dloi, iloi, Last_Bias;
__IO float pid_feeback1, pid_feeback2, pid_feeback3;
float kp = 1.65;
float ki = 0.4;
float kd = 0;

float r_k = 0;
float pre_error;
float inte, deri;


void PID_DIGITAL(__IO float* Save_PID_Feeback, int32_t Target, int32_t Current, float kp, float ki, float kd) {
	static int32_t Bias;
	//static float Pwm;
	__IO float Pwm;

	Bias = Target - Current;

	ploi = Bias;

	dloi = Bias - Last_Bias;

	iloi += Bias;

	iloi = constrain(iloi, -10, 10);

	Last_Bias = Bias;

	Pwm = kp*ploi + ki*iloi*0.02 + kd*dloi/0.02;

	Pwm = constrain(Pwm, -100, 100);

	*Save_PID_Feeback = Pwm ;

#if (DEBUG_ENCODER == 1)
	//xprintf("error: %d			", (int32_t)error);
	//xprintf("Target: %d			", (int32_t)Target);
	xprintf("Current: %d			\n", (int32_t)Current);
	//xprintf("pid_feeback1: %d			\n", (int32_t)Pwm);
#endif
}

void PID_DeltaRobot(__IO float* Save_PID_Feeback, float deta_run, float t_run, float current, float kp, float ki, float kd) {
	float error;

	float s;
	__IO float pwm;

	// Tính độ dốc cần đạt được
	s = (float)deta_run / t_run;

	/* r_k trong trường hợp này sẽ là setpoint và được cộng
	dồn qua nhiều lần tương ứng với số bậc thang cần đạt được của PID */
	r_k += (s  * 0.02);

	 if (ABS(r_k) > ABS(deta_run)) {
		r_k = deta_run;
	 }

	// Bo xu ly PID
	error = r_k - current;
	inte += error * 0.02;	// Tích phân
	deri = (error - pre_error) / 0.02; // Đạo hàm
	inte = constrain(inte, -50, 50);

	// Output
	pwm = (kp * error) + (ki * inte) + (kd * deri);
	pwm = constrain(pwm, -100, 100);
	*Save_PID_Feeback = pwm;

	pre_error = error;
#if (DEBUG_ENCODER == 1)
	//xprintf("error: %d			", (int32_t)error);
	xprintf("r_k: %d			", (int32_t)r_k);
	xprintf("current: %d			\n", (int32_t)current);
	xprintf("pid_feeback1: %d			\n", (int32_t)pwm);
#endif
}

void timer4_irq() {
	task_entry_interrupt();

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) {

		//PID_DeltaRobot(&pid_feeback2, pid_degree , 0.5, enc_fb2.Pos_encoder_feedback, kp_545, ki_545, kd_545);
		PID_DIGITAL(&pid_feeback2, pid_degree, enc_fb2.Pos_encoder_feedback, kp_545, ki_545, kd_545);
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}

	task_exit_interrupt();
}

void uart2_irq(void) {
	// uint8_t c;
	task_entry_interrupt();
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET) {
		//c = (uint8_t)USART_ReceiveData(USART2);
		//xprintf("%d:", c);
		//USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		sys_irq_uart_cpu_serial_if();
	}
	task_exit_interrupt();
}

void uart3_irq(void) {
}

void spi2_irq(void) {
}
