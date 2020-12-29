/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   05/09/2016
 ******************************************************************************
**/
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>

#include "sys_cfg.h"
#include "system.h"

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "system_stm32f4xx.h"
#include "core_cm4.h"
#include "core_cmFunc.h"

#include "../../common/xprintf.h"

#include "../../sys/sys_dbg.h"
#include "../../sys/sys_def.h"
#include "../../sys/sys_ctrl.h"
#include "../../sys/sys_io.h"
#include "../../sys/sys_dbg.h"
#include "../../ak/ak.h"

/* Private define */
static uint32_t delay_coeficient = 0;
static void xputchar(uint8_t c);

/******************************************************************************
* system configure function
*******************************************************************************/
void sys_cfg_clock() {
	RCC_HSICmd(ENABLE);

	while(RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	SystemCoreClockUpdate();

	/* NVIC configuration */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
}

void sys_cfg_tick() {
	SysTick_Config(SystemCoreClock / 10000);
}

void sys_cfg_console() {

	GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
	USART_InitTypeDef USART_InitStructure; // this is for the USART1 initilization
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	/* enable APB2 peripheral clock for USART1
	 * note that only USART1 and USART6 are connected to APB2
	 * the other USARTs are connected to APB1
	 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* enable the peripheral clock for the pins used by
	 * USART1, PB6 for TX and PB7 for RX
	 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* This sequence sets up the TX and RX pins
	 * so they work correctly with the USART1 peripheral
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; 			// the pins are configured as alternate function so the USART peripheral has access to them
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// this defines the IO speed and has nothing to do with the baudrate!
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;			// this defines the output type as push pull mode (as opposed to open drain)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// this activates the pullup resistors on the IO pins
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// now all the values are passed to the GPIO_Init() function which sets the GPIO registers

	/* The RX and TX pins are now connected to their AF
	 * so that the USART1 can take over control of the
	 * pins
	 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	/* Now the USART_InitStruct is used to define the
	 * properties of USART1
	 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);	// again all the properties are passed to the USART_Init function which takes care of all the bit setting


	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);							 // the properties are passed to the NVIC_Init function which takes care of the low level stuff

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
	xfunc_out = xputchar;
}

void sys_cfg_svc() {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = SVCall_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void sys_cfg_pendsv() {
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = PendSV_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0xF;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void sys_cfg_update_info() {
	extern uint32_t _start_flash;
	extern uint32_t _end_flash;
	extern uint32_t _start_ram;
	extern uint32_t _end_ram;
	extern uint32_t _data;
	extern uint32_t _edata;
	extern uint32_t _bss;
	extern uint32_t _ebss;
	extern uint32_t __heap_start__;
	extern uint32_t __heap_end__;
	extern uint32_t _estack;

	RCC_ClocksTypeDef RCC_Clocks;

	RCC_GetClocksFreq(&RCC_Clocks);

	system_info.cpu_clock = RCC_Clocks.HCLK_Frequency;
	system_info.tick      = 1;
	system_info.console_baudrate = SYS_CONSOLE_BAUDRATE;
	system_info.flash_used = ((uint32_t)&_end_flash - (uint32_t)&_start_flash) + ((uint32_t)&_edata - (uint32_t)&_data);
	system_info.ram_used = (uint32_t)&_end_ram - (uint32_t)&_start_ram;
	system_info.data_init_size = (uint32_t)&_edata - (uint32_t)&_data;
	system_info.data_non_init_size = (uint32_t)&_ebss - (uint32_t)&_bss;
	system_info.stack_size = (uint32_t)&_estack - (uint32_t)&_end_ram;
	system_info.heap_size = (uint32_t)&__heap_end__ - (uint32_t)&__heap_start__;

	delay_coeficient = system_info.cpu_clock /1000000;

	/* kernel banner */
	SYS_PRINT("\n");
	SYS_PRINT("   __    _  _ \n");
	SYS_PRINT("  /__\\  ( )/ )\n");
	SYS_PRINT(" /(__)\\ (   ( \n");
	SYS_PRINT("(__)(__)(_)\\_)\n");
	SYS_PRINT("Wellcome to Active Kernel %s\n", AK_VERSION);
	SYS_PRINT("\n");


	/* system banner */
	SYS_PRINT("system information:\n");
	SYS_PRINT("\tFLASH used:\t%d bytes\n", system_info.flash_used);
	SYS_PRINT("\tSRAM used:\t%d bytes\n", system_info.ram_used);
	SYS_PRINT("\t\tdata init size:\t\t%d bytes\n", system_info.data_init_size);
	SYS_PRINT("\t\tdata non_init size:\t%d bytes\n", system_info.data_non_init_size);
	SYS_PRINT("\t\tstack size:\t\t%d bytes\n", system_info.stack_size);
	SYS_PRINT("\t\theap size:\t\t%d bytes\n", system_info.heap_size);
	SYS_PRINT("\n");
	SYS_PRINT("\tcpu clock:\t%d Hz\n", system_info.cpu_clock);
	SYS_PRINT("\ttime tick:\t%d ms\n", system_info.tick);
	SYS_PRINT("\tconsole:\t%d bps\n", system_info.console_baudrate);
	SYS_PRINT("\n\n");
}

/******************************************************************************
* system utilities function
*******************************************************************************/
void xputchar(uint8_t c) {
	/* wait last transmission completed */
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);

	/* put transnission data */
	USART_SendData(USARTx, (uint8_t)c);

	/* wait transmission completed */
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

void sys_ctrl_shell_put_char(uint8_t c) {
	/* wait last transmission completed */
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);

	/* put transnission data */
	USART_SendData(USARTx, (uint8_t)c);

	/* wait transmission completed */
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
}

uint8_t sys_ctrl_shell_get_char() {
	volatile uint8_t c;

	if (USART_GetITStatus(USARTx, USART_IT_RXNE) == SET) {
		c = (uint8_t)USART_ReceiveData(USARTx);
		USART_ClearITPendingBit(USARTx, USART_IT_RXNE);
	}

	return c;
}

void sys_ctrl_reset() {
	NVIC_SystemReset();
}

void sys_ctrl_delay_us(volatile uint32_t count) {
	volatile uint32_t delay_value = 0;
	delay_value = count*delay_coeficient - 10 ;
	while(delay_value--);
}

void sys_ctrl_delay_ms(volatile uint32_t count) {
#if 0
	volatile uint32_t current_time = 0;
	volatile uint32_t current;
	volatile int32_t start = sys_ctrl_millis();

	while(current_time < count) {
		current = sys_ctrl_millis();

		if (current < start) {
			current_time += ((uint32_t)0xFFFFFFFF - start) + current;
		}
		else {
			current_time += current - start;
		}

		start = current;
	}
#else
	volatile uint32_t delay = 1000 * count;
	sys_ctrl_delay_us(delay);
#endif
}

void sys_ctr_sleep_wait_for_irq() {
	__WFI();
}

uint32_t sys_ctr_get_exception_number() {
	volatile uint32_t exception_number = (uint32_t)__get_IPSR();
	return exception_number;
}

void sys_ctr_restart_app() {
	volatile uint32_t normal_stack_pointer	=	(uint32_t) *(volatile uint32_t*)(APP_START_ADDR);
	volatile uint32_t normal_jump_address	=	(uint32_t) *(volatile uint32_t*)(APP_START_ADDR + 4);


	p_jump_func jump_to_normal = (p_jump_func)normal_jump_address;

	/* Disable interrupt */
	DISABLE_INTERRUPTS();

	__DMB();

	/* update interrupt vertor table */
	SCB->VTOR = APP_START_ADDR;

	/* set stack pointer */
	__asm volatile ("MSR msp, %0\n" : : "r" (normal_stack_pointer) : "sp");

	__DSB();

	/* jump to normal program */
	jump_to_normal();

	while(1);
}

void sys_cfg_restore_app() {
	extern uint32_t _isr_vector;
	volatile uint32_t normal_stack_pointer	=	(uint32_t) *(volatile uint32_t*)(APP_START_ADDR);
	volatile uint32_t normal_jump_address	=	(uint32_t) *(volatile uint32_t*)(APP_START_ADDR + 4);

	p_jump_func jump_to_normal = (p_jump_func)normal_jump_address;

	/* update interrupt vertor table */
	SCB->VTOR = (uint32_t)&_isr_vector; /* Vector Table Relocation in Internal FLASH. */

	/* set stack pointer */
	__DSB();
	__asm volatile ("MSR msp, %0\n" : : "r" (normal_stack_pointer) : "sp");
	__DSB();

	/* jump to normal program */
	jump_to_normal();

	while(1);
}

void sys_ctrl_independent_watchdog_init() {
	/* enable PCLK1 for watchdog timer */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_WWDG, ENABLE);

	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_256);		/* IWDG counter clock: 37KHz(LSI) / 256 = 0.144 KHz */
	IWDG_SetReload(0xFFF);						/* Set counter reload, T = (1/IWDG counter clock) * Reload_counter  = 30s  */
	IWDG_ReloadCounter();
	IWDG_Enable();
}

void sys_ctrl_independent_watchdog_reset() {
	ENTRY_CRITICAL();
	IWDG_ReloadCounter();
	EXIT_CRITICAL();
}

static uint32_t sys_ctrl_soft_counter = 0;
static uint32_t sys_ctrl_soft_time_out;

void sys_ctrl_soft_watchdog_init(uint32_t time_out) {
	TIM_TimeBaseInitTypeDef  timer_100us;
	NVIC_InitTypeDef NVIC_InitStructure;
	sys_ctrl_soft_time_out = time_out;

	/* timer 10ms to polling receive IR signal */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	timer_100us.TIM_Period = 3196;		/* HCLK=8Mhz and 3196 respective HCLK=32Mhz;*/
	timer_100us.TIM_Prescaler = 1000;
	timer_100us.TIM_ClockDivision = 0;
	timer_100us.TIM_CounterMode = TIM_CounterMode_Down;

	TIM_TimeBaseInit(TIM7, &timer_100us);

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	/* highest priority level */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM7, ENABLE);
}

void sys_ctrl_soft_watchdog_reset() {
	ENTRY_CRITICAL();
	sys_ctrl_soft_counter = 0;
	EXIT_CRITICAL();
}

void sys_ctrl_soft_watchdog_enable() {
	ENTRY_CRITICAL();
	TIM_Cmd(TIM7, ENABLE);
	EXIT_CRITICAL();
}

void sys_ctrl_soft_watchdog_disable() {
	ENTRY_CRITICAL();
	TIM_Cmd(TIM7, DISABLE);
	EXIT_CRITICAL();
}

void sys_ctrl_soft_watchdog_increase_counter() {
	sys_ctrl_soft_counter++;
	if (sys_ctrl_soft_counter >= sys_ctrl_soft_time_out) {
		TIM_Cmd(TIM7, DISABLE);
		FATAL("SWDG", 0x01);
	}
}

void sys_ctrl_get_firmware_info(firmware_header_t* header) {
	extern uint32_t _start_flash;
	extern uint32_t _end_flash;
	extern uint32_t _data;
	extern uint32_t _edata;

	uint32_t check_sum = 0;
	uint32_t len_of_flash = (uint32_t)&_end_flash - (uint32_t)&_start_flash + ((uint32_t)&_edata - (uint32_t)&_data) + sizeof(uint32_t);

	for (uint32_t index = (uint32_t)&(_start_flash); index <= ((uint32_t)&(_start_flash) + len_of_flash); index += sizeof(uint32_t)) {
		check_sum += *((uint32_t*)index);
	}

	header->psk = FIRMWARE_PSK;
	header->checksum = (check_sum & 0xFFFF);
	header->bin_len = len_of_flash;
}

void ak_irq_io_entry_trigger() {
#if defined(AK_IO_IRQ_ANALYZER)
	GPIO_ResetBits(LED_LIFE_IO_PORT, LED_LIFE_IO_PIN);
#endif
}

void ak_irq_io_exit_trigger() {
#if defined(AK_IO_IRQ_ANALYZER)
	GPIO_SetBits(LED_LIFE_IO_PORT, LED_LIFE_IO_PIN);
#endif
}
