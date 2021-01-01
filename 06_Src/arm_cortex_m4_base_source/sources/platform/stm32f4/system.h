/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   05/09/2016
 ******************************************************************************
**/
#ifndef __SYSTEM_H__
#define __SYSTEM_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "sys_cfg.h"
#include "system.h"
#include "stm32f4.h"

#include "system_stm32f4xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "core_cm4.h"
#define STM32F40_41xxx

#define IRQ_PRIO_SYS_SYSTEMSTICK		1
#define IRQ_PRIO_SYS_SVC				1
#define IRQ_PRIO_SYS_PENDSV				15
#define IRQ_PRIO_UART0_CONSOLE			2
#define IRQ_PRIO_TIMER7_SOFT_WATCHDOG	0

#define min(a, b)  ({  __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _b : _a; })

#define max(a, b) 	({  __typeof__ (a) _a = (a);	__typeof__ (b) _b = (b); _a > _b ? _a : _b; })

#define constrain(x, lower_limit, upper_limit)   ({ __typeof__ (x) _x = (x); __typeof__ (lower_limit) _lower_limit = (lower_limit);  __typeof__ (upper_limit) _upper_limit = (upper_limit);  min(max(_x, _lower_limit), _upper_limit); })

#define ABS(n)     (((n) < 0) ? -(n) : (n))

#define DEBUG_ENCODER	1

#define kp_545		2
#define ki_545		0
#define kd_545		0

typedef struct {
	uint32_t cpu_clock;
	uint32_t tick;
	uint32_t console_baudrate;
	uint32_t flash_used;
	uint32_t ram_used;
	uint32_t data_init_size;
	uint32_t data_non_init_size;
	uint32_t stack_size;
	uint32_t heap_size;
} system_info_t;

extern system_info_t system_info;
extern __IO float pid_feeback1, pid_feeback2, pid_feeback3;


extern void PID_DIGITAL(__IO float* Save_PID_Feeback, int32_t Target, int32_t Current, float kp, float ki, float kd);
void PID_DeltaRobot(__IO float* Save_PID_Feeback, float deta_run, float t_run, float current, float kp, float ki, float kd);
typedef void (*p_jump_func)(void);

#ifdef __cplusplus
}
#endif

#endif //__SYSTEM_H__
