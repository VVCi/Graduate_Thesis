#include "../ak/fsm.h"
#include "../ak/port.h"
#include "../ak/message.h"

#include "../sys/sys_ctrl.h"
#include "../sys/sys_dbg.h"
#include "../common/xprintf.h"

#include "app.h"
#include "app_dbg.h"

#include "task_list.h"
#include "task_life.h"
#include <math.h>

#define IsUpper(c)	(((c)>='A')&&((c)<='Z'))
#define IsLower(c)	(((c)>='a')&&((c)<='z'))
#define IsDigit(c)	(((c)>='0')&&((c)<='9'))

led_t led_life;

void task_life(ak_msg_t* msg) {
	switch (msg->sig) {
	case AC_LIFE_SYSTEM_CHECK: {
		led_toggle(&led_life);
	}
		break;

	default:
		break;
	}
}
