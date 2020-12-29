/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   23/11/2016
 ******************************************************************************
**/

#include <stdint.h>

#include "../ak/ak.h"
#include "../ak/task.h"
#include "../ak/timer.h"
#include "../ak/message.h"

#include "../common/cmd_line.h"
#include "../common/utils.h"
#include "../common/xprintf.h"

#include "../sys/sys_ctrl.h"
#include "../sys/sys_io.h"
#include "../sys/sys_dbg.h"

#include "app.h"
#include "app_data.h"

#include "task_rf24_if.h"

void rf_printf(uint8_t* buf, uint32_t len) {
	(void)buf;
	(void)len;
}
