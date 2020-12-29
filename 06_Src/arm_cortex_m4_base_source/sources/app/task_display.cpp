#include "../ak/fsm.h"
#include "../ak/port.h"
#include "../ak/message.h"
#include "../ak/timer.h"

#include "../sys/sys_ctrl.h"
#include "../sys/sys_dbg.h"

#include "../common/view_render.h"

#include "app.h"
#include "app_dbg.h"

#include "task_list.h"
#include "task_display.h"

void task_display(ak_msg_t* msg) {
	switch (msg->sig) {
	case AC_DISPLAY_INITIAL: {
		APP_DBG("AC_DISPLAY_INITIAL\n");
		view_render.initialize();
		task_post_pure_msg(AC_TASK_DISPLAY_ID, AC_DISPLAY_SHOW_ON_LOGO);
	}
		break;

	case AC_DISPLAY_SHOW_ON_LOGO: {
		APP_DBG("AC_DISPLAY_SHOW_ON_LOGO\n");
		view_render.display_on();
		view_render.clear();

		/* ak logo */
		view_render.setTextSize(1);
		view_render.setTextColor(WHITE);
		view_render.setCursor(24, 3);
		view_render.print("   __    _  _ ");
		view_render.setCursor(24, 10);
		view_render.print("  /__\\  ( )/ )");
		view_render.setCursor(24, 20);
		view_render.print(" /(__)\\ (   (");
		view_render.setCursor(24, 30);
		view_render.print("(__)(__)(_)\\_)");
		view_render.setCursor(24, 45);
		view_render.print("Active Kernel");
		view_render.update();

		timer_set(AC_TASK_DISPLAY_ID, AC_DISPLAY_SHOW_OFF_LOGO, AC_DISPLAY_LOGO_INTERVAL, TIMER_ONE_SHOT);
	}
		break;

	case AC_DISPLAY_SHOW_OFF_LOGO: {
		APP_DBG("AC_DISPLAY_SHOW_OFF_LOGO\n");
		view_render.display_off();
	}
		break;

	default:
		break;
	}
}
