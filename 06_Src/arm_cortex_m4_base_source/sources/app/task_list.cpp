#include "task_list.h"
#include "../ak/timer.h"

task_t app_task_table[] = {
	/*************************************************************************/
	/* SYSTEM TASK */
	/*************************************************************************/
	{TASK_TIMER_TICK_ID,		TASK_PRI_LEVEL_7,		task_timer_tick		},

	/*************************************************************************/
	/* APP TASK */
	/*************************************************************************/
	{AC_TASK_FW_ID			,	TASK_PRI_LEVEL_2	,	task_fw				},
	{AC_TASK_SHELL_ID		,	TASK_PRI_LEVEL_2	,	task_shell			},
	{AC_TASK_LIFE_ID		,	TASK_PRI_LEVEL_6	,	task_life			},
	{AC_TASK_IF_ID			,	TASK_PRI_LEVEL_4	,	task_if				},
	{AC_TASK_RF24_IF_ID		,	TASK_PRI_LEVEL_4	,	task_rf24_if		},
	{AC_TASK_UART_IF_ID		,	TASK_PRI_LEVEL_4	,	task_uart_if		},
	{AC_TASK_DBG_ID			,	TASK_PRI_LEVEL_4	,	task_dbg			},
	{AC_TASK_DISPLAY_ID		,	TASK_PRI_LEVEL_4	,	task_display		},
	/*************************************************************************/
	/* RF24 NETWORK TASK */
	/*************************************************************************/
	{RF24_PHY_ID			,	TASK_PRI_LEVEL_3	,	task_rf24_phy		},
	{RF24_MAC_ID			,	TASK_PRI_LEVEL_4	,	task_rf24_mac		},
	{AC_TASK_ENCODER_PID_ID	,	TASK_PRI_LEVEL_4	,	task_encoder_pid	},
	{RF24_NWK_ID			,	TASK_PRI_LEVEL_5	,	task_rf24_nwk		},

	/*************************************************************************/
	/* END OF TABLE */
	/*************************************************************************/
	{AK_TASK_EOT_ID,			TASK_PRI_LEVEL_0,		(pf_task)0			}
};
