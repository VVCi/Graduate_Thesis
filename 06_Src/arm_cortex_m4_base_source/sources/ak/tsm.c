#include "tsm.h"
#include "ak_dbg.h"

#include "sys_dbg.h"

void tsm_init(tsm_tbl_t* tsm_tbl, tsm_t** tbl, tsm_state_t state) {
	if (tsm_tbl == (tsm_tbl_t*)0) {
		FATAL("TSM", 0x01);
	}

	/* assign state table */
	tsm_tbl->table = tbl;

	/* init state */
	tsm_tran(tsm_tbl, state);
}

void tsm_dispatch(tsm_tbl_t* tsm_tbl, ak_msg_t* msg) {
	uint8_t state_change_flag = AK_FLAG_OFF;

	tsm_t* respective_table = tsm_tbl->table[tsm_tbl->state];

	/* search tsm state respective */
	while (respective_table->sig != msg->sig) {
		respective_table++;
	}

	/* checking and update next state */
	if (tsm_tbl->state != respective_table->next_state) {
		state_change_flag = AK_FLAG_ON;
	}

	if (state_change_flag == AK_FLAG_ON) {
		tsm_tran(tsm_tbl, respective_table->next_state);
	}

	/* handle message which respective state */
	if (respective_table->tsm_func != TSM_FUNCTION_NULL) {
		respective_table->tsm_func(msg);
	}
}

void tsm_tran(tsm_tbl_t *tsm_tbl, tsm_state_t state) {
	tsm_tbl->state = state;

	/* signal state changed */
	if (tsm_tbl->on_state != ((on_tsm_state)0)) {
		tsm_tbl->on_state(tsm_tbl->state);
	}
}
