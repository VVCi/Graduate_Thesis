/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   05/09/2016
 ******************************************************************************
**/
#include "ak_dbg.h"

#include "timer.h"

#include "sys_dbg.h"
#include "task_list.h"

/* define message data is transfered between interrupt heart beat and timer task */
struct ak_timer_payload_irq_t {
	uint32_t counter;
	uint32_t enable_post_msg;
};

static volatile struct ak_timer_payload_irq_t ak_timer_payload_irq = {0, AK_DISABLE};

/* data to manage memory of timer message */
static ak_timer_t timer_pool[AK_TIMER_POOL_SIZE];
static ak_timer_t* free_list_timer_pool;
static ak_timer_t* timer_list_head;

/* allocate/free memory of timer message */
static void         timer_msg_pool_init();
static ak_timer_t*  get_timer_msg();
static void         free_timer_msg(ak_timer_t* msg);

static uint8_t timer_remove_msg(task_id_t des_task_id, timer_sig_t sig);

void timer_msg_pool_init() {
	uint32_t index;

	ENTRY_CRITICAL();

	timer_list_head       = TIMER_MSG_NULL;
	free_list_timer_pool = (ak_timer_t*)timer_pool;

	for (index = 0; index < AK_TIMER_POOL_SIZE; index++) {
		if (index == (AK_TIMER_POOL_SIZE - 1)) {
			timer_pool[index].next = TIMER_MSG_NULL;
		}
		else {
			timer_pool[index].next = (ak_timer_t*)&timer_pool[index + 1];
		}
	}

	EXIT_CRITICAL();
}

ak_timer_t* get_timer_msg() {
	ak_timer_t* allocate_timer;

	ENTRY_CRITICAL();

	allocate_timer = free_list_timer_pool;

	if (allocate_timer == TIMER_MSG_NULL) {
		FATAL("MT", 0x30);
	}
	else {
		free_list_timer_pool = allocate_timer->next;
	}

	EXIT_CRITICAL();

	return allocate_timer;
}

void free_timer_msg(ak_timer_t* msg) {

	ENTRY_CRITICAL();

	msg->next = free_list_timer_pool;
	free_list_timer_pool = msg;

	EXIT_CRITICAL();
}

void task_timer_tick(ak_msg_t* msg) {
	ak_msg_t* timer_msg;

	ak_timer_t* timer_list;
	ak_timer_t* timer_del = TIMER_MSG_NULL; /* MUST-BE assign TIMER_MSG_NULL when entry function */

	int32_t temp_counter = 0;   /* keep this variable for preemtive kernel mode */
	int32_t irq_counter = 0;

	/* permit receive message from interrupt heart beat */

	ENTRY_CRITICAL();

	timer_list = timer_list_head;

	/* save current interrupt counter value */
	irq_counter = ak_timer_payload_irq.counter;

	/* reset interrupt counter && enable interrupt post message to timer task */
	ak_timer_payload_irq.counter = 0;
	ak_timer_payload_irq.enable_post_msg = AK_ENABLE;

	EXIT_CRITICAL();

	switch (msg->sig) {
	case TIMER_TICK:
		/* query timer node */
		while(timer_list != TIMER_MSG_NULL) {

			ENTRY_CRITICAL();

			timer_list->counter -= irq_counter;
			temp_counter = timer_list->counter;

			EXIT_CRITICAL();

			/* time-out counter */
			if (temp_counter <= 0) {

				/* send timer message to owner task */
				timer_msg = get_pure_msg();
				set_msg_sig(timer_msg, timer_list->sig);
				task_post(timer_list->des_task_id, timer_msg);

				ENTRY_CRITICAL();

				/* periodic timer */
				if (timer_list->period) {
					/* update timer counter */
					timer_list->counter = timer_list->period;
				}
				/* one-shot timer, remove timer node from timer list */
				else {
					/* mask timer node to delete */
					timer_del = timer_list;
				}

				EXIT_CRITICAL();
			}

			/* next node */
			timer_list = timer_list->next;

			/* remove node expired */
			if (timer_del) {
				timer_remove_msg(timer_del->des_task_id, timer_del->sig);
				timer_del = TIMER_MSG_NULL;
			}
		}
		break;

	default:
		break;
	}
}

void timer_init() {
	/* init timer message pool */
	timer_msg_pool_init();

	/* init data transfer between interrupt heart beat and timer task */
	ENTRY_CRITICAL();

	ak_timer_payload_irq.counter = 0;
	ak_timer_payload_irq.enable_post_msg = AK_ENABLE;

	EXIT_CRITICAL();
}

void timer_tick(int32_t t) {
	/* check available timer */
	if (timer_list_head != TIMER_MSG_NULL) {
		/* if previous message is not handled, just only increase counter */
		if (ak_timer_payload_irq.enable_post_msg == AK_DISABLE) {
			ak_timer_payload_irq.counter = ak_timer_payload_irq.counter + t;
		}
		/* if previous message is handled, post new message to timer task */
		else {
			/* get a pure message */
			ak_msg_t* msg = get_pure_msg();

			/* disable post message until this message is handled */
			ak_timer_payload_irq.enable_post_msg = AK_DISABLE;

			if (ak_timer_payload_irq.counter == 0) {
				ak_timer_payload_irq.counter = t;
			}

			/* post new message */
			set_msg_sig(msg, TIMER_TICK);
			task_post(TASK_TIMER_TICK_ID, msg);
		}
	}
}

uint8_t timer_set(task_id_t des_task_id, timer_sig_t sig, int32_t duty, timer_type_t type) {
	ak_timer_t* timer_msg;

	ENTRY_CRITICAL();

	timer_msg = timer_list_head;

	while (timer_msg != TIMER_MSG_NULL) {
		/* check timer node */
		if (timer_msg->des_task_id == des_task_id &&
				timer_msg->sig == sig) {

			timer_msg->counter = duty;

			EXIT_CRITICAL();

			return TIMER_RET_OK;
		}
		else {
			timer_msg = timer_msg->next;
		}
	}

	/* if timer does not exist, create new timer */

	/* get timer mesage from timer message pool */
	timer_msg = get_timer_msg();

	/* assign properties of timer node */
	timer_msg->des_task_id  = des_task_id;
	timer_msg->sig      = sig;
	timer_msg->counter  = (int32_t)duty;

	switch (type) {
	case TIMER_ONE_SHOT:
		timer_msg->period       = (int32_t)0;
		break;

	case TIMER_PERIODIC:
		timer_msg->period       = (int32_t)duty;
		break;

	default: {
		FATAL("MT", 0x32);
	}
		break;
	}

	/* insert node to timer list */
	if (timer_list_head == TIMER_MSG_NULL) {
		timer_msg->next = TIMER_MSG_NULL;
		timer_list_head = timer_msg;
	}
	else {
		timer_msg->next = timer_list_head;
		timer_list_head = timer_msg;
	}

	EXIT_CRITICAL();

	return TIMER_RET_OK;
}

uint8_t timer_remove_msg(task_id_t des_task_id, timer_sig_t sig) {
	ak_timer_t* timer_msg;
	ak_timer_t* timer_msg_prev;

	ENTRY_CRITICAL();

	timer_msg = timer_list_head;
	timer_msg_prev = timer_msg;

	while (timer_msg != TIMER_MSG_NULL) {

		/* check timer node */
		if (timer_msg->des_task_id == des_task_id &&
				timer_msg->sig == sig) {

			/* the head node */
			if (timer_msg == timer_list_head) {

				/* remove node */
				timer_list_head = timer_msg->next;
			}
			else {
				/* remove node */
				timer_msg_prev->next = timer_msg->next;
			}

			/* free node memory */
			free_timer_msg(timer_msg);

			EXIT_CRITICAL();

			return TIMER_RET_OK;
		}
		else {
			/* move on next node */
			timer_msg_prev = timer_msg;
			timer_msg = timer_msg->next;
		}
	}

	EXIT_CRITICAL();

	return TIMER_RET_NG;
}

uint8_t timer_remove_attr(task_id_t des_task_id, timer_sig_t sig) {
	/* remove timer message in timer queue message */
	uint8_t ret = timer_remove_msg(des_task_id, sig);

	/* remove timer message in task queue message */
	task_remove_msg(des_task_id, sig);

	return ret;
}

uint32_t timer_used() {
	uint32_t used = 0;
	ak_timer_t* head_pool = timer_list_head;
	while(head_pool != TIMER_MSG_NULL) {
		used++;
		head_pool = head_pool->next;
	}
	return used;
}

ak_timer_t* timer_get_list() {
	return timer_list_head;
}
