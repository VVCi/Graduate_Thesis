/**
 ******************************************************************************
 * @author: ThanNT
 * @date:   13/08/2016
 ******************************************************************************
**/

#include "ak.h"
#include "ak_dbg.h"
#include "message.h"
#include "task.h"

#include "utils.h"
#include "sys_dbg.h"

/* common pool memory */
static ak_msg_common_t    msg_common_pool[AK_COMMON_MSG_POOL_SIZE];
static ak_msg_t*          free_list_common_msg_pool;

/* pure pool memory */
static ak_msg_pure_t      msg_pure_pool[AK_PURE_MSG_POOL_SIZE];
static ak_msg_t*          free_list_pure_msg_pool;

/* dynamic pool memory */
static ak_msg_dynamic_t   msg_dynamic_pool[AK_DYNAMIC_MSG_POOL_SIZE];
static ak_msg_t*          free_list_dynamic_msg_pool;

static dynamic_pdu_t data_msg_dynamic_pool[AK_DYNAMIC_DATA_POOL_SIZE];
static dynamic_pdu_t* head_data_msg_dynamic_pool;
static dynamic_pdu_t* tail_data_msg_dynamic_pool;

static void common_msg_pool_init();
static void pure_msg_pool_init();
static void dynamic_msg_pool_init();

static void free_common_msg(ak_msg_t* msg);
static void free_pure_msg(ak_msg_t* msg);
static void free_dynamic_msg(ak_msg_t* msg);

static uint32_t	get_pool_msg_free(ak_msg_t* pool_msg);

static dynamic_pdu_t* get_data_dynamic_pdu_pool(uint32_t size);
static uint8_t get_data_dynamic_pdu(dynamic_pdu_t* dynamic_pdu, uint8_t* data, uint32_t size);
static uint8_t set_data_dynamic_pdu(dynamic_pdu_t* dynamic_pdu, uint8_t* data, uint32_t size);
static void free_data_dynamic_pdu_pool(dynamic_pdu_t* dynamic_pdu);

void msg_init() {
	/* init common message pool */
	common_msg_pool_init();

	/* init pure message pool */
	pure_msg_pool_init();

	/* init dynamic message pool */
	dynamic_msg_pool_init();
}

void msg_free(ak_msg_t* msg) {
	uint8_t pool_type = 0;

	/* decrease reference counter of message */
	msg_dec_ref_count(msg);

	if (get_msg_ref_count(msg) == 0) {

		/* get type of message */
		pool_type = get_msg_type(msg);

		switch (pool_type) {
		case COMMON_MSG_TYPE:
			free_common_msg(msg);
			break;

		case PURE_MSG_TYPE:
			free_pure_msg(msg);
			break;

		case DYNAMIC_MSG_TYPE:
			free_dynamic_msg(msg);
			break;

		default:
			FATAL("MF", 0x20);
			break;
		}
	}
}

void msg_inc_ref_count(ak_msg_t* msg) {
	if (get_msg_ref_count(msg) < AK_MSG_REF_COUNT_MAX) {
		msg->ref_count++;
	}
	else {
		FATAL("MF", 0x21);
	}
}

void msg_dec_ref_count(ak_msg_t* msg) {
	if (get_msg_ref_count(msg) > 0) {
		msg->ref_count--;
	}
	else {
		FATAL("MF", 0x28);
	}
}

uint32_t get_pool_msg_free(ak_msg_t* pool_msg) {
	uint32_t used = 0;
	ak_msg_t* head_pool = pool_msg;
	while(head_pool != AK_MSG_NULL) {
		used++;
		head_pool = head_pool->next;
	}
	return used;
}

void* ak_malloc(size_t size) {
	extern uint32_t __heap_end__;
	static uint8_t* ak_heap = NULL;

	if (ak_heap != NULL) {
		if (((uint32_t)ak_heap + size + __AK_MALLOC_CTRL_SIZE) > ((uint32_t)&__heap_end__)) {
			FATAL("ak_malloc", 0x01);
		}
	}

	ak_heap = malloc(size);

	if (ak_heap == NULL) {
		FATAL("ak_malloc", 0x02);
	}

	return ak_heap;
}

void ak_free(void* ptr) {
	free(ptr);
}

/*****************************************************************************
 * common message function define.
 *****************************************************************************/
void common_msg_pool_init() {
	uint32_t index;

	ENTRY_CRITICAL();

	free_list_common_msg_pool = (ak_msg_t*)msg_common_pool;

	for (index = 0; index < AK_COMMON_MSG_POOL_SIZE; index++) {
		msg_common_pool[index].msg_header.ref_count |= COMMON_MSG_TYPE;
		if (index == (AK_COMMON_MSG_POOL_SIZE - 1)) {
			msg_common_pool[index].msg_header.next = AK_MSG_NULL;
		}
		else {
			msg_common_pool[index].msg_header.next = (ak_msg_t*)&msg_common_pool[index + 1];
		}
	}

	EXIT_CRITICAL();
}

uint32_t get_common_msg_pool_used() {
	return (AK_COMMON_MSG_POOL_SIZE - get_pool_msg_free((ak_msg_t*)free_list_common_msg_pool));
}

ak_msg_t* get_common_msg() {
	ak_msg_t* allocate_massage;

	ENTRY_CRITICAL();

	allocate_massage = free_list_common_msg_pool;

	if (allocate_massage == AK_MSG_NULL) {
		FATAL("MF", 0x21);
	}
	else {
		free_list_common_msg_pool = allocate_massage->next;
	}

	if (get_msg_type(allocate_massage) != COMMON_MSG_TYPE) {
		FATAL("MF", 0x27);
	}

	msg_inc_ref_count(allocate_massage);

	allocate_massage->src_task_id = get_current_task_id();

	EXIT_CRITICAL();

	return allocate_massage;
}

void free_common_msg(ak_msg_t* msg) {

	ENTRY_CRITICAL();

	msg->next = free_list_common_msg_pool;
	free_list_common_msg_pool = msg;

	EXIT_CRITICAL();
}

uint8_t set_data_common_msg(ak_msg_t* msg, uint8_t* data, uint8_t size) {
	ak_msg_common_t* msg_common = (ak_msg_common_t*)msg;

	/* check messge null */
	if ((ak_msg_t*)msg_common == AK_MSG_NULL) {
		FATAL("MF", 0x22);
	}

	/* check message type */
	if ((msg_common->msg_header.ref_count & AK_MSG_TYPE_MASK) != COMMON_MSG_TYPE) {
		FATAL("MF", 0x23);
	}

	/* check data lenght */
	if (size > AK_COMMON_MSG_DATA_SIZE) {
		FATAL("MF", 0x24);
	}

	/* set data message */
	msg_common->len = size;
	memcpy(msg_common->data, data, size);
	return AK_MSG_OK;
}

uint8_t* get_data_common_msg(ak_msg_t* msg) {
	ak_msg_common_t* msg_common = (ak_msg_common_t*)msg;

	/* check messge null */
	if ((ak_msg_t*)msg_common == AK_MSG_NULL) {
		FATAL("MF", 0x25);
	}

	/* check message type */
	if ((msg_common->msg_header.ref_count & AK_MSG_TYPE_MASK) != COMMON_MSG_TYPE) {
		FATAL("MF", 0x26);
	}

	return (uint8_t*)msg_common->data;
}

uint8_t get_data_len_common_msg(ak_msg_t* msg) {
	ak_msg_common_t* msg_common = (ak_msg_common_t*)msg;

	/* check messge null */
	if ((ak_msg_t*)msg_common == AK_MSG_NULL) {
		FATAL("MF", 0x38);
	}

	return ((uint8_t)msg_common->len);
}

/*****************************************************************************
 * pure message function define.
 *****************************************************************************/
void pure_msg_pool_init() {
	uint32_t index;

	ENTRY_CRITICAL();

	free_list_pure_msg_pool = (ak_msg_t*)msg_pure_pool;

	for (index = 0; index < AK_PURE_MSG_POOL_SIZE; index++) {
		msg_pure_pool[index].msg_header.ref_count |= PURE_MSG_TYPE;
		if (index == (AK_PURE_MSG_POOL_SIZE - 1)) {
			msg_pure_pool[index].msg_header.next = AK_MSG_NULL;
		}
		else {
			msg_pure_pool[index].msg_header.next = (ak_msg_t*)&msg_pure_pool[index + 1];
		}
	}

	EXIT_CRITICAL();
}

uint32_t get_pure_msg_pool_used() {
	return (AK_PURE_MSG_POOL_SIZE - get_pool_msg_free((ak_msg_t*)free_list_common_msg_pool));
}

ak_msg_t* get_pure_msg() {
	ak_msg_t* allocate_massage;

	ENTRY_CRITICAL();

	allocate_massage = free_list_pure_msg_pool;

	if (allocate_massage == AK_MSG_NULL) {
		FATAL("MF", 0x31);
	}
	else {
		free_list_pure_msg_pool = allocate_massage->next;
	}

	if (get_msg_type(allocate_massage) != PURE_MSG_TYPE) {
		FATAL("MF", 0x37);
	}

	msg_inc_ref_count(allocate_massage);

	allocate_massage->src_task_id = get_current_task_id();

	EXIT_CRITICAL();

	return allocate_massage;
}

void free_pure_msg(ak_msg_t* msg) {

	ENTRY_CRITICAL();

	msg->next = free_list_pure_msg_pool;
	free_list_pure_msg_pool = msg;

	EXIT_CRITICAL();
}

/*****************************************************************************
 * dynamic message function define.
 *****************************************************************************/
void dynamic_msg_pool_init() {
	uint32_t index;

	ENTRY_CRITICAL();

	/* init dynamic pool message */
	free_list_dynamic_msg_pool = (ak_msg_t*)msg_dynamic_pool;

	for (index = 0; index < AK_DYNAMIC_MSG_POOL_SIZE; index++) {
		msg_dynamic_pool[index].msg_header.ref_count |= DYNAMIC_MSG_TYPE;
		if (index == (AK_DYNAMIC_MSG_POOL_SIZE - 1)) {
			msg_dynamic_pool[index].msg_header.next = AK_MSG_NULL;
		}
		else {
			msg_dynamic_pool[index].msg_header.next = (ak_msg_t*)&msg_dynamic_pool[index + 1];
		}
	}

	/* init dynamic pool data */
	head_data_msg_dynamic_pool = (dynamic_pdu_t*)data_msg_dynamic_pool;
	for (index = 0; index < AK_DYNAMIC_DATA_POOL_SIZE; index++) {
		if (index == (AK_DYNAMIC_DATA_POOL_SIZE - 1)) {
			data_msg_dynamic_pool[index].next = NULL;
			tail_data_msg_dynamic_pool = (dynamic_pdu_t*)&data_msg_dynamic_pool[index];
		}
		else {
			data_msg_dynamic_pool[index].next = (dynamic_pdu_t*)&data_msg_dynamic_pool[index + 1];
		}
	}

	EXIT_CRITICAL();
}

void free_dynamic_msg(ak_msg_t* msg) {
	ak_msg_dynamic_t* ak_msg_dynamic = (ak_msg_dynamic_t*)msg;

	ENTRY_CRITICAL();

	/* free dynmic message pool */
	msg->next = free_list_dynamic_msg_pool;
	free_list_dynamic_msg_pool = msg;

	/* free data of dynamic message pool */
	if (ak_msg_dynamic->data != ((dynamic_pdu_t*)0)) {
		free_data_dynamic_pdu_pool(ak_msg_dynamic->data);
	}

	EXIT_CRITICAL();
}

ak_msg_t* get_dynamic_msg() {
	ak_msg_t* allocate_massage;

	ENTRY_CRITICAL();

	allocate_massage = free_list_dynamic_msg_pool;

	if (allocate_massage == AK_MSG_NULL) {
		FATAL("MF", 0x41);
	}
	else {
		free_list_dynamic_msg_pool = allocate_massage->next;
	}

	if (get_msg_type(allocate_massage) != DYNAMIC_MSG_TYPE) {
		FATAL("MF", 0x51);
	}

	/* increase referrence counter */
	msg_inc_ref_count(allocate_massage);

	/* update source task id */
	allocate_massage->src_task_id = get_current_task_id();

	/* assign NULL data message */
	((ak_msg_dynamic_t*)allocate_massage)->data = ((dynamic_pdu_t*)0);

	EXIT_CRITICAL();

	return allocate_massage;
}

uint32_t get_dynamic_msg_pool_used() {
	return (AK_DYNAMIC_MSG_POOL_SIZE - get_pool_msg_free((ak_msg_t*)free_list_dynamic_msg_pool));
}

uint8_t set_data_dynamic_msg(ak_msg_t* msg, uint8_t* data, uint32_t size) {
	ak_msg_dynamic_t* msg_dynamic = (ak_msg_dynamic_t*)msg;

	/* check messge null */
	if ((ak_msg_t*)msg_dynamic == AK_MSG_NULL) {
		FATAL("MF", 0x42);
	}

	/* check message type */
	if ((msg_dynamic->msg_header.ref_count & AK_MSG_TYPE_MASK) != DYNAMIC_MSG_TYPE) {
		FATAL("MF", 0x43);
	}

	/* set data message */
	msg_dynamic->len = size;

	msg_dynamic->data = get_data_dynamic_pdu_pool(size);
	set_data_dynamic_pdu(msg_dynamic->data, data, size);

	return AK_MSG_OK;
}

uint8_t get_data_dynamic_msg(ak_msg_t* msg, uint8_t* data, uint32_t size) {
	ak_msg_dynamic_t* msg_dynamic = (ak_msg_dynamic_t*)msg;
	/* check messge null */
	if ((ak_msg_t*)msg_dynamic == AK_MSG_NULL) {
		FATAL("MF", 0x45);
	}

	/* check message type */
	if ((msg_dynamic->msg_header.ref_count & AK_MSG_TYPE_MASK) != DYNAMIC_MSG_TYPE) {
		FATAL("MF", 0x46);
	}

	/* check length */
	if (msg_dynamic->len != size) {
		FATAL("MF", 0x47);
	}

	get_data_dynamic_pdu(msg_dynamic->data, data, size);

	return AK_MSG_OK;
}

uint32_t get_data_len_dynamic_msg(ak_msg_t* msg) {
	return ((ak_msg_dynamic_t*)msg)->len;
}

dynamic_pdu_t* get_data_dynamic_pdu_pool(uint32_t size) {
	uint32_t i, size_pdu;
	dynamic_pdu_t *ptemp, *phead;

	size_pdu = size / AK_DYNAMIC_PDU_SIZE;
	if ((size % AK_DYNAMIC_PDU_SIZE) != 0) {
		size_pdu++;
	}

	ENTRY_CRITICAL();

	phead = head_data_msg_dynamic_pool;
	ptemp = head_data_msg_dynamic_pool;

	for (i = 0; i < size_pdu; i++) {
		if (ptemp->next == NULL) {
			FATAL("MF", 0x48);
		}
		ptemp = ptemp->next;
	}

	head_data_msg_dynamic_pool = ptemp->next;
	ptemp->next = NULL;

	EXIT_CRITICAL();

	return phead;
}

uint8_t get_data_dynamic_pdu(dynamic_pdu_t* dynamic_pdu, uint8_t* data, uint32_t size) {
	dynamic_pdu_t *phead = dynamic_pdu;
	uint32_t i;

	for (i = 0; i < size; i++) {
		/* switch to next pdu */
		if ((i != 0) && (i % AK_DYNAMIC_PDU_SIZE == 0)) {
			if (phead->next == NULL) {
				FATAL("MF", 0x49);
			}

			phead = phead->next;
		}

		/* assign data */
		data[i] = phead->data_unit[i % AK_DYNAMIC_PDU_SIZE];
	}

	return AK_MSG_NG;
}

uint8_t set_data_dynamic_pdu(dynamic_pdu_t* dynamic_pdu, uint8_t* data, uint32_t size) {
	dynamic_pdu_t *ptemp = dynamic_pdu;
	uint32_t index = 0, pdu_size, pdu_remain;

	pdu_size = size / AK_DYNAMIC_PDU_SIZE;
	pdu_remain = size % AK_DYNAMIC_PDU_SIZE;

	while (index < pdu_size) {
		memcpy(ptemp->data_unit, (uint8_t*)&data[index * AK_DYNAMIC_PDU_SIZE], AK_DYNAMIC_PDU_SIZE);

		if (ptemp->next == NULL) {
			FATAL("MF", 0x50);
		}

		ptemp = ptemp->next;
		index++;
	}

	if (pdu_remain != 0) {
		memcpy(ptemp->data_unit, (uint8_t*)&data[index * AK_DYNAMIC_PDU_SIZE], pdu_remain);
	}

	return AK_MSG_OK;
}

void free_data_dynamic_pdu_pool(dynamic_pdu_t* dynamic_pdu) {

	ENTRY_CRITICAL();

	tail_data_msg_dynamic_pool->next = dynamic_pdu;

	while (tail_data_msg_dynamic_pool->next != NULL) {
		tail_data_msg_dynamic_pool = tail_data_msg_dynamic_pool->next;
	}

	EXIT_CRITICAL();
}

/*****************************************************************************
 * debug message function define.
 *****************************************************************************/
void msg_dbg_dum(ak_msg_t* msg) {
	xprintf("stid:%d dtid:%d rfc:%02X sig:%d Istid:%d Idtid:%d Ist:%d Idt:%d Isig:%d\n",	\
			msg->src_task_id,	\
			msg->des_task_id,	\
			msg->ref_count,		\
			msg->sig,			\
			msg->if_src_task_id,\
			msg->if_des_task_id,\
			msg->if_src_type,	\
			msg->if_des_type,	\
			msg->if_sig			\
			);
}
