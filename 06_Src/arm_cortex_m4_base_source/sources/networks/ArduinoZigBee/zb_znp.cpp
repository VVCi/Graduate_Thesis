#include <unistd.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sys_dbg.h"
#include "sys_ctrl.h"
#include "sys_io.h"

#include "sys_cfg.h"
#include "zb_znp.h"
#include "task_zigbee.h"

#include "task_sensor.h"
#include "task_test.h"

#include "io_cfg.h"
#include "app_dbg.h"

#define ZB_ZNP_EN		1

#if (ZB_ZNP_EN == 1)
#define ZB_ZNP(fmt, ...)		xprintf(fmt, "ZB_ZNP", ##__VA_ARGS__)
#else
#define ZB_ZNP(fmt, ...)
#endif

#define RX_BUFFER_SIZE		MAX_LEN_ZNP

#define znp_delay_ms sys_ctrl_delay_ms

uint8_t zb_znp::get_sequence_send() {
	if (m_sequence_send == 0xFF) {
		m_sequence_send = 0;
	} else {
		m_sequence_send++;
	}
	return m_sequence_send;
}

int polling_rev_buf_read_len;

uint8_t zb_znp::update() {
	uint8_t polling_rev_buf[RX_BUFFER_SIZE];
	polling_rev_buf_read_len = zread(polling_rev_buf, RX_BUFFER_SIZE);
	if (polling_rev_buf_read_len > 0) {
		znp_frame_parser(polling_rev_buf, polling_rev_buf_read_len);
	}
	return 0;
}

uint8_t zb_znp::update_irq(uint8_t byte) {
	return znp_frame_parser(byte);
}

int zb_znp::zwrite(uint8_t* data, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		io_uart_interface_transfer(*(data + i));
//		xprintf("%02X.", *(data + i));
	}
	return 0;
}

int zb_znp::zread(uint8_t* data, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		if (ring_buffer_char_is_empty(&ring_buffer_znp_rev) == false) {
			*(data + i) = ring_buffer_char_get(&ring_buffer_znp_rev);
			//xprintf("%02X ", *(data + i));
		}
		else {
			break;
		}
	}
	return 0;
}

void zb_znp::zdisable_rev_async() {

}

void zb_znp::zenable_rev_async() {

}

uint8_t zb_znp::set_tc_require_key_exchange(uint8_t bdb_trust_center_require_key_exchange) {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(APP_CNF_BDB_SET_TC_REQUIRE_KEY_EXCHANGE);
	i++;
	m_znp_buf[i] = LO_UINT16(APP_CNF_BDB_SET_TC_REQUIRE_KEY_EXCHANGE);
	i++;
	m_znp_buf[i] = bdb_trust_center_require_key_exchange;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();

	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(APP_CNF_BDB_SET_TC_REQUIRE_KEY_EXCHANGE_SRSP);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::znp_frame_parser(uint8_t* data, uint32_t len) {
	uint8_t ch, ret = ZNP_PARSER_IDLE;
	uint32_t rx_remain;

	while (len) {
		ch = *data++;
		len--;

		switch (znp_frame.state) {
		case SOP_STATE:
			if (SOF_CHAR == ch) {
				znp_frame.state = LEN_STATE;
				ret = ZNP_PARSER_DOING;
			}
			break;

		case LEN_STATE:
			ret = ZNP_PARSER_DOING;
			znp_frame.zigbee_msg.len = ch;
			znp_frame.zigbee_msg_data_index = 0;
			znp_frame.state = CMD0_STATE;
			break;

		case CMD0_STATE:
			ret = ZNP_PARSER_DOING;
			znp_frame.zigbee_msg.cmd0 = ch;
			znp_frame.state = CMD1_STATE;
			break;

		case CMD1_STATE:
			ret = ZNP_PARSER_DOING;
			znp_frame.zigbee_msg.cmd1 = ch;
			if (znp_frame.zigbee_msg.len) {
				znp_frame.state = DATA_STATE;
			} else {
				znp_frame.state = FCS_STATE;
			}
			break;

		case DATA_STATE: {
			ret = ZNP_PARSER_DOING;
			znp_frame.zigbee_msg.data[znp_frame.zigbee_msg_data_index++] = ch;

			rx_remain = znp_frame.zigbee_msg.len - znp_frame.zigbee_msg_data_index;

			if (len >= rx_remain) {
				memcpy((uint8_t*) (znp_frame.zigbee_msg.data + znp_frame.zigbee_msg_data_index), data, rx_remain);
				znp_frame.zigbee_msg_data_index += rx_remain;
				len -= rx_remain;
				data += rx_remain;
			} else {
				memcpy((uint8_t*) (znp_frame.zigbee_msg.data + znp_frame.zigbee_msg_data_index), data, len);
				znp_frame.zigbee_msg_data_index += len;
				len = 0;
			}

			if (znp_frame.zigbee_msg.len == znp_frame.zigbee_msg_data_index) {
				znp_frame.state = FCS_STATE;
			}
		}
			break;

		case FCS_STATE: {
			znp_frame.state = SOP_STATE;

			znp_frame.frame_fcs = ch;

			if (znp_frame.frame_fcs == znp_frame_calc_fcs(znp_frame.zigbee_msg)) {

				if (!znp_frame.zigbee_msg_denied_handle) {
					zigbee_message_handler(znp_frame.zigbee_msg);
				}

				ret = ZNP_PARSER_DONE;
			} else {
				/* TODO: handle checksum incorrect */
				ret = ZNP_PARSER_ERR;
			}
		}
			break;

		default:
			break;
		}
	}

	ret = ZNP_PARSER_DONE;
	return ret;
}

uint8_t zb_znp::znp_frame_parser(uint8_t data) {
	uint8_t ret = ZNP_PARSER_IDLE;
	switch (znp_frame.state) {
	case SOP_STATE:
		if (SOF_CHAR == data) {
			znp_frame.state = LEN_STATE;
			ret = ZNP_PARSER_DOING;
		}
		break;

	case LEN_STATE:
		ret = ZNP_PARSER_DOING;
		znp_frame.zigbee_msg.len = data;
		znp_frame.zigbee_msg_data_index = 0;
		znp_frame.state = CMD0_STATE;
		break;

	case CMD0_STATE:
		ret = ZNP_PARSER_DOING;
		znp_frame.zigbee_msg.cmd0 = data;
		znp_frame.state = CMD1_STATE;
		break;

	case CMD1_STATE:
		ret = ZNP_PARSER_DOING;
		znp_frame.zigbee_msg.cmd1 = data;
		if (znp_frame.zigbee_msg.len) {
			znp_frame.state = DATA_STATE;
		} else {
			znp_frame.state = FCS_STATE;
		}
		break;

	case DATA_STATE: {
		ret = ZNP_PARSER_DOING;
		znp_frame.zigbee_msg.data[znp_frame.zigbee_msg_data_index++] = data;

		if (znp_frame.zigbee_msg.len == znp_frame.zigbee_msg_data_index) {
			znp_frame.state = FCS_STATE;
		}
	}
		break;

	case FCS_STATE: {
		znp_frame.state = SOP_STATE;

		znp_frame.frame_fcs = data;

		if (znp_frame.frame_fcs == znp_frame_calc_fcs(znp_frame.zigbee_msg)) {

			if (!znp_frame.zigbee_msg_denied_handle) {
				zigbee_message_handler(znp_frame.zigbee_msg);
			}

			ret = ZNP_PARSER_DONE;
		} else {
			/* TODO: handle checksum incorrect */
			ret = ZNP_PARSER_ERR;
		}
	}
		break;

	default:
		break;
	}

	return ret;
}

uint8_t zb_znp::znp_frame_calc_fcs(zigbee_msg_t& zigbee_msg) {
	uint8_t i;
	uint8_t xorResult;
	xorResult = zigbee_msg.len ^ zigbee_msg.cmd0 ^ zigbee_msg.cmd1;
	for (i = 0; i < zigbee_msg.len; i++) {
		xorResult = xorResult ^ zigbee_msg.data[i];
	}
	return (xorResult);
}

uint8_t zb_znp::waiting_for_message(uint16_t cmd) {
	uint8_t waiting_for_message_buf[RX_BUFFER_SIZE];
	int waiting_for_message_buf_read_len;
	int retry_time = 50;
	znp_frame.zigbee_msg_denied_handle = 1;
	while(ring_buffer_char_is_empty(&ring_buffer_znp_rev));
	while (retry_time > 0) {
		waiting_for_message_buf_read_len = zread(waiting_for_message_buf, RX_BUFFER_SIZE);
		if (znp_frame_parser(waiting_for_message_buf, waiting_for_message_buf_read_len) == ZNP_PARSER_DONE) {
			if (BUILD_UINT16(znp_frame.zigbee_msg.cmd1, znp_frame.zigbee_msg.cmd0) == cmd) {
//				APP_PRINT("cmd: %04x\n", cmd);
				znp_frame.zigbee_msg_denied_handle = 0;
				return znp_frame.zigbee_msg.data[0];
			}
		}
		znp_delay_ms(1);
		retry_time--;
	}
	znp_frame.zigbee_msg_denied_handle = 0;
	return ZNP_RET_NG;
}

uint8_t zb_znp::waiting_for_status(uint16_t cmd, uint8_t status) {
	uint8_t waiting_for_status_buf[RX_BUFFER_SIZE];
	int waiting_for_status_buf_read_len;
	int retry_time = 50;
	znp_frame.zigbee_msg_denied_handle = 1;
	while(ring_buffer_char_is_empty(&ring_buffer_znp_rev));
	while (retry_time > 0) {
		waiting_for_status_buf_read_len = zread(waiting_for_status_buf, RX_BUFFER_SIZE);
		if (znp_frame_parser(waiting_for_status_buf, waiting_for_status_buf_read_len) == ZNP_PARSER_DONE) {
			if (BUILD_UINT16(znp_frame.zigbee_msg.cmd1, znp_frame.zigbee_msg.cmd0) == cmd &&
					znp_frame.zigbee_msg.data[0] == status) {
				znp_frame.zigbee_msg_denied_handle = 0;
				return znp_frame.zigbee_msg.data[0];
			}
		}
		znp_delay_ms(1);
		retry_time--;
	}
	znp_frame.zigbee_msg_denied_handle = 0;
	return ZNP_RET_NG;
}

uint8_t zb_znp::get_msg_return(uint16_t cmd, uint8_t status, uint8_t* rx_buffer, uint32_t* len) {
	int rx_read_len;
	int retry_time = 50;
	znp_frame.zigbee_msg_denied_handle = 1;
	while(ring_buffer_char_is_empty(&ring_buffer_znp_rev));
	while (retry_time > 0) {
		rx_read_len = zread(rx_buffer, RX_BUFFER_SIZE);
		if (znp_frame_parser(rx_buffer, rx_read_len) == ZNP_PARSER_DONE) {
			if (BUILD_UINT16(znp_frame.zigbee_msg.cmd1, znp_frame.zigbee_msg.cmd0) == cmd &&
					znp_frame.zigbee_msg.data[0] == status) {
				memcpy(rx_buffer, znp_frame.zigbee_msg.data, znp_frame.zigbee_msg.len);
				*len = znp_frame.zigbee_msg.len;

				znp_frame.zigbee_msg_denied_handle = 0;
				return ZNP_RET_OK;
			}
		}
		znp_delay_ms(1);
		retry_time--;
	}
	znp_frame.zigbee_msg_denied_handle = 0;
	return ZNP_RET_NG;
}

uint8_t zb_znp::calc_fcs(uint8_t* msg, uint8_t len) {
	uint8_t result = 0;
	while (len--) {
		result ^= *msg++;
	}
	return result;
}

uint8_t zb_znp::app_cnf_set_allowrejoin_tc_policy(uint8_t mode) {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(APP_CNF_SET_ALLOWREJOIN_TC_POLICY);
	i++;
	m_znp_buf[i] = LO_UINT16(APP_CNF_SET_ALLOWREJOIN_TC_POLICY);
	i++;
	m_znp_buf[i] = mode;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(APP_CNF_SET_ALLOWREJOIN_TC_POLICY | 0x6000);
	zenable_rev_async();

	return ret;
}

#define HARD_RESET				0x00
#define SOFT_RESET				0x01
uint8_t zb_znp::soft_reset() {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(SYS_RESET_REQ);
	i++;
	m_znp_buf[i] = LO_UINT16(SYS_RESET_REQ);
	i++;
	m_znp_buf[i] = SOFT_RESET;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(SYS_RESET_IND);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::set_startup_options(uint8_t opt) {
	uint8_t i = 0;
	if (opt > (STARTOPT_CLEAR_CONFIG + STARTOPT_CLEAR_STATE)) {
		return ZNP_RET_NG;
	}

	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_WRITE_CONFIGURATION);
	i++;

	m_znp_buf[i] = ZCD_NV_STARTUP_OPTION;
	i++;
	m_znp_buf[i] = ZCD_NV_STARTUP_OPTION_LEN;
	i++;
	m_znp_buf[i] = opt;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(ZB_WRITE_CONFIGURATION | 0x6000);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::set_panid(uint16_t pan_id) {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = ZCD_NV_PANID;
	i++;
	m_znp_buf[i] = ZCD_NV_PANID_LEN;
	i++;
	m_znp_buf[i] = LO_UINT16(pan_id);
	i++;
	m_znp_buf[i] = HI_UINT16(pan_id);
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(ZB_WRITE_CONFIGURATION | 0x6000);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::set_zigbee_device_type(uint8_t dev_type) {
	uint8_t i = 0;
	if (dev_type > END_DEVICE) {
		return ZNP_RET_NG;
	}
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_WRITE_CONFIGURATION);
	i++;

	m_znp_buf[i] = ZCD_NV_LOGICAL_TYPE;
	i++;
	m_znp_buf[i] = ZCD_NV_LOGICAL_TYPE_LEN;
	i++;
	m_znp_buf[i] = dev_type;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(ZB_WRITE_CONFIGURATION | 0x6000);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::set_transmit_power(uint8_t tx_power_db) {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(SYS_SET_TX_POWER);
	i++;
	m_znp_buf[i] = LO_UINT16(SYS_SET_TX_POWER);
	i++;
	m_znp_buf[i] = tx_power_db;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t znpCmdExeResult = waiting_for_message(SYS_SET_TX_POWER | 0x6000);
	zenable_rev_async();

	if (znpCmdExeResult == tx_power_db) {
		return ZNP_RET_OK;
	}
	return ZNP_RET_NG;
}

uint8_t zb_znp::set_channel_mask(uint8_t primary, uint32_t channelmask) {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(APP_CNF_BDB_SET_CHANNEL);
	i++;
	m_znp_buf[i] = LO_UINT16(APP_CNF_BDB_SET_CHANNEL);
	i++;

	m_znp_buf[i] = primary;
	i++;
	m_znp_buf[i] = LO_UINT16(channelmask);
	i++;
	m_znp_buf[i] = (channelmask & 0xFF00) >> 8;
	i++;
	m_znp_buf[i] = (channelmask & 0xFF0000) >> 16;
	i++;
	m_znp_buf[i] = channelmask >> 24;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(APP_CNF_BDB_SET_CHANNEL | 0x6000);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::af_register_generic_application(uint8_t endpoint) {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(AF_REGISTER);
	i++;
	m_znp_buf[i] = LO_UINT16(AF_REGISTER);
	i++;
	m_znp_buf[i] = endpoint;
	i++;
	m_znp_buf[i] = LO_UINT16(DEFAULT_PROFILE_ID);
	i++;
	m_znp_buf[i] = HI_UINT16(DEFAULT_PROFILE_ID);
	i++;
	m_znp_buf[i] = LO_UINT16(DEVICE_ID);
	i++;
	m_znp_buf[i] = HI_UINT16(DEVICE_ID);
	i++;
	m_znp_buf[i] = DEVICE_VERSION;
	i++;
	m_znp_buf[i] = LATENCY_NORMAL;
	i++;
	m_znp_buf[i] = 0;
	i++;				// number of binding input clusters
	m_znp_buf[i] = 0;
	i++;				// number of binding output clusters
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(AF_REGISTER | 0x6000);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::zb_app_register_request() {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_APP_REGISTER_REQUEST);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_APP_REGISTER_REQUEST);
	i++;
	m_znp_buf[i] = DEFAULT_ENDPOINT;
	i++;
	m_znp_buf[i] = LO_UINT16(DEFAULT_PROFILE_ID);
	i++;
	m_znp_buf[i] = HI_UINT16(DEFAULT_PROFILE_ID);
	i++;
	m_znp_buf[i] = LO_UINT16(DEVICE_ID);
	i++;
	m_znp_buf[i] = HI_UINT16(DEVICE_ID);
	i++;
	m_znp_buf[i] = DEVICE_VERSION;
	i++;
	m_znp_buf[i] = LATENCY_NORMAL;
	i++;
	m_znp_buf[i] = 0;
	i++;				// number of binding input clusters
	m_znp_buf[i] = 0;
	i++;				// number of binding output clusters

	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(ZB_APP_REGISTER_REQUEST | 0x6000);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::zb_start_request() {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_START_REQUEST);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_START_REQUEST);
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(ZB_START_CONFIRM);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::zdo_start_application() {
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZDO_STARTUP_FROM_APP);
	i++;
	m_znp_buf[i] = LO_UINT16(ZDO_STARTUP_FROM_APP);
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_status(ZDO_STATE_CHANGE_IND, 0x09);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::set_callbacks(uint8_t cb) {
	if ((cb != CALLBACKS_ENABLED) && (cb != CALLBACKS_DISABLED)) {
		return ZNP_RET_NG;
	}
	uint8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = ZCD_NV_ZDO_DIRECT_CB;
	i++;
	m_znp_buf[i] = ZCD_NV_ZDO_DIRECT_CB_LEN;
	i++;
	m_znp_buf[i] = cb;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(ZB_WRITE_CONFIGURATION | 0x6000);
	zenable_rev_async();

	return ret;
}

uint8_t  zb_znp::start_coordinator(uint8_t opt) {
	uint8_t znpCmdExeResult;
	uint8_t zb_read_configuration_buf[RX_BUFFER_SIZE];
	int zb_read_configuration_buf_len;

	ZB_ZNP("start_coordinator\n");
	znpCmdExeResult = soft_reset();
	if (znpCmdExeResult == ZNP_RET_NG) {
		ZB_ZNP("[Start Coodinator] ERR: soft_reset()\n");
		return znpCmdExeResult;
	}
	ZB_ZNP("[Start Coodinator] OK: soft_reset()\n");

	znpCmdExeResult = zb_read_configuration(ZCD_NV_PANID, zb_read_configuration_buf, (uint32_t*) &zb_read_configuration_buf_len);

	if (zb_read_configuration_buf_len >= 5 && \
			zb_read_configuration_buf[3] == 0xFF && \
			zb_read_configuration_buf[4] == 0xFF) {
		opt = 1;
	}

	APP_DBG("[Start Coodinator] opt = %d\n", opt);

	if (opt == 0) {
		ZB_ZNP("Skipping startup option !\n");
		for (int i = 1; i < 0x20; i++) {
			if (i == 0x01 || i == 0x0A) {
				znpCmdExeResult = af_register_generic_application(i);
				if (znpCmdExeResult != ZNP_RET_OK) {
					ZB_ZNP("[Start Coodinator] ERR: af_register_generic_application\n");
					return znpCmdExeResult;
				}
			}
		}
		ZB_ZNP("[Start Coodinator] OK: af_register_generic_application\n");

		znpCmdExeResult = bdb_start_commissioning(COMMISSIONING_MODE_STEERING, 1, 1);		// 0x02 is Network Steering
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: COMMISSIONING_MODE_STEERING\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: COMMISSIONING_MODE_STEERING\n");

	} else {
		znpCmdExeResult = set_startup_options(DEFAULT_STARTUP_OPTIONS);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: DEFAULT_STARTUP_OPTIONS\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("DEFAULT_STARTUP_OPTIONS\n");

		znpCmdExeResult = soft_reset();
		if (znpCmdExeResult == ZNP_RET_NG) {
			ZB_ZNP("[Start Coodinator] ERR: soft_reset()\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: soft_reset()");

		znpCmdExeResult = set_panid((uint16_t) PAN_ID);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: set_panid(%x)\n", PAN_ID);
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: set_panid(%x)\n", PAN_ID);

		znpCmdExeResult = set_zigbee_device_type(COORDINATOR);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: set_zigbee_device_type(COORDINATOR)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: set_zigbee_device_type(COORDINATOR)\n");

		//Set primary channel mask & disable secondary channel mask
		znpCmdExeResult = set_channel_mask(CHANNEL_TRUE, (uint32_t) DEFAULT_CHANNEL_MASK);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: set_channel_mask(CHANNEL_TRUE, DEFAULT_CHANNEL_MASK)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: set_channel_mask(CHANNEL_TRUE, DEFAULT_CHANNEL_MASK)\n");

		znpCmdExeResult = set_channel_mask(CHANNEL_FALSE, (uint32_t) 0);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: set_channel_mask(CHANNEL_FALSE)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: set_channel_mask(CHANNEL_FALSE)\n");

		znpCmdExeResult = app_cnf_set_allowrejoin_tc_policy(1);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: app_cnf_set_allowrejoin_tc_policy(1)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: app_cnf_set_allowrejoin_tc_policy(1)\n");

		znpCmdExeResult = set_transmit_power(DEFAULT_TX_POWER);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: set_transmit_power(DEFAULT_TX_POWER)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: set_transmit_power(DEFAULT_TX_POWER)\n");

		// Set ZCD_NV_ZDO_DIRECT_CB
		znpCmdExeResult = set_callbacks(CALLBACKS_ENABLED);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: set_callbacks(CALLBACKS_ENABLED)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: set_callbacks(CALLBACKS_ENABLED)\n");

		for (int i = 1; i < 0x20; i++) {
			if (i == 0x01 || i == 0x0A) {
				znpCmdExeResult = af_register_generic_application(i);
				if (znpCmdExeResult != ZNP_RET_OK) {
					ZB_ZNP("[Start Coodinator] ERR: af_register_generic_application(%d)\n", i);
					return znpCmdExeResult;
				}
			}
		}
		ZB_ZNP("[Start Coodinator] OK: af_register_generic_application()\n");
		// Start commissioning using network formation as parameter to start coordinator
		znpCmdExeResult = bdb_start_commissioning(COMMISSIONING_MODE_INFORMATION, 1, 1);		// 0x04 is Network Formation
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Coodinator] ERR: COMMISSIONING_MODE_INFORMATION\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Coodinator] OK: COMMISSIONING_MODE_INFORMATION\n");
	}

//	znpCmdExeResult = set_tc_require_key_exchange(0);
//	if (znpCmdExeResult != ZNP_RET_OK) {
//		ZB_ZNP("[Start Coodinator] ERR: set_tc_require_key_exchange(1)\n");
//		return znpCmdExeResult;
//	}
//	ZB_ZNP("[Start Coodinator] OK: set_tc_require_key_exchange(0)\n");

	znpCmdExeResult = set_permit_joining_req(ALL_ROUTER_AND_COORDINATOR, ZB_COODINATOR_DISABLE_PERMIT_JOIN, 1);
	if (znpCmdExeResult != ZNP_RET_OK) {
		ZB_ZNP("[Start Coodinator] ERR: set_permit_joining_req(ALL_ROUTER_AND_COORDINATOR, ZB_COODINATOR_DISABLE_PERMIT_JOIN, 1)\n");
		return znpCmdExeResult;
	}
	ZB_ZNP("[Start Coodinator] OK: set_permit_joining_req(ALL_ROUTER_AND_COORDINATOR, ZB_COODINATOR_DISABLE_PERMIT_JOIN, 1)\n");

	/*get MAC address of coordinator*/
	znpCmdExeResult = get_mac_addr_req(SHORT_ADDRESS_COORDINATOR, 0x00, 0x00);
	if (znpCmdExeResult != ZNP_RET_OK) {
		ZB_ZNP("[Start Coodinator] ERR: get_mac_addr_req(ALL_ROUTER_AND_COORDINATOR, 0x00, 0x00);\n");
		return znpCmdExeResult;
	}
	ZB_ZNP("[Start Coodinator] OK: get_mac_addr_req(ALL_ROUTER_AND_COORDINATOR, 0x00, 0x00);\n");

	return ZNP_RET_OK;
}

uint8_t  zb_znp::start_router(uint8_t opt) {
	uint8_t znpCmdExeResult;
	uint8_t rx_buffer[RX_BUFFER_SIZE];
	int rx_read_len;

	ZB_ZNP("start_router()\n");

	znpCmdExeResult = zb_read_configuration(ZCD_NV_PANID, rx_buffer, (uint32_t*) &rx_read_len);

	ZB_ZNP("[Start Router] opt = %d\n", opt);

	if (opt == 0) {
		ZB_ZNP("Skipping startup option !\n");
		for (int i = 1; i < 0x20; i++) {
			if (i == 0x01 || i == 0x0A) {
				znpCmdExeResult = af_register_generic_application(i);
				if (znpCmdExeResult != ZNP_RET_OK) {
					ZB_ZNP("[Start Router] ERR: af_register_generic_application\n");
					return znpCmdExeResult;
				}
			}
		}
		znpCmdExeResult = bdb_start_commissioning(COMMISSIONING_MODE_STEERING, 1, 0);		// 0x02 is Network Steering
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Router] OK: af_register_generic_application\n");
			return znpCmdExeResult;
		}
	}
	else {
		znpCmdExeResult = set_startup_options(DEFAULT_STARTUP_OPTIONS);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Router] ERR: set_startup_options(DEFAULT_STARTUP_OPTIONS)\n");

			if (zigbee_error_flag == 1) {
				sys_ctrl_delay_ms(10);
				sys_ctrl_reset();
			}

			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Router] OK: set_startup_options(DEFAULT_STARTUP_OPTIONS)\n");

		znpCmdExeResult = soft_reset();
		if (znpCmdExeResult == ZNP_RET_NG) {
			ZB_ZNP("[Start Router] ERR: set_reset()\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Router] OK: set_reset()\n");

		znpCmdExeResult = set_zigbee_device_type(ROUTER);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Router] ERR: set_zigbee_device_type(ROUTER)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Router] OK: set_zigbee_device_type(ROUTER)\n");

		znpCmdExeResult = soft_reset();
		if (znpCmdExeResult == ZNP_RET_NG) {
			ZB_ZNP("[Start Router] ERR: set_reset()\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Router] OK: set_reset()\n");

		//Set primary channel mask & disable secondary channel mask
		znpCmdExeResult = set_channel_mask(CHANNEL_TRUE, (uint32_t) DEFAULT_CHANNEL_MASK);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Router] ERR: set_channel_mask(CHANNEL_TRUE, (uint32_t) DEFAULT_CHANNEL_MASK)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Router] OK: set_channel_mask(CHANNEL_TRUE, (uint32_t) DEFAULT_CHANNEL_MASK)\n");

		znpCmdExeResult = set_channel_mask(CHANNEL_FALSE, (uint32_t) 0);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Router] ERR: set_channel_mask(CHANNEL_FALSE, (uint32_t) 0)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Router] OK: set_channel_mask(CHANNEL_FALSE, (uint32_t) 0)\n");

		for (int i = 1; i < 0x20; i++) {
			if (i == 0x01 || i == 0x0A) {
				znpCmdExeResult = af_register_generic_application(i);
				if (znpCmdExeResult != ZNP_RET_OK) {
					ZB_ZNP("[Start Router] ERR: af_register_generic_application(%d)\n", i);
					return znpCmdExeResult;
				}
			}
		}
		ZB_ZNP("[Start Router] OK: af_register_generic_application()\n");

		znpCmdExeResult = bdb_start_commissioning(COMMISSIONING_MODE_STEERING, 1, 10);
		if (znpCmdExeResult != ZNP_RET_OK) {
			ZB_ZNP("[Start Router] ERR: bdb_start_commissioning(COMMISSIONING_MODE_STEERING, 1, 10)\n");
			return znpCmdExeResult;
		}
		ZB_ZNP("[Start Router] OK: bdb_start_commissioning(COMMISSIONING_MODE_STEERING, 1, 10)\n");
	}
	return ZNP_RET_OK;
}

uint8_t zb_znp::bdb_start_commissioning(uint8_t mode_config, uint8_t mode_receiving, uint8_t flag_waiting) {
	(void)mode_receiving;
	int8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(APP_CNF_BDB_START_COMMISSIONING);
	i++;
	m_znp_buf[i] = LO_UINT16(APP_CNF_BDB_START_COMMISSIONING);
	i++;
	m_znp_buf[i] = mode_config;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

//	uint8_t ret = ZNP_RET_OK;
	if (flag_waiting == 0) {
		return waiting_for_status(APP_CNF_BDB_COMMISSIONING_NOTIFICATION, ZNP_RET_OK);
	}
	zenable_rev_async();
	return ZNP_RET_OK;
}

uint8_t zb_znp::util_get_device_info() {
	int8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(UTIL_GET_DEVICE_INFO);
	i++;
	m_znp_buf[i] = LO_UINT16(UTIL_GET_DEVICE_INFO);
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	if (zwrite(m_znp_buf, i) < 0) {
		return ZNP_RET_NG;
	}

	return ZNP_RET_OK;
}

uint8_t zb_znp::zdo_mgmt_leave_req(uint16_t short_add, uint8_t ieee_addr[8], uint8_t flags) {
	int8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZDO_MGMT_LEAVE_REQ);
	i++;
	m_znp_buf[i] = LO_UINT16(ZDO_MGMT_LEAVE_REQ);
	i++;
	m_znp_buf[i] = LO_UINT16(short_add);
	i++;
	m_znp_buf[i] = HI_UINT16(short_add);
	i++;
	memcpy((uint8_t*) &m_znp_buf[i], ieee_addr, 8);
	i += 8;
	m_znp_buf[i] = flags;
	i++;

	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	if (zwrite(m_znp_buf, i) < 0) {
		return ZNP_RET_NG;
	}

	return ZNP_RET_OK;
}

uint8_t zb_znp::set_permit_joining_req(uint16_t short_addr, uint8_t timeout, uint8_t flag_waiting) {
	int8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_PERMIT_JOINING_REQUEST);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_PERMIT_JOINING_REQUEST);
	i++;
	m_znp_buf[i] = LO_UINT16(short_addr);
	i++;
	m_znp_buf[i] = HI_UINT16(short_addr);
	i++;
	m_znp_buf[i] = timeout;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = ZNP_RET_OK;
	if (flag_waiting == 0) {
		ret =  waiting_for_message(ZDO_MGMT_PERMIT_JOIN_RSP);
	}
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::get_mac_addr_req(uint16_t short_addr, uint8_t req_type, uint8_t start_index) {
	int8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZDO_IEEE_ADDR_REQ);
	i++;
	m_znp_buf[i] = LO_UINT16(ZDO_IEEE_ADDR_REQ);
	i++;
	m_znp_buf[i] = LO_UINT16(short_addr);
	i++;
	m_znp_buf[i] = HI_UINT16(short_addr);
	i++;
	m_znp_buf[i] = req_type;
	i++;
	m_znp_buf[i] = start_index;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	if (zwrite(m_znp_buf, i) < 0) {
		return ZNP_RET_NG;
	}

	return ZNP_RET_OK;
}

uint8_t zb_znp::send_af_data_req(af_data_request_t af_data_request) {
	int32_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(AF_DATA_REQUEST);
	i++;
	m_znp_buf[i] = LO_UINT16(AF_DATA_REQUEST);
	i++;
	m_znp_buf[i] = LO_UINT16(af_data_request.dst_address);
	i++;
	m_znp_buf[i] = HI_UINT16(af_data_request.dst_address);
	i++;
	m_znp_buf[i] = af_data_request.dst_endpoint;
	i++;
	m_znp_buf[i] = af_data_request.src_endpoint;
	i++;
	m_znp_buf[i] = HI_UINT16(af_data_request.cluster_id);
	i++;
	m_znp_buf[i] = LO_UINT16(af_data_request.cluster_id);
	i++;
	m_znp_buf[i] = af_data_request.trans_id;
	i++;
	m_znp_buf[i] = af_data_request.options;
	i++;
	m_znp_buf[i] = af_data_request.radius;
	i++;
	m_znp_buf[i] = af_data_request.len;
	i++;
	memcpy((uint8_t*) &m_znp_buf[i], af_data_request.data, af_data_request.len);
	if (m_znp_buf[i] & 0x04) {
		m_znp_buf[i + 3] = get_sequence_send();
	} else {
		m_znp_buf[i + 1] = get_sequence_send();
	}
	i += af_data_request.len;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	if (zwrite(m_znp_buf, i) < 0) {
		return ZNP_RET_NG;
	}

	return ZNP_RET_OK;
}

uint8_t zb_znp::set_security_mode(uint8_t security_mode) {
	if (security_mode > SECURITY_MODE_COORD_DIST_KEYS) {
		return ZNP_RET_NG;
	}
	int32_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = ZCD_NV_SECURITY_MODE;
	i++;
	m_znp_buf[i] = ZCD_NV_SECURITY_MODE_LEN;
	i++;
	m_znp_buf[i] = (security_mode > 0);
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = ZNP_RET_NG;
	ret = waiting_for_message(ZB_WRITE_CONFIGURATION | 0x6000);

	if ((security_mode != SECURITY_MODE_OFF) && ret != ZNP_RET_NG) {
		i = 0;
		m_znp_buf[i] = ZNP_SOF;
		i++;
		m_znp_buf[i] = 0;
		i++;
		m_znp_buf[i] = HI_UINT16(ZB_WRITE_CONFIGURATION);
		i++;
		m_znp_buf[i] = LO_UINT16(ZB_WRITE_CONFIGURATION);
		i++;
		m_znp_buf[i] = ZCD_NV_PRECFGKEYS_ENABLE;
		i++;
		m_znp_buf[i] = ZCD_NV_PRECFGKEYS_ENABLE_LEN;
		i++;
		m_znp_buf[i] = (security_mode == SECURITY_MODE_PRECONFIGURED_KEYS);
		i++;
		m_znp_buf[1] = i - 4;
		m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
		i++;

		if (zwrite(m_znp_buf, i) < 0) {
			ret = ZNP_RET_NG;
		}
		else {
			ret = waiting_for_message(ZB_WRITE_CONFIGURATION | 0x6000);
		}
	}

	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::set_security_key(uint8_t* key) {
	int32_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_WRITE_CONFIGURATION);
	i++;
	m_znp_buf[i] = ZCD_NV_PRECFGKEY;
	i++;
	m_znp_buf[i] = ZCD_NV_PRECFGKEY_LEN;
	i++;
	memcpy((uint8_t*) &m_znp_buf[i], key, ZCD_NV_PRECFGKEY_LEN);
	i += ZCD_NV_PRECFGKEY_LEN;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = waiting_for_message(ZB_WRITE_CONFIGURATION | 0x6000);
	zenable_rev_async();

	return ret;
}

uint8_t zb_znp::zb_get_device_info(uint8_t param, uint8_t* rx_buffer, uint32_t* len) {
	uint32_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_GET_DEVICE_INFO);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_GET_DEVICE_INFO);
	i++;
	m_znp_buf[i] = param;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = get_msg_return((ZB_GET_DEVICE_INFO | 0x6000), param, rx_buffer, len);
	zenable_rev_async();
	return ret;
}

uint8_t zb_znp::zb_read_configuration(uint8_t config_id, uint8_t* rx_buffer, uint32_t* len) {
	int8_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZB_READ_CONFIGURATION);
	i++;
	m_znp_buf[i] = LO_UINT16(ZB_READ_CONFIGURATION);
	i++;
	m_znp_buf[i] = config_id;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	zdisable_rev_async();
	if (zwrite(m_znp_buf, i) < 0) {
		zenable_rev_async();
		return ZNP_RET_NG;
	}

	uint8_t ret = get_msg_return(ZB_READ_CONFIGURATION_RSP, ZNP_RET_OK, rx_buffer, len);
	zenable_rev_async();

	return  ret;
}

uint8_t zb_znp::zdo_binding_req(binding_req_t binding_req) {
	int32_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZDO_BIND_REQ);
	i++;
	m_znp_buf[i] = LO_UINT16(ZDO_BIND_REQ);
	i++;
	m_znp_buf[i] = binding_req.src_short_addr[0];
	i++;
	m_znp_buf[i] = binding_req.src_short_addr[1];
	i++;
	memcpy((uint8_t*) &m_znp_buf[i], (uint8_t*) binding_req.src_ieee_addr, 8);
	i += 8;
	m_znp_buf[i] = binding_req.src_endpoint;
	i++;
	m_znp_buf[i] = LO_UINT16(binding_req.cluster_id);
	i++;
	m_znp_buf[i] = HI_UINT16(binding_req.cluster_id);
	i++;
	m_znp_buf[i] = binding_req.dst_mode;
	i++;
	if (binding_req.dst_mode == ADDRESS_64_BIT) {
		memcpy((uint8_t*) &m_znp_buf[i], (uint8_t*) binding_req.dst_address, 8);
		i += 8;
	} else {
		memcpy((uint8_t*) &m_znp_buf[i], (uint8_t*) binding_req.dst_address, 2);
		i += 2;
	}
	m_znp_buf[i] = binding_req.dst_endpoint;
	i++;

	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	if (zwrite(m_znp_buf, i) < 0) {
		return ZNP_RET_NG;
	}

	return ZNP_RET_OK;
}

uint8_t zb_znp::zdo_simple_desc_req(uint16_t dst_addr, uint8_t dst_enpoint) {
	int32_t i = 0;
	m_znp_buf[i] = ZNP_SOF;
	i++;
	m_znp_buf[i] = 0;
	i++;
	m_znp_buf[i] = HI_UINT16(ZDO_SIMPLE_DESC_REQ);
	i++;
	m_znp_buf[i] = LO_UINT16(ZDO_SIMPLE_DESC_REQ);
	i++;
	m_znp_buf[i] = LO_UINT16(dst_addr); //dst_addr
	i++;
	m_znp_buf[i] = HI_UINT16(dst_addr); // dst_addr
	i++;
	m_znp_buf[i] = LO_UINT16(dst_addr); // NWKAddrOfInterest
	i++;
	m_znp_buf[i] = HI_UINT16(dst_addr); //NWKAddrOfInterest
	i++;
	m_znp_buf[i] = dst_enpoint;
	i++;
	m_znp_buf[1] = i - 4;
	m_znp_buf[i] = calc_fcs((uint8_t *) &m_znp_buf[1], (i - 1));
	i++;

	if (zwrite(m_znp_buf, i) < 0) {
		return ZNP_RET_NG;
	}

	return ZNP_RET_OK;
}
