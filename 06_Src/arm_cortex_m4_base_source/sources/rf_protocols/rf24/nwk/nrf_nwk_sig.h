#ifndef __NRF_NWK_SIG_H__
#define __NRF_NWK_SIG_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include "../../../ak/ak.h"

/*****************************************************************************/
/*  RF24_PHY task define
 */
/*****************************************************************************/
/* private define */
/* define timer */
/* define signal */
enum {
	RF24_PHY_INIT = AK_USER_DEFINE_SIG,
	RF24_PHY_IRQ_TX_MAX_RT,
	RF24_PHY_IRQ_TX_DS,
	RF24_PHY_IRQ_RX_DR,
	RF24_PHY_IRQ_ACK_PR,
	RF24_PHY_SEND_FRAME_REQ,
	RF24_PHY_IRQ_CLEAR_REQ,
	RF24_PHY_REV_MODE_REQ,
	RF24_PHY_SEND_MODE_REQ,
};

/*****************************************************************************/
/*  RF24_MAC task define
 */
/*****************************************************************************/
/* private define */
/* define timer */
/* define signal */
enum {
	RF24_MAC_INIT = AK_USER_DEFINE_SIG,
	RF24_MAC_HANDLE_MSG_OUT,
	RF24_MAC_SEND_FRAME,
	RF24_MAC_SEND_FRAME_DONE,
	RF24_MAC_SEND_FRAME_ERR,
	RF24_MAC_RECV_FRAME,
	RF24_MAC_RECV_FRAME_TO,
	RF24_MAC_HANDLE_MSG_IN,
};

/*****************************************************************************/
/*  RF24_NWK task define
 */
/*****************************************************************************/
/* private define */
/* define timer */
/* define signal */
enum {
	RF24_NWK_INIT = AK_USER_DEFINE_SIG,
	RF24_NWK_PDU_FULL,
	RF24_NWK_PURE_MSG_OUT,
	RF24_NWK_COMMON_MSG_OUT,
	RF24_NWK_DYNAMIC_MSG_OUT,
	RF24_NWK_DATA_MSG_OUT,
	RF24_NWK_SEND_MSG_DONE,
	RF24_NWK_RECV_MSG,
	RF24_NWK_SEND_MSG_ERR_SDF,
	RF24_NWK_SEND_MSG_ERR_BUSY,
};

#ifdef __cplusplus
}
#endif

#endif //__NRF_NWK_SIG_H__
