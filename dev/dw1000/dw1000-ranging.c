/*
 * Copyright (c) 2017, University of Trento.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 3. The name of the author may not be used to endorse or promote
 *    products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file
 *      Contiki DW1000 Driver ranging module
 *
 * \author
 *      Timofei Istomin <tim.ist@gmail.com>
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "core/net/linkaddr.h"
#include "dev/radio.h"
#include "net/netstack.h"
/*---------------------------------------------------------------------------*/
#include "deca_device_api.h"
#include "deca_regs.h"
#include "dw1000-ranging.h"
#include "process.h"
#include "deca_range_tables.h"
/*---------------------------------------------------------------------------*/
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#if DW1000_RANGING_ENABLED

#define DEBUG 0
#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...) do {} while(0)
#endif

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

process_event_t ranging_event;
struct process *req_process;
static ranging_data_t ranging_data;
static int status;
static dw1000_rng_type_t rng_type;

typedef enum {
  S_WAIT_POLL, /* 0 */
  S_WAIT_SS1,  /* 1 */
  S_WAIT_DS1,  /* 2 */
  S_WAIT_DS2,  /* 3 */
  S_WAIT_DS3,  /* 4 */
  S_RANGING_DONE,      /* 5 */
  S_RANGING_DONE_MSG4, /* 6 */
  S_ABORT,     /* 7 */
  S_RESET      /* 8 */
} state_t;

static state_t state;
static state_t old_state;

/* Packet lengths for different messages (include the 2-byte CRC!) */
#define PKT_LEN_POLL 12

#define PKT_LEN_SS1 20

#define PKT_LEN_DS1 12
#define PKT_LEN_DS2 24
#define PKT_LEN_DS3 24

/* Packet types for different messages */
#define MSG_TYPE_SS0 0xE0
#define MSG_TYPE_SS1 0xE1

#define MSG_TYPE_DS0 0xD0
#define MSG_TYPE_DS1 0xD1
#define MSG_TYPE_DS2 0xD2
#define MSG_TYPE_DS3 0xD3

/* Indexes to access some of the fields in the frames defined above. */
#define IDX_SN 2
#define IDX_TYPE 9

/*SS*/
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14

/*DS*/
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18

#define DISTANCE_MSG_POLL_RX_IDX 10
#define DISTANCE_MSG_RESP_TX_IDX 14
#define DISTANCE_MSG_FINAL_RX_IDX 18

static uint8_t my_seqn;
static uint8_t recv_seqn;
static linkaddr_t ranging_with; /* the current ranging peer */

#define RX_BUF_LEN 24
static uint8_t tx_buf[RX_BUF_LEN];
static uint8_t rx_buf[RX_BUF_LEN];

typedef struct {
  /* SS and DS timeouts */
  uint32_t poll_rx_to_resp_tx_dly_uus;
  uint32_t poll_tx_to_resp_rx_dly_uus;
  uint16_t resp_rx_timeout_uus;

/* DS timeouts */
  uint32_t resp_tx_to_final_rx_dly_uus;
  uint32_t resp_rx_to_final_tx_dly_uus;
  uint16_t final_rx_timeout_uus;
  uint16_t distance_rx_timeout_uus;

  uint32_t finish_delay; /* assumes millisecond clock tick! */
} ranging_conf_t;

/* EXPERIMENTAL VALUE */
const ranging_conf_t ranging_conf_6M8 = {
/* SS and DS timeouts */
  .poll_rx_to_resp_tx_dly_uus = 450,
  .poll_tx_to_resp_rx_dly_uus = 0,
  .resp_rx_timeout_uus = 500,

/* DS timeouts */
  .resp_tx_to_final_rx_dly_uus = 0,
  .resp_rx_to_final_tx_dly_uus = 350,
  .final_rx_timeout_uus = 450,
  .distance_rx_timeout_uus = 400,

  .finish_delay = 1, /* assumes millisecond clock tick! */
};

const ranging_conf_t ranging_conf_110K = {
/* SS and DS timeouts */
  .poll_rx_to_resp_tx_dly_uus = 3000,
  .poll_tx_to_resp_rx_dly_uus = 0,
  .resp_rx_timeout_uus = 4000,

/* DS timeouts */
  .resp_tx_to_final_rx_dly_uus = 0,
  .resp_rx_to_final_tx_dly_uus = 3000,
  .final_rx_timeout_uus = 4500,
  .distance_rx_timeout_uus = 3500, /* 3000 kind of works, too */

  .finish_delay = 3, /* assumes millisecond clock tick! */
};

typedef struct ranging_ant_delay_t {
  uint16_t tx_ant_dly;
  uint16_t rx_ant_dly;
} ranging_ant_delay_t;

const ranging_ant_delay_t ranging_ant_ch2 = {
  /* ch. 2 (old demo, 110Kbps) */
  .tx_ant_dly = 16496,
  .rx_ant_dly = 16496,
};

const ranging_ant_delay_t ranging_ant_ch4 = {
  /* ch. 4 (Pablo's best, 6.8Mbps) */
  .tx_ant_dly = 16455,
  .rx_ant_dly = 16455,
};

const ranging_ant_delay_t ranging_ant_ch5 = {
  /* ch. 5, 6.8Mbps */
  .tx_ant_dly = 16436,
  .rx_ant_dly = 16436,
};

static ranging_conf_t ranging_conf;
static ranging_ant_delay_t ranging_ant_delay;
/*---------------------------------------------------------------------------*/
#define tx_buf_set_src() do { tx_buf[7] = linkaddr_node_addr.u8[1]; tx_buf[8] = linkaddr_node_addr.u8[0]; } while(0)
#define tx_buf_set_dst() do { tx_buf[5] = ranging_with.u8[1]; tx_buf[6] = ranging_with.u8[0]; } while(0)
#define tx_buf_set_dst_from_src() do { tx_buf[5] = rx_buf[7]; tx_buf[6] = rx_buf[8]; } while(0)
#define rx_buf_check_dst() (rx_buf[5] == linkaddr_node_addr.u8[1] && rx_buf[6] == linkaddr_node_addr.u8[0]) /* TODO: check also PANID */
#define rx_buf_check_src() (rx_buf[7] == ranging_with.u8[1] && rx_buf[8] == ranging_with.u8[0])
/*---------------------------------------------------------------------------*/
static inline uint64_t
get_rx_timestamp_u64(void)
{
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for(i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}
/*---------------------------------------------------------------------------*/
static inline uint64_t
get_tx_timestamp_u64(void)
{
  uint8_t ts_tab[5];
  uint64_t ts = 0;
  int i;
  dwt_readtxtimestamp(ts_tab);
  for(i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}
/*---------------------------------------------------------------------------*/
static inline void
msg_get_ts(uint8_t *ts_field, uint32_t *ts)
{
  int i;
  *ts = 0;
  for(i = 0; i < 4; i++) {
    *ts |= ts_field[i] << (i * 8);
  }
}
/*---------------------------------------------------------------------------*/
static inline void
msg_set_ts(uint8_t *ts_field, uint64_t ts)
{
  int i;
  for(i = 0; i < 4; i++) {
    ts_field[i] = (uint8_t)ts;
    ts >>= 8;
  }
}
/*---------------------------------------------------------------------------*/
PROCESS(dw1000_rng_process, "DW1000 dbg process");
/*---------------------------------------------------------------------------*/
void
dw1000_ranging_init()
{

#if DW1000_DATA_RATE == DWT_BR_6M8
  ranging_conf = ranging_conf_6M8;
  ranging_ant_delay = ranging_ant_ch4;
#else /* DW1000_DATA_RATE == DWT_BR_6M8 */
  ranging_conf = ranging_conf_110K;
  ranging_ant_delay = ranging_ant_ch2;
#endif

  /* Apply default antenna delay value. */
  dwt_setrxantennadelay(ranging_ant_delay.rx_ant_dly);
  dwt_settxantennadelay(ranging_ant_delay.tx_ant_dly);

  /* Fill in the constant part of the TX buffer */
  tx_buf[0] = 0x41;
  tx_buf[1] = 0x88;
  tx_buf[3] = IEEE802154_PANID & 0xff;
  tx_buf[4] = IEEE802154_PANID >> 8;

  process_start(&dw1000_rng_process, NULL);
  state = S_WAIT_POLL;
}
/*---------------------------------------------------------------------------*/
/* XXX now it works only with 2-byte addresses */
bool
dw1000_range_with(linkaddr_t *lladdr, dw1000_rng_type_t type)
{
  int8_t irq_status;
  bool ret;
  if(type != DW1000_RNG_SS && type != DW1000_RNG_DS) {
    return false;
  }

  if(!ranging_event) {
    return false; /* first call the init function */
  }
  if(req_process != PROCESS_NONE) {
    return false; /* already ranging */
  }
  irq_status = dw1000_disable_interrupt();

  if(state != S_WAIT_POLL) {
    ret = false;
    goto enable_interrupts;
  }

  dwt_forcetrxoff();

  ranging_with = *lladdr;
  ranging_data.distance = 0;
  ranging_data.status = 0;
  my_seqn++;
  rng_type = type;
  /* PRINTF("dwr: rng start %d type %d\n", my_seqn, rng_type); */

  /* Write frame data to DW1000 and prepare transmission. */
  tx_buf[IDX_SN] = my_seqn;
  tx_buf[IDX_TYPE] = (rng_type == DW1000_RNG_SS) ? MSG_TYPE_SS0 : MSG_TYPE_DS0;
  tx_buf_set_dst();
  tx_buf_set_src();

  /* Set expected response's delay and timeout.
   * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
  dwt_setrxaftertxdelay(ranging_conf.poll_tx_to_resp_rx_dly_uus);
  dwt_setrxtimeout(ranging_conf.resp_rx_timeout_uus);

  /* Write frame data to DW1000 and prepare transmission. */
  /* dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS); */
  dwt_writetxdata(PKT_LEN_POLL, tx_buf, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(PKT_LEN_POLL, 0, 1); /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
   * set by dwt_setrxaftertxdelay() has elapsed. */

  if(dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
    ret = false;
    goto enable_interrupts;
  }

  ret = true; /* the request is successful */
  state = (rng_type == DW1000_RNG_SS) ? S_WAIT_SS1 : S_WAIT_DS1;
  req_process = PROCESS_CURRENT();

enable_interrupts:
  dw1000_enable_interrupt(irq_status);
  /* PRINTF("dwr3: %d\n", my_seqn); */
  return ret;
}
/*---------------------------------------------------------------------------*/
/* Timestamps needed for SS computations */
uint32_t ss_poll_tx_ts, ss_resp_rx_ts, ss_poll_rx_ts, ss_resp_tx_ts;
/* Timestamps needed for DS computations */
uint32_t ds_poll_tx_ts, ds_resp_rx_ts, ds_final_tx_ts;
uint32_t ds_poll_rx_ts, ds_resp_tx_ts, ds_final_rx_ts;
/*---------------------------------------------------------------------------*/
/* Callback to process ranging good frame events
 */
void
dw1000_rng_ok_cb(const dwt_cb_data_t *cb_data)
{
  uint16_t pkt_len = cb_data->datalength;

  /* if(! (cb_data->rx_flags & DWT_CB_DATA_RX_FLAG_RNG)) {
   *  goto abort; // got a non-ranging packet, abort the ranging session
   * } */

  if(state == S_WAIT_POLL) {
    if(pkt_len != PKT_LEN_POLL) {
      status = 11;
      goto abort;
    }

    dwt_readrxdata(rx_buf, pkt_len - DW1000_CRC_LEN, 0);

    if(!rx_buf_check_dst()) {
      status = 12;
      goto abort;
    }
    if(rx_buf[IDX_TYPE] == MSG_TYPE_SS0) {  /* --- Single-sided poll --- */

      /* Timestamps of frames transmission/reception.
       * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
      uint64_t poll_rx_ts_64;
      uint64_t resp_tx_ts_64;
      uint32_t resp_tx_time;

      /* Retrieve poll reception timestamp. */
      poll_rx_ts_64 = get_rx_timestamp_u64();

      /* Compute final message transmission time. */
      resp_tx_time = (poll_rx_ts_64 + (ranging_conf.poll_rx_to_resp_tx_dly_uus * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
      resp_tx_ts_64 = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + ranging_ant_delay.tx_ant_dly;

      /* Write all timestamps in the final message. */
      msg_set_ts(&tx_buf[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts_64);
      msg_set_ts(&tx_buf[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts_64);

      /* Write and send the response message. */
      recv_seqn = rx_buf[IDX_SN];
      tx_buf[IDX_SN] = rx_buf[IDX_SN];
      tx_buf[IDX_TYPE] = MSG_TYPE_SS1;
      tx_buf_set_src();
      tx_buf_set_dst_from_src();

      dwt_writetxdata(PKT_LEN_SS1, tx_buf, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(PKT_LEN_SS1, 0, 1); /* Zero offset in TX buffer, ranging. */
      dwt_setrxaftertxdelay(0);
      dwt_setrxtimeout(0);
      int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
      if(ret != DWT_SUCCESS) {
        status = 14;
        goto abort;
      }
      return; /* In this case, no other processing nor state change are needed */
    } else if(rx_buf[IDX_TYPE] == MSG_TYPE_DS0) { /* --- Double-sided poll --- */
      uint32_t resp_tx_time;
      uint64_t poll_rx_ts_64;

      /* Retrieve poll and store poll reception timestamp. */
      poll_rx_ts_64 = get_rx_timestamp_u64();
      ds_poll_rx_ts = (uint32_t)poll_rx_ts_64;

      /* Set send time for response. See NOTE 9 below. */
      resp_tx_time = (poll_rx_ts_64 + (ranging_conf.poll_rx_to_resp_tx_dly_uus * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
      dwt_setrxaftertxdelay(ranging_conf.resp_tx_to_final_rx_dly_uus);
      dwt_setrxtimeout(ranging_conf.final_rx_timeout_uus);

      /* Write and send the response message. See NOTE 10 below.*/
      recv_seqn = rx_buf[IDX_SN];
      tx_buf[IDX_SN] = rx_buf[IDX_SN];
      tx_buf[IDX_TYPE] = MSG_TYPE_DS1;
      tx_buf_set_src();
      tx_buf_set_dst_from_src();
      ranging_with.u8[1] = rx_buf[7];
      ranging_with.u8[0] = rx_buf[8];

      dwt_writetxdata(PKT_LEN_DS1, tx_buf, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(PKT_LEN_DS1, 0, 1); /* Zero offset in TX buffer, ranging. */
      int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
      if(ret != DWT_SUCCESS) {
        status = 15;
        goto abort;
      }
      state = S_WAIT_DS2;
      return; /* done with this packet */
    } else {
      status = 13;
      goto abort;
    }
  } else if(state == S_WAIT_SS1) { /* --- We are waiting for the SS1 response ---- */
    if(pkt_len != PKT_LEN_SS1) {
      status = 1;
      goto abort;
    }

    dwt_readrxdata(rx_buf, pkt_len - DW1000_CRC_LEN, 0);

    if(!rx_buf_check_dst()) {
      status = 2;
      goto abort;
    }
    if(!rx_buf_check_src()) {
      status = 3;
      goto abort; /* got reply from a wrong node */
    }

    /* Check that the frame is the expected response */
    if(rx_buf[IDX_TYPE] != MSG_TYPE_SS1) {
      status = 4;
      goto abort;
    }
  
    /* Retrieve poll transmission and response reception timestamps. */
    ss_poll_tx_ts = dwt_readtxtimestamplo32();
    ss_resp_rx_ts = dwt_readrxtimestamplo32();

    /* Get timestamps embedded in response message. */
    msg_get_ts(&rx_buf[RESP_MSG_POLL_RX_TS_IDX], &ss_poll_rx_ts);
    msg_get_ts(&rx_buf[RESP_MSG_RESP_TX_TS_IDX], &ss_resp_tx_ts);

    state = S_RANGING_DONE;
    dwt_setrxtimeout(0);
    dwt_rxenable(0);
    goto finish;
  } else if(state == S_WAIT_DS1) { /* --- We are waiting for the DS1 response --- */
    if(pkt_len != PKT_LEN_DS1) {
      status = 41;
      goto abort;
    }

    dwt_readrxdata(rx_buf, pkt_len - DW1000_CRC_LEN, 0);

    if(!rx_buf_check_dst()) {
      status = 42;
      goto abort;
    }
    if(!rx_buf_check_src()) {
      status = 43;
      goto abort; /* got reply from a wrong node */
    }

    /* Check that the frame is the expected response */
    if(rx_buf[IDX_TYPE] != MSG_TYPE_DS1) {
      status = 44;
      goto abort;
    }

    uint64_t final_tx_ts_64;
    uint32_t final_tx_time;
    uint64_t poll_tx_ts_64, resp_rx_ts_64;

    /* Retrieve poll transmission and response reception timestamp. */
    poll_tx_ts_64 = get_tx_timestamp_u64();
    resp_rx_ts_64 = get_rx_timestamp_u64();

    tx_buf[IDX_TYPE] = MSG_TYPE_DS2;

    /* Compute final message transmission time. See NOTE 10 below. */
    final_tx_time = (resp_rx_ts_64 + (ranging_conf.resp_rx_to_final_tx_dly_uus * UUS_TO_DWT_TIME)) >> 8;
    dwt_setdelayedtrxtime(final_tx_time);

    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(ranging_conf.distance_rx_timeout_uus);

    /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
    final_tx_ts_64 = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + ranging_ant_delay.tx_ant_dly;

    ds_poll_tx_ts = (uint32_t)poll_tx_ts_64;
    ds_resp_rx_ts = (uint32_t)resp_rx_ts_64;
    ds_final_tx_ts = (uint32_t)final_tx_ts_64;

    /* Write all timestamps in the final message. */
    msg_set_ts(&tx_buf[FINAL_MSG_POLL_TX_TS_IDX], ds_poll_tx_ts);
    msg_set_ts(&tx_buf[FINAL_MSG_RESP_RX_TS_IDX], ds_resp_rx_ts);
    msg_set_ts(&tx_buf[FINAL_MSG_FINAL_TX_TS_IDX], ds_final_tx_ts);

    dwt_writetxdata(PKT_LEN_DS2, tx_buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(PKT_LEN_DS2, 0, 1); /* Zero offset in TX buffer, ranging. */
    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

    if(ret != DWT_SUCCESS) {
      status = 45;
      goto abort;
    }
    state = S_WAIT_DS3;
    return;
  } else if(state == S_WAIT_DS2) { /* --- We are waiting for the DS2 response --- */
    if(pkt_len != PKT_LEN_DS2) {
      status = 51;
      goto abort;
    }

    dwt_readrxdata(rx_buf, pkt_len - DW1000_CRC_LEN, 0);

    if(!rx_buf_check_dst()) {
      status = 52;
      goto abort;
    }
    if(!rx_buf_check_src()) {
      status = 53;
      goto abort; /* got reply from a wrong node */
    }

    /* Check that the frame is the expected response */
    if(rx_buf[IDX_TYPE] != MSG_TYPE_DS2) {
      status = 54;
      goto abort;
    }

    dwt_readrxdata(rx_buf, pkt_len - DW1000_CRC_LEN, 0);

    /* ds_poll_rx_ts was stored on the previous step */
    /* Retrieve response transmission and final reception timestamps. */
    ds_resp_tx_ts = (uint32_t)get_tx_timestamp_u64();
    ds_final_rx_ts = (uint32_t)get_rx_timestamp_u64();

    /* Get timestamps embedded in the final message. */
    msg_get_ts(&rx_buf[FINAL_MSG_POLL_TX_TS_IDX], &ds_poll_tx_ts);
    msg_get_ts(&rx_buf[FINAL_MSG_RESP_RX_TS_IDX], &ds_resp_rx_ts);
    msg_get_ts(&rx_buf[FINAL_MSG_FINAL_TX_TS_IDX], &ds_final_tx_ts);

    recv_seqn = rx_buf[IDX_SN];
    tx_buf[IDX_SN] = rx_buf[IDX_SN];
    tx_buf[IDX_TYPE] = MSG_TYPE_DS3;
    tx_buf_set_src();
    tx_buf_set_dst_from_src();

    /* Reply with timestamps */
    memcpy(&tx_buf[DISTANCE_MSG_POLL_RX_IDX], &ds_poll_rx_ts, 4);
    memcpy(&tx_buf[DISTANCE_MSG_RESP_TX_IDX], &ds_resp_tx_ts, 4);
    memcpy(&tx_buf[DISTANCE_MSG_FINAL_RX_IDX], &ds_final_rx_ts, 4);

    dwt_writetxdata(PKT_LEN_DS3, tx_buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(PKT_LEN_DS3, 0, 1); /* Zero offset in TX buffer, ranging. */
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

    /* done with this ranging session but still sending the 4th message */
    state = S_RANGING_DONE_MSG4;

    goto finish;
  } else if(state == S_WAIT_DS3) { /* --- We are waiting for the DS3 response --- */
    if(pkt_len != PKT_LEN_DS3) {
      status = 61;
      goto abort;
    }

    dwt_readrxdata(rx_buf, pkt_len - DW1000_CRC_LEN, 0);

    if(!rx_buf_check_dst()) {
      status = 62;
      goto abort;
    }
    if(!rx_buf_check_src()) {
      status = 63;
      goto abort; /* got reply from a wrong node */
    }

    /* Check that the frame is the expected response */
    if(rx_buf[IDX_TYPE] != MSG_TYPE_DS3) {
      status = 64;
      goto abort;
    }

    dwt_readrxdata(rx_buf, pkt_len - DW1000_CRC_LEN, 0);

    msg_get_ts(&rx_buf[DISTANCE_MSG_POLL_RX_IDX], &ds_poll_rx_ts);
    msg_get_ts(&rx_buf[DISTANCE_MSG_RESP_TX_IDX], &ds_resp_tx_ts);
    msg_get_ts(&rx_buf[DISTANCE_MSG_FINAL_RX_IDX], &ds_final_rx_ts);

    state = S_RANGING_DONE;
    dwt_setrxtimeout(0);
    dwt_rxenable(0);
    goto finish;
  }

abort: /* In case we got anything unexpected */
  old_state = state;
  state = S_ABORT;
  dwt_forcetrxoff();
  dwt_rxreset(); /* just to check */
  dwt_setrxtimeout(0);
  dwt_rxenable(0);
finish:
  process_poll(&dw1000_rng_process);
}
/*---------------------------------------------------------------------------*/
static double
ss_tof_calc()
{
  int32_t rtd_init, rtd_resp;
  /* Compute time of flight. */
  rtd_init = ss_resp_rx_ts - ss_poll_tx_ts;
  rtd_resp = ss_resp_tx_ts - ss_poll_rx_ts;
  return ((rtd_init - rtd_resp) / 2.0) * DWT_TIME_UNITS;
}
/*---------------------------------------------------------------------------*/
static double
ds_tof_calc()
{
  double Ra, Rb, Da, Db;
  int64_t tof_dtu;

  /* Compute time of flight.
   * 32-bit subtractions give correct answers even if clock has wrapped. */
  Ra = (double)(ds_resp_rx_ts - ds_poll_tx_ts);
  Rb = (double)(ds_final_rx_ts - ds_resp_tx_ts);
  Da = (double)(ds_final_tx_ts - ds_resp_rx_ts);
  Db = (double)(ds_resp_tx_ts - ds_poll_rx_ts);
  tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

  return (double)(tof_dtu * DWT_TIME_UNITS);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(dw1000_rng_process, ev, data)
{
  static struct etimer abort_timer;
  PROCESS_BEGIN();

  if(!ranging_event) {
    ranging_event = process_alloc_event();
  }

  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    PRINTF("dwr: my %d their %d, ost %d st %d ss %d\n", my_seqn, recv_seqn, old_state, state, status);
    old_state = 0;
    status = 0;
    if(state == S_RESET || state == S_ABORT) {
      ranging_data.status = 0;
    } else if(state == S_RANGING_DONE || state == S_RANGING_DONE_MSG4) {
      double tof;
      double not_corrected;

      if(rng_type == DW1000_RNG_SS) {
        tof = ss_tof_calc();
      } else {
        tof = ds_tof_calc();
      }

      not_corrected = tof * SPEED_OF_LIGHT;
      ranging_data.raw_distance = not_corrected;
#if DW1000_COMPENSATE_BIAS
      ranging_data.distance = not_corrected - dwt_getrangebias(DW1000_CHANNEL, not_corrected, DW1000_PRF);
#else
      ranging_data.distance = not_corrected;
#endif

      /* PRINTF("dwr: %d done %f, after bias %f\n", my_seqn, not_corrected, ranging_data.distance); */
      ranging_data.status = 1;
    }
    if(req_process != PROCESS_NONE) {
      if(ranging_data.status == 0 || state == S_RANGING_DONE_MSG4) {
        /* delay to let the 4th message be transmitted
         * or to let our peer timeout if we were interrupted
         * in the middle of a ranging sequence */
        etimer_set(&abort_timer, ranging_conf.finish_delay);
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&abort_timer));
      }
      process_post(req_process, ranging_event, &ranging_data);
      req_process = PROCESS_NONE;
    }
    state = S_WAIT_POLL;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
bool
dw1000_is_ranging()
{
  return state != S_WAIT_POLL;
}
/*---------------------------------------------------------------------------*/
/* Should be called with interrupts disabled */
void
dw1000_range_reset()
{
  switch(state) {
  case S_WAIT_SS1:
  case S_WAIT_DS1:
  case S_WAIT_DS2:
  case S_WAIT_DS3:
  /* case S_ABORT: */
  /* case S_RANGING_DONE: */
  /* case S_RANGING_DONE_MSG4: */
    old_state = state;
    state = S_RESET;
    process_poll(&dw1000_rng_process);
    break;
  default:
    break;
  }
}
/*---------------------------------------------------------------------------*/
int8_t
dw1000_range_reconfigure(dwt_config_t *config)
{
  switch(config->dataRate) {
  case DWT_BR_6M8:
    ranging_conf = ranging_conf_6M8;
    break;

  case DWT_BR_110K:
    ranging_conf = ranging_conf_110K;
    break;

  default:
    return -1; /* error: unsupported data rate */
  }

  switch(config->chan) {
  case 2:
    ranging_ant_delay = ranging_ant_ch2;
    break;

  case 4:
    ranging_ant_delay = ranging_ant_ch4;
    break;

  case 5:
    ranging_ant_delay = ranging_ant_ch5;
    break;

  default:
    return -2; /* error: unsupported channel */
  }

  return 0; /* success */
}
#endif /* DW1000_RANGING_ENABLED */
