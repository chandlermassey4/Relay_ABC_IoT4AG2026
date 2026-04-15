/*
 * Board B — Relay Node
 * Receives MSG_DATA from A, forwards to C, waits for ACK from C, then ACKs A.
 *
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>

LOG_MODULE_REGISTER(app);

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

/* ------------------------------------------------------------------
 * Device IDs — must match CONFIG_DEVICE_ID in each board's prj.conf
 * ------------------------------------------------------------------ */
#define DEVICE_ID_A  1
#define DEVICE_ID_B  2
#define DEVICE_ID_C  3

#define DATA_LEN_MAX 64

#define MSG_DATA 0x01
#define MSG_ACK  0x02

/*
 * Frame layout (all frames):
 *   Byte 0:   type      (MSG_DATA or MSG_ACK)
 *   Byte 1:   seq
 *   Byte 2:   sender_id high byte
 *   Byte 3:   sender_id low byte
 *   Byte 4+:  payload (MSG_DATA only)
 */
#define FRAME_HDR_LEN  4   /* type + seq + sender_id (2 bytes) */

static bool exit;
static uint16_t device_id;
static uint64_t modem_time;

/* Relay buffer — holds the frame received from A for forwarding to C */
static uint8_t relay_buf[DATA_LEN_MAX];
static size_t  relay_len;
static uint8_t relay_seq;

K_SEM_DEFINE(operation_sem, 0, 1);
K_SEM_DEFINE(deinit_sem,    0, 1);
K_SEM_DEFINE(data_sem,      0, 1);  /* fired when valid MSG_DATA arrives from A */
K_SEM_DEFINE(ack_sem,       0, 1);  /* fired when valid MSG_ACK arrives from C  */

/* ---------- PHY header ---------- */
struct phy_ctrl_field_common {
    uint32_t packet_length      : 4;
    uint32_t packet_length_type : 1;
    uint32_t header_format      : 3;
    uint32_t short_network_id   : 8;
    uint32_t transmitter_id_hi  : 8;
    uint32_t transmitter_id_lo  : 8;
    uint32_t df_mcs             : 3;
    uint32_t reserved           : 1;
    uint32_t transmit_power     : 4;
    uint32_t pad                : 24;
};

/* ---------- forward declarations ---------- */
static int transmit(uint32_t handle, void *data, size_t data_len);
static int receive(uint32_t handle);

/* ---------- callbacks ---------- */
static void on_init(const struct nrf_modem_dect_phy_init_event *evt)
{
    if (evt->err) { LOG_ERR("Init failed %d", evt->err); exit = true; return; }
    k_sem_give(&operation_sem);
}

static void on_deinit(const struct nrf_modem_dect_phy_deinit_event *evt)
{
    if (evt->err) { LOG_ERR("Deinit failed %d", evt->err); return; }
    k_sem_give(&deinit_sem);
}

static void on_activate(const struct nrf_modem_dect_phy_activate_event *evt)
{
    if (evt->err) { LOG_ERR("Activate failed %d", evt->err); exit = true; return; }
    k_sem_give(&operation_sem);
}

static void on_deactivate(const struct nrf_modem_dect_phy_deactivate_event *evt)
{
    if (evt->err) { LOG_ERR("Deactivate failed %d", evt->err); return; }
    k_sem_give(&deinit_sem);
}

static void on_configure(const struct nrf_modem_dect_phy_configure_event *evt)
{
    if (evt->err) { LOG_ERR("Configure failed %d", evt->err); return; }
    k_sem_give(&operation_sem);
}

static void on_radio_config(const struct nrf_modem_dect_phy_radio_config_event *evt)
{
    if (evt->err) { LOG_ERR("Radio config failed %d", evt->err); return; }
    k_sem_give(&operation_sem);
}

static void on_op_complete(const struct nrf_modem_dect_phy_op_complete_event *evt)
{
    LOG_DBG("op_complete status %d", evt->err);
    k_sem_give(&operation_sem);
}

static void on_cancel(const struct nrf_modem_dect_phy_cancel_event *evt)
{
    LOG_DBG("cancel status %d", evt->err);
    k_sem_give(&operation_sem);
}

static void on_link_config(const struct nrf_modem_dect_phy_link_config_event *evt)
    { LOG_DBG("link_config status %d", evt->err); }

static void on_capability_get(const struct nrf_modem_dect_phy_capability_get_event *evt)
    { LOG_DBG("capability_get status %d", evt->err); }

static void on_bands_get(const struct nrf_modem_dect_phy_band_get_event *evt)
    { LOG_DBG("bands_get status %d", evt->err); }

static void on_latency_info_get(const struct nrf_modem_dect_phy_latency_info_event *evt)
    { LOG_DBG("latency_info status %d", evt->err); }

static void on_time_get(const struct nrf_modem_dect_phy_time_get_event *evt)
    { LOG_DBG("time_get status %d", evt->err); }

static void on_rssi(const struct nrf_modem_dect_phy_rssi_event *evt)
    { LOG_DBG("rssi carrier %d", evt->carrier); }

static void on_pcc(const struct nrf_modem_dect_phy_pcc_event *evt)
{
    LOG_INF("B: Header from device %d",
        evt->hdr.hdr_type_1.transmitter_id_hi << 8 |
        evt->hdr.hdr_type_1.transmitter_id_lo);
}

static void on_pcc_crc_err(const struct nrf_modem_dect_phy_pcc_crc_failure_event *evt)
    { LOG_DBG("pcc crc err"); }

static void on_pdc_crc_err(const struct nrf_modem_dect_phy_pdc_crc_failure_event *evt)
    { LOG_DBG("pdc crc err"); }

static void on_stf_cover_seq_control(const struct nrf_modem_dect_phy_stf_control_event *evt)
    { LOG_WRN("Unexpectedly in %s", __func__); }

static void on_test_rf_tx_cw_ctrl(const struct nrf_modem_dect_phy_test_rf_tx_cw_control_event *evt)
    { LOG_WRN("Unexpectedly in %s", __func__); }

/*
 * on_pdc — B handles two cases:
 *
 *   MSG_DATA from A:
 *     - Validate sender_id == DEVICE_ID_A
 *     - Copy frame into relay_buf
 *     - Overwrite sender_id field with B's own ID so C knows who forwarded it
 *     - Signal main() via data_sem to forward to C
 *
 *   MSG_ACK from C:
 *     - Validate sender_id == DEVICE_ID_C
 *     - Signal main() via ack_sem that C confirmed delivery
 *
 *   Anything else is logged and ignored.
 */
static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt)
{
    if (evt->len < FRAME_HDR_LEN) {
        LOG_WRN("B: Frame too short (%d bytes), ignoring", evt->len);
        return;
    }

    const uint8_t *data = (const uint8_t *)evt->data;
    uint8_t  type      = data[0];
    uint8_t  seq       = data[1];
    uint16_t sender_id = ((uint16_t)data[2] << 8) | data[3];

    if (type == MSG_DATA) {
        if (sender_id != DEVICE_ID_A) {
            LOG_WRN("B: MSG_DATA from unexpected sender %d, ignoring", sender_id);
            return;
        }

        /* Guard against oversized frames */
        if (evt->len > DATA_LEN_MAX) {
            LOG_WRN("B: Frame too large (%d bytes), ignoring", evt->len);
            return;
        }

        // Message from correct sender
        relay_len = evt->len;
        memcpy(relay_buf, data, relay_len);

        /* Replace sender_id in the copy with B's own ID.
         * C will see this frame as coming from B, not A. */
        relay_buf[2] = (device_id >> 8) & 0xff;
        relay_buf[3] = (device_id & 0xff);

        relay_seq = seq;

        LOG_INF("B: MSG_DATA from A, seq %d, payload: %.*s",
                seq, (int)(relay_len - FRAME_HDR_LEN), &data[FRAME_HDR_LEN]);

        k_sem_give(&data_sem);

    } else if (type == MSG_ACK) {
        if (sender_id != DEVICE_ID_C) {
            LOG_WRN("B: MSG_ACK from unexpected sender %d, ignoring", sender_id);
            return;
        }

        // Correct device id
        LOG_INF("B: MSG_ACK from C for seq %d", seq);
        k_sem_give(&ack_sem);

    } else {
        LOG_WRN("B: Unknown frame type 0x%02x from sender %d", type, sender_id);
    }
}

static void dect_phy_event_handler(const struct nrf_modem_dect_phy_event *evt)
{
    modem_time = evt->time;
    switch (evt->id) {
    case NRF_MODEM_DECT_PHY_EVT_INIT:         on_init(&evt->init);                         break;
    case NRF_MODEM_DECT_PHY_EVT_DEINIT:       on_deinit(&evt->deinit);                     break;
    case NRF_MODEM_DECT_PHY_EVT_ACTIVATE:     on_activate(&evt->activate);                 break;
    case NRF_MODEM_DECT_PHY_EVT_DEACTIVATE:   on_deactivate(&evt->deactivate);             break;
    case NRF_MODEM_DECT_PHY_EVT_CONFIGURE:    on_configure(&evt->configure);               break;
    case NRF_MODEM_DECT_PHY_EVT_RADIO_CONFIG: on_radio_config(&evt->radio_config);         break;
    case NRF_MODEM_DECT_PHY_EVT_COMPLETED:    on_op_complete(&evt->op_complete);           break;
    case NRF_MODEM_DECT_PHY_EVT_CANCELED:     on_cancel(&evt->cancel);                     break;
    case NRF_MODEM_DECT_PHY_EVT_RSSI:         on_rssi(&evt->rssi);                         break;
    case NRF_MODEM_DECT_PHY_EVT_PCC:          on_pcc(&evt->pcc);                           break;
    case NRF_MODEM_DECT_PHY_EVT_PCC_ERROR:    on_pcc_crc_err(&evt->pcc_crc_err);           break;
    case NRF_MODEM_DECT_PHY_EVT_PDC:          on_pdc(&evt->pdc);                           break;
    case NRF_MODEM_DECT_PHY_EVT_PDC_ERROR:    on_pdc_crc_err(&evt->pdc_crc_err);           break;
    case NRF_MODEM_DECT_PHY_EVT_TIME:         on_time_get(&evt->time_get);                 break;
    case NRF_MODEM_DECT_PHY_EVT_CAPABILITY:   on_capability_get(&evt->capability_get);     break;
    case NRF_MODEM_DECT_PHY_EVT_BANDS:        on_bands_get(&evt->band_get);                break;
    case NRF_MODEM_DECT_PHY_EVT_LATENCY:      on_latency_info_get(&evt->latency_get);      break;
    case NRF_MODEM_DECT_PHY_EVT_LINK_CONFIG:  on_link_config(&evt->link_config);           break;
    case NRF_MODEM_DECT_PHY_EVT_STF_CONFIG:
        on_stf_cover_seq_control(&evt->stf_cover_seq_control);                             break;
    case NRF_MODEM_DECT_PHY_EVT_TEST_RF_TX_CW_CONTROL_CONFIG:
        on_test_rf_tx_cw_ctrl(&evt->test_rf_tx_cw_control);                               break;
    }
}

/* ---------- PHY config ---------- */
static struct nrf_modem_dect_phy_config_params dect_phy_config_params = {
    .band_group_index       = ((CONFIG_CARRIER >= 525 && CONFIG_CARRIER <= 551)) ? 1 : 0,
    .harq_rx_process_count  = 4,
    .harq_rx_expiry_time_us = 5000000,
};

/* ---------- transmit ---------- */
static int transmit(uint32_t handle, void *data, size_t data_len)
{
    struct phy_ctrl_field_common header = {
        .header_format      = 0x0,
        .packet_length_type = 0x0,
        .packet_length      = 0x01,
        .short_network_id   = (CONFIG_NETWORK_ID & 0xff),
        .transmitter_id_hi  = (device_id >> 8),
        .transmitter_id_lo  = (device_id & 0xff),
        .transmit_power     = CONFIG_TX_POWER,
        .reserved           = 0,
        .df_mcs             = CONFIG_MCS,
    };
    struct nrf_modem_dect_phy_tx_params tx_op_params = {
        .start_time             = 0,
        .handle                 = handle,
        .network_id             = CONFIG_NETWORK_ID,
        .phy_type               = 0,
        .lbt_rssi_threshold_max = 0,
        .carrier                = CONFIG_CARRIER,
        .lbt_period             = NRF_MODEM_DECT_LBT_PERIOD_MAX,
        .phy_header             = (union nrf_modem_dect_phy_hdr *)&header,
        .data                   = data,
        .data_size              = data_len,
    };
    return nrf_modem_dect_phy_tx(&tx_op_params);
}

/* ---------- receive ---------- */
static int receive(uint32_t handle)
{
    struct nrf_modem_dect_phy_rx_params rx_op_params = {
        .start_time              = 0,
        .handle                  = handle,
        .network_id              = CONFIG_NETWORK_ID,
        .mode                    = NRF_MODEM_DECT_PHY_RX_MODE_SINGLE_SHOT,
        .rssi_interval           = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
        .link_id                 = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
        .rssi_level              = -100, // changed
        .carrier                 = CONFIG_CARRIER,
        .duration                = CONFIG_RX_PERIOD_S * MSEC_PER_SEC *
                                   NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ,
        .filter.short_network_id         = CONFIG_NETWORK_ID & 0xff,
        .filter.is_short_network_id_used = 1,
        .filter.receiver_identity        = 0,
    };
    return nrf_modem_dect_phy_rx(&rx_op_params);
}

/* ---------- main ---------- */
int main(void)
{
    int err;
    uint32_t tx_handle = 0;
    uint32_t rx_handle = 1;

    LOG_INF("Board B — relay node, starting");

    err = nrf_modem_lib_init();
    if (err) { LOG_ERR("modem init failed %d", err); return err; }

    err = nrf_modem_dect_phy_event_handler_set(dect_phy_event_handler);
    if (err) { LOG_ERR("handler set failed %d", err); return err; }

    err = nrf_modem_dect_phy_init();
    if (err) { LOG_ERR("phy init failed %d", err); return err; }
    k_sem_take(&operation_sem, K_FOREVER);
    if (exit) return -EIO;

    err = nrf_modem_dect_phy_configure(&dect_phy_config_params);
    if (err) { LOG_ERR("configure failed %d", err); return err; }
    k_sem_take(&operation_sem, K_FOREVER);
    if (exit) return -EIO;

    err = nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
    if (err) { LOG_ERR("activate failed %d", err); return err; }
    k_sem_take(&operation_sem, K_FOREVER);
    if (exit) return -EIO;

    /* Use manually assigned ID from prj.conf instead of hardware ID */
    device_id = CONFIG_DEVICE_ID;
    LOG_INF("B: Device ID: %d", device_id);

    while (1) {

    /* -------------------------------------------------------
     * STEP 1: Listen for MSG_DATA from A
     * ------------------------------------------------------- */
    LOG_INF("B: Waiting for data from A...");
    receive(rx_handle);
    k_sem_take(&data_sem, K_FOREVER);           
    k_sem_take(&operation_sem, K_FOREVER);      // block main execution until recieve gives it back

    /* -------------------------------------------------------
     * STEP 2: ACK A immediately after recieval (may need to keep sending acknowledgement until new seq number comes)
     * ------------------------------------------------------- */
    uint8_t ack_to_a[FRAME_HDR_LEN];
    ack_to_a[0] = MSG_ACK;
    ack_to_a[1] = relay_seq;
    ack_to_a[2] = (device_id >> 8) & 0xff;
    ack_to_a[3] = (device_id & 0xff);

    LOG_INF("B: ACKing A immediately for seq %d", relay_seq);
    err = transmit(tx_handle, ack_to_a, sizeof(ack_to_a));
    if (err) {
        LOG_ERR("B: TX ACK to A failed %d", err);
    }
    k_sem_take(&operation_sem, K_FOREVER);         // block main execution until transmit occurs

    /* -------------------------------------------------------
     * STEP 3: Forward frame to C, retry until C ACKs
     * ------------------------------------------------------- */
    while (1) {
        LOG_INF("B: Forwarding seq %d to C", relay_seq);

        err = transmit(tx_handle, relay_buf, relay_len);
        if (err) {
            LOG_ERR("B: TX to C failed %d", err);
        }
        k_sem_take(&operation_sem, K_FOREVER);

        /* Open a receive window to catch C's ACK */
        receive(rx_handle);

        if (k_sem_take(&ack_sem, K_SECONDS(5)) == 0) { // on_pdc gives(ack) for correct message so this will fail if give never occurs
            k_sem_take(&operation_sem, K_FOREVER);
            LOG_INF("B: C confirmed delivery, seq %d complete", relay_seq);
            break;
        }

        LOG_WRN("B: Timeout waiting for ACK from C, retransmitting");
        k_sem_take(&operation_sem, K_FOREVER);
    	}
	}   // return to waiting for new message from A

    /* Unreachable under normal operation */
    nrf_modem_dect_phy_deactivate();
    k_sem_take(&deinit_sem, K_FOREVER);
    nrf_modem_dect_phy_deinit();
    k_sem_take(&deinit_sem, K_FOREVER);
    nrf_modem_lib_shutdown();

    return 0;
}