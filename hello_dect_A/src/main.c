/*
 * Board A — Originator
 * Sends MSG_DATA to B, waits for ACK from B.
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
    * These are used to know which board each message was transmitted from
 * ------------------------------------------------------------------ */
#define DEVICE_ID_A  1
#define DEVICE_ID_B  2
#define DEVICE_ID_C  3

#define DATA_LEN_MAX 64

// Message type identifiers
#define MSG_DATA 0x01
#define MSG_ACK  0x02

/*
 * Frame layout (all frames):
 *   Byte 0:   type                 (MSG_DATA or MSG_ACK)
 *   Byte 1:   seq                  (# for data packet. 1, 2, 3 ...)
 *   Byte 2:   sender_id high byte
 *   Byte 3:   sender_id low byte
 *   Byte 4+:  payload              (MSG_DATA only)
 */
#define FRAME_HDR_LEN  4

static bool exit;
static uint16_t device_id;
static uint64_t modem_time;

K_SEM_DEFINE(operation_sem, 0, 1);  // when operation_sem = 1 the main is allowed to run, otherwise sleep
K_SEM_DEFINE(deinit_sem,    0, 1);
K_SEM_DEFINE(ack_sem,       0, 1);  /* fired when valid MSG_ACK arrives from B */

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
    k_sem_give(&operation_sem); // enable main to run
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
    LOG_INF("A: Header from device %d",
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
 * on_pdc — A only expects MSG_ACK from B.
 * Validates sender_id before signalling ack_sem.
 */
static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt)
{
    if (evt->len < FRAME_HDR_LEN) {
        LOG_WRN("A: Frame too short (%d bytes), ignoring", evt->len);
        return;
    }

    const uint8_t *data = (const uint8_t *)evt->data;
    uint8_t  type      = data[0];
    uint8_t  seq       = data[1];
    uint16_t sender_id = ((uint16_t)data[2] << 8) | data[3];

    if (type == MSG_ACK) {
        if (sender_id != DEVICE_ID_B) {
            LOG_WRN("A: ACK from unexpected sender %d, ignoring", sender_id);
            return;
        }
        LOG_INF("A: ACK from B for seq %d", seq);
        k_sem_give(&ack_sem);
    } else {
        LOG_WRN("A: Unexpected frame type 0x%02x from sender %d", type, sender_id);
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
        .rssi_level              = -100,   // modular rssi level which computes what the gain should likely be
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
    uint8_t  tx_buf[DATA_LEN_MAX];
    uint8_t  seq = 0;

    LOG_INF("Board A — originator, starting");

    err = nrf_modem_lib_init();
    if (err) { LOG_ERR("modem init failed %d", err); return err; }

    err = nrf_modem_dect_phy_event_handler_set(dect_phy_event_handler);
    if (err) { LOG_ERR("handler set failed %d", err); return err; }

    err = nrf_modem_dect_phy_init();
    if (err) { LOG_ERR("phy init failed %d", err); return err; }
    k_sem_take(&operation_sem, K_FOREVER);  // wait for init to complete
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
    LOG_INF("A: Device ID: %d", device_id);

    while (1) {
        /* -------------------------------------------------------
         * Build frame:
         *   [MSG_DATA][seq][sender_id_hi][sender_id_lo][payload]
         * ------------------------------------------------------- */
        tx_buf[0] = MSG_DATA;
        tx_buf[1] = seq;
        tx_buf[2] = (device_id >> 8) & 0xff;
        tx_buf[3] = (device_id & 0xff);
        int payload_len = snprintf((char *)&tx_buf[FRAME_HDR_LEN],
                                   DATA_LEN_MAX - FRAME_HDR_LEN,
                                   "Hello %d", seq);
        size_t frame_len = FRAME_HDR_LEN + payload_len + 1; /* +1 for null terminator */

        /* -------------------------------------------------------
         * Retry loop — keep sending until B ACKs
         * ------------------------------------------------------- */
        while (1) {
            LOG_INF("A: Sending seq %d", seq);

            err = transmit(tx_handle, tx_buf, frame_len);
            if (err) {
                LOG_ERR("A: TX failed %d", err);
            }
            k_sem_take(&operation_sem, K_FOREVER);  // sleep until TX is done

            /* Open receive window to catch B's ACK */
            receive(rx_handle);

            // 
            if (k_sem_take(&ack_sem, K_SECONDS(5)) == 0) {  //ksem_take sucessfully returns 1
                /* ACK received — consume op_complete and move on */
                k_sem_take(&operation_sem, K_FOREVER);      // wait for recieve operation to completely finish
                LOG_INF("A: seq %d acknowledged by B", seq);
                break;
            }

            /* Timeout — consume op_complete and retransmit (receive will complete after 5 seconds no matter what
            This is set in the in the receive function */
            LOG_WRN("A: Timeout, retransmitting seq %d", seq);
            k_sem_take(&operation_sem, K_FOREVER);
        }

        seq++;
        k_sleep(K_SECONDS(1));
    }

    /* Unreachable under normal operation */
    nrf_modem_dect_phy_deactivate();
    k_sem_take(&deinit_sem, K_FOREVER);
    nrf_modem_dect_phy_deinit();
    k_sem_take(&deinit_sem, K_FOREVER);
    nrf_modem_lib_shutdown();

    return 0;
}