/*
 * Board C — Sink Node
 * Receives MSG_DATA from B, logs it, and sends ACK back to B.
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

#define DEVICE_ID_B  2
#define DEVICE_ID_C  3

#define MSG_DATA 0x01
#define MSG_ACK  0x02

/*
 * Frame layout (all frames):
 *   Byte 0:   type      (MSG_DATA or MSG_ACK)
 *   Byte 1:   seq
 *   Byte 2:   sender_id high byte
 */
#define FRAME_HDR_LEN 4

static bool exit;
static uint16_t device_id;
static uint64_t modem_time;
static uint8_t last_seq;

K_SEM_DEFINE(operation_sem, 0, 1);
K_SEM_DEFINE(deinit_sem,    0, 1);
K_SEM_DEFINE(data_sem,      0, 1);

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
static void on_init(const struct nrf_modem_dect_phy_init_event *evt) // this is where failure is occuring
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
/*
static void on_rssi(const struct nrf_modem_dect_phy_rssi_event *evt)
{
    LOG_INF("RSSI event: carrier=%d RSSI=%d dBm time=%llu",
            evt->carrier,
            evt->rssi,
            (unsigned long long)modem_time);
}
*/

static void on_pcc(const struct nrf_modem_dect_phy_pcc_event *evt)
{
    LOG_INF("C: Header from device %d",
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

// Physical data channel (PDC) NEW code here
static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt)
{
    if (evt->len < FRAME_HDR_LEN) {
        LOG_WRN("C: Frame too short (%d bytes), ignoring", evt->len);
        return;
    }

    /* ---- RSSI extraction (same as hello_dect) ---- */
    int rssi_int  = evt->rssi_2 / 2;
    int rssi_frac = (evt->rssi_2 & 0x1) ? 5 : 0;

    const uint8_t *data = (const uint8_t *)evt->data;
    uint8_t  type      = data[0];
    uint8_t  seq       = data[1];
    uint16_t sender_id = ((uint16_t)data[2] << 8) | data[3];

    if (type != MSG_DATA) {
        LOG_WRN("C: Unexpected frame type 0x%02x from sender %d", type, sender_id);
        return;
    }

    if (sender_id != DEVICE_ID_B) {
        LOG_WRN("C: MSG_DATA from unexpected sender %d, ignoring", sender_id);
        return;
    }

    size_t payload_len = evt->len - FRAME_HDR_LEN;

    LOG_INF("C: MSG_DATA from B | RSSI: %d.%d dBm | seq %d | payload: %.*s",
            rssi_int, rssi_frac,
            seq,
            (int)payload_len,
            &data[FRAME_HDR_LEN]);

    last_seq = seq;
    k_sem_give(&data_sem);
}

/*
static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt)
{
    if (evt->len < FRAME_HDR_LEN) {
        LOG_WRN("C: Frame too short (%d bytes), ignoring", evt->len);
        return;
    }

    const uint8_t *data = (const uint8_t *)evt->data;
    uint8_t  type      = data[0];
    uint8_t  seq       = data[1];
    uint16_t sender_id = ((uint16_t)data[2] << 8) | data[3];

    if (type != MSG_DATA) {
        LOG_WRN("C: Unexpected frame type 0x%02x from sender %d", type, sender_id);
        return;
    }

    if (sender_id != DEVICE_ID_B) {
        LOG_WRN("C: MSG_DATA from unexpected sender %d, ignoring", sender_id);
        return;
    }

    // Correct id and data type
    size_t payload_len = evt->len - FRAME_HDR_LEN;
    LOG_INF("C: MSG_DATA from B, seq %d, payload: %.*s",
            seq, (int)payload_len, &data[FRAME_HDR_LEN]);

    last_seq = seq;
    k_sem_give(&data_sem);
}
*/

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
        .rssi_level              = -100,
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

    LOG_INF("Board C — sink node, starting");

    err = nrf_modem_lib_init();
    if (err) { LOG_ERR("modem init failed %d", err); return err; }

    err = nrf_modem_dect_phy_event_handler_set(dect_phy_event_handler);
    if (err) { LOG_ERR("handler set failed %d", err); return err; }

    err = nrf_modem_dect_phy_init();        // this fails sometimes
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

    // Set the device ID so each board knows who they are communications with
    device_id = CONFIG_DEVICE_ID;
    LOG_INF("C: Device ID: %d", device_id);

    while (1) {

        /* STEP 1: Open an RX window and wait for it to close */
        LOG_INF("C: Waiting for data from B...");

        err = receive(rx_handle);
        if (err) {
            LOG_ERR("C: RX failed to start %d", err);
            continue;
        }
        k_sem_take(&operation_sem, K_FOREVER); /* wait for RX window to close */

        /* If no valid MSG_DATA arrived in the window, try again */
        if (k_sem_take(&data_sem, K_NO_WAIT) != 0) {
            LOG_INF("C: No data received, retrying...");
            continue;   // jump back to the beginning of the while loop
        }

        /* STEP 2: Send ACK back to B */
        uint8_t ack[FRAME_HDR_LEN];
        ack[0] = MSG_ACK;
        ack[1] = last_seq;
        ack[2] = (device_id >> 8) & 0xff;
        ack[3] = (device_id & 0xff);

        LOG_INF("C: Sending ACK to B for seq %d", last_seq);

        err = transmit(tx_handle, ack, sizeof(ack));
        if (err) {
            LOG_ERR("C: TX ACK failed %d", err);
            continue;
        }
        k_sem_take(&operation_sem, K_FOREVER); /* wait for TX to complete */

        LOG_INF("C: ACK sent, cycle complete for seq %d", last_seq);
    }

    /* Unreachable under normal operation */
    nrf_modem_dect_phy_deactivate();
    k_sem_take(&deinit_sem, K_FOREVER);
    nrf_modem_dect_phy_deinit();
    k_sem_take(&deinit_sem, K_FOREVER);
    nrf_modem_lib_shutdown();

    return 0;
}