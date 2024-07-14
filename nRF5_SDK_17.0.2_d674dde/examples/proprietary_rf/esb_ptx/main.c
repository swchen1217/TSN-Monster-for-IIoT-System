/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>

#include "sdk_common.h"

#include "nrf.h"
#include "nrf_esb.h"
#include "nrf_error.h"
#include "nrf_esb_error_codes.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "app_util.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"

#include "app_error.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"

// @MWNL TimeSync Lib
#include "time_sync.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "app_timer.h"
#include "nrf_atomic.h"

// @MWNL URLLC Lib
#include "urllc.h"

// @MWNL TimeSync Begin
static bool m_gpio_trigger_enabled;

static void ts_evt_callback(const ts_evt_t *evt);
static void ts_gpio_trigger_enable(void);
static void ts_gpio_trigger_disable(void);

APP_TIMER_DEF(sync_pkt_send_timer); // timer udddddddsed for triggering sending sync pkt

static nrf_atomic_flag_t urllc_pkt_in_progress = false; // this flag is set when urllc pkt is processing
// @MWNL TimeSync End

// @MWNL ESB Begin
static nrf_esb_payload_t tx_payload = NRF_ESB_CREATE_PAYLOAD(0, 0x01, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00);
static nrf_esb_payload_t rx_payload;
// @MWNL ESB End

// @MWNL USBD Begin
#define LED_USB_RESUME (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX (BSP_BOARD_LED_3)

#define BTN_CDC_DATA_SEND 0
#define BTN_CDC_NOTIFY_SEND 1

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT NRF_DRV_USBD_EPOUT1

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

#define READ_SIZE 1

static char m_usbd_rx_buffer[READ_SIZE];
static char m_usbd_tx_buffer[NRF_DRV_USBD_EPSIZE];
// static char m_echo_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;

/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const *p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const *p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);

    switch (event)
    {
    case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
    {
        bsp_board_led_on(LED_CDC_ACM_OPEN);

        /*Setup first transfer*/
        ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                               m_usbd_rx_buffer,
                                               READ_SIZE);
        // NRF_LOG_DEBUG("char: %d", m_usbd_rx_buffer[0]);
        UNUSED_VARIABLE(ret);
        break;
    }
    case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        bsp_board_led_off(LED_CDC_ACM_OPEN);
        break;
    case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        bsp_board_led_invert(LED_CDC_ACM_TX);
        break;
    case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
    {
        // NRF_LOG_DEBUG("run RX DONE.");
        // @MWNL
        ret_code_t ret;
        // the first byte is read in case : APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN
        static uint8_t index = 0;
        static uint8_t type = 0;
        static uint8_t val_len = 0;
        static char m_usbd_rx_data[NRF_DRV_USBD_EPSIZE];

        // NRF_LOG_INFO("Bytes waiting: %d", app_usbd_cdc_acm_bytes_stored(p_cdc_acm));
        // index = 0;
        do
        {
            NRF_LOG_WARNING("RX: char: %d", m_usbd_rx_buffer[0]);
            // NRF_LOG_FLUSH();
            memcpy(&m_usbd_rx_data[index++], &m_usbd_rx_buffer[0], READ_SIZE);

            ret = app_usbd_cdc_acm_read(&m_app_cdc_acm,
                                        m_usbd_rx_buffer,
                                        READ_SIZE);

            if (index >= TLV_HEADER_LEN)
            {
                type = m_usbd_rx_data[TLV_TYPE_INDEX];
                val_len = m_usbd_rx_data[TLV_LENGTH_INDEX];
                // NRF_LOG_DEBUG("%d, %d", type, val_len);
            }

            // if ((type != CDC_ACM_DATA && index >= TLV_HEADER_LEN) || (type == CDC_ACM_DATA && index >= TLV_HEADER_LEN + val_len))
            if (index >= TLV_HEADER_LEN + val_len)
            {
                NRF_LOG_DEBUG("TLV OK");
                NRF_LOG_FLUSH();

                // bsp_board_led_invert(LED_CDC_ACM_RX);
                // bsp_board_led_on(LED_CDC_ACM_RX);
                // bsp_board_led_off(LED_CDC_ACM_RX);

                switch (type)
                {
                case CDC_ACM_DATA:
                {
                    NRF_LOG_WARNING("USBD RX: CDC_ACM_DATA");

                    nrf_atomic_flag_set(&urllc_pkt_in_progress);

                    urllc_payload urllc_pkt;
                    urllc_pkt.header.message_id = URLLC_DATA_PKT;
                    memcpy(&urllc_pkt.seq_num, &m_usbd_rx_data[TLV_HEADER_LEN], sizeof(uint32_t));
                    memcpy(&urllc_pkt.data, &m_usbd_rx_data[TLV_HEADER_LEN + sizeof(uint32_t)], val_len - sizeof(uint32_t));
                    urllc_pkt.data[val_len - sizeof(uint32_t)] = '\0';
                    urllc_pkt.time_stamp = TIME_SYNC_TIMESTAMP_TO_USEC(ts_timestamp_get_ticks_u64());

                    NRF_LOG_DEBUG("USBD RX: %d", urllc_pkt.seq_num);

                    // echo
                    // size_t size = sprintf(m_usbd_tx_buffer, "%d,%s", urllc_pkt.seq_num, urllc_pkt.data);
                    // app_usbd_cdc_acm_write(&m_app_cdc_acm, m_usbd_tx_buffer, size);

                    tx_payload.noack = true;
                    tx_payload.length = sizeof(urllc_pkt);
                    // nrf_esb_set_retransmit_count(5);
                    // current counter val + 0.5sec is the ch_tick_target
                    memcpy(tx_payload.data, &urllc_pkt, sizeof(urllc_pkt));

                    bsp_board_led_on(LED_CDC_ACM_RX);

                    ret_code_t err = nrf_esb_write_payload(&tx_payload);

                    if (err == NRF_SUCCESS)
                    {

                        NRF_LOG_INFO("Sending urllc data pkt succeed");
                    }
                    else
                    {
                        NRF_LOG_WARNING("Sending urllc data packet failed: %d", err);
                    }

                    nrf_atomic_flag_clear(&urllc_pkt_in_progress);
                    
                    bsp_board_led_off(LED_CDC_ACM_RX);
                }
                break;
                case CDC_ACM_CHN_SET:
                {
                    NRF_LOG_WARNING("USBD RX: CDC_ACM_CHN_SET");
                    // NRF_LOG_FLUSH();

                    uint8_t new_chn = m_usbd_rx_data[TLV_HEADER_LEN];

                    // uint32_t new_chn;
                    // memcpy(&new_chn, &m_usbd_rx_data[TLV_HEADER_LEN], sizeof(uint32_t));

                    NRF_LOG_WARNING("CHN: %d", new_chn);

                    ret_code_t err = nrf_esb_set_rf_channel(new_chn);

                    if (err == NRF_SUCCESS)
                    {
                        NRF_LOG_WARNING("nrf_esb_set_rf_channel succeed:");
                        uint32_t now_chn;
                        nrf_esb_get_rf_channel(&now_chn);
                        NRF_LOG_WARNING("%d", now_chn);
                    }
                    else
                    {
                        NRF_LOG_WARNING("nrf_esb_set_rf_channel failed: %d", err);
                    }

                }
                break;
                case CDC_ACM_TS_1:
                {
                    NRF_LOG_DEBUG("USBD RX: CDC_ACM_TS_1");
                    NRF_LOG_FLUSH();

                    static bool m_send_sync_pkt = false;

                    if (m_send_sync_pkt)
                    {
                        m_send_sync_pkt = false;
                        m_gpio_trigger_enabled = false;

                        bsp_board_leds_off();

                        uint32_t err_code = ts_tx_stop();
                        APP_ERROR_CHECK(err_code);

                        NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");
                    }
                    else
                    {
                        m_send_sync_pkt = true;

                        bsp_board_leds_on();

                        uint32_t err_code = ts_tx_start(TIME_SYNC_FREQ_AUTO);
                        APP_ERROR_CHECK(err_code);
  
                        ts_gpio_trigger_enable();

                        NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
                    }
                }
                break;
                case CDC_ACM_TS_2:
                {
                    NRF_LOG_DEBUG("USBD RX: CDC_ACM_TS_2");

                    uint64_t time_ticks;
                    uint32_t time_usec;

                    time_ticks = ts_timestamp_get_ticks_u64();
                    time_usec = TIME_SYNC_TIMESTAMP_TO_USEC(time_ticks);

                    NRF_LOG_INFO("Timestamp: %d us (%d, %d)", time_usec, time_usec / 1000000, time_usec / 1000);
                }
                break;
                default:
                {
                    NRF_LOG_WARNING("USBD RX: TYPE NOT DEFINED");
                    bsp_board_led_invert(LED_CDC_ACM_RX);
                }
                break;
                }

                index = 0;
            }

        } while (ret == NRF_SUCCESS);

        // bsp_board_led_invert(LED_CDC_ACM_RX);
        break;
    }
    default:
        break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
    case APP_USBD_EVT_DRV_SUSPEND:
        bsp_board_led_off(LED_USB_RESUME);
        break;
    case APP_USBD_EVT_DRV_RESUME:
        bsp_board_led_on(LED_USB_RESUME);
        break;
    case APP_USBD_EVT_STARTED:
        break;
    case APP_USBD_EVT_STOPPED:
        app_usbd_disable();
        bsp_board_leds_off();
        break;
    case APP_USBD_EVT_POWER_DETECTED:
        NRF_LOG_INFO("USB power detected");

        if (!nrf_drv_usbd_is_enabled())
        {
            app_usbd_enable();
        }
        break;
    case APP_USBD_EVT_POWER_REMOVED:
        NRF_LOG_INFO("USB power removed");
        app_usbd_stop();
        break;
    case APP_USBD_EVT_POWER_READY:
        NRF_LOG_INFO("USB ready");
        app_usbd_start();
        break;
    default:
        break;
    }
}

// @MWNL USBD End

// @MWNL ESB Begin
void nrf_esb_event_handler(nrf_esb_evt_t const *p_event)
{
    switch (p_event->evt_id)
    {
    case NRF_ESB_EVENT_TX_SUCCESS:
        NRF_LOG_DEBUG("TX SUCCESS EVENT");
        break;
    case NRF_ESB_EVENT_TX_FAILED:
        NRF_LOG_DEBUG("TX FAILED EVENT");
        (void)nrf_esb_flush_tx();
        (void)nrf_esb_start_tx();
        break;
    case NRF_ESB_EVENT_RX_RECEIVED:
        NRF_LOG_DEBUG("RX RECEIVED EVENT");
        while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
        {
            if (rx_payload.length > 0)
            {
                NRF_LOG_DEBUG("RX RECEIVED PAYLOAD");
            }
        }
        break;
    }
}

void clocks_start(void)
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
        ;
}

uint32_t esb_init(void)
{
    uint32_t ret;
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

    nrf_esb_config_t nrf_esb_config = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.protocol = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.retransmit_delay = 600;
    nrf_esb_config.bitrate = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.event_handler = nrf_esb_event_handler;
    nrf_esb_config.mode = NRF_ESB_MODE_PTX;
    nrf_esb_config.selective_auto_ack = true;

    ret = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(ret);

    ret = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(ret);

    ret = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(ret);

    ret = nrf_esb_set_prefixes(addr_prefix, NRF_ESB_PIPE_COUNT);
    VERIFY_SUCCESS(ret);

    return ret;
}
// @MWNL ESB End

// @MWNL GPIO Begin
void gpio_init(void)
{
    nrf_gpio_range_cfg_output(8, 15);
    bsp_board_init(BSP_INIT_LEDS);
}
// @MWNL GPIO End

// @MWNL TimeSync Begin
static void ts_gpio_trigger_enable(void)
{
    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_target;
    uint32_t err_code;

    if (m_gpio_trigger_enabled)
    {
        return;
    }

    // Round up to nearest second to next 1000 ms to start toggling.
    // If the receiver has received a valid sync packet within this time, the GPIO toggling polarity will be the same.

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

    time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + (1000 * 2);
    time_target = (time_target / 1000) * 1000;

    err_code = ts_set_trigger(time_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    APP_ERROR_CHECK(err_code);

    nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

    m_gpio_trigger_enabled = true;
}

static void ts_gpio_trigger_disable(void)
{
    m_gpio_trigger_enabled = false;
}

static void ts_evt_callback(const ts_evt_t *evt)
{
    APP_ERROR_CHECK_BOOL(evt != NULL);

    switch (evt->type)
    {
    case TS_EVT_SYNCHRONIZED:
        NRF_LOG_INFO("TS_EVT_SYNCHRONIZED.");
        ts_gpio_trigger_enable();
        break;
    case TS_EVT_DESYNCHRONIZED:
        NRF_LOG_INFO("TS_EVT_DESYNCHRONIZED.");
        ts_gpio_trigger_disable();
        break;
    case TS_EVT_TRIGGERED:

        if (m_gpio_trigger_enabled)
        {
            uint32_t tick_target;

            tick_target = evt->params.triggered.tick_target + 1;

            uint32_t err_code = ts_set_trigger(tick_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            // Ensure pin is low when triggering is stopped
            nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);
        }
        break;
    default:
        APP_ERROR_CHECK_BOOL(false);
        break;
    }
}

static void sync_timer_init(void)
{
    uint32_t err_code;

    // Debug pin:
    // nRF52-DK (PCA10040) Toggle P0.24 from sync timer to allow pin measurement
    // nRF52840-DK (PCA10056) Toggle P1.14 from sync timer to allow pin measurement
#if defined(BOARD_PCA10040)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#elif defined(BOARD_PCA10056)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#else
#warning Debug pin not set
#endif

    ts_init_t init_ts =
        {
            .high_freq_timer[0] = NRF_TIMER3,
            .high_freq_timer[1] = NRF_TIMER4,
            .egu = NRF_EGU3,
            .egu_irq_type = SWI3_EGU3_IRQn,
            .evt_handler = ts_evt_callback,
        };

    err_code = ts_init(&init_ts);
    APP_ERROR_CHECK(err_code);

    ts_rf_config_t rf_config =
        {
            .rf_chn = 80,
            .rf_addr = {0xDE, 0xAD, 0xBE, 0xEF, 0x19}};

    // err_code = ts_enable(&rf_config);
    err_code = ts_enable();
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Started listening for beacons.");
    NRF_LOG_INFO("Press Button 1 to start transmitting sync beacons");
    NRF_LOG_INFO("GPIO toggling will begin when transmission has started.");
}

// send sync packet periodically (Period : SYNC_INTERVAL)
static void sync_pkt_send_timer_handler(void *p_context)
{
    // NRF_LOG_WARNING("run sync_pkt_send_timer_handler......");
    if (urllc_pkt_in_progress)
    {
        NRF_LOG_WARNING("urllc_pkt_in_progress");
    }
    if (!urllc_pkt_in_progress)
    {
        // if (true)
        //   NRF_LOG_WARNING("run sync_pkt_send......");

        tx_payload.length = sizeof(sync_pkt_t); // set tx_payload length packet length
        tx_payload.noack = true;                // sync packet is not required ack
        sync_pkt_t *p_pkt;
        p_pkt = (sync_pkt_t *)malloc(sizeof(sync_pkt_t));
        p_pkt->header.message_id = SYNC_PKT;
        take_sync_timer_val(p_pkt);
        // nrf_gpio_pin_set(TEST_PIN);
        memcpy(tx_payload.data, p_pkt, sizeof(sync_pkt_t));
        ret_code_t ret = nrf_esb_write_payload(&tx_payload);
        APP_ERROR_CHECK(ret);
        if (ret == NRF_SUCCESS)
        {
            NRF_LOG_INFO("Sending sync packet succeed");
        }
        else
        {
            NRF_LOG_WARNING("Sending sync failed: %d", ret);
        }
        free(p_pkt);
    }
    // nrf_gpio_pin_clear(TEST_PIN);
}

static void config_timers()
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&sync_pkt_send_timer,
                                APP_TIMER_MODE_REPEATED,
                                sync_pkt_send_timer_handler);
    // err_code = app_timer_create(&chn_pkt_send_timer,
    //                              APP_TIMER_MODE_SINGLE_SHOT,
    //                              ch_pkt_send_timer_handler);

    APP_ERROR_CHECK(err_code);
}
// @MWNL TimeSync End

int main(void)
{
    ret_code_t ret;

    // @MWNL USBD Begin
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler};
    // @MWNL USBD End

    gpio_init();

    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // @MWNL USBD Begin
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);

    while (!nrf_drv_clock_lfclk_is_running())
    {
        /* Just waiting */
    }

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    app_usbd_serial_num_generate();

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USBD CDC ACM example started.");

    app_usbd_class_inst_t const *class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);

    if (USBD_POWER_DETECTION)
    {
        ret = app_usbd_power_events_enable();
        APP_ERROR_CHECK(ret);
    }
    else
    {
        NRF_LOG_INFO("No USB power detection enabled\r\nStarting USB now");

        app_usbd_enable();
        app_usbd_start();
    }
    // @MWNL USBD End

    // @MWNL ESB Begin
    clocks_start();

    ret = esb_init();
    APP_ERROR_CHECK(ret);

    NRF_LOG_DEBUG("Enhanced ShockBurst Transmitter Example started.");
    // @MWNL ESB End

    // @MWNL TimeSync Begin
    sync_timer_init();
    ts_gpio_trigger_enable();

    config_timers();
    // send sync packet each (SYNC_INTERVAL)ms using app timer
    ret = app_timer_start(sync_pkt_send_timer, APP_TIMER_TICKS(SYNC_INTERVAL), NULL);
    APP_ERROR_CHECK(ret);
    // @MWNL TimeSync End

    while (true)
    {
        // @MWNL USBD Begin
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }

        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        /* Sleep CPU only if there was no interrupt since last loop processing */
        __WFE();
        // @MWNL USBD End
    }
}
