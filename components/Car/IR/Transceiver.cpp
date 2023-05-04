#include "Transceiver.hpp"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#define LOG_TAG "Transceiver"
#define EXAMPLE_IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us

Transceiver::Transceiver(int receivePin, int sendPin)
{
    setupRxChannel(receivePin);
    setupTxChannel(sendPin);
}

void Transceiver::setupTxChannel(int txPin)
{
    rmt_tx_channel_config_t txConfig;
    txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    txConfig.mem_block_symbols = 64;
    txConfig.resolution_hz = EXAMPLE_IR_RESOLUTION_HZ;
    txConfig.trans_queue_depth = 4;
    txConfig.gpio_num = txPin;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&txConfig, &mTxCh));

    rmt_carrier_config_t tx_carrier_cfg;
    tx_carrier_cfg.duty_cycle = 0.33;
    tx_carrier_cfg.frequency_hz = 38000;
    tx_carrier_cfg.flags.polarity_active_low = false;
    ESP_ERROR_CHECK(rmt_apply_carrier(mTxCh, &tx_carrier_cfg));

    mTxCallbacks.on_trans_done = this->onSendDoneImpl;
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(mTxCh, &mTxCallbacks, this));
}

void Transceiver::setupRxChannel(int rxPin)
{
    rmt_rx_channel_config_t rxConfig;
    rxConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    rxConfig.mem_block_symbols = 64;
    rxConfig.resolution_hz = EXAMPLE_IR_RESOLUTION_HZ;
    rxConfig.gpio_num = rxPin;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rxConfig, &mRxCh));

    rmt_carrier_config_t rx_carrier_cfg;
    rx_carrier_cfg.duty_cycle = .33;
    rx_carrier_cfg.frequency_hz = 25000;
    rx_carrier_cfg.flags.polarity_active_low = false;
    ESP_ERROR_CHECK(rmt_apply_carrier(mRxCh, &rx_carrier_cfg));

    mRxCallbacks.on_recv_done = this->onReceiveImpl;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(mRxCh, &mRxCallbacks, this));
}

bool Transceiver::onReceiveImpl(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    return static_cast<Transceiver *>(user_ctx)->onReceive(edata);
}

bool Transceiver::onReceive(const rmt_rx_done_event_data_t *edata)
{
    for (int i = 0; i < edata->num_symbols; i++)
    {
        ESP_LOGI(LOG_TAG, "Received: %d", edata->received_symbols[i].val);
    }
    return false;
}

bool Transceiver::onSendDoneImpl(rmt_channel_handle_t rx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
    return static_cast<Transceiver *>(user_ctx)->onSendDone(edata);
}

bool Transceiver::onSendDone(const rmt_tx_done_event_data_t *edata)
{
    ESP_LOGI(LOG_TAG, "Sent: %d symbols", edata->num_symbols);
    return false;
}

void Transceiver::send()
{
}
