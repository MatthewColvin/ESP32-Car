#include "Transceiver.hpp"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#include "freertos/queue.h"

#define LOG_TAG "Transceiver"
#define RX_QUEUE_TASK_NAME "RX_Processor"
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

    // rmt_carrier_config_t tx_carrier_cfg;
    // tx_carrier_cfg.duty_cycle = 0.33;
    // tx_carrier_cfg.frequency_hz = 38000;
    // tx_carrier_cfg.flags.polarity_active_low = false;
    // ESP_ERROR_CHECK(rmt_apply_carrier(mTxCh, &tx_carrier_cfg));

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

    // rmt_carrier_config_t rx_carrier_cfg;
    // rx_carrier_cfg.duty_cycle = .33;
    // rx_carrier_cfg.frequency_hz = 25000;
    // rx_carrier_cfg.flags.polarity_active_low = false;
    // ESP_ERROR_CHECK(rmt_apply_carrier(mRxCh, &rx_carrier_cfg));

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
        xQueueSendFromISR(mRxQueue, &edata->received_symbols[i], nullptr);
    }
    return false;
}

bool Transceiver::onSendDoneImpl(rmt_channel_handle_t rx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
    return static_cast<Transceiver *>(user_ctx)->onSendDone(edata);
}

bool Transceiver::onSendDone(const rmt_tx_done_event_data_t *edata)
{
    return false;
}

void Transceiver::proccessRxQueueImpl(void *aThis)
{
    static_cast<Transceiver *>(aThis)->proccessRxQueue();
}

void Transceiver::proccessRxQueue()
{
    while (true)
    {
        static constexpr auto msToWaitforVal = 30;
        if (!readyForSymbol)
        {
            mReceivedSymbols.resize(mReceivedSymbols.size() + 1);
            readyForSymbol = true;
        }
        auto isValueProcessed =
            xQueueReceive(mRxQueue, &mReceivedSymbols[mReceivedSymbols.size() - 1], msToWaitforVal / portTICK_PERIOD_MS);

        if (isValueProcessed == pdTRUE)
        {
            readyForSymbol = false;
            ESP_LOGI(LOG_TAG, "ValueReceived: %d", mReceivedSymbols.end()->val);
        }
    }
}

void Transceiver::enableRx()
{
    rmt_enable(mRxCh);
    xTaskCreate(this->proccessRxQueueImpl, RX_QUEUE_TASK_NAME, 4096, this, 4, &mQueueProcessor);
};

void Transceiver::disableRx()
{
    rmt_disable(mRxCh);
    vTaskDelete(mQueueProcessor);
};

void Transceiver::enableTx() { rmt_enable(mTxCh); };

void Transceiver::disableTx() { rmt_disable(mTxCh); };

void Transceiver::receive()
{
    rmt_receive_config_t aReceiveCfg;
    aReceiveCfg.signal_range_max_ns = 200;
    aReceiveCfg.signal_range_min_ns = 20;

    static constexpr auto numSymbolsToReceive = 10;
    rmt_symbol_word_t buffer[numSymbolsToReceive];
    ESP_ERROR_CHECK(rmt_receive(mRxCh, &buffer, sizeof(buffer), &aReceiveCfg));
    for (int i = 0; i < numSymbolsToReceive; i++)
    {
        ESP_LOGI(LOG_TAG, "Symbol %d: %d", i, buffer[i].val);
    }
}

void Transceiver::send()
{
    rmt_copy_encoder_config_t encoderConfig;
    rmt_encoder_handle_t cpyEncoder;
    rmt_new_copy_encoder(&encoderConfig, &cpyEncoder);

    rmt_transmit_config_t aTransmitConfig;
    aTransmitConfig.loop_count = 0;

    mFakePayload++;
    ESP_ERROR_CHECK(rmt_transmit(mTxCh, cpyEncoder, &mFakePayload, 1, &aTransmitConfig));
    ESP_LOGI(LOG_TAG, "Sent %d", mFakePayload);
}
