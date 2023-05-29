#include "Transceiver.hpp"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#include "freertos/queue.h"

#define EXAMPLE_IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us

#define LOG_TAG "Transceiver"
#define RX_QUEUE_TASK_NAME "RX_Processor"

Transceiver::Transceiver(gpio_num_t receivePin, gpio_num_t sendPin)
{

    mReceivedSymbols.resize(mPacketSize);
    setupRxChannel(receivePin);
    setupTxChannel(sendPin);
}

void Transceiver::setupTxChannel(gpio_num_t txPin)
{
    rmt_tx_channel_config_t txConfig;
    txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    txConfig.mem_block_symbols = mPacketSize;
    txConfig.resolution_hz = EXAMPLE_IR_RESOLUTION_HZ;
    txConfig.trans_queue_depth = 4;
    txConfig.gpio_num = txPin;
    txConfig.flags.invert_out = false;
    txConfig.flags.with_dma = false;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&txConfig, &mTxCh));

    rmt_carrier_config_t tx_carrier_cfg;
    tx_carrier_cfg.duty_cycle = 0.33;
    tx_carrier_cfg.frequency_hz = 38000;
    tx_carrier_cfg.flags.polarity_active_low = false;
    ESP_ERROR_CHECK(rmt_apply_carrier(mTxCh, &tx_carrier_cfg));

    mTxCallbacks.on_trans_done = this->onSendDoneImpl;
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(mTxCh, &mTxCallbacks, this));
}

void Transceiver::setupRxChannel(gpio_num_t rxPin)
{
    rmt_rx_channel_config_t rxConfig;
    rxConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    rxConfig.mem_block_symbols = mPacketSize;
    rxConfig.resolution_hz = EXAMPLE_IR_RESOLUTION_HZ;
    rxConfig.gpio_num = rxPin;
    rxConfig.flags.with_dma = false;
    rxConfig.flags.io_loop_back = false;
    rxConfig.flags.invert_in = false;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rxConfig, &mRxCh));

    mRxCallbacks.on_recv_done = this->onReceiveImpl;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(mRxCh, &mRxCallbacks, this));
}

bool Transceiver::onReceiveImpl(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx)
{
    return static_cast<Transceiver *>(user_ctx)->onReceive(edata);
}

bool Transceiver::onReceive(const rmt_rx_done_event_data_t *edata)
{
    BaseType_t high_task_wakeup = pdFALSE;
    xQueueSendFromISR(mRxQueue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}

bool Transceiver::onSendDoneImpl(rmt_channel_handle_t rx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx)
{
    return static_cast<Transceiver *>(user_ctx)->onSendDone(edata);
}

bool Transceiver::onSendDone(const rmt_tx_done_event_data_t *edata)
{
    return false;
}

void Transceiver::receiveTaskImpl(void *aThis)
{
    static_cast<Transceiver *>(aThis)->receiveTask();
}

void Transceiver::receiveTask()
{
    rmt_receive_config_t receiveCfg;
    receiveCfg.signal_range_min_ns = 1250;     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
    receiveCfg.signal_range_max_ns = 12000000; // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early
    rmt_symbol_word_t buffer[mPacketSize];
    ESP_ERROR_CHECK(rmt_receive(mRxCh, &buffer, sizeof(buffer), &receiveCfg));

    while (true)
    {
        static constexpr auto msToWaitforVal = 1000;

        if (xQueueReceive(mRxQueue, &mEventBeingProc, msToWaitforVal / portTICK_PERIOD_MS))
        {
            mNecParser.Parse();
            // if (result.has_value())
            // {
            //     auto [addr, data, isRepeat] = result.value();
            // }
            // example_parse_nec_frame(mEventBeingProc.received_symbols, mEventBeingProc.num_symbols);
            ESP_ERROR_CHECK(rmt_receive(mRxCh, &buffer, sizeof(buffer), &receiveCfg));
        }
        else
        {
            // ESP_LOGI(LOG_TAG, "No Symbols Received");
            ESP_ERROR_CHECK(rmt_receive(mRxCh, &buffer, sizeof(buffer), &receiveCfg));
        }
    }
}

void Transceiver::enableRx()
{
    rmt_enable(mRxCh);
    xTaskCreate(this->receiveTaskImpl, RX_QUEUE_TASK_NAME, 4096, this, configMAX_PRIORITIES - 1, &mReceiveProccess);
};

void Transceiver::disableRx()
{
    rmt_disable(mRxCh);
    vTaskDelete(mReceiveProccess);
};

void Transceiver::enableTx() { rmt_enable(mTxCh); };

void Transceiver::disableTx() { rmt_disable(mTxCh); };

void Transceiver::send()
{

    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = EXAMPLE_IR_RESOLUTION_HZ,
    };
    rmt_encoder_handle_t nec_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &nec_encoder));

    rmt_transmit_config_t aTransmitConfig;
    aTransmitConfig.loop_count = 0;

    const ir_nec_scan_code_t scan_code = {
        .address = 0x0440,
        .command = mFakePayload,
    };

    mFakePayload++;
    ESP_ERROR_CHECK(rmt_transmit(mTxCh, nec_encoder, &scan_code, sizeof(scan_code), &aTransmitConfig));
    ESP_LOGI(LOG_TAG, "Sent %d", mFakePayload);
}
