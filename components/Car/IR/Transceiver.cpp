#include "Transceiver.hpp"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#include "freertos/queue.h"

#define LOG_TAG "Transceiver"
#define RX_QUEUE_TASK_NAME "RX_Processor"

Transceiver::Transceiver(int receivePin, int sendPin)
{
    setupRxChannel(receivePin);
    setupTxChannel(sendPin);
}

Transceiver::~Transceiver()
{
    disableRx();
    disableTx();
    if (mRxCh)
    {
        rmt_del_channel(mRxCh);
    }
    if (mTxCh)
    {
        rmt_del_channel(mTxCh);
    }
    if (mRxQueue)
    {
        vQueueDelete(mRxQueue);
    }
}

void Transceiver::setupTxChannel(int txPin)
{
    rmt_tx_channel_config_t txConfig;
    txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    txConfig.mem_block_symbols = mPacketSize;
    txConfig.resolution_hz = Transceiver::IR_RESOLUTION_HZ;
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
    rxConfig.mem_block_symbols = mPacketSize;
    rxConfig.resolution_hz = Transceiver::IR_RESOLUTION_HZ;
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

    rmt_rx_done_event_data_t eventToProccess;
    while (true)
    {
        static constexpr auto msToWaitforVal = 1000;

        if (xQueueReceive(mRxQueue, &eventToProccess, msToWaitforVal / portTICK_PERIOD_MS))
        {
            auto result = mNecParser.Parse(eventToProccess);
            if (result.has_value())
            {
                auto [addr, data, isRepeat] = result.value();
                mDataReceivedHandler(addr, data, isRepeat);
            }
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
    if (mIsRxEnabled)
    {
        return;
    } // early return if we are already ready to Receive
    rmt_enable(mRxCh);
    xTaskCreate(this->receiveTaskImpl, RX_QUEUE_TASK_NAME, 4096, this, 5, &mReceiveProccess);
    mIsRxEnabled = true;
};

void Transceiver::disableRx()
{
    if (!mIsRxEnabled)
    {
        return;
    } // early return if we are already are disabled
    rmt_disable(mRxCh);
    vTaskDelete(mReceiveProccess);
    mIsRxEnabled = false;
};

void Transceiver::enableTx()
{
    if (mIsTxEnabled)
    {
        return;
    }
    ESP_ERROR_CHECK(rmt_enable(mTxCh));
    ir_nec_encoder_config_t nec_encoder_cfg = {
        .resolution = Transceiver::IR_RESOLUTION_HZ,
    };
    ESP_ERROR_CHECK(rmt_new_ir_nec_encoder(&nec_encoder_cfg, &mNecEncoder));
    mIsTxEnabled = true;
    ESP_LOGI(LOG_TAG, "IR Transmit Enabled");
};

void Transceiver::disableTx()
{
    if (!mIsTxEnabled)
    {
        return;
    }
    rmt_disable(mTxCh);
    if (mNecEncoder)
    {
        rmt_del_encoder(mNecEncoder);
    }
    mIsTxEnabled = false;
};

void Transceiver::send(uint16_t address, uint16_t command, uint16_t numRepeats)
{
    rmt_transmit_config_t aTransmitConfig;
    aTransmitConfig.loop_count = 0; // Todo this has to be zero
    aTransmitConfig.flags.eot_level = 1;

    const ir_nec_scan_code_t scan_code = {
        .address = address,
        .command = command,
    };
    // TODO figure out why this is not sending more than one time.
    ESP_ERROR_CHECK(rmt_encoder_reset(mNecEncoder));
    ESP_ERROR_CHECK(rmt_transmit(mTxCh, mNecEncoder, &scan_code, sizeof(scan_code), &aTransmitConfig));
}
