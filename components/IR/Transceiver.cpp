#include "Transceiver.hpp"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#include "freertos/queue.h"

#define LOG_TAG "Transceiver"
#define RX_QUEUE_TASK_NAME "RX_Processor"

Transceiver::Transceiver(gpio_num_t receivePin, gpio_num_t sendPin) : mRxPin(receivePin), mTxPin(sendPin)
{
    // Tx Configs
    mTxConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    mTxConfig.mem_block_symbols = mPacketSize;
    mTxConfig.resolution_hz = Transceiver::IR_RESOLUTION_HZ;
    mTxConfig.trans_queue_depth = 4;
    mTxConfig.gpio_num = mTxPin;
    mTxConfig.flags.invert_out = false;
    mTxConfig.flags.io_loop_back = false;
    mTxConfig.flags.io_od_mode = false;
    mTxConfig.flags.with_dma = false;

    mTxCarrierConfig.duty_cycle = 0.33;
    mTxCarrierConfig.frequency_hz = 38000;
    mTxCarrierConfig.flags.polarity_active_low = false;

    mTxCallbacks.on_trans_done = this->onSendDoneImpl;
    setupTxChannel();

    // Rx Configs
    mRxConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    mRxConfig.mem_block_symbols = mPacketSize;
    mRxConfig.resolution_hz = Transceiver::IR_RESOLUTION_HZ;
    mRxConfig.gpio_num = mRxPin;
    mRxConfig.flags.with_dma = false;
    mRxConfig.flags.io_loop_back = false;
    mRxConfig.flags.invert_in = false;

    mRxCallbacks.on_recv_done = this->onReceiveImpl;
    setupRxChannel();
}

Transceiver::~Transceiver()
{
    teardownTxChannel();
    teardownRxChannel();
}

void Transceiver::teardownTxChannel()
{
    disableTx();
    if (mTxCh)
    {
        rmt_del_channel(mTxCh);
    }
}

void Transceiver::setupTxChannel()
{
    ESP_ERROR_CHECK(rmt_new_tx_channel(&mTxConfig, &mTxCh));
    ESP_ERROR_CHECK(rmt_apply_carrier(mTxCh, &mTxCarrierConfig));
    ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(mTxCh, &mTxCallbacks, this));
}

void Transceiver::teardownRxChannel()
{
    disableRx();
    if (mRxCh)
    {
        rmt_del_channel(mRxCh);
    }
}

void Transceiver::setupRxChannel()
{
    ESP_ERROR_CHECK(rmt_new_rx_channel(&mRxConfig, &mRxCh));
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

void Transceiver::send(uint16_t address, uint16_t data)
{
    if (!mIsTxEnabled)
    {
        ESP_LOGE(LOG_TAG, "Must enableTx() before using send()");
        return;
    }
    rmt_transmit_config_t aTransmitConfig;
    aTransmitConfig.loop_count = 0; // Todo this has to be zero
    aTransmitConfig.flags.eot_level = 1;

    const ir_nec_scan_code_t scan_code = {
        .address = address,
        .command = data,
    };
    ESP_ERROR_CHECK(rmt_transmit(mTxCh, mNecEncoder, &scan_code, sizeof(scan_code), &aTransmitConfig));
    ESP_LOGI(LOG_TAG, "Sent - address:%d data:%c)", address, data);
    // This teardown and setup feels like a bug but its the only way I could get it to send properly.
    teardownTxChannel();
    setupTxChannel();
    enableTx();
}
