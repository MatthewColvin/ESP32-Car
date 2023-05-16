#include "Transceiver.hpp"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"

#include "freertos/queue.h"

#define LOG_TAG "Transceiver"
#define RX_QUEUE_TASK_NAME "RX_Processor"
#define EXAMPLE_IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us
#define EXAMPLE_IR_NEC_DECODE_MARGIN 200 // Tolerance for parsing RMT symbols into bit stream

/**
 * @brief NEC timing spec
 */
#define NEC_LEADING_CODE_DURATION_0 9000
#define NEC_LEADING_CODE_DURATION_1 4500
#define NEC_PAYLOAD_ZERO_DURATION_0 560
#define NEC_PAYLOAD_ZERO_DURATION_1 560
#define NEC_PAYLOAD_ONE_DURATION_0 560
#define NEC_PAYLOAD_ONE_DURATION_1 1690
#define NEC_REPEAT_CODE_DURATION_0 9000
#define NEC_REPEAT_CODE_DURATION_1 2250

/**
 * @brief Saving NEC decode results
 */
static uint16_t s_nec_code_address;
static uint16_t s_nec_code_command;

/**
 * @brief Check whether a duration is within expected range
 */
static inline bool nec_check_in_range(uint32_t signal_duration, uint32_t spec_duration)
{
    return (signal_duration < (spec_duration + EXAMPLE_IR_NEC_DECODE_MARGIN)) &&
           (signal_duration > (spec_duration - EXAMPLE_IR_NEC_DECODE_MARGIN));
}

/**
 * @brief Check whether a RMT symbol represents NEC logic zero
 */
static bool nec_parse_logic0(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ZERO_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ZERO_DURATION_1);
}

/**
 * @brief Check whether a RMT symbol represents NEC logic one
 */
static bool nec_parse_logic1(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_PAYLOAD_ONE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_PAYLOAD_ONE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC address and command
 */
static bool nec_parse_frame(rmt_symbol_word_t *rmt_nec_symbols)
{
    rmt_symbol_word_t *cur = rmt_nec_symbols;
    uint16_t address = 0;
    uint16_t command = 0;
    bool valid_leading_code = nec_check_in_range(cur->duration0, NEC_LEADING_CODE_DURATION_0) &&
                              nec_check_in_range(cur->duration1, NEC_LEADING_CODE_DURATION_1);
    if (!valid_leading_code)
    {
        return false;
    }
    cur++;
    for (int i = 0; i < 16; i++)
    {
        if (nec_parse_logic1(cur))
        {
            address |= 1 << i;
        }
        else if (nec_parse_logic0(cur))
        {
            address &= ~(1 << i);
        }
        else
        {
            return false;
        }
        cur++;
    }
    for (int i = 0; i < 16; i++)
    {
        if (nec_parse_logic1(cur))
        {
            command |= 1 << i;
        }
        else if (nec_parse_logic0(cur))
        {
            command &= ~(1 << i);
        }
        else
        {
            return false;
        }
        cur++;
    }
    // save address and command
    s_nec_code_address = address;
    s_nec_code_command = command;
    return true;
}

/**
 * @brief Check whether the RMT symbols represent NEC repeat code
 */
static bool nec_parse_frame_repeat(rmt_symbol_word_t *rmt_nec_symbols)
{
    return nec_check_in_range(rmt_nec_symbols->duration0, NEC_REPEAT_CODE_DURATION_0) &&
           nec_check_in_range(rmt_nec_symbols->duration1, NEC_REPEAT_CODE_DURATION_1);
}

/**
 * @brief Decode RMT symbols into NEC scan code and print the result
 */
static void example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num)
{
    printf("NEC frame start---\r\n");
    for (size_t i = 0; i < symbol_num; i++)
    {
        printf("{%d:%d},{%d:%d}\r\n", rmt_nec_symbols[i].level0, rmt_nec_symbols[i].duration0,
               rmt_nec_symbols[i].level1, rmt_nec_symbols[i].duration1);
    }
    printf("---NEC frame end: ");
    // decode RMT symbols
    switch (symbol_num)
    {
    case 34: // NEC normal frame
        if (nec_parse_frame(rmt_nec_symbols))
        {
            printf("Address=%04X, Command=%04X\r\n\r\n", s_nec_code_address, s_nec_code_command);
        }
        break;
    case 2: // NEC repeat frame
        if (nec_parse_frame_repeat(rmt_nec_symbols))
        {
            printf("Address=%04X, Command=%04X, repeat\r\n\r\n", s_nec_code_address, s_nec_code_command);
        }
        break;
    default:
        printf("Unknown NEC frame\r\n\r\n");
        break;
    }
}

Transceiver::Transceiver(gpio_num_t receivePin, gpio_num_t sendPin, const uint packetSize) : mPacketSize(packetSize)
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

    // rmt_carrier_config_t tx_carrier_cfg;
    // tx_carrier_cfg.duty_cycle = 0.33;
    // tx_carrier_cfg.frequency_hz = 38000;
    // tx_carrier_cfg.flags.polarity_active_low = false;
    // ESP_ERROR_CHECK(rmt_apply_carrier(mTxCh, &tx_carrier_cfg));

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
    rxConfig.flags.invert_in = false;
    rxConfig.flags.with_dma = false;
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
    while (true)
    {
        static constexpr auto msToWaitforVal = 1000;

        if (xQueueReceive(mRxQueue, &mEventBeingProc, msToWaitforVal / portTICK_PERIOD_MS))
        {
            example_parse_nec_frame(mEventBeingProc.received_symbols, mEventBeingProc.num_symbols);
        }

        ESP_LOGI(LOG_TAG, "ValueReceived: %d", mEventBeingProc.num_symbols);
    }
}

void Transceiver::enableRx()
{
    rmt_enable(mRxCh);
    xTaskCreate(this->receiveTaskImpl, RX_QUEUE_TASK_NAME, 4096, this, 4, &mQueueProcessor);
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
    aReceiveCfg.signal_range_min_ns = 1250;     // the shortest duration for NEC signal is 560us, 1250ns < 560us, valid signal won't be treated as noise
    aReceiveCfg.signal_range_max_ns = 12000000; // the longest duration for NEC signal is 9000us, 12000000ns > 9000us, the receive won't stop early

    rmt_symbol_word_t buffer[mPacketSize];
    ESP_ERROR_CHECK(rmt_receive(mRxCh, &buffer, sizeof(buffer), &aReceiveCfg));
    for (int i = 0; i < mPacketSize; i++)
    {
        ESP_LOGI(LOG_TAG, "Symbol %d: %lu", i, buffer[i].val);
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
