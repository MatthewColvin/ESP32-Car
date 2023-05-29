#pragma once
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ir_nec_encoder.h"

#include <vector>
#include <functional>
class Transceiver
{
public:
    typedef std::function<void(uint16_t, uint16_t, bool)> RxHandlerTy;

    Transceiver(gpio_num_t recievePin, gpio_num_t sendPin, const uint packetSize);
    void send();
    void receive();

    void enableRx();
    void disableRx();
    void enableTx();
    void disableTx();

    void mSetReceiveHandler(RxHandlerTy aReceiveHandler) { mDataReceivedHandler = aReceiveHandler; };

private:
    void example_parse_nec_frame(rmt_symbol_word_t *rmt_nec_symbols, size_t symbol_num);

    void setupRxChannel(gpio_num_t rxPin);
    void setupTxChannel(gpio_num_t txPin);

    static bool onReceiveImpl(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx);
    bool onReceive(const rmt_rx_done_event_data_t *edata);
    static bool onSendDoneImpl(rmt_channel_handle_t rx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx);
    bool onSendDone(const rmt_tx_done_event_data_t *edata);

    rmt_rx_event_callbacks_t mRxCallbacks;
    rmt_tx_event_callbacks_t mTxCallbacks;

    rmt_channel_handle_t mRxCh = nullptr;
    rmt_channel_handle_t mTxCh = nullptr;

    QueueHandle_t mRxQueue = xQueueCreate(5, sizeof(rmt_rx_done_event_data_t));
    bool readyForSymbol = false;
    std::vector<rmt_symbol_word_t> mReceivedSymbols;
    RxHandlerTy mDataReceivedHandler = nullptr;

    TaskHandle_t mReceiveProccess;
    rmt_rx_done_event_data_t mEventBeingProc;
    static void receiveTaskImpl(void *aThis);
    void receiveTask();

    uint16_t mFakePayload = 0;
    const uint mPacketSize = 0;
};