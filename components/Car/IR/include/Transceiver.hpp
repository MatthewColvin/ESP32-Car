#pragma once
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include <vector>

class Transceiver
{
public:
    Transceiver(int recievePin, int sendPin);
    void send();
    void receive();

    void enableRx();
    void disableRx();
    void enableTx();
    void disableTx();

private:
    void setupRxChannel(int rxPin);
    void setupTxChannel(int txPin);

    static bool onReceiveImpl(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx);
    bool onReceive(const rmt_rx_done_event_data_t *edata);
    static bool onSendDoneImpl(rmt_channel_handle_t rx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx);
    bool onSendDone(const rmt_tx_done_event_data_t *edata);

    rmt_rx_event_callbacks_t mRxCallbacks;
    rmt_tx_event_callbacks_t mTxCallbacks;

    rmt_channel_handle_t mRxCh = nullptr;
    rmt_channel_handle_t mTxCh = nullptr;

    QueueHandle_t mRxQueue = xQueueCreate(50, sizeof(rmt_symbol_word_t));
    bool readyForSymbol = false;
    std::vector<rmt_symbol_word_t> mReceivedSymbols;
    TaskHandle_t mQueueProcessor;
    static void proccessRxQueueImpl(void *aThis);
    void proccessRxQueue();

    uint8_t mFakePayload = 0;
};