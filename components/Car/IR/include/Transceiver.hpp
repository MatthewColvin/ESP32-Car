#pragma once
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"

class Transceiver
{
public:
    Transceiver(int recievePin, int sendPin);
    void send();

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
};