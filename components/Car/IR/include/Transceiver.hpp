#pragma once
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ir_nec_encoder.h"
#include "IrNECParser.hpp"

#include <vector>
#include <functional>
class Transceiver
{
public:
    typedef std::function<void(uint16_t, uint16_t, bool)> RxHandlerTy;

    Transceiver(int recievePin, int sendPin);
    void send();
    inline void mSetReceiveHandler(RxHandlerTy aReceiveHandler) { mDataReceivedHandler = aReceiveHandler; };

    void enableRx();
    void disableRx();
    void enableTx();
    void disableTx();

private:
    static constexpr auto IR_RESOLUTION_HZ = 1000000; // 1MHz resolution, 1 tick = 1us;

    void setupRxChannel(int rxPin);
    void setupTxChannel(int txPin);
    rmt_channel_handle_t mRxCh = nullptr;
    rmt_channel_handle_t mTxCh = nullptr;

    // Callback for the remote library to put data on the Receive Queue
    rmt_rx_event_callbacks_t mRxCallbacks;
    static bool onReceiveImpl(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx);
    bool onReceive(const rmt_rx_done_event_data_t *edata);

    // Todo figure out what these are for...
    rmt_tx_event_callbacks_t mTxCallbacks;
    static bool onSendDoneImpl(rmt_channel_handle_t rx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx);
    bool onSendDone(const rmt_tx_done_event_data_t *edata);

    QueueHandle_t mRxQueue = xQueueCreate(5, sizeof(rmt_rx_done_event_data_t));
    IrNECParser mNecParser;
    RxHandlerTy mDataReceivedHandler = nullptr;

    TaskHandle_t mReceiveProccess;            // Task Handle for Receiving incoming transmissions
    void receiveTask();                       // Task running on mReceiveProccess
    static void receiveTaskImpl(void *aThis); // Wrapper for Task so it can be used with FreeRtos
    rmt_rx_done_event_data_t mEventBeingProc; // Most recent received rmt_symbols

    uint16_t mFakePayload = 0;
    const uint mPacketSize = 64;
};