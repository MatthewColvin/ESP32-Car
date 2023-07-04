#pragma once
#include "driver/rmt_types.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "hal/gpio_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "ir_nec_encoder.h"
#include "IrNECParser.hpp"

#include <vector>
#include <functional>
class Transceiver
{
public:
    Transceiver(gpio_num_t receivePin, gpio_num_t sendPin);
    ~Transceiver();

    /**
     * @brief Send an IR Transmission with given parameters
     *
     * @param address - NEC Address
     * @param command - NEC Command
     */
    void send(uint16_t address, uint16_t data);

    typedef std::function<void(uint16_t, uint16_t, bool)> RxHandlerTy;

    /**
     * @brief Set a function to handle the incoming ir transmissions
     *
     * @param aReceiveHandler - callable that handles incoming ir Data
     */
    inline void mSetReceiveHandler(RxHandlerTy aReceiveHandler) { mDataReceivedHandler = aReceiveHandler; };

    /**
     * @brief Enable Reception of IR Signals
     */
    void enableRx();
    /**
     * @brief Disable Reception of IR Signals
     */
    void disableRx();
    /**
     * @brief Enable Transmission of IR Signals
     */
    void enableTx();
    /**
     * @brief Disable Transmission of IR Signals
     */
    void disableTx();

private:
    static constexpr auto IR_RESOLUTION_HZ = 1000000; // 1MHz resolution, 1 tick = 1us;
    static constexpr auto mPacketSize = 64;

    // Receive
    const gpio_num_t mRxPin;
    bool mIsRxEnabled = false;
    void setupRxChannel();
    void teardownRxChannel();
    rmt_channel_handle_t mRxCh = nullptr;
    rmt_rx_channel_config_t mRxConfig;

    // Callback for the remote library to put data on the Receive Queue
    rmt_rx_event_callbacks_t mRxCallbacks;
    static bool onReceiveImpl(rmt_channel_handle_t rx_chan, const rmt_rx_done_event_data_t *edata, void *user_ctx);
    bool onReceive(const rmt_rx_done_event_data_t *edata);

    TaskHandle_t mReceiveProccess;            // Task Handle for Receiving incoming transmissions
    void receiveTask();                       // Task running on mReceiveProccess
    static void receiveTaskImpl(void *aThis); // Wrapper for Task so it can be used with FreeRtos
    QueueHandle_t mRxQueue = xQueueCreate(5, sizeof(rmt_rx_done_event_data_t));

    IrNECParser mNecParser;                     // Parser To Parse things coming off the Queue
    RxHandlerTy mDataReceivedHandler = nullptr; // Callback for user of class to handle parsed data from the Queue

    // Transmit
    const gpio_num_t mTxPin;
    bool mIsTxEnabled = false;
    void setupTxChannel();
    void teardownTxChannel();
    rmt_channel_handle_t mTxCh = nullptr;
    rmt_tx_channel_config_t mTxConfig;
    rmt_carrier_config_t mTxCarrierConfig;

    // Todo figure out what these are for...
    rmt_tx_event_callbacks_t mTxCallbacks;
    static bool onSendDoneImpl(rmt_channel_handle_t rx_chan, const rmt_tx_done_event_data_t *edata, void *user_ctx);
    bool onSendDone(const rmt_tx_done_event_data_t *edata);

    rmt_encoder_handle_t mNecEncoder = nullptr;
};