#include "Transciever.hpp"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"

#define EXAMPLE_IR_RESOLUTION_HZ 1000000 // 1MHz resolution, 1 tick = 1us

Transciever::Transciever(int recievePin, int sendPin)
{
    rmt_tx_channel_config_t txConfig;
    txConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    txConfig.mem_block_symbols = 64;
    txConfig.resolution_hz = EXAMPLE_IR_RESOLUTION_HZ;
    txConfig.trans_queue_depth = 4;
    txConfig.gpio_num = sendPin;

    rmt_rx_channel_config_t rxConfig;
    rxConfig.clk_src = RMT_CLK_SRC_DEFAULT;
    rxConfig.mem_block_symbols = 64;
    rxConfig.resolution_hz = EXAMPLE_IR_RESOLUTION_HZ;
    rxConfig.gpio_num = recievePin;

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rxConfig, &rxCh));
    ESP_ERROR_CHECK(rmt_new_tx_channel(&txConfig, &txCh));
}

void Transciever::send()
{
}
