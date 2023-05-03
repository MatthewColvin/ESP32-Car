#pragma once
#include "driver/rmt_types.h"

class Transciever
{
public:
    Transciever(int recievePin, int sendPin);
    void send();
    void onRecieve();

private:
    rmt_channel_handle_t rxCh = nullptr;
    rmt_channel_handle_t txCh = nullptr;
};