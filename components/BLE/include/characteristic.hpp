#pragma once

#include "esp_gattc_api.h"

class Characteristic
{
    Characteristic(esp_gattc_char_elem_t anIdfCharacteristic);

private:
    esp_gattc_char_elem_t mCharacteristic;
};