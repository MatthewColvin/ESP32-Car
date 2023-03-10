#pragma once
#include "esp_gattc_api.h"

#include "characteristic.hpp"

#include "vector"

class Service
{
public:
    typedef esp_ble_gattc_cb_param_t::gattc_search_res_evt_param espServiceTy;

    Service(Service::espServiceTy anEspService);

    std::vector<esp_gattc_char_elem_t> getCharacteristics(uint8_t aGattIf,
                                                          uint8_t propertiesFilter = 0b1111111,
                                                          Characteristic::FilterType filtertype = Characteristic::FilterType::Any,
                                                          std::vector<int> uuidFilter = {});

    // Expose internals for API CALL integration for now
    uint16_t conn_id() const { return mService.conn_id; };
    uint16_t start_handle() const { return mService.start_handle; }
    uint16_t end_handle() const { return mService.end_handle; }
    esp_gatt_id_t srvc_id() const { return mService.srvc_id; }
    bool is_primary() const { return mService.is_primary; }

private:
    espServiceTy mService;
    std::vector<Characteristic> characteristics;
};