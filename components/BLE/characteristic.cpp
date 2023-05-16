// BLE Component
#include "characteristic.hpp"
// ESP API
#include "esp_log.h"
// RTOS
// STD
#include <algorithm>

#define LOG_TAG "Characteristic"

Characteristic::Characteristic(uint8_t aDeviceGattIf,
                               uint8_t aServiceConnId,
                               esp_gattc_char_elem_t anIdfCharacteristic) : mDeviceGattIf(aDeviceGattIf),
                                                                            mServiceConnId(aServiceConnId),
                                                                            mCharacteristic(anIdfCharacteristic)
{
}

std::vector<esp_gattc_descr_elem_t> Characteristic::getDescriptors() const
{
    esp_gatt_status_t status = ESP_GATT_OK;
    std::vector<esp_gattc_descr_elem_t> descriptors;
    uint16_t numDescriptions; // outside scope of loop for end read check
    do
    {
        numDescriptions = 1; // only fetch one at a time
        esp_gattc_descr_elem_t charaDescription;
        status = esp_ble_gattc_get_all_descr(mDeviceGattIf,
                                             mServiceConnId,
                                             mCharacteristic.char_handle,
                                             &charaDescription,
                                             &numDescriptions, // this will update to total number of descriptions
                                             descriptors.size());
        if (status == ESP_GATT_OK)
        {
            descriptors.push_back(charaDescription);
        }
        else if (status != ESP_GATT_NOT_FOUND)
        {
            ESP_LOGE(LOG_TAG, "FETCH DESC STATUS: %d", status);
        }
    } while (descriptors.size() < numDescriptions && // Check num descriptions
             status != ESP_GATT_NOT_FOUND);

    return descriptors;
}

void Characteristic::describe() const
{
    ESP_LOGI(LOG_TAG, "Name: %s Handle: %d UUID: %X Properties - Ext_Prop: %d Auth: %d Indicate: %d Notify: %d Write: %d Write_NR: %d Read: %d BroadCast: %d ",
             uuidstr().c_str(),
             mCharacteristic.char_handle,
             uuid(),
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_EXT_PROP) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_AUTH) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_INDICATE) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_NOTIFY) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_WRITE) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_WRITE_NR) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_READ) > 0,
             (mCharacteristic.properties & ESP_GATT_CHAR_PROP_BIT_BROADCAST) > 0);

    for (auto descriptor : getDescriptors())
    {
        ESP_LOGI(LOG_TAG, "-Descriptor UUID: %d", descriptor.uuid.uuid.uuid16);
    }
}

void Characteristic::read()
{
    esp_ble_gattc_read_char(mDeviceGattIf, mServiceConnId, mCharacteristic.char_handle, ESP_GATT_AUTH_REQ_NO_MITM);
}

void Characteristic::readDescriptors()
{
    for (esp_gattc_descr_elem_t descriptor : getDescriptors())
    {
        ESP_LOGI(LOG_TAG, "Descriptor handle:%d", descriptor.handle);
        esp_ble_gattc_read_char_descr(mDeviceGattIf, mServiceConnId, descriptor.handle, ESP_GATT_AUTH_REQ_NO_MITM);
    }
}

void Characteristic::write(uint8_t *value, uint16_t len)
{
    esp_ble_gattc_write_char(mDeviceGattIf, mServiceConnId, mCharacteristic.char_handle, len, value, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NO_MITM);
}

bool Characteristic::matchesFilters(uint8_t aFilter, PropFilterType aType, std::vector<int> uuidFilter) const
{
    bool isUUIDWanted = uuidFilter.empty() || std::find(uuidFilter.begin(), uuidFilter.end(), uuid()) != uuidFilter.end();
    if (isUUIDWanted)
    {
        if (aType == PropFilterType::Any)
        {
            return (mCharacteristic.properties & aFilter) > 0;
        }
        // all
        return (mCharacteristic.properties & aFilter) == aFilter;
    }
    return false;
}

int Characteristic::uuid() const
{
    int retval = 0;
    switch (mCharacteristic.uuid.len)
    {
    case ESP_UUID_LEN_16:
        retval = mCharacteristic.uuid.uuid.uuid16;
        break;
    case ESP_UUID_LEN_32:
        retval = mCharacteristic.uuid.uuid.uuid32;
        break;
    case ESP_UUID_LEN_128:
        ESP_LOGE(LOG_TAG, "C++ API NOT SUPPORTING 128BIT UUIDS FOR NOW");
        break;
    default:
        ESP_LOGE(LOG_TAG, "UUID IMPROPER len");
    }
    return retval;
}

std::string Characteristic::uuidstr() const
{
    switch (uuid())
    {
    case ESP_GATT_UUID_CHAR_DECLARE: /*  Characteristic Declaration*/
        return std ::string("ESP_GATT_UUID_CHAR_DECLARE");
    case ESP_GATT_UUID_CHAR_EXT_PROP: /*  Characteristic Extended Properties */
        return std::string("ESP_GATT_UUID_CHAR_EXT_PROP");
    case ESP_GATT_UUID_CHAR_DESCRIPTION: /*  Characteristic User Description*/
        return std::string("ESP_GATT_UUID_CHAR_DESCRIPTION");
    case ESP_GATT_UUID_CHAR_CLIENT_CONFIG: /*  Client Characteristic Configuration */
        return std::string("ESP_GATT_UUID_CHAR_CLIENT_CONFIG");
    case ESP_GATT_UUID_CHAR_SRVR_CONFIG: /*  Server Characteristic Configuration */
        return std::string("ESP_GATT_UUID_CHAR_SRVR_CONFIG");
    case ESP_GATT_UUID_CHAR_PRESENT_FORMAT: /*  Characteristic Presentation Format*/
        return std::string("ESP_GATT_UUID_CHAR_PRESENT_FORMAT");
    case ESP_GATT_UUID_CHAR_AGG_FORMAT: /*  Characteristic Aggregate Format*/
        return std::string("ESP_GATT_UUID_CHAR_AGG_FORMAT");
    case ESP_GATT_UUID_CHAR_VALID_RANGE: /*  Characteristic Valid Range */
        return std::string("ESP_GATT_UUID_CHAR_VALID_RANGE");
    case ESP_GATT_UUID_EXT_RPT_REF_DESCR: /*  External Report Reference */
        return std::string("ESP_GATT_UUID_EXT_RPT_REF_DESCR");
    case ESP_GATT_UUID_RPT_REF_DESCR: /*  Report Reference */
        return std::string("ESP_GATT_UUID_RPT_REF_DESCR");
    case ESP_GATT_UUID_NUM_DIGITALS_DESCR: /*  Number of Digitals */
        return std::string("ESP_GATT_UUID_NUM_DIGITALS_DESCR");
    case ESP_GATT_UUID_VALUE_TRIGGER_DESCR: /*  Value Trigger Setting */
        return std::string("ESP_GATT_UUID_VALUE_TRIGGER_DESCR");
    case ESP_GATT_UUID_ENV_SENSING_CONFIG_DESCR: /*  Environmental Sensing Configuration */
        return std::string("ESP_GATT_UUID_ENV_SENSING_CONFIG_DESCR");
    case ESP_GATT_UUID_ENV_SENSING_MEASUREMENT_DESCR: /*  Environmental Sensing Measurement */
        return std::string("ESP_GATT_UUID_ENV_SENSING_MEASUREMENT_DESCR");
    case ESP_GATT_UUID_ENV_SENSING_TRIGGER_DESCR: /*  Environmental Sensing Trigger Setting */
        return std::string("ESP_GATT_UUID_ENV_SENSING_TRIGGER_DESCR");
    case ESP_GATT_UUID_TIME_TRIGGER_DESCR: /*  Time Trigger Setting */
        return std::string("ESP_GATT_UUID_TIME_TRIGGER_DESCR");

        /* GAP Profile Attributes */
    case ESP_GATT_UUID_GAP_DEVICE_NAME:
        return std::string("ESP_GATT_UUID_GAP_DEVICE_NAME");
    case ESP_GATT_UUID_GAP_ICON:
        return std::string("ESP_GATT_UUID_GAP_ICON");
    case ESP_GATT_UUID_GAP_PREF_CONN_PARAM:
        return std::string("ESP_GATT_UUID_GAP_PREF_CONN_PARAM");
    case ESP_GATT_UUID_GAP_CENTRAL_ADDR_RESOL:
        return std::string("ESP_GATT_UUID_GAP_CENTRAL_ADDR_RESOL");

        /* Attribute Profile Attribute UUID */
    case ESP_GATT_UUID_GATT_SRV_CHGD:
        return std::string("ESP_GATT_UUID_GATT_SRV_CHGD");

        /* Link ESP_Loss Service */
    case ESP_GATT_UUID_ALERT_LEVEL: /* Alert Level */
        return std::string("ESP_GATT_UUID_ALERT_LEVEL");
    case ESP_GATT_UUID_TX_POWER_LEVEL: /* TX power level */
        return std::string("ESP_GATT_UUID_TX_POWER_LEVEL");

        /* Current Time Service */
    case ESP_GATT_UUID_CURRENT_TIME: /* Current Time */
        return std::string("ESP_GATT_UUID_CURRENT_TIME");
    case ESP_GATT_UUID_LOCAL_TIME_INFO: /* Local time info */
        return std::string("ESP_GATT_UUID_LOCAL_TIME_INFO");
    case ESP_GATT_UUID_REF_TIME_INFO: /* reference time information */
        return std::string("ESP_GATT_UUID_REF_TIME_INFO");

        /* Phone alert */
    case ESP_GATT_UUID_ALERT_STATUS: /* alert status */
        return std::string("ESP_GATT_UUID_ALERT_STATUS");
    case ESP_GATT_UUID_RINGER_CP: /* ringer control point */
        return std::string("ESP_GATT_UUID_RINGER_CP");
    case ESP_GATT_UUID_RINGER_SETTING: /* ringer setting */
        return std::string("ESP_GATT_UUID_RINGER_SETTING");

        /* Glucose Service */
    case ESP_GATT_UUID_GM_MEASUREMENT:
        return std::string("ESP_GATT_UUID_GM_MEASUREMENT");
    case ESP_GATT_UUID_GM_CONTEXT:
        return std::string("ESP_GATT_UUID_GM_CONTEXT");
    case ESP_GATT_UUID_GM_CONTROL_POINT:
        return std::string("ESP_GATT_UUID_GM_CONTROL_POINT");
    case ESP_GATT_UUID_GM_FEATURE:
        return std::string("ESP_GATT_UUID_GM_FEATURE");

        /* device information characteristic */
    case ESP_GATT_UUID_SYSTEM_ID:
        return std::string("ESP_GATT_UUID_SYSTEM_ID");
    case ESP_GATT_UUID_MODEL_NUMBER_STR:
        return std::string("ESP_GATT_UUID_MODEL_NUMBER_STR");
    case ESP_GATT_UUID_SERIAL_NUMBER_STR:
        return std::string("ESP_GATT_UUID_SERIAL_NUMBER_STR");
    case ESP_GATT_UUID_FW_VERSION_STR:
        return std::string("ESP_GATT_UUID_FW_VERSION_STR");
    case ESP_GATT_UUID_HW_VERSION_STR:
        return std::string("ESP_GATT_UUID_HW_VERSION_STR");
    case ESP_GATT_UUID_SW_VERSION_STR:
        return std::string("ESP_GATT_UUID_SW_VERSION_STR");
    case ESP_GATT_UUID_MANU_NAME:
        return std::string("ESP_GATT_UUID_MANU_NAME");
    case ESP_GATT_UUID_IEEE_DATA:
        return std::string("ESP_GATT_UUID_IEEE_DATA");
    case ESP_GATT_UUID_PNP_ID:
        return std::string("ESP_GATT_UUID_PNP_ID");

        /* HID characteristics */
    case ESP_GATT_UUID_HID_INFORMATION:
        return std::string("ESP_GATT_UUID_HID_INFORMATION");
    case ESP_GATT_UUID_HID_REPORT_MAP:
        return std::string("ESP_GATT_UUID_HID_REPORT_MAP");
    case ESP_GATT_UUID_HID_CONTROL_POINT:
        return std::string("ESP_GATT_UUID_HID_CONTROL_POINT");
    case ESP_GATT_UUID_HID_REPORT:
        return std::string("ESP_GATT_UUID_HID_REPORT");
    case ESP_GATT_UUID_HID_PROTO_MODE:
        return std::string("ESP_GATT_UUID_HID_PROTO_MODE");
    case ESP_GATT_UUID_HID_BT_KB_INPUT:
        return std::string("ESP_GATT_UUID_HID_BT_KB_INPUT");
    case ESP_GATT_UUID_HID_BT_KB_OUTPUT:
        return std::string("ESP_GATT_UUID_HID_BT_KB_OUTPUT");
    case ESP_GATT_UUID_HID_BT_MOUSE_INPUT:
        return std::string("ESP_GATT_UUID_HID_BT_MOUSE_INPUT");

        /// Heart Rate Measurement
    case ESP_GATT_HEART_RATE_MEAS:
        return std::string("ESP_GATT_HEART_RATE_MEAS");
        /// Body Sensor Location
    case ESP_GATT_BODY_SENSOR_LOCATION:
        return std::string("ESP_GATT_BODY_SENSOR_LOCATION");
        /// Heart Rate Control Point
    case ESP_GATT_HEART_RATE_CNTL_POINT:
        return std::string("ESP_GATT_HEART_RATE_CNTL_POINT");

        /* Battery Service characteristics */
    case ESP_GATT_UUID_BATTERY_LEVEL:
        return std::string("ESP_GATT_UUID_BATTERY_LEVEL");

        /* Sensor Service */
    case ESP_GATT_UUID_SC_CONTROL_POINT:
        return std::string("ESP_GATT_UUID_SC_CONTROL_POINT");
    case ESP_GATT_UUID_SENSOR_LOCATION:
        return std::string("ESP_GATT_UUID_SENSOR_LOCATION");

        /* Runners speed and cadence service */
    case ESP_GATT_UUID_RSC_MEASUREMENT:
        return std::string("ESP_GATT_UUID_RSC_MEASUREMENT");
    case ESP_GATT_UUID_RSC_FEATURE:
        return std::string("ESP_GATT_UUID_RSC_FEATURE");

        /* Cycling speed and cadence service */
    case ESP_GATT_UUID_CSC_MEASUREMENT:
        return std::string("ESP_GATT_UUID_CSC_MEASUREMENT");
    case ESP_GATT_UUID_CSC_FEATURE:
        return std::string("ESP_GATT_UUID_CSC_FEATURE");

        /* Scan ESP_Parameter characteristics */
    case ESP_GATT_UUID_SCAN_INT_WINDOW:
        return std::string("ESP_GATT_UUID_SCAN_INT_WINDOW");
    case ESP_GATT_UUID_SCAN_REFRESH:
        return std::string("ESP_GATT_UUID_SCAN_REFRESH");

    default:
        return std::string("unknown");
    }
}