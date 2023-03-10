idf_component_register(SRCS "device.cpp" "ble.cpp" "service.cpp" "characteristic.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES bt driver esp_common nvs_flash)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)