idf_component_register(SRCS "BTClassicHID.cpp" "HIDDevice.cpp" "esp_hid_gap.c"
                       INCLUDE_DIRS "./include"
                       REQUIRES bt esp_common esp_hid)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
