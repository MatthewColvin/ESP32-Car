idf_component_register(SRCS "ble.cpp"  "device.cpp" "service.cpp" "characteristic.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES bt driver esp_common nvs_flash)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)


find_package(Doxygen
             OPTIONAL_COMPONENTS dot mscgen dia)

set(DOXYGEN_PROJECT_BRIEF "BLE LIB")
set(DOXYGEN_GENERATE_HTML YES)
set(DOXYGEN_HTML_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/docs")
doxygen_add_docs(BLE_DOCS 
                ./include
                )