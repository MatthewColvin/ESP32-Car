idf_component_register(SRC_DIRS "./"
                       INCLUDE_DIRS "./include"
                       REQUIRES bt driver esp_common nvs_flash)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
