idf_component_register(SRCS "led.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES esp_common driver)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)