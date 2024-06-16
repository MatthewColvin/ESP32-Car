idf_component_register(SRCS "LightSensor.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES esp_common driver esp_timer )

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)