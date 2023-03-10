idf_component_register(SRCS "joystick.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES BLE)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)