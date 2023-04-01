idf_component_register(SRCS "mocute052.cpp" "joystick.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES BLE btHIDHost)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)