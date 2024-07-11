idf_component_register(SRCS "servo.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES esp_common driver LED Joystick)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)