idf_component_register(SRCS "motor.cpp" "car.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES esp_common driver Joystick)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
