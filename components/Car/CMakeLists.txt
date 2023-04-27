idf_component_register(SRCS "motor.cpp" "car.cpp" "./motorMixing/TankMix.cpp" "./motorMixing/IMotorMixingStrategy.cpp"
                       INCLUDE_DIRS "./include" "./motorMixing/include"
                       REQUIRES esp_common driver Joystick)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
