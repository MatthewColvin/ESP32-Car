idf_component_register(SRC_DIRS "./" "./motorMixing/" "./IR"
                       INCLUDE_DIRS "./include" "./motorMixing/include" "./IR/include"
                       REQUIRES esp_common driver Joystick)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
