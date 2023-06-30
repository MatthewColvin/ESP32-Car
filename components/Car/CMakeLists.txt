idf_component_register(SRC_DIRS "./" "./motorMixing/" 
                       INCLUDE_DIRS "./include" "./motorMixing/include" 
                       REQUIRES esp_common driver Joystick bdc_motor)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
