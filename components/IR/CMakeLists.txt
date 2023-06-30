idf_component_register(SRC_DIRS "./"
                       INCLUDE_DIRS "./include" 
                       REQUIRES esp_common driver )

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
