idf_component_register(SRCS "motor.cpp"
                       INCLUDE_DIRS "./include"
                       REQUIRES driver )

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
