idf_component_register(SRCS "main.cpp" 
                    INCLUDE_DIRS "."
                    REQUIRES Servo Car LED Servo)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)

