idf_component_register(SRCS "main.cpp" 
                    INCLUDE_DIRS "."
                    REQUIRES ServoMotor Car LED IR)

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)

