idf_component_register(SRCS "main.cpp" "ble.cpp"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
