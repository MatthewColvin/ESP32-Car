idf_component_register(SRCS "ble.cpp" "spp_client_demo.cpp"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PUBLIC -std=gnu++2b)
