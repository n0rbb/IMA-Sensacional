idf_component_register(SRCS "esp32s2_sensacional.c"                          
                        INCLUDE_DIRS "include"
                        REQUIRES driver
                                 Wrapper_BME280
                                 Wrapper_SGP40
                        )