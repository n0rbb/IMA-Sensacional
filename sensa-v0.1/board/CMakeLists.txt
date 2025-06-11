idf_component_register(SRCS "esp32s2_sensacional.c"                          
                        INCLUDE_DIRS "include"
                        REQUIRES driver
                                 Sensor_BME280
                                 Sensor_SGP40
                        )