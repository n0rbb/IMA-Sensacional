idf_component_register(SRCS "../Sensor_SGP40/sensirion_i2c.c" "../Sensor_SGP40/sensirion_common.c"
                            "sensirion_i2c_hal_esp32.c" "../Sensor_SGP40/sgp40_i2c.c"
                    INCLUDE_DIRS "../Sensor_SGP40"
                    REQUIRES driver
                             freertos
                             )