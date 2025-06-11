idf_component_register(SRCS "sensirion_i2c.c" "sensirion_common.c"
                            "sensirion_i2c_hal.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver)