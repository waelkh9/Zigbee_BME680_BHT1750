idf_component_register(SRCS "bh1750_i2c.c" "bh1750_i2c_hal.c" 
                    INCLUDE_DIRS "."
                    REQUIRES i2cdev log esp_idf_lib_helpers)