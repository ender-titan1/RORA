idf_component_register(
    SRCS "drv8825.c"
    INCLUDE_DIRS "include"
    REQUIRES "driver"
    REQUIRES esp_driver_rmt
)
