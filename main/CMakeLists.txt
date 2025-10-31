idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES drv8825
    PRIV_REQUIRES motion_planner
    PRIV_REQUIRES satellite
    PRIV_REQUIRES esp_wifi nvs_flash
)