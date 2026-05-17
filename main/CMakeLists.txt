idf_component_register(
    SRCS "satellite.c" "core.c" "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES drv8825 motion_planner coordinator control_system realtime esp_wifi nvs_flash
)