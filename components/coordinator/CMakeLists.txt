idf_component_register(
    SRCS "coordinator.c" "coordinator_core.c" "coordinator_sat.c"

    INCLUDE_DIRS "include"
    REQUIRES esp_wifi nvs_flash
    REQUIRES "drv8825"
)
