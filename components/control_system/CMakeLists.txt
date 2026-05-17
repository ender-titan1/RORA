idf_component_register(
    SRCS "control_system.c"
    INCLUDE_DIRS "include"
    REQUIRES "drv8825" "coordinator" "motion_planner" "realtime"
)
