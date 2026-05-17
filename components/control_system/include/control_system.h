#pragma once

#include "drv8825.h"
#include "motion_planner.h"
#include "realtime.h"

// Top level wrepper around the realtime and motion_planner components
// Handles both realtime motion and pre-computed sequences

typedef struct {
    control_mode_t mode;
    union {
        mp_motion_planner_t *motion_planner;
        rt_controller_t *realtime_controller;
    } used_system;
} control_system_t;

