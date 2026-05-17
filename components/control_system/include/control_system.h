#pragma once

#include "drv8825.h"
#include "motion_planner.h"
#include "realtime.h"

// Top level wrapper around the realtime and motion_planner components
// Handles both realtime motion and pre-computed sequences

typedef struct __attribute__((packed)) {
    uint8_t control_mode;
    uint8_t integrator_mode;
} configure_command_payload_t;

typedef struct {
    control_mode_t mode;
    integrator_mode_t integrator_mode;
    union {
        mp_motion_planner_t *motion_planner;
        controller_specific_buffers_t *motion_planner_bufs;
        rt_controller_t *realtime_controller;
    } backend;
} control_system_t;

void init_control_system(control_system_t *controller, uint8_t* peer_mac, control_mode_t control_mode, integrator_mode_t integrator_mode, joint_t* joints, size_t joints_len);
void reset_control_system(control_system_t *controller);