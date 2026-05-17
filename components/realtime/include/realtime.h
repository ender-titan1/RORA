#pragma once

#include "common.h"

typedef struct {
    integrator_mode_t integrator_mode;

    joint_t *joints_arr;
    size_t joints_len;
} rt_controller_t;

void init_realtime(rt_controller_t *rt);
void rt_demo(rt_controller_t *rt);