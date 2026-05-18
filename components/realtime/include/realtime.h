#pragma once

#include "common.h"

#define RT_UPDATE_LOOP_HZ 500
#define RT_UPDATE_PERIOD_MS (1000 / RT_UPDATE_LOOP_HZ)

typedef struct {
    integrator_mode_t integrator_mode;

    joint_t *joints_arr;
    size_t joints_len;
} rt_controller_t;

void init_realtime(rt_controller_t *rt);
void rt_demo(rt_controller_t *rt);