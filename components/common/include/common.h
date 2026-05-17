#pragma once

#include "drv8825.h"

typedef struct {
    drv8825_t *motor;
    uint16_t pinion_teeth;
    uint16_t output_teeth;
    bool disable_by_default;
} joint_t;