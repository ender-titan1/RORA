#pragma once

#include "drv8825.h"

typedef enum {
    ACCEL = 1,
    DECEL = -1,
} mp_accel_type_t;

typedef enum {
    EASE_LINEAR = 0,
    EASE_SINE = 1,
    EASE_CUBIC = 2,
} ease_type_t;

typedef double (*mp_easing_func_t)(double, double, mp_accel_type_t);

typedef struct {
    drv8825_t *motor;
    uint16_t pinion_teeth;
    uint16_t output_teeth;
} mp_joint_t;

typedef struct __attribute__((packed)) {
    uint8_t ease_type;
    float accel_time_s;
    float decel_time_s;
    float duration_s;
    float resolution;
} mp_movement_curve_config_t;

typedef struct {
    double *points;
    mp_movement_curve_config_t *cfg;
    uint16_t n_points;
} mp_movement_curve_t;

typedef struct {
    mp_joint_t *joint;
    int degrees;
    float duration_s;
    uint8_t direction;
    mp_movement_curve_t* profile;
} mp_joint_command_t;

typedef struct {
    uint16_t index;
    double remainder;
} mp_integration_remainder_t;

int compare_remainders_desc_cb(const void *a, const void *b);
bool create_drv8825_command(mp_joint_command_t *cmd, drv8825_command_t *out_cmd);
void create_eased_movement_curve(mp_movement_curve_config_t *cfg, mp_movement_curve_t *curve);
void delete_eased_movement_curve(mp_movement_curve_t *curve);
void delete_drv8825_command(drv8825_command_t *cmd);
double mp_ease_linear(double t, double d, mp_accel_type_t a);
double mp_ease_sine(double t, double d, mp_accel_type_t a);
double mp_ease_cubic(double t, double d, mp_accel_type_t a);
