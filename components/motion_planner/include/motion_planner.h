#pragma once

#include "drv8825.h"
#include "common.h"

#define MOTION_MAX_COUNT 8
#define MAX_SATELLITES 4
#define MAC_ADDR_LEN 6

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
    joint_t *joint;
    int degrees;
    float duration_s;
    uint8_t direction;
    mp_movement_curve_t* profile;
} mp_joint_command_t;

typedef struct {
    uint16_t index;
    double remainder;
} mp_integration_remainder_t;

typedef struct {
    mp_movement_curve_t *curves;
    drv8825_command_t *commands;
    joint_t *joints;
    uint8_t *local_commands;
        
    size_t curve_buf_size;
    size_t command_buf_size;
    size_t joint_buf_size;
    size_t local_commands_size;
} controller_specific_buffers_t;

typedef struct mp_linked_motion mp_linked_motion_t;
struct __attribute__((packed)) mp_linked_motion {
    uint8_t command_ids[MOTION_MAX_COUNT];
    uint8_t overrides_direction[MOTION_MAX_COUNT];
    uint8_t count;
    float wait_time;
    struct mp_linked_motion *next;
};

typedef struct {
    controller_specific_buffers_t *core_buffers;
    uint8_t satellite_addrs[MAX_SATELLITES][MAC_ADDR_LEN];

    mp_linked_motion_t *motion;
} mp_motion_planner_t;

int compare_remainders_desc_cb(const void *a, const void *b);
bool create_drv8825_command(mp_joint_command_t *cmd, drv8825_command_t *out_cmd);

void create_eased_movement_curve(mp_movement_curve_config_t *cfg, mp_movement_curve_t *curve);
double mp_ease_linear(double t, double d, mp_accel_type_t a);
double mp_ease_sine(double t, double d, mp_accel_type_t a);
double mp_ease_cubic(double t, double d, mp_accel_type_t a);

void init_buffers(controller_specific_buffers_t *bufs, size_t command_buf_size, size_t curve_buf_size, size_t joint_buf_size);
mp_motion_planner_t *init_motion_planner(size_t command_buf_size, size_t curve_buf_size, size_t joints_arr_size);

void generate_empty_motion(mp_linked_motion_t *motions_arr, size_t count);
void compile_command(mp_motion_planner_t *planner, size_t on_controller, size_t output_id, size_t curve_id, size_t joint_id, int angle, uint8_t dir, mp_movement_curve_config_t *cfg);
void execute_local_commands_in_motion(mp_linked_motion_t *motion, controller_specific_buffers_t *bufs);
void execute_motion_globally(mp_motion_planner_t *planner);

void demo_motion(mp_motion_planner_t *planner);

void delete_local_buffers(controller_specific_buffers_t *bufs);
void delete_motion_planner(mp_motion_planner_t *planner);
void delete_eased_movement_curve(mp_movement_curve_t *curve);
void delete_drv8825_command(drv8825_command_t *cmd);
