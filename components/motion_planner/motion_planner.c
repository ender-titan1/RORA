#include <stdio.h>
#include <math.h>
#include "motion_planner.h"
#include "esp_log.h"

#define EPSILON 0.00001

#define MAX(a, b) (a > b ? a : b)

static const char* TAG = "Motion Planner";

int compare_remainders_desc_cb(const void *a, const void *b)
{
    const mp_integration_remainder_t *ra = (const mp_integration_remainder_t *)a;
    const mp_integration_remainder_t *rb = (const mp_integration_remainder_t *)b;
    
    if (ra->remainder < rb->remainder)
        return 1;

    if (ra->remainder > rb->remainder)
        return -1;

    return 0;
}

bool create_drv8825_command(mp_joint_command_t *cmd, drv8825_command_t *out_cmd)
{
    mp_joint_t *joint = cmd->joint;
    float gear_ratio = joint->output_teeth/joint->pinion_teeth; // How many times the motor has to rotate to rotate the joint 360 degrees
    float motor_degrees = cmd->degrees * gear_ratio;
    drv8825_direction dir = (cmd->direction == CW) ? CCW : CW;

    uint16_t steps = motor_degrees/360.0f * joint->motor->stepsPerRotation;

    if (cmd->profile == NULL)
    {
        double f_max = steps/cmd->duration_s;
        double t_pulse_us = (1/f_max) * 1e6;
        double pulse_us = t_pulse_us/2;

        out_cmd->motor = joint->motor;
        out_cmd->moving = true;
        out_cmd->degrees = motor_degrees;
        out_cmd->direction = dir;
        
        drv8825_pulse_provider_t p = {
            .type = CONSTANT,
            .payload = {
                .constant_pulse_us = {
                    .level0 = 1,
                    .duration0 = pulse_us,
                    .level1 = 0,
                    .duration1 = pulse_us,
                }
            }
        };
        
        out_cmd->pulse = p;

        return true;
    }

    if (cmd->duration_s != cmd->profile->cfg->duration_s)
    {
        ESP_LOGE(TAG, "Failed to create DRV8825 command! (STEP: %s)", cmd->joint->motor->pinSTEP);
        ESP_LOGE(TAG, "Duration of command != duration of movement profile!");
        ESP_LOGE(TAG, "Aborting movement profile");

        return false;
    }

    ESP_LOGI(TAG, "Computing movement profile");

    double dt = cmd->profile->cfg->resolution;
    int n = cmd->profile->n_points;

    ESP_LOGI(TAG, "[DT %f] [N %d]", dt, n);

    // Integrate the curve to find amount of arbirary units travelled

    double total_area = 0;
    for (int i = 0; i < n - 1; i++)
    {
        // Area of a trapezoid = h(a+b)/2
        total_area += (dt *(cmd->profile->points[i] + cmd->profile->points[i + 1]))/2;
    }

    ESP_LOGI(TAG, "Total Area: %f", total_area);

    if (total_area == 0)
    {
        ESP_LOGE(TAG, "Failed to create DRV8825 command! (STEP: %s)", cmd->joint->motor->pinSTEP);
        ESP_LOGE(TAG, "Movement curve area == 0");
        ESP_LOGE(TAG, "Aborting movement profile");

        return false;
    }

    // Find the amount of steps in each segment

    double steps_f_per_seg[n - 1];
    int total_steps = 0;
    for (int i = 0; i < n - 1; i++)
    {
        double seg_area = (dt * (cmd->profile->points[i] + cmd->profile->points[i + 1]))/2;
        double fraction = seg_area/total_area;
        double seg_steps = fraction * steps;
        steps_f_per_seg[i] = seg_steps;
        total_steps += floor(seg_steps);

        //ESP_LOGI(TAG, "SEG %3d | AREA %f | STEPS %f (%f-%f)", i, seg_area, seg_steps, cmd->profile->points[i], cmd->profile->points[i + 1]);
    }

    int r_steps = steps - total_steps;

    if (r_steps > 0)
    {
        ESP_LOGI(TAG, "Underfilled by %3d steps", r_steps);

        mp_integration_remainder_t remainders[n - 1];

        for (int i = 0; i < n - 1; i++)
        {
            remainders[i].index = i;
            remainders[i].remainder = steps_f_per_seg[i] - floor(steps_f_per_seg[i]);
            //ESP_LOGI(TAG, "SEG %d | REMAINDER %f", i, remainders[i].remainder);
        }

        qsort(remainders, n - 1, sizeof(mp_integration_remainder_t), compare_remainders_desc_cb);

        for (int i = 0; i < r_steps && i < n - 1; i++)
        {
            int idx = remainders[i].index;
            steps_f_per_seg[idx] += 1;
            total_steps++;
            //ESP_LOGI(TAG, "Adding step to SEG %d with remainder %f", idx, remainders[i].remainder);
        }
    }

    double pulse_us_per_seg[n - 1];
    for (int i = 0; i < n - 1; i++)
    {
        if (floor(steps_f_per_seg[i]) == 0)
            continue;

        double s_per_step = dt / floor(steps_f_per_seg[i]);
        double pulse_us = (s_per_step * 1e6) / 2.0;
        pulse_us_per_seg[i] = pulse_us;

        //ESP_LOGI(TAG, "SEG %3d | STEPS %3d | PULSE US %f (%f)", i, (int)floor(steps_f_per_seg[i]), pulse_us, pulse_us * 2.0);
    }

    rmt_symbol_word_t *rmt_symbols = malloc(steps * sizeof(rmt_symbol_word_t));
    if (!rmt_symbols)
    {
        return false;
    }

    size_t total_written = 0;
    for (int i = 0; i < n - 1; i++)
    {
        for (int j = 0; j < floor(steps_f_per_seg[i]); j++)
        {
            rmt_symbol_word_t s = {
                .level0 = 1,
                .duration0 = pulse_us_per_seg[i],
                .level1 = 0,
                .duration1 = pulse_us_per_seg[i],
            };
            rmt_symbols[total_written] = s;
            total_written++;
        }
    }


    ESP_LOGI(TAG, "Profile computation finished");
    ESP_LOGI(TAG, "[Steps/expected %d/%d]", total_steps, steps);

    out_cmd->motor = joint->motor;
    out_cmd->moving = true;
    out_cmd->degrees = motor_degrees;
    out_cmd->direction = dir;
    out_cmd->pulse.type = VARIABLE;
    out_cmd->pulse.payload.variable_pulse_us = rmt_symbols;

    return true;
}

void create_eased_movement_curve(mp_movement_curve_config_t *cfg, mp_movement_curve_t *curve)
{
    if (cfg->resolution <= EPSILON)
    {
        ESP_LOGW(TAG, "Resolution too fine, setting to acceptable resolution: %f", 100 * EPSILON);
        cfg->resolution = 100 * EPSILON;
    }

    size_t max_points = (size_t)(cfg->duration_s / cfg->resolution) + 2;
    double *points = malloc(max_points * sizeof(double));
    uint16_t i = 0;

    //ESP_LOGI(TAG, "ACCEL");

    // ACCEL
    for (double t = 0.0; t < (cfg->accel_time_s + EPSILON); t += cfg->resolution)
    {
        points[i] = cfg->easing_func(t, cfg->accel_time_s, ACCEL);
        //ESP_LOGI(TAG, "%d| %f: %f", i, t, points[i]);
        i++;
    }

    //ESP_LOGI(TAG, "HOLD");

    // Hold
    double hold_time = cfg->duration_s - (cfg->accel_time_s + cfg->decel_time_s);
    for (double t = 0.0; t < (hold_time + EPSILON); t += cfg->resolution)
    {
        points[i] = 1.0;
        //ESP_LOGI(TAG, "%d| %f: %f", i, t, points[i]);
        i++;
    }

    //ESP_LOGI(TAG, "DECEL");
        
    // DECEL
    for (double t = 0.0; t <= (cfg->decel_time_s + EPSILON); t += cfg->resolution)
    {
        points[i] = cfg->easing_func(t, cfg->decel_time_s, DECEL);
        //ESP_LOGI(TAG, "%d| %f: %f", i, t, points[i]);
        i++;
    }

    curve->cfg = cfg;
    curve->n_points = i;
    curve->points = points;
}

void delete_eased_movement_curve(mp_movement_curve_t *curve)
{
    free(curve->points);
}

void delete_drv8825_command(drv8825_command_t *cmd)
{
    if (cmd->pulse.type == VARIABLE)
    {
        free(cmd->pulse.payload.variable_pulse_us);
    }
}

double mp_ease_linear(double t, double d, mp_accel_type_t a)
{
    double a_d = (double)a;
    return a_d*t*(1.0/d)+MAX(-a_d, 0.0);
}

double mp_ease_sine(double t, double d, mp_accel_type_t a)
{
    double a_d = (double)a;
    double x = t * (1.0/d);
    return -a_d * ((1+cos(M_PI*x))/2) + MAX(a_d, 0);
}

double mp_ease_cubic(double t, double d, mp_accel_type_t a)
{
    double a_d = (double)a;
    double x = t * (1.0/d);
    return a_d*(3*pow(x, 2) - 2*pow(x, 3)) + MAX(-a_d, 0);
}
