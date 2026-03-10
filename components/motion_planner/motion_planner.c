#include <stdio.h>
#include <math.h>
#include "motion_planner.h"
#include "esp_log.h"
#include "coordinator_common.h"
#include "coordinator_core.h"

#define EPSILON 0.00001
#define DEMO_FAST 1
#define DEMO_NORMAL 0
#define DEMO_SLOW 2

#define MAX(a, b) (a > b ? a : b)

static const char* TAG = "motion_planner";

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
    float gear_ratio = ((float)joint->output_teeth)/((float)joint->pinion_teeth); // How many times the motor has to rotate to rotate the joint 360 degrees
    float motor_degrees = ((float)cmd->degrees) * gear_ratio;
    uint8_t dir = (cmd->direction == CW) ? CCW : CW;

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

    if (total_area == 0.0)
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
    out_cmd->disable = cmd->joint->disable_by_default;
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

    mp_easing_func_t easing_func;

    switch (cfg->ease_type)
    {
    case EASE_LINEAR:
        easing_func = mp_ease_linear;
        break;
    case EASE_SINE:
        easing_func = mp_ease_sine;
        break;
    case EASE_CUBIC:
        easing_func = mp_ease_cubic;
        break;
    default:
        ESP_LOGW(TAG, "Unknown easing type detected, falling back to sine");
        easing_func = mp_ease_sine;
        break;
    }

    size_t max_points = (size_t)(cfg->duration_s / cfg->resolution) + 2;
    double *points = malloc(max_points * sizeof(double));
    uint16_t i = 0;

    //ESP_LOGI(TAG, "ACCEL");

    // ACCEL
    for (double t = 0.0; t < (cfg->accel_time_s + EPSILON); t += cfg->resolution)
    {
        points[i] = easing_func(t, cfg->accel_time_s, ACCEL);
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
        points[i] = easing_func(t, cfg->decel_time_s, DECEL);
        //ESP_LOGI(TAG, "%d| %f: %f", i, t, points[i]);
        i++;
    }

    memcpy(&curve->cfg, &cfg, sizeof(mp_movement_curve_config_t));
    curve->n_points = i;
    curve->points = points;
}

void delete_local_buffers(controller_specific_buffers_t *bufs)
{
    for (size_t i = 0; i < bufs->curve_buf_size; i++)
    {
        delete_eased_movement_curve(&bufs->curves[i]);
    }

    free(bufs->curves);

    for (size_t i = 0; i < bufs->command_buf_size; i++)
    {
        delete_drv8825_command(&bufs->commands[i]);
    }

    free(bufs->commands);
    free(bufs->joints);
    free(bufs->local_commands);
    free(bufs);
}

// This code is almost certainly some form of broken
void delete_motion_planner(mp_motion_planner_t *planner)
{
    mp_linked_motion_t *cur = planner->motion;

    while (cur) 
    {
        mp_linked_motion_t *next = cur->next;
        free(cur);
        cur = next;
    }

    delete_local_buffers(planner->core_buffers);
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

void init_buffers(controller_specific_buffers_t *bufs, size_t command_buf_size, size_t curve_buf_size, size_t joint_buf_size)
{
    bufs->commands = malloc(sizeof(drv8825_command_t) * command_buf_size);
    bufs->curves = malloc(sizeof(mp_movement_curve_t) * curve_buf_size);
    bufs->joints = malloc(sizeof(mp_joint_t) * joint_buf_size);
    bufs->local_commands = malloc(sizeof(uint8_t) * command_buf_size);

    bufs->command_buf_size = command_buf_size;
    bufs->curve_buf_size = curve_buf_size;
    bufs->joint_buf_size = joint_buf_size;
    bufs->local_commands_size = command_buf_size;

    for (size_t i = 0; i < command_buf_size; i++)
        bufs->local_commands[i] = 0x00;
}

mp_motion_planner_t *init_motion_planner(size_t command_buf_size, size_t curve_buf_size, size_t joints_arr_size)
{
    mp_motion_planner_t* handle = calloc(1, sizeof(mp_motion_planner_t));
    handle->core_buffers = malloc(sizeof(controller_specific_buffers_t));
    init_buffers(handle->core_buffers, command_buf_size, curve_buf_size, joints_arr_size);

    return handle;
}

void generate_empty_motion(mp_linked_motion_t *motions_arr, size_t count)
{
    if (motions_arr == NULL || count == 0)
        return;

    for (size_t i = 0; i < count; i++) 
    {
        for (size_t j = 0; j < MOTION_MAX_COUNT; j++)
        {
            motions_arr[i].overrides_direction[j] = NO_OVERRIDE;
        }
        
        motions_arr[i].wait_time = 0.3;

        if (i + 1 < count)
            motions_arr[i].next = &motions_arr[i + 1];
        else
            motions_arr[i].next = NULL;
    }
}

void compile_command(mp_motion_planner_t *planner, size_t on_controller, size_t output_id, size_t curve_id, size_t joint_id, int angle, uint8_t dir, mp_movement_curve_config_t *cfg)
{
    ESP_LOGI(TAG, "Compiling command: %i", output_id);

    if (on_controller == 0)
    {
        if (cfg != NULL)
        {
            ESP_LOGI(TAG, "Command compilation will override curve id %i", curve_id);
            create_eased_movement_curve(cfg, &planner->core_buffers->curves[curve_id]);  
        }

        mp_joint_command_t command = {
            .degrees = angle,
            .direction = dir,
            .joint = &planner->core_buffers->joints[joint_id],
            .profile = &planner->core_buffers->curves[curve_id],
            .duration_s = planner->core_buffers->curves[curve_id].cfg->duration_s
        };

        create_drv8825_command(&command, &planner->core_buffers->commands[output_id]);
        // Kinda stupid but it works
        planner->core_buffers->local_commands[output_id] = output_id + 1;
        return;
    }

    if (cfg != NULL)
    {
        ESP_LOGI(TAG, "Command compilation will override curve id %i", curve_id);
        data_frame_t frame = {
            .command = CMD_MP_CURVE,
        };
        mp_curve_command_payload_t payload = {
            .curve_id = curve_id,
            .cfg = *cfg,
        };
        memcpy(frame.payload, &payload, sizeof(mp_curve_command_payload_t));
        transmit_frame(planner->satellite_addrs[on_controller - 1], &frame);
        if (!await_response())
        {
            ESP_LOGE(TAG, "Failed to recieve ACK on curve command!");
        }
    }

    data_frame_t frame = {
        .command = CMD_MP_COMPUTE,
    };
    mp_joint_command_payload_t payload = {
        .command_id = output_id,
        .joint_id = joint_id,
        .curve_id = curve_id,
        .degrees = angle,
        .dir = dir
    };
    memcpy(&frame.payload, &payload, sizeof(mp_joint_command_payload_t));
    transmit_frame(planner->satellite_addrs[on_controller - 1], &frame);
    if (!await_response())
    {
        ESP_LOGE(TAG, "Failed to recieve ACK on compute command!");
    }
}

void execute_local_commands_in_motion(mp_linked_motion_t *motion, controller_specific_buffers_t *bufs)
{
    drv8825_command_t commands[motion->count];
    uint8_t count = 0;
    for (size_t i = 0; i < motion->count; i++)
    {
        drv8825_command_t cmd = bufs->commands[motion->command_ids[i]];

        for (size_t j = 0; j < bufs->local_commands_size; j++)
        {
            // Skip check if current value is 0, as that just means no data is present
            if (bufs->local_commands[j] == 0)
                continue;

            // If the command being loaded is local to this controller, execute.
            // Else, skip this command and move onto the next one in this motion.
            if (bufs->local_commands[j] - 1 == motion->command_ids[i]) 
                goto execute;
        }

        // Skip command loaded on a different controller
        continue;
execute:

        commands[count] = bufs->commands[motion->command_ids[i]];
        uint8_t ovr = motion->overrides_direction[i];
        if (ovr != NO_OVERRIDE)
            commands[count].direction = ovr;

        count++;
    }

    execute_sync(count, commands, NO_OVERRIDE);
}

void execute_motion_globally(mp_motion_planner_t *planner)
{
    mp_linked_motion_t *cur = planner->motion;
    while (cur) 
    {
        // Send EXECUTE commands to all satellites
        data_frame_t frame = {
            .command = CMD_EXECUTE,
        };
        memcpy(&frame.payload, &cur, sizeof(mp_linked_motion_t));
        for (size_t j = 0; j < MAX_SATELLITES; j++)
        {
            if (planner->satellite_addrs[j][0] == 0x00)
                continue;

            transmit_frame(planner->satellite_addrs[j], &frame);
        }

        // Execute commands locally
        execute_local_commands_in_motion(cur, planner->core_buffers);

        cur = cur->next;
    }
}

void demo_motion(mp_motion_planner_t *planner)
{
    ESP_LOGI(TAG, "%i", planner->core_buffers->joints[0].output_teeth);

    mp_movement_curve_config_t cfg = {
        .accel_time_s = 0.2,
        .decel_time_s = 0.2,
        .duration_s = 1,
        .ease_type = EASE_CUBIC,
        .resolution = 0.01,
    };

    // REMEMBER: Curves are stored per-conrtoller, so you have to pass in a new cfg even
    // when reusing the same curve when it's on a different controller
    
    // Compile: ID 1 |     base |  90 deg |  CW | new curve (1s; 0.2s a/d) (normal)
    compile_command(planner, 1, 1, DEMO_NORMAL, 0, 90, CW, &cfg);
    // Compile: ID 4 | shoulder |  20 deg | CCW | normal curve
    compile_command(planner, 0, 4, DEMO_NORMAL, 0, 20, CCW, &cfg);
    
    cfg.duration_s = 0.75;
    cfg.accel_time_s = 0.15,
    cfg.decel_time_s = 0.15;
    
    // Compile: ID 2 | shoulder |  45 deg | CCW | new curve (0.75s, 0.15s a/d) (fast)
    compile_command(planner, 0, 2, DEMO_FAST, 0, 45, CCW, &cfg);
    
    cfg.duration_s = 2;
    cfg.accel_time_s = 0.4;
    cfg.decel_time_s = 0.4;
    
    // Compile: ID 3 |     base | 180 deg | CCW | new curve (2s; 0.4s a/d) (slow)
    compile_command(planner, 1, 3, DEMO_SLOW, 0, 180, CCW, &cfg);

    mp_linked_motion_t *motions = malloc(sizeof(mp_linked_motion_t) * 3);
    generate_empty_motion(motions, 3); 
    
    // Synched move of base (90 CW normal) and shoulder (45 CCW fast)
    mp_linked_motion_t *m0 = &motions[0];
    m0->command_ids[0] = 1;
    m0->command_ids[1] = 2;
    m0->count = 2;
    
    // Move of shoulder with overriden direction (45 CW fast)
    mp_linked_motion_t *m1 = &motions[1];
    m1->command_ids[0] = 2;
    m1->overrides_direction[0] = CW;
    m1->count = 1;
    
    // Synched move of base (180 CCW slow) and shoulder (20 CCW normal)
    mp_linked_motion_t *m2 = &motions[2];
    m2->command_ids[0] = 3;
    m2->command_ids[1] = 4;
    m2->count = 2;

    planner->motion = &motions[0];
}