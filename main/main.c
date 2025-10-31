#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "drv8825.h"
#include "motion_planner.h"
#include "esp_random.h"
#include "esp_log.h"
#include "satellite.h"

#define M1_STEP GPIO_NUM_15
#define M1_DIR GPIO_NUM_2
#define M1_EN GPIO_NUM_4

#define M2_EN GPIO_NUM_23
#define M2_STEP GPIO_NUM_22
#define M2_DIR GPIO_NUM_21

// CORE MAC 68:25:DD:20:29:3C
// SATELLITE MAC EC:E3:34:17:D0:C8

void app_main(void)
{
    wifi_init();

    return;

    drv8825_t shoulder_motor_nema23 = {
        .pinSTEP = M1_STEP,
        .pinDIR = M1_DIR,
        .pinEN = M1_EN,
        .stepsPerRotation = 200 * 32,
        .channelRMT = NULL,
        .encoderRMT = NULL,
    };

    drv8825_t base_motor_nema17 = {
        .pinSTEP = M2_STEP,
        .pinDIR = M2_DIR,
        .pinEN = M2_EN,
        .stepsPerRotation = 200 * 32,
        .channelRMT = NULL,
        .encoderRMT = NULL,
    };

    mp_joint_t base = {
        .motor = &base_motor_nema17,
        .pinion_teeth = 25,
        .output_teeth = 125,
    };

    mp_joint_t shoulder = {
        .motor = &shoulder_motor_nema23,
        .pinion_teeth = 20,
        .output_teeth = 70,
    };

    attach_motor(&shoulder_motor_nema23);
    attach_motor(&base_motor_nema17);
    
    mp_movement_curve_config_t cfg = {
        .duration_s = 1,
        .accel_time_s = 0.25,
        .decel_time_s = 0.25,
        .easing_func = mp_ease_sine,
        .resolution = 0.01,
    };
    mp_movement_curve_t curve;
    create_eased_movement_curve(&cfg, &curve);

    mp_joint_command_t joint_cmd_base = {
        .joint = &base,
        .degrees = 180,
        .duration_s = 1,
        .direction = CW,
        .profile = &curve
    };
    drv8825_command_t cmd_base;
    bool result = create_drv8825_command(&joint_cmd_base, &cmd_base);
    if (!result)
    {
        return;
    }

    mp_joint_command_t joint_cmd_shoulder = {
        .joint = &shoulder,
        .degrees = 120,
        .duration_s = 1,
        .direction = CW,
        .profile = &curve,
    };
    drv8825_command_t cmd_shoulder;
    bool result2 = create_drv8825_command(&joint_cmd_shoulder, &cmd_shoulder);
    if (!result2)
    {
        return;
    }

    drv8825_direction dir = CW;

    while (1)
    {
        cmd_base.direction = dir;
        cmd_shoulder.direction = dir;
        drv8825_command_t arr[] = {
            //cmd_base,
            cmd_shoulder
        };
        execute_sync(1, arr);
        dir = (dir == CW) ? CCW : CW;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Clean up
    delete_drv8825_command(&cmd_base);
    delete_eased_movement_curve(&curve);
    detach_motor(&base_motor_nema17);
    detach_motor(&shoulder_motor_nema23);
}