#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "drv8825.h"
#include "motion_planner.h"
#include "esp_random.h"
#include "esp_log.h"
#include "coordinator_common.h"

#if CONFIG_DEVICE_ROLE_CORE
    #include "coordinator_core.h"
#elif CONFIG_DEVICE_ROLE_SAT
    #include "coordinator_sat.h"
#endif

#define M1_STEP GPIO_NUM_15
#define M1_DIR GPIO_NUM_2
#define M1_EN GPIO_NUM_4

static uint8_t core_mac[6] = { 0x68, 0x25, 0xDD, 0x20, 0x29, 0x3C };
static uint8_t sat_mac[6] = { 0xEC, 0xE3, 0x34, 0x17, 0xD0, 0xC8 };

// Satellite-only global vars for callbacks

#if CONFIG_DEVICE_ROLE_SAT
volatile mp_movement_curve_t sat_curve = {0};
volatile mp_joint_command_t sat_cmd = {0};
#endif

void connect_to_peer()
{
#if CONFIG_DEVICE_ROLE_CORE
    connect(sat_mac);
#elif CONFIG_DEVICE_ROLE_SAT
    connect(core_mac);
#endif
}

#if CONFIG_DEVICE_ROLE_CORE

void core_main()
{
    drv8825_t shoulder_motor_nema23 = {
        .pinSTEP = M1_STEP,
        .pinDIR = M1_DIR,
        .pinEN = M1_EN,
        .stepsPerRotation = 200 * 32,
        .channelRMT = NULL,
        .encoderRMT = NULL,
    };

    mp_joint_t shoulder = {
        .motor = &shoulder_motor_nema23,
        .pinion_teeth = 20,
        .output_teeth = 70,
    };

    attach_motor(&shoulder_motor_nema23);

    sat_handshake(sat_mac);

    // Create movement curve on the core
    mp_movement_curve_config_t cfg = {
        .duration_s = 1,
        .accel_time_s = 0.25,
        .decel_time_s = 0.25,
        .easing_func = mp_ease_sine,
        .resolution = 0.01,
    };
    mp_movement_curve_t curve;
    create_eased_movement_curve(&cfg, &curve);

    // Create movement curve on the satellite
    mp_movement_curve_config_payload_t cfg_payload = {
        .duration_s = cfg.duration_s,
        .accel_time_s = cfg.accel_time_s,
        .decel_time_s = cfg.decel_time_s,
        .resolution = cfg.resolution,
        .ease = EASE_SINE,
    };
    data_frame_t curve_cmd = {
        .command = CMD_MP_CURVE,
        .payload = {0},
    };
    memcpy(curve_cmd.payload, (uint8_t*)&cfg_payload, sizeof(cfg_payload));
    sat_transmit_command(sat_mac, &curve_cmd);

    delete_eased_movement_curve(&curve);
}

#elif CONFIG_DEVICE_ROLE_SAT

void sat_cmd_curve(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI("SAT", "Curve command received");

    mp_movement_curve_config_payload_t payload;
    memcpy(&payload, cmd->payload, sizeof(payload));

    mp_movement_curve_config_t cfg = {
        .duration_s = payload.duration_s,
        .accel_time_s = payload.accel_time_s,
        .decel_time_s = payload.decel_time_s,
        .resolution = payload.resolution,
    };

    switch (payload.ease)
    {
    case EASE_LINEAR:
        cfg.easing_func = mp_ease_linear;
        break;
    case EASE_SINE:
        cfg.easing_func = mp_ease_linear;
        break;
    case EASE_CUBIC:
        cfg.easing_func = mp_ease_cubic;
        break;
    default:
        ESP_LOGW("SAT", "Unknown easing type detected, falling back to sine");
        cfg.easing_func = mp_ease_sine;
        break;
    }

    create_eased_movement_curve(&cfg, &sat_curve);
}

void sat_main()
{
    drv8825_t base_motor_nema17 = {
        .pinSTEP = GPIO_NUM_18,
        .pinDIR = GPIO_NUM_5,
        .pinEN = GPIO_NUM_23,
        .stepsPerRotation = 200 * 32,
        .channelRMT = NULL,
        .encoderRMT = NULL,
    };

    mp_joint_t base = {
        .motor = &base_motor_nema17,
        .pinion_teeth = 25,
        .output_teeth = 125,
    };

    attach_motor(&base_motor_nema17);

    bind_cmd_callbacks(sat_cmd_curve, NULL, NULL, NULL);
}

#endif

void app_main(void)
{
    wifi_init();
    connect_to_peer();

#if CONFIG_DEVICE_ROLE_CORE
    core_main();
#elif CONFIG_DEVICE_ROLE_SAT
    sat_main();
#endif
}