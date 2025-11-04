#include "satellite.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "sat_main";

static uint8_t core_mac[6] = { 0x68, 0x25, 0xDD, 0x20, 0x29, 0x3C };

static volatile mp_movement_curve_config_t sat_curve_cfg = {0};
static volatile mp_movement_curve_t sat_curve = {0};
static volatile mp_joint_command_t sat_cmd = {0};
static volatile drv8825_command_t sat_drv8825_cmd = {0};
static drv8825_t base_motor_nema17 = {0};
static mp_joint_t base = {0};

static void sat_cmd_curve(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "MP Curve command received");

    memcpy(&sat_curve_cfg, cmd->payload, sizeof(mp_movement_curve_config_t));

    create_eased_movement_curve(&sat_curve_cfg, &sat_curve);
}

// THIS CODE ALSO LEAKS - and this one I will have to fix, but not now
static void sat_cmd_compute(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "MP Compute command received");

    mp_joint_command_payload_t payload;
    memcpy(&payload, cmd->payload, sizeof(payload));

    mp_joint_command_t mp_cmd = {
        .joint = &base,
        .profile = &sat_curve,
        .duration_s = sat_curve.cfg->duration_s,
        .degrees = payload.degrees,
        .direction = payload.dir,
    };

    create_drv8825_command(&mp_cmd, &sat_drv8825_cmd);
}

static void sat_cmd_exec(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "Execute command received");
    execute(&sat_drv8825_cmd);

    data_frame_t ack = {
        .command = ACK,
        .payload = {0}
    };

    transmit_frame(core_mac, &ack);
}

void sat_main()
{
    wifi_init();
    coordinator_connect(core_mac);

    base_motor_nema17.pinSTEP = GPIO_NUM_18,
    base_motor_nema17.pinDIR = GPIO_NUM_5,
    base_motor_nema17.pinEN = GPIO_NUM_23,
    base_motor_nema17.stepsPerRotation = 200 * 32,
    base_motor_nema17.activeLow = true,
    base_motor_nema17.channelRMT = NULL,
    base_motor_nema17.encoderRMT = NULL,

    base.motor = &base_motor_nema17,
    base.pinion_teeth = 25,
    base.output_teeth = 125,

    attach_motor(&base_motor_nema17);

    bind_cmd_callbacks(sat_cmd_curve, sat_cmd_compute, sat_cmd_exec, NULL);
}