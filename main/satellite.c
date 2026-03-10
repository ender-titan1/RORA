#include "satellite.h"
#include <string.h>
#include "esp_log.h"

static const char *TAG = "sat_main";

static uint8_t core_mac[6] = { 0x68, 0x25, 0xDD, 0x20, 0x29, 0x3C };

static volatile controller_specific_buffers_t *buffers;

static drv8825_t base_motor_nema17 = {
    .pinSTEP = GPIO_NUM_18,
    .pinDIR = GPIO_NUM_5,
    .pinEN = GPIO_NUM_23,
    .stepsPerRotation = 200 * 32,
    .activeLow = true,
    .channelRMT = NULL,
    .encoderRMT = NULL,
};
static mp_joint_t base = {
    .motor = &base_motor_nema17,
    .pinion_teeth = 25,
    .output_teeth = 125,
    .disable_by_default = false
};

static void sat_cmd_curve(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "MP Curve command received");

    mp_curve_command_payload_t payload;
    memcpy(&payload, cmd->payload, sizeof(mp_curve_command_payload_t));

    create_eased_movement_curve(&payload.cfg, &buffers->curves[payload.curve_id]);
}

// THIS CODE ALSO LEAKS - and this one I will have to fix, but not now
static void sat_cmd_compute(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "MP Compute command received");

    mp_joint_command_payload_t payload;
    memcpy(&payload, cmd->payload, sizeof(payload));

    mp_joint_command_t mp_cmd = {
        .joint = &buffers->joints[payload.joint_id],
        .profile = &buffers->curves[payload.curve_id],
        .degrees = payload.degrees,
        .direction = payload.dir,
    };

    mp_cmd.duration_s = mp_cmd.profile->cfg->duration_s;

    create_drv8825_command(&mp_cmd, &buffers->commands[payload.command_id]);

    // Kinda stupid but it works
    buffers->local_commands[payload.command_id] = payload.command_id + 1;
}

static void sat_cmd_exec(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "Execute command received");

    mp_linked_motion_t motion;
    memcpy(&motion, cmd->payload, sizeof(mp_linked_motion_t));

    // Execute the motion, only executing the local commands
    // The `next` value in the recieved motion is a pointer and is completely useless,
    // but I see no point making a separate struct just for transmission, so we will just ignore it
    execute_local_commands_in_motion(&motion, buffers);

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

    attach_motor(&base_motor_nema17);
    disable_motor(&base_motor_nema17);

    bind_cmd_callbacks(sat_cmd_curve, sat_cmd_compute, sat_cmd_exec, NULL);
    buffers = malloc(sizeof(controller_specific_buffers_t));
    init_buffers(buffers, 16, 8, 2);
    buffers->joints[0] = base;
}