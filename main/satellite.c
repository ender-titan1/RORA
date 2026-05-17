#include "satellite.h"
#include <string.h>
#include "esp_log.h"
#include "control_system.h"

static const char *TAG = "sat_main";

static uint8_t core_mac[6] = { 0xAC, 0xA7, 0x04, 0x2C, 0x59, 0x28 };

static control_system_t *controller = NULL;

static drv8825_t base_motor_nema17 = {
    .pin_STEP = GPIO_NUM_3,
    .pin_DIR = GPIO_NUM_2,
    .pin_EN = GPIO_NUM_4,
    .steps_per_rotation = 200 * 32,
    .active_low = true,
    .rmt_channel = NULL,
    .rmt_encoder = NULL,
};
static joint_t base = {
    .motor = &base_motor_nema17,
    .pinion_teeth = 25,
    .output_teeth = 125,
    .disable_by_default = false
};

static void sat_cmd_curve(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "MP Curve command received");

    if (controller == NULL)
    {
        ESP_LOGE(TAG, "Cannot perform command, control system uninitialized");
        return;
    }

    if (controller->mode != OFFLINE)
    {
        ESP_LOGE(TAG, "Satellite offline mode callback called despite REALTIME mode");
        return;
    }
    
    mp_curve_command_payload_t payload;
    memcpy(&payload, cmd->payload, sizeof(mp_curve_command_payload_t));

    create_eased_movement_curve(&payload.cfg, &controller->used_system.motion_planner_bufs->curves[payload.curve_id]);
}

// THIS CODE ALSO LEAKS - and this one I will have to fix, but not now
static void sat_cmd_compute(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "MP Compute command received");

    if (controller == NULL)
    {
        ESP_LOGE(TAG, "Cannot perform command, control system uninitialized");
        return;
    }

    if (controller->mode != OFFLINE)
    {
        ESP_LOGE(TAG, "Satellite offline mode callback called despite REALTIME mode");
        return;
    }
    
    mp_joint_command_payload_t payload;
    memcpy(&payload, cmd->payload, sizeof(mp_joint_command_payload_t));
    
    mp_joint_command_t mp_cmd = {
        .joint = &controller->used_system.motion_planner_bufs->joints[payload.joint_id],
        .profile = &controller->used_system.motion_planner_bufs->curves[payload.curve_id],
        .degrees = payload.degrees,
        .direction = payload.dir,
    };

    mp_cmd.duration_s = mp_cmd.profile->cfg->duration_s;

    create_drv8825_command(&mp_cmd, &controller->used_system.motion_planner_bufs->commands[payload.command_id]);

    // Kinda stupid but it works
    controller->used_system.motion_planner_bufs->local_commands[payload.command_id] = payload.command_id + 1;
}

static void sat_cmd_exec(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "Execute command received");

    if (controller == NULL)
    {
        ESP_LOGE(TAG, "Cannot perform command, control system uninitialized");
        return;
    }

    if (controller->mode != OFFLINE)
    {
        ESP_LOGE(TAG, "Satellite offline mode callback called despite REALTIME mode");
        return;
    }
    
    mp_linked_motion_t motion;
    memcpy(&motion, cmd->payload, sizeof(mp_linked_motion_t));

    // Execute the motion, only executing the local commands
    // The `next` value in the recieved motion is a pointer and is completely useless,
    // but I see no point making a separate struct just for transmission, so we will just ignore it
    execute_local_commands_in_motion(&motion, controller->used_system.motion_planner_bufs);

    data_frame_t ack = {
        .command = ACK,
        .payload = {0}
    };

    transmit_frame(core_mac, &ack);
}

static void sat_cmd_config(data_frame_t *cmd, uint8_t len)
{
    ESP_LOGI(TAG, "Configure command received");

    configure_command_payload_t cfg;
    memcpy(&cfg, cmd->payload, sizeof(configure_command_payload_t));

    if (controller == NULL)
    {
        controller = malloc(sizeof(control_system_t));
    }

    joint_t joints[1] = {
        base
    };
    init_control_system(controller, core_mac, cfg.control_mode, cfg.integrator_mode, joints, 1);

    if (cfg.control_mode == OFFLINE)
    {
        coordinator_curve_callback = sat_cmd_curve;
        coordinator_compute_callback = sat_cmd_compute;
        coordinator_exec_callback = sat_cmd_exec;
    }
}

void sat_main()
{
    wifi_init();
    coordinator_connect(core_mac);
    coordinator_configure_callback = sat_cmd_config;
}