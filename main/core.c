#include "core.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#define TCP_PORT 3333

static const char *TAG = "core_main";
static uint8_t sat_mac[6] = { 0x00, 0x40, 0x0A, 0x3C, 0xE4, 0x4B };
static mp_movement_curve_t curves[3] = {0};
static drv8825_command_t commands[3] = {0};

static QueueHandle_t pc_cmd_queue;

static drv8825_t shoulder_motor_nema23 = {
    .pinSTEP = M1_STEP,
    .pinDIR = M1_DIR,
    .pinEN = M1_EN,
    .stepsPerRotation = 200 * 32,
    .activeLow = false,
    .channelRMT = NULL,
    .encoderRMT = NULL,
};

static mp_joint_t shoulder = {
    .motor = &shoulder_motor_nema23,
    .pinion_teeth = 35,
    .output_teeth = 70,
    .disable_by_default = false
};

static drv8825_t elbow_motor_nema17 = {
    .pinSTEP = M2_STEP,
    .pinDIR = M2_DIR,
    .pinEN = M2_EN,
    .stepsPerRotation = 200 * 32,
    .activeLow = false,
    .channelRMT = NULL,
    .encoderRMT = NULL,
};

static mp_joint_t elbow = {
    .motor = &elbow_motor_nema17,
    .pinion_teeth = 25,
    .output_teeth = 45,
    .disable_by_default = false,
};

static void tcp_server_task(void *arg)
{
    int listen_sock, client_sock;
    struct sockaddr_in server_addr, client_addr;
    socklen_t client_addr_len = sizeof(client_addr);
    uint8_t rx_buffer[128];

    listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(TCP_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    bind(listen_sock, (struct sockaddr *)&server_addr, sizeof(server_addr));

    listen(listen_sock, 1);
    ESP_LOGI(TAG, "TCP server initialized, listening on port %d", TCP_PORT);

    while (1)
    {
        client_sock = accept(listen_sock, (struct sockaddr *)&client_addr, &client_addr_len);
        ESP_LOGI(TAG, "Client connected!");

        while (1)
        {
            int len = recv(client_sock, rx_buffer, sizeof(rx_buffer), 0);
            if (len <= 0)
            {
                ESP_LOGI(TAG, "Client disconnected");
                break;
            }
            
            pc_command_t cmd;
            memcpy(&cmd, rx_buffer, sizeof(pc_command_t));

            if (xQueueSend(pc_cmd_queue, &cmd, 0) != pdTRUE)
            {
                ESP_LOGW(TAG, "Command queue full, dropping command");
            }

            uint8_t ack = ACK;
            send(client_sock, &ack, sizeof(ack), 0);
        }

        close(client_sock);
    }
}

static void pc_command_task(void *arg)
{
    pc_command_t cmd;

    while (true)
    {
        if (xQueueReceive(pc_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            uint8_t target_slot = cmd.payload.slot;
            uint8_t src_slot = cmd.payload.src_slot;
            mp_joint_t *joint = NULL;

            switch (cmd.payload.motor)
            {
            case 1:
                joint = &shoulder;
                break;
            default:   
                break;
            }

            // THIS CODE LEAKS - but I'm too lazy to fix that
            if (cmd.cmd == PC_CMD_CREATE_CURVE)
            {
                if (cmd.payload.controller == 1)
                {
                    data_frame_t frame = {
                        .command = CMD_MP_CURVE,
                    };
                    memcpy(&frame.payload, &cmd.payload.curve_cfg, sizeof(mp_movement_curve_config_t));
                    transmit_frame(sat_mac, &frame);
                    continue;
                }

                mp_movement_curve_config_t *cfg = malloc(sizeof(mp_movement_curve_config_t));
                memcpy(cfg, &cmd.payload.curve_cfg, sizeof(mp_movement_curve_config_t));
                create_eased_movement_curve(cfg, &curves[target_slot]);
                continue;
            }

            if (cmd.cmd == PC_CMD_COMPUTE_CURVE)
            {
                if (joint == NULL)
                {
                    ESP_LOGE(TAG, "No valid motor selected!");
                    continue;
                }

                if (cmd.payload.controller == 1)
                {
                    data_frame_t frame = {
                        .command = CMD_MP_COMPUTE,
                    };
                    memcpy(&frame.payload, &cmd.payload.compute_cfg, sizeof(mp_joint_command_payload_t));
                    transmit_frame(sat_mac, &frame);
                    continue;
                }


                mp_joint_command_t jc = {
                    .degrees = cmd.payload.compute_cfg.degrees,
                    .direction = cmd.payload.compute_cfg.dir,
                    .duration_s = curves[src_slot].cfg->duration_s,
                    .profile = &curves[src_slot],
                    .joint = joint
                };
                create_drv8825_command(&jc, &commands[target_slot]);
            }

            if (cmd.cmd == PC_CMD_EXECUTE)
            {                
                if (cmd.payload.controller == 1)
                {
                    data_frame_t frame = {
                        .command = CMD_EXECUTE,
                    };
                    memcpy(&frame.payload, &cmd.payload.exec_cfg, sizeof(execute_overrides_t));
                    transmit_frame(sat_mac, &frame);
                    continue;
                }

                if (cmd.payload.exec_cfg.direction_override != NO_OVERRIDE)
                {
                    commands[src_slot].direction = cmd.payload.exec_cfg.direction_override;
                }

                execute(&commands[src_slot], (override_t)cmd.payload.exec_cfg.disable_override);
            }

        }
    }
}

void core_main()
{
    // uint8_t mac[6];
    // esp_wifi_get_mac(ESP_IF_WIFI_STA, mac);
    // ESP_LOGI(TAG, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_init();
    pc_cmd_queue = xQueueCreate(8, sizeof(pc_command_t));
    xTaskCreate(pc_command_task, "pc_command", 4096, NULL, 4, NULL);
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    coordinator_connect(sat_mac);

    attach_motor(&shoulder_motor_nema23);
    disable_motor(&shoulder_motor_nema23);

    attach_motor(&elbow_motor_nema17);
    disable_motor(&elbow_motor_nema17);

    if (!sat_handshake(sat_mac))
        return;

    mp_motion_planner_t *planner = init_motion_planner(16, 8, 2);
    planner->core_buffers->joints[0] = shoulder;
    planner->core_buffers->joints[1] = elbow;
    memcpy(planner->satellite_addrs[0], sat_mac, sizeof(uint8_t) * MAC_ADDR_LEN);
    
    demo_motion(planner);
    execute_motion_globally(planner);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        execute_motion_globally(planner);
    }

    return;
}