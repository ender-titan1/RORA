#include "core.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#define TCP_PORT 3333

static const char *TAG = "core_main";
static uint8_t sat_mac[6] = { 0xEC, 0xE3, 0x34, 0x17, 0xD0, 0xC8 };
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
    .pinion_teeth = 20,
    .output_teeth = 70,
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
            if (cmd.payload.controller != 0)
            {
                ESP_LOGW(TAG, "Satellite commands not implemented yet!");
                continue;
            }

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

                mp_joint_command_t jc = {
                    .degrees = cmd.payload.cmd.degrees,
                    .direction = cmd.payload.cmd.dir,
                    .duration_s = curves[src_slot].cfg->duration_s,
                    .profile = &curves[src_slot],
                    .joint = joint
                };
                ESP_LOGI(TAG, "%f", curves[src_slot].cfg->duration_s);
                ESP_LOGI(TAG, "%d", joint->motor->pinSTEP);
                create_drv8825_command(&jc, &commands[target_slot]);
            }

            if (cmd.cmd == PC_CMD_EXECUTE)
            {
                execute(&commands[src_slot]);
                disable_motor(commands[src_slot].motor);
            }

        }
    }
}

void core_main()
{
    wifi_init();
    pc_cmd_queue = xQueueCreate(8, sizeof(pc_command_t));
    xTaskCreate(pc_command_task, "pc_command", 4096, NULL, 4, NULL);
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    coordinator_connect(sat_mac);

    attach_motor(&shoulder_motor_nema23);
    disable_motor(&shoulder_motor_nema23);

    sat_handshake(sat_mac);

    return;

    ESP_LOGW(TAG, "Creating movement curve...");

    // Create movement curve on the core
    mp_movement_curve_config_t cfg = {
        .duration_s = 1,
        .accel_time_s = 0.25,
        .decel_time_s = 0.25,
        .ease_type = EASE_SINE,
        .resolution = 0.01,
    };
    mp_movement_curve_t curve;
    create_eased_movement_curve(&cfg, &curve);

    // Create movement curve on the satellite
    data_frame_t curve_cmd = {
        .command = CMD_MP_CURVE,
        .payload = {0},
    };
    memcpy(curve_cmd.payload, (uint8_t*)&cfg, sizeof(cfg));
    transmit_frame(sat_mac, &curve_cmd);
    if (!await_response())
    {
        ESP_LOGW(TAG, "MP Curve command - no ACK");
    }

    ESP_LOGW(TAG, "Computing joint command...");

    // Crete movement command on the core
    mp_joint_command_t jc = {
        .joint = &shoulder,
        .profile = &curve,
        .degrees = 90,
        .direction = CW,
        .duration_s = 1,
    };
    drv8825_command_t motor_cmd = {0};
    create_drv8825_command(&jc, &motor_cmd);

    // Create movement command on the satellite
    mp_joint_command_payload_t jcp = {
        .degrees = 180,
        .dir = CCW,
    };
    data_frame_t compute_cmd = {
        .command = CMD_MP_COMPUTE,
        .payload = {0},
    };
    memcpy(compute_cmd.payload, (uint8_t*)&jcp, sizeof(jcp));
    transmit_frame(sat_mac, &compute_cmd);
    if (!await_response())
    {
        ESP_LOGW(TAG, "MP Compute command - no ACK");
    }

    // Execute
    data_frame_t execute_cmd = {
        .command = CMD_EXECUTE,
        .payload = {0},
    };
    transmit_frame(sat_mac, &execute_cmd);
    await_response();
    execute(&motor_cmd);

    detach_motor(&shoulder_motor_nema23);
    delete_drv8825_command(&motor_cmd);
    delete_eased_movement_curve(&curve);
}