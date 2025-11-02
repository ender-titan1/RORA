#include "coordinator_sat.h"
#include "esp_log.h"

const char *TAG = "Coordinator Satellite";

sat_callback_func curve_callback;
sat_callback_func compute_callback;
sat_callback_func exec_callback;
sat_callback_func cleanup_callback;

static void sat_on_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    data_frame_t cmd;
    memcpy(&cmd, data, sizeof(data_frame_t));

    uint8_t generated_crc = crc8_gen(data, len - 1);
    uint8_t original_crc = cmd.crc;

    if (generated_crc != original_crc)
    {
        ESP_LOGW(TAG, "Data received, CRC8 invalid (expected %02X, got %02X)", original_crc, generated_crc);
        return;
    }

    switch (cmd.command)
    {
    case CMD_MP_CURVE:
        curve_callback(&cmd, len);
        break;
    case CMD_MP_COMPUTE:
        compute_callback(&cmd, len);
        break;
    case CMD_EXECUTE:
        exec_callback(&cmd, len);
        break;
    case CMD_CLEANUP:
        cleanup_callback(&cmd, len);
        break;
    case CMD_HANDSHAKE:
        ESP_LOGI(TAG, "Handshake command received. Responding.");
        break;
    default:
        ESP_LOGE(TAG, "Unknown command sent: %02X", cmd.command);
        break;
    }

    // Send NACK for exec command, we'll send ACK once the movement has been completed
    uint8_t ack_nack = (cmd.command == CMD_EXECUTE) ? NACK : ACK;

    data_frame_t ack = {
        .command = ack_nack,
        .payload = {0}
    };

    ack.crc = crc8_gen((uint8_t*)&ack, sizeof(data_frame_t) - 1);
    esp_now_send(recv_info->src_addr, (uint8_t*)&ack, sizeof(ack));
}

void sat_init()
{
    esp_now_register_recv_cb(sat_on_recv_cb);
    ESP_LOGI(TAG, "Initialized device as SAT");
}

void bind_cmd_callbacks(sat_callback_func curve_cb, sat_callback_func compute_cb, sat_callback_func exec_cb, sat_callback_func cleanup_cb)
{
    curve_callback = curve_cb;
    compute_callback = compute_cb;
    exec_callback = exec_cb;
    cleanup_callback = cleanup_cb;
}
