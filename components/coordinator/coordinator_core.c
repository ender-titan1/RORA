#include "coordinator_core.h"
#include "esp_log.h"

static const char *TAG = "coordinator";
static volatile bool ack_received = false;

static void core_on_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    data_frame_t *frame = (data_frame_t*)data;
    uint8_t generated_crc = crc8_gen(data, len - 1);
    uint8_t original_crc = frame->crc;

    if (generated_crc != original_crc)
    {
        ESP_LOGW(TAG, "Data received, CRC8 invalid (expected %02X, got %02X)", original_crc, generated_crc);
        ack_received = false;
        return;
    }

    if (frame->command == ACK)
    {
        //ESP_LOGI(TAG, "ACK received");
        ack_received = true;
        return;
    }

    if (frame->command == NACK)
    {
        ESP_LOGI(TAG, "NACK received, waiting for ACK");
        ack_received = false;
        return;
    }

    ESP_LOGW(TAG, "Data received, but no meaningful payload found!");
    ack_received = false;
    return;
}

void core_init()
{
    esp_now_register_recv_cb(core_on_recv_cb);
    ESP_LOGI(TAG, "Initialized device as CORE");
}

bool await_response()
{
    uint32_t start_ticks = xTaskGetTickCount();
    while (!ack_received) 
    {
        if (xTaskGetTickCount() - start_ticks > pdMS_TO_TICKS(100))
            break;

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    bool ack = ack_received;
    ack_received = false;
    return ack;
}

bool sat_handshake(uint8_t *mac)
{
    ack_received = false;
    uint32_t start_ticks = xTaskGetTickCount();
    data_frame_t handshake_cmd = {
        .command = CMD_HANDSHAKE,
        .payload = {0},
    };

    while (xTaskGetTickCount() - start_ticks < pdMS_TO_TICKS(HANDSHAKE_TIMEOUT_MS)) 
    {
        transmit_frame(mac, &handshake_cmd);

        ESP_LOGI(TAG, "Handshake sent...");

        vTaskDelay(pdMS_TO_TICKS(HANDSHAKE_TIME_BETWEEN_RETRY));

        if (ack_received)
        {
            ack_received = false;
            return true;
        }

        ESP_LOGW(TAG, "Hanshake failed! Retrying...");
    }

    ESP_LOGE(TAG, "Satellite unreachable!");

    ack_received = false;

    return false;
}
