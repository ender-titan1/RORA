#include "coordinator_sat.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

static const char *TAG = "coordinator";

static QueueHandle_t sat_cmd_queue;

static void sat_on_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (len != sizeof(data_frame_t))
    {
        ESP_LOGE(TAG, "Len != sizeof(data_frame_t)");
        return;
    }

    data_frame_t cmd;
    memcpy(&cmd, data, sizeof(data_frame_t));

    uint8_t generated_crc = crc8_gen(data, len - 1);
    uint8_t original_crc = cmd.crc;

    if (generated_crc != original_crc)
    {
        ESP_LOGW(TAG, "Data received, CRC8 invalid (expected %02X, got %02X)", original_crc, generated_crc);
        return;
    }

    queued_data_frame_t qcmd = {
        .frame = cmd,
        .len = len,
    };
    
    memcpy(qcmd.mac, recv_info->src_addr, sizeof(uint8_t) * MAC_ADDR_LEN);
    
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xQueueSendFromISR(sat_cmd_queue, &qcmd, &higher_priority_task_woken) != pdTRUE)
    {
        ESP_LOGW(TAG, "Command queue full, dropping command");
    }

    if (higher_priority_task_woken)
    {
        portYIELD_FROM_ISR();
    }
}

static void sat_command_task(void *arg)
{
    queued_data_frame_t frame;

    while (true)
    {
        if (xQueueReceive(sat_cmd_queue, &frame, portMAX_DELAY) == pdTRUE)
        {
            data_frame_t cmd = frame.frame;
            int len = frame.len;

            data_frame_t ack = {
                .command = ACK,
                .payload = {0}
            };
            transmit_frame(frame.mac, &ack);

            switch (cmd.command)
            {
            case CMD_CONFIGURE:
                coordinator_configure_callback(&cmd, len);
                break;
            case CMD_OFFLINE_MP_CURVE:
                coordinator_curve_callback(&cmd, len);
                break;
            case CMD_OFFLINE_MP_COMPUTE:
                coordinator_compute_callback(&cmd, len);
                break;
            case CMD_OFFLINE_EXECUTE:
                coordinator_exec_callback(&cmd, len);
                break;
            case CMD_HANDSHAKE:
                ESP_LOGI(TAG, "Handshake command received. Responding.");
                break;
            default:
                ESP_LOGE(TAG, "Unknown command sent: %02X", cmd.command);
                break;
            }

            taskYIELD();
        }
    }
}

void sat_init()
{
    sat_cmd_queue = xQueueCreate(16, sizeof(queued_data_frame_t));
    xTaskCreate(sat_command_task, "sat_command_task", 8192, NULL, 5, NULL);

    esp_now_register_recv_cb(sat_on_recv_cb);
    ESP_LOGI(TAG, "Initialized device as SAT");
}
