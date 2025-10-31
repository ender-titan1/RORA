#include "coordinator_core.h"
#include "esp_log.h"

const uint8_t *TAG = "Coordinator Core";

static void core_on_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{

}

void core_init()
{
    esp_now_register_recv_cb(core_on_recv_cb);
    ESP_LOGI(TAG, "Initialized device as CORE");
}

void sat_transmit_command(uint8_t *mac, sat_command_t *cmd)
{
    esp_now_send(mac, (uint8_t*)cmd, sizeof(sat_command_t));
}
