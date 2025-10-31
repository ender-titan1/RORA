#include "coordinator_sat.h"
#include "esp_log.h"

const uint8_t *TAG = "Coordinator Satellite";

static void sat_on_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{

}

void sat_init()
{
    esp_now_register_recv_cb(sat_on_recv_cb);
    ESP_LOGI(TAG, "Initialized device as SAT");
}
