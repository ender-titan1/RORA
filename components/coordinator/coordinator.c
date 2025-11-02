#include <stdio.h>
#include <string.h>
#include "coordinator_common.h"
#include "esp_err.h"

#if CONFIG_DEVICE_ROLE_CORE
    #include "coordinator_core.h"
#elif CONFIG_DEVICE_ROLE_SAT
    #include "coordinator_sat.h"
#endif


void wifi_init()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());

#if CONFIG_DEVICE_ROLE_CORE
    core_init();
#elif CONFIG_DEVICE_ROLE_SAT
    sat_init();
#endif
}

void connect(uint8_t *mac)
{
    esp_now_peer_info_t peer = {0};
    peer.channel = 0;
    peer.encrypt = false;
    memcpy(peer.peer_addr, mac, 6*sizeof(uint8_t));
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

void transmit_frame(uint8_t *mac, data_frame_t *cmd)
{
    cmd->crc = crc8_gen((uint8_t*)cmd, sizeof(data_frame_t) - 1);
    esp_now_send(mac, (uint8_t*)cmd, sizeof(data_frame_t));
}

uint8_t crc8_gen(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    const uint8_t poly = 0x07;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ poly;
            else
                crc <<= 1;
        }
    }
    return crc;
}
