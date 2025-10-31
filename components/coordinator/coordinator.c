#include <stdio.h>
#include <string.h>
#include "coordinator_common.h"
#include "esp_err.h"


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
}

void connect(uint8_t *mac)
{
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    peer->channel = 0;
    peer->encrypt = false;
    memcpy(peer->peer_addr, mac, 6*sizeof(uint8_t));
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);
}