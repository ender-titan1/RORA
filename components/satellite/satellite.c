#include <stdio.h>
#include <string.h>
#include "satellite.h"
#include "esp_err.h"

static void on_data_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{

}

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
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv_cb));
#endif
}

void satellite_connect(satellite_t *sat)
{
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    peer->channel = 0;
    peer->encrypt = false;
    memcpy(peer->peer_addr, sat->mac, 6*sizeof(uint8_t));
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);
}

void sat_transmit_command(satellite_t *sat, sat_command_t *cmd)
{
    esp_now_send(sat->mac, (uint8_t*)cmd, sizeof(sat_command_t));
}
