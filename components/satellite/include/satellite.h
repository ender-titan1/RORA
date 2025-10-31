#pragma once
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t payload[32];
    uint8_t crc;
} sat_command_t;

typedef struct {
    uint8_t *mac;
} satellite_t;

void wifi_init();
void satellite_connect(satellite_t *sat);
void sat_transmit_command(satellite_t *sat, sat_command_t *cmd);