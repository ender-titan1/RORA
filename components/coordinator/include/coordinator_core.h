#pragma once
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t payload[32];
    uint8_t crc;
} sat_command_t;

void core_init();
void sat_transmit_command(uint8_t *mac, sat_command_t *cmd);