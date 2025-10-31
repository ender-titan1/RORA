#pragma once
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t payload[32];
    uint8_t crc;
} sat_command_t;

#if CONFIG_DEVICE_ROLE_CORE
    #include "coordinator_core.h"
#elif CONFIG_DEVICE_ROLE_SAT
    #include "coordinator_sat.h"
#endif

void wifi_init();
void connect(uint8_t *mac);
