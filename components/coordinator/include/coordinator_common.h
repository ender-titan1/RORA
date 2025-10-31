#pragma once
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"

#if CONFIG_DEVICE_ROLE_CORE
    #include "coordinator_core.h"
#elif CONFIG_DEVICE_ROLE_SAT
    #include "coordinator_sat.h"
#endif

void wifi_init();
void connect(uint8_t *mac);
