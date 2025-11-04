#pragma once
#include "coordinator_common.h"

#define HANDSHAKE_TIMEOUT_MS 1000
#define HANDSHAKE_TIME_BETWEEN_RETRY 100

void core_init();
bool await_response();
bool sat_handshake(uint8_t *mac);