#pragma once
#include "coordinator_common.h"

typedef void (*sat_callback_func)(data_frame_t *cmd, uint8_t len);

typedef struct {
    data_frame_t frame;
    int len;
    uint8_t mac[MAC_ADDR_LEN];
} queued_data_frame_t;

sat_callback_func coordinator_configure_callback;
sat_callback_func coordinator_curve_callback;
sat_callback_func coordinator_compute_callback;
sat_callback_func coordinator_exec_callback;

void sat_init();