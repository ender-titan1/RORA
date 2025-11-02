#pragma once
#include "coordinator_common.h"

typedef void (*sat_callback_func)(data_frame_t *cmd, uint8_t len);

void sat_init();
void bind_cmd_callbacks(sat_callback_func curve_cb, sat_callback_func compute_cb, sat_callback_func exec_cb, sat_callback_func cleanup_cb);