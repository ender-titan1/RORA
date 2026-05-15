#pragma once
#include "drv8825.h"
#include "motion_planner.h"
#include "coordinator_common.h"
#include "coordinator_core.h"

#define M1_STEP GPIO_NUM_6
#define M1_DIR GPIO_NUM_5
#define M1_EN GPIO_NUM_4

#define M2_STEP GPIO_NUM_2
#define M2_DIR GPIO_NUM_1
#define M2_EN GPIO_NUM_3

typedef enum {
    // Basic commands
    PC_CMD_CREATE_CURVE = 0xF0,
    PC_CMD_COMPUTE_CURVE = 0xFA,
    PC_CMD_QUEUE_MOVEMENT = 0xFB,
    PC_CMD_EXECUTE = 0xFC,

    // Complex commands
    PC_CMD_MOVE = 0xE0,
} pc_command_type_t;

typedef struct __attribute__((packed)) {
    uint8_t slot;
    uint8_t src_slot;
    uint8_t controller;
    uint8_t motor;
    union {
        mp_movement_curve_config_t curve_cfg;
        mp_joint_command_payload_t compute_cfg;
        execute_overrides_t exec_cfg;
    };
} pc_command_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t cmd;
    pc_command_payload_t payload;
} pc_command_t;

void core_main();