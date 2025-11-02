#pragma once
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "drv8825.h"

#define CMD_HANDSHAKE 0xF0
#define CMD_MP_CURVE 0xA0
#define CMD_MP_COMPUTE 0xB0
#define CMD_EXECUTE 0x01
#define CMD_CLEANUP 0xFF

#define ACK 0x06
#define NACK 0x15

typedef enum {
    EASE_LINEAR = 0,
    EASE_SINE = 1,
    EASE_CUBIC = 2
} ease_type_t;

typedef struct __attribute__((packed)) {
    float degrees;
    drv8825_direction dir;
} mp_joint_command_payload_t;

typedef struct __attribute__((packed)) {
    ease_type_t ease;
    float accel_time_s;
    float decel_time_s;
    float duration_s;
    float resolution;
} mp_movement_curve_config_payload_t;

typedef struct __attribute__((packed)) {
    uint8_t command;
    uint8_t payload[32];
    uint8_t crc;
} data_frame_t;

void wifi_init();
void connect(uint8_t *mac);
uint8_t crc8_gen(const uint8_t *data, uint8_t len);

