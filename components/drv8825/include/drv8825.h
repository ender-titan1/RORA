#pragma once

#include "driver/gpio.h"
#include "driver/rmt_tx.h"

#define CCW 0
#define CW 1

typedef struct {
    int pinSTEP;
    int pinDIR;
    int pinEN;
    int stepsPerRotation;

    bool activeLow;

    rmt_channel_handle_t channelRMT;
    rmt_encoder_handle_t encoderRMT;
} drv8825_t;

typedef struct {
    enum {
        CONSTANT = 0,
        VARIABLE = 1,
    } type;

    union {
        rmt_symbol_word_t constant_pulse_us;
        rmt_symbol_word_t *variable_pulse_us;
    } payload;
} drv8825_pulse_provider_t;

typedef struct {
    drv8825_t *motor;
    bool moving;

    int degrees;
    uint8_t direction;
    drv8825_pulse_provider_t pulse;
} drv8825_command_t;

typedef struct {
    drv8825_pulse_provider_t pulse;
    uint16_t loop_count;
} rmt_stepper_loop_encoder_data_t;

size_t rmt_stepper_loop_encode(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg);

void attach_motor(drv8825_t *motor);
uint16_t prepare(drv8825_command_t *command, const char* tag);
void execute(drv8825_command_t *command);
void execute_sync(uint8_t count, drv8825_command_t *commands, bool disable);
void disable_motor(drv8825_t *motor);
void detach_motor(drv8825_t *motor);
