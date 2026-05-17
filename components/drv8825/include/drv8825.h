#pragma once

#include "driver/gpio.h"
#include "driver/rmt_tx.h"

#define CCW 0
#define CW 1

#define RT_TX_BUF_SIZE 32
#define RT_RMT_SYMBOL_LOW_DURATION_US 5
#define RT_RMT_IDLE_SYMBOL_DURATION_US 100

typedef enum {
    OFFLINE,
    REALTIME
} control_mode_t;

typedef enum {
    POSITION_DRIVEN,
    VELOCITY_DRIVEN
} integrator_mode_t;

typedef enum {
    FALSE = 0x00,
    TRUE = 0x01,
    NO_OVERRIDE = 0xFF,
} override_t;

typedef struct __attribute__((packed)) {
    uint8_t direction_override;
    uint8_t disable_override; 
} execute_overrides_t;

typedef struct {
    int32_t current_step;
    int32_t target_step;

    float current_velocity;
    float target_velocity;

    float acceleration;

    float dt_s;
} drv8825_rt_motor_state_t;

typedef struct {
    uint16_t tx_buffer[RT_TX_BUF_SIZE];

    volatile uint16_t read_idx;
    volatile uint16_t write_idx;

    volatile uint16_t count;
} drv8825_rt_buffer_t;

typedef struct {
    int pin_STEP;
    int pin_DIR;
    int pin_EN;
    int steps_per_rotation;

    bool active_low;

    rmt_channel_handle_t rmt_channel;
    rmt_encoder_handle_t rmt_encoder;

    // Realtime control
    drv8825_rt_motor_state_t rt_state;
    drv8825_rt_buffer_t rt_buffer;
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
    bool disable;

    int degrees;
    uint8_t direction;
    drv8825_pulse_provider_t pulse;
} drv8825_command_t;

typedef struct {
    drv8825_pulse_provider_t pulse;
    uint16_t loop_count;
} rmt_stepper_loop_encoder_data_t;

void attach_motor(drv8825_t *motor);
void drv8825_set_control_mode(control_mode_t mode);
void detach_motor(drv8825_t *motor);

// ===== REALTIME MODE =====
void tx_buf_push(drv8825_t *motor, uint16_t pulse_time_us);
bool tx_buf_pop(drv8825_t *motor, uint16_t *out);
void integrate_and_update_motor_state(drv8825_t *motor, integrator_mode_t mode, float dt_s);
void fill_tx_buffer(drv8825_t *motor, integrator_mode_t mode);

// ===== OFFLINE MODE =====
uint16_t prepare(drv8825_command_t *command, const char* tag);
void execute(drv8825_command_t *command, override_t disable);
void execute_sync(uint8_t count, drv8825_command_t *commands, override_t disable);
void disable_motor(drv8825_t *motor);
