#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "drv8825.h"
#include "esp_log.h"
#include "math.h"

static const char* TAG = "drv8825";

static control_mode_t control_mode = OFFLINE;

static size_t rmt_stepper_realtime_encode(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    // No space to write pulses
    if (symbols_free == 0)
    {
        return 0;
    }

    drv8825_t *motor = (drv8825_t*)data;

    size_t written = 0;
    while (written < symbols_free) 
    {
        // Never set *done, as the transmission must be continuous for RT control. Instead, keep sending idle symbols
        if (motor->rt_buffer.count == 0) 
        {
            symbols[written++] = (rmt_symbol_word_t){
                .level0 = 0,
                .duration0 = RT_RMT_IDLE_SYMBOL_DURATION_US / 2,
                .level1 = 0,
                .duration1 = RT_RMT_IDLE_SYMBOL_DURATION_US / 2
            };
            continue;
        }

        uint16_t pulse_time_us = 0;
        if (tx_buf_pop(motor, &pulse_time_us)) {
            symbols[written++] = (rmt_symbol_word_t){
                .level0 = 0,
                .duration0 = RT_RMT_SYMBOL_LOW_DURATION_US,
                .level1 = 1,
                .duration1 = pulse_time_us - RT_RMT_SYMBOL_LOW_DURATION_US
            };
        }
    }

    return written;
}

static size_t rmt_stepper_loop_encode(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    // No space to write pulses
    if (symbols_free == 0)
    {
        return 0;
    }

    rmt_stepper_loop_encoder_data_t *d = (rmt_stepper_loop_encoder_data_t*)data;
    
    // Prevent overwrites
    if (symbols_written >= d->loop_count)
    {
        *done = true;
        return 0;
    }
    
    size_t symbols_to_write = d->loop_count - symbols_written;

    if (symbols_to_write > symbols_free)
        symbols_to_write = symbols_free;

    if (d->pulse.type == CONSTANT)
    {
        // Repeat the same pulse until memory is full
        for (size_t i = 0; i < symbols_to_write; i++)
        {
            symbols[i] = d->pulse.payload.constant_pulse_us;
        }
    }
    else
    {
        for (size_t i = 0; i < symbols_to_write; i++)
        {
            symbols[i] = d->pulse.payload.variable_pulse_us[symbols_written + i];
        }
    }
    
    *done = (symbols_to_write + symbols_written == d->loop_count);

    return symbols_to_write;
}

void attach_motor(drv8825_t* motor)
{
    ESP_LOGI(TAG, "Attaching motor...");
    gpio_reset_pin(motor->pin_DIR);
    gpio_set_pull_mode(motor->pin_DIR, GPIO_PULLDOWN_ONLY);
    gpio_set_direction(motor->pin_DIR, GPIO_MODE_OUTPUT);
    gpio_set_level(motor->pin_DIR, 0);
    ESP_LOGI(TAG, "DIR pin set: %d", motor->pin_DIR);

    gpio_reset_pin(motor->pin_EN);
    gpio_set_pull_mode(motor->pin_EN, GPIO_FLOATING);
    gpio_set_direction(motor->pin_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(motor->pin_EN, motor->active_low);
    ESP_LOGI(TAG, "EN pin set: %d", motor->pin_EN);

    ESP_LOGI(TAG, "Initializing RMT channel at STEP pin %d", motor->pin_STEP);

    rmt_tx_channel_config_t tx_config = {
        .gpio_num = motor->pin_STEP,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = 1 * 1000 * 1000,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };

    rmt_new_tx_channel(&tx_config, &motor->rmt_channel);

    ESP_LOGI(TAG, "Initializing encoder for mode: %s", control_mode == OFFLINE ? "OFFLINE" : "REALTIME");

    if (control_mode == OFFLINE) {
        rmt_simple_encoder_config_t encoder_config = {
            .callback = rmt_stepper_loop_encode,
            .min_chunk_size = 10
        };
        rmt_new_simple_encoder(&encoder_config, &motor->rmt_encoder);
    } else {

        motor->rt_buffer = (drv8825_rt_buffer_t){
            .count = 0,
            .read_idx = 0,
            .write_idx = 0,
            .tx_buffer = {0}
        };

        rmt_simple_encoder_config_t encoder_config = {
            .callback = rmt_stepper_realtime_encode,
            .min_chunk_size = 10
        };
        rmt_new_simple_encoder(&encoder_config, &motor->rmt_encoder);
    }

    rmt_enable(motor->rmt_channel);

    if (control_mode == REALTIME)
    {
        ESP_LOGI(TAG, "Starting RMT transmission for realtime control...");

        gpio_set_level(motor->pin_EN, motor->active_low ? 0 : 1);

        rmt_transmit_config_t c = {
            .flags.eot_level = 0
        };
        rmt_transmit(motor->rmt_channel, motor->rmt_encoder, motor, sizeof(*motor), &c);
    }

    ESP_LOGI(TAG, "Motor succesfully attached!");
}

void drv8825_set_control_mode(control_mode_t mode)
{
    control_mode = mode;
}

void detach_motor(drv8825_t* motor)
{
    ESP_LOGI(TAG, "Detaching motor with STEP pin %d...", motor->pin_STEP);
    rmt_disable(motor->rmt_channel);
    rmt_del_channel(motor->rmt_channel);
    rmt_del_encoder(motor->rmt_encoder);
    ESP_LOGI(TAG, "Motor succesfully detached!");
}

void tx_buf_push(drv8825_t *motor, uint16_t pulse_time_us)
{
    drv8825_rt_buffer_t *buf = &motor->rt_buffer;

    if (buf->count >= RT_TX_BUF_SIZE)
    {
        ESP_LOGW(TAG, "Cannot push to RMT TX buffer. Maximum capacity reached");
        return;
    }
    
    // Equivalent to: write_idx mod BUF_SIZE (ONLY IF buf size is power of 2)
    buf->write_idx = buf->write_idx & (RT_TX_BUF_SIZE - 1);
    buf->tx_buffer[buf->write_idx] = pulse_time_us;
    buf->count++;
    buf->write_idx++;
}

bool tx_buf_pop(drv8825_t *motor, uint16_t *out)
{
    drv8825_rt_buffer_t *buf = &motor->rt_buffer;
    
    if (buf->count == 0)
    {
        ESP_LOGW(TAG, "Cannot pop from RMT TX buffer. 0 items in buffer");
        return false;
    }
    
    // Equivalent to: read_idx mod BUF_SIZE (ONLY IF buf size is power of 2)
    buf->read_idx = buf->read_idx & (RT_TX_BUF_SIZE - 1);
    *out = buf->tx_buffer[buf->read_idx];
    buf->read_idx++;
    buf->count--;
    return true;
}

void integrate_and_update_motor_state(drv8825_t *motor, integrator_mode_t mode, float dt_s)
{
    drv8825_rt_motor_state_t *rt_state = &motor->rt_state;

    int32_t error = rt_state->target_step - rt_state->current_step;

    if (error == 0 && mode != VELOCITY_DRIVEN)
    {
        rt_state->current_velocity = 0;
        rt_state->moving = false;
        return;
    }

    float direction = (error > 0) ? 1.0f : -1.0f;

    // Initial start velocity
    if (fabsf(rt_state->current_velocity) < 1.0f && mode == VELOCITY_DRIVEN)
    {
        rt_state->current_velocity = copysignf(rt_state->min_start_vel, direction);
    }

    float vel_desired;
    float max_delta_v = rt_state->acceleration * dt_s;

    if (mode == POSITION_DRIVEN)
    {
        vel_desired = direction * rt_state->target_velocity;

        // D = v^2/2a
        float stopping_dist = (rt_state->current_velocity * rt_state->current_velocity) / (2.0f * rt_state->acceleration);
        
        // Start braking if at or within stopping range
        if ((float)abs(error) <= stopping_dist)
            vel_desired = 0;
    } 
    else 
    {
        vel_desired = rt_state->target_velocity;
    }

    float vel_error = vel_desired - rt_state->current_velocity;

    // Clamp velocity
    if (fabsf(vel_error) > max_delta_v)
        vel_error = copysignf(max_delta_v, vel_error);
    
    rt_state->current_velocity += vel_error;
}

void fill_tx_buffer(drv8825_t* motor, integrator_mode_t mode)
{
    if (control_mode != REALTIME)
    {
        ESP_LOGE(TAG, "Attempted to used realtime stepper API in offline mode");
        return; 
    }

    drv8825_rt_motor_state_t *rt_state = &motor->rt_state;

    if (!rt_state->moving)
        return;

    // There's probably some stupidity here with the units, but it works
    float dt_s = (uint32_t)(1e6f / rt_state->min_start_vel) * 1e-6f;

    while (motor->rt_buffer.count < RT_TX_BUF_SIZE)
    {
        int32_t error = rt_state->target_step - rt_state->current_step;

        if (error == 0 && mode != VELOCITY_DRIVEN)
        {
            rt_state->moving = false;
            return;
        }

        integrate_and_update_motor_state(motor, mode, dt_s);

        float abs_speed = fabsf(rt_state->current_velocity);

        if (abs_speed < 1.0f)
            return;

        uint32_t interval_us = (uint32_t)(1e6f / abs_speed);

        dt_s = interval_us * 1e-6f;

        gpio_set_level(motor->pin_DIR, rt_state->current_velocity > 0 ? CW : CCW);
        tx_buf_push(motor, interval_us);

        rt_state->current_step += rt_state->current_velocity > 0 ? 1 : -1; 
    }
}

uint16_t prepare(drv8825_command_t *command, const char* tag)
{
    drv8825_t *motor = command->motor;

    ESP_LOGI(tag, "Movement command issued for motor with STEP pin %d", motor->pin_STEP);
    ESP_LOGI(tag, "Rotation %d degress %s", command->degrees, command->direction ? "CW" : "CCW");
    float fraction = command->degrees/360.0f;
    uint16_t steps = (uint16_t)(fraction * motor->steps_per_rotation);

    gpio_set_level(motor->pin_DIR, (uint8_t)command->direction);
    gpio_set_level(motor->pin_EN, motor->active_low ? 0 : 1);

    vTaskDelay(pdMS_TO_TICKS(3));

    return steps;
}

void execute(drv8825_command_t *command, override_t disable)
{
    if (control_mode != OFFLINE) {
        ESP_LOGE(TAG, "Attempted to used offline stepper API in realtime mode");
        return;
    }

    if (!command->moving)
        return;

    drv8825_t *motor = command->motor;
    uint16_t steps = prepare(command, TAG);
    ESP_LOGI(TAG, "Executing move: [STEP pin %d] [Steps %d]", motor->pin_STEP, steps);

    rmt_transmit_config_t c = {
        .flags.eot_level = 0
    };
    rmt_stepper_loop_encoder_data_t data = {
        .pulse = command->pulse,
        .loop_count = steps,
    };
    rmt_transmit(motor->rmt_channel, motor->rmt_encoder, &data, sizeof(data), &c);
    rmt_tx_wait_all_done(motor->rmt_channel, 10000);

    if (disable == TRUE || (disable == NO_OVERRIDE && command->disable))
    {
        disable_motor(command->motor);
    }

}

void execute_sync(uint8_t count, drv8825_command_t *commands, override_t disable)
{
    if (control_mode != OFFLINE) {
        ESP_LOGE(TAG, "Attempted to used offline stepper API in realtime mode");
        return;
    }

    if (count == 0)
        return;
    
    ESP_LOGI(TAG, "Begin Sync Execute");
    ESP_LOGI(TAG, "%d commands issued", count);
    
    // Run a check to make sure the same motor is not getting multiple commands
    uint8_t step_pins[count];
    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t step = commands[i].motor->pin_STEP;
        for (uint8_t j = 0; j < i; j++)
        {
            if (step == step_pins[j])
            {
                ESP_LOGE(TAG, "Multiple commands issued for motor with STEP pin %d", step);
                ESP_LOGE(TAG, "Aborting sync execute!");
                return;
            }
        }

        step_pins[i] = step;
    }

    ESP_LOGI(TAG, "Preparing commands");

    rmt_stepper_loop_encoder_data_t* data = calloc(count, sizeof(rmt_stepper_loop_encoder_data_t));
    for (uint8_t i = 0; i < count; i++)
    {
        drv8825_command_t command = commands[i];
        uint16_t steps = prepare(&command, TAG);  

        rmt_stepper_loop_encoder_data_t d = {
            .pulse = command.pulse,
            .loop_count = steps
        };
        data[i] = d;

        ESP_LOGI(TAG, "Command: [STEP# %d] [Steps %d]", command.motor->pin_STEP, steps);
    }

    ESP_LOGI(TAG, "Beginning transmission");

    rmt_transmit_config_t c = {
        .flags.eot_level = 0
    };

    for (uint8_t i = 0; i < count; i++)
    {
        drv8825_t *motor = commands[i].motor;
        rmt_transmit(motor->rmt_channel, motor->rmt_encoder, &data[i], sizeof(rmt_stepper_loop_encoder_data_t), &c);
    }

    for (uint8_t i = 0; i < count; i++)
    {
        rmt_tx_wait_all_done(commands[i].motor->rmt_channel, 10000);
    }

    free(data);

    ESP_LOGI(TAG, "Transmission completed succesfully!");

    // Make sure we disable the motor after the move
    for (uint8_t i = 0; i < count; i++)
    {
        if (disable == TRUE || (disable == NO_OVERRIDE && commands[i].disable))
        {
            disable_motor(commands[i].motor);
        }
    }
}

void disable_motor(drv8825_t *motor)
{
    gpio_set_level(motor->pin_EN, motor->active_low);
}
