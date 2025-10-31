#include <stdio.h>
#include "drv8825.h"
#include "esp_log.h"

static const char* TAG = "DRV8825";
static const char* SYNC_TAG = "DRV8825|SYNC";

size_t rmt_stepper_loop_encode(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg)
{
    // No space to write pulses
    if (symbols_free == 0)
    {
        return 0;
    }

    rmt_stepper_loop_encoder_data_t *d = (rmt_stepper_loop_encoder_data_t*)data;
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
    gpio_set_direction(motor->pinDIR, GPIO_MODE_OUTPUT);
    gpio_set_level(motor->pinDIR, 0);
    ESP_LOGI(TAG, "DIR pin set: %d", motor->pinDIR);

    gpio_set_direction(motor->pinEN, GPIO_MODE_OUTPUT);
    gpio_set_level(motor->pinEN, 0);
    ESP_LOGI(TAG, "EN pin set: %d", motor->pinEN);

    ESP_LOGI(TAG, "Initializing RMT channel at STEP pin %d", motor->pinSTEP);

    rmt_tx_channel_config_t tx_config = {
        .gpio_num = motor->pinSTEP,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .mem_block_symbols = 64,
        .resolution_hz = 1 * 1000 * 1000,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false
    };

    rmt_new_tx_channel(&tx_config, &motor->channelRMT);

    rmt_simple_encoder_config_t encoder_config = {
        .callback = rmt_stepper_loop_encode,
        .min_chunk_size = 10
    };
    rmt_new_simple_encoder(&encoder_config, &motor->encoderRMT);

    rmt_enable(motor->channelRMT);

    ESP_LOGI(TAG, "Motor succesfully attached!");
}

void detach_motor(drv8825_t* motor)
{
    ESP_LOGI(TAG, "Detaching motor with STEP pin %d...", motor->pinSTEP);
    rmt_disable(motor->channelRMT);
    rmt_del_channel(motor->channelRMT);
    rmt_del_encoder(motor->encoderRMT);
    ESP_LOGI(TAG, "Motor succesfully detached!");
}

uint16_t prepare(drv8825_command_t *command, const char* tag)
{
    drv8825_t *motor = command->motor;

    ESP_LOGI(tag, "Movement command issued for motor with STEP pin %d", motor->pinSTEP);
    ESP_LOGI(tag, "Rotation %d degress %s", command->degrees, command->direction ? "CW" : "CCW");
    float fraction = command->degrees/360.0f;
    uint16_t steps = (uint16_t)(fraction * motor->stepsPerRotation);

    gpio_set_level(motor->pinDIR, (uint8_t)command->direction);
    gpio_set_level(motor->pinEN, 1);

    return steps;
}

void execute(drv8825_command_t *command)
{
    if (!command->moving)
        return;

    drv8825_t *motor = command->motor;
    uint16_t steps = prepare(command, TAG);
    ESP_LOGI(TAG, "Executing move: [STEP pin %d] [Steps %d]", motor->pinSTEP, steps);

    rmt_transmit_config_t c = {};
    rmt_stepper_loop_encoder_data_t data = {
        .pulse = command->pulse,
        .loop_count = steps
    };
    rmt_transmit(motor->channelRMT, motor->encoderRMT, &data, sizeof(data), &c);
    rmt_tx_wait_all_done(motor->channelRMT, 10000);

    gpio_set_level(motor->pinEN, 0);
}

void execute_sync(uint8_t count, drv8825_command_t *commands)
{
    ESP_LOGI(TAG, "SYNC EXECUTE BEGIN");
    ESP_LOGI(SYNC_TAG, "%d commands issued", count);

    // Run a check to make sure the same motor is not getting multiple commands
    uint8_t step_pins[count];
    for (uint8_t i = 0; i < count; i++)
    {
        uint8_t step = commands[i].motor->pinSTEP;
        for (uint8_t j = 0; j < i; j++)
        {
            if (step == step_pins[j])
            {
                ESP_LOGE(SYNC_TAG, "Multiple commands issued for motor with STEP pin %d", step);
                ESP_LOGE(SYNC_TAG, "Aborting sync execute!");
                return;
            }
        }

        step_pins[i] = step;
    }

    ESP_LOGI(SYNC_TAG, "Preparing commands");

    rmt_stepper_loop_encoder_data_t data[count];
    for (uint8_t i = 0; i < count; i++)
    {
        drv8825_command_t command = commands[i];
        uint16_t steps = prepare(&command, SYNC_TAG);

        rmt_stepper_loop_encoder_data_t d = {
            .pulse = command.pulse,
            .loop_count = steps
        };
        data[i] = d;

        ESP_LOGI(SYNC_TAG, "Command: [STEP pin|%d] [Steps|%d]", command.motor->pinSTEP, steps);
    }

    ESP_LOGI(SYNC_TAG, "Beginning transmission");

    rmt_transmit_config_t c = {};

    for (uint8_t i = 0; i < count; i++)
    {
        drv8825_t *motor = commands[i].motor;
        rmt_transmit(motor->channelRMT, motor->encoderRMT, &data[i], sizeof(rmt_stepper_loop_encoder_data_t), &c);
    }

    for (uint8_t i = 0; i < count; i++)
    {
        rmt_tx_wait_all_done(commands[i].motor->channelRMT, 10000);
    }

    ESP_LOGI(SYNC_TAG, "Transmission completed succesfully!");

    // Make sure we disable the motor after the move
    for (uint8_t i = 0; i < count; i++)
    {
        gpio_set_level(commands[i].motor->pinEN, 0);
    }
}

