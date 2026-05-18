#include "realtime.h"
#include "freertos/FreeRTOS.h"

static void motor_update(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    rt_controller_t *rt = (rt_controller_t*)arg;

    while (true)
    {
        xTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(RT_UPDATE_PERIOD_MS));

        for (size_t i = 0; i < rt->joints_len; i++)
        {
            fill_tx_buffer(rt->joints_arr[i].motor, rt->integrator_mode);
        }
    }
}

void init_realtime(rt_controller_t *rt)
{
    xTaskCreatePinnedToCore(motor_update, "motor_update", 4096, rt, 10, NULL, 1);
}

void rt_demo(rt_controller_t *rt)
{
    rt->joints_arr[0].motor->rt_state.target_velocity = 4000;
    rt->joints_arr[0].motor->rt_state.moving = true;
}
