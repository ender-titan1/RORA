#include <stdio.h>
#include "freertos/FreeRTOS.h"

#if CONFIG_DEVICE_ROLE_CORE
    #include "core.h"
#elif CONFIG_DEVICE_ROLE_SAT
    #include "satellite.h"
#endif

void app_main(void)
{
#if CONFIG_DEVICE_ROLE_CORE
    core_main();
#elif CONFIG_DEVICE_ROLE_SAT
    sat_main();
#endif
}