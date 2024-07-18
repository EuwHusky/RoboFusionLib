#include "drv_delay.h"

#include "FreeRTOS.h"
#include "task.h"

void rflOsDelayMs(uint32_t ms)
{
    vTaskDelay(ms);
}
