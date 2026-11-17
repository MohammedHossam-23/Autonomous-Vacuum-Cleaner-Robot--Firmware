#include "stm32f4xx_hal.h"

static uint32_t g_fake_tick = 0;

uint32_t HAL_GetTick(void)
{
    return g_fake_tick;
}

void host_hal_set_tick(uint32_t tick)
{
    g_fake_tick = tick;
}
