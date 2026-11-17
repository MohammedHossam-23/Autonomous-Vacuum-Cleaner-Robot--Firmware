/*
 * Minimal HAL shim so firmware modules that only touch timers and the tick
 * counter (currently the odometry module) can be compiled and tested on the host.
 *
 * This header shadows the real stm32f4xx_hal.h via include order. It is only
 * used by the host build, never by the firmware build.
 */
#ifndef STM32F4XX_HAL_H
#define STM32F4XX_HAL_H

#include <stdint.h>

typedef struct
{
    uint32_t CNT;
} TIM_HandleTypeDef;

#define TIM_CHANNEL_ALL 0x0000003CU

#define __HAL_TIM_GET_COUNTER(handle) ((handle)->CNT)

static inline int HAL_TIM_Encoder_Start(TIM_HandleTypeDef *handle, uint32_t channel)
{
    (void)handle;
    (void)channel;
    return 0;
}

uint32_t HAL_GetTick(void);
void host_hal_set_tick(uint32_t tick);

#endif /* STM32F4XX_HAL_H */
