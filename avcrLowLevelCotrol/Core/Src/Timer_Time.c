/*
 * Timer_Time.c
 *
 *  Created on: Jun 16, 2026
 *      Author: Mohammed Hossam
 */


#include "Timer_Time.h"

/* Static variable to retain the timer tick count from the previous execution loop */
static uint32_t last_micros = 0;

void Time_Init(void) {
    /* Start the hardware timer base */
    HAL_TIM_Base_Start(&htim2);

    /* Capture the initial baseline tick count in microseconds */
    last_micros = __HAL_TIM_GET_COUNTER(&htim2);
}

float Time_GetDeltaTime(uint32_t *pLastMicros) {
    /* 1. Read the current hardware timer counter (in microseconds) */
    uint32_t current_micros = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t elapsed_micros = 0;
    uint32_t local_last_micros = *pLastMicros;

    /* 2. Handle timer overflow/roll-over condition safely */
    if (current_micros >= local_last_micros) {
        /* Standard calculation when no overflow occurred */
        elapsed_micros = current_micros - local_last_micros;
    } else {
        /* Calculation accounting for 32-bit register wrap-around back to zero */
        elapsed_micros = (0xFFFFFFFF - local_last_micros) + current_micros + 1;
    }

    /* 3. Update the history tracking variable for the next loop execution */
    *pLastMicros = current_micros;

    /* 4. Convert microseconds into a floating-point representation of seconds */
    float dt = (float)elapsed_micros / 1000000.0f;

    /* Fail-safe guard: prevent a zero value to eliminate potential 'division by zero' in math operations */
    if (dt < 0.000001f) {
        dt = 0.000001f;
    }

    return dt;
}

/**
 * @brief  Returns the current system timestamp in microseconds (us).
 * @return uint32_t: Microseconds elapsed since timer start.
 */
uint32_t Time_GetStatusMicros(void)
{
	return __HAL_TIM_GET_COUNTER(&htim2);
}

/**
 * @brief  Returns the current system timestamp in milliseconds (ms).
 * @return uint32_t: Milliseconds elapsed since timer start.
 */
uint32_t Time_GetStatusMillis(void)
{
	return (__HAL_TIM_GET_COUNTER(&htim2) / 1000);
}
