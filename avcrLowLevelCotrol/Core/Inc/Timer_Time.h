/*
 * Timer_Time.h
 *
 *  Created on: Jun 16, 2026
 *      Author: Mohammed Hossam
 */

#ifndef INC_TIMER_TIME_H_
#define INC_TIMER_TIME_H_

#include "main.h" // Required for HAL and timer handler definitions

/* External declaration of the hardware timer used for microsecond tracking */
extern TIM_HandleTypeDef htim2;

/**
 * @brief  Initializes the high-resolution timer for time tracking.
 * @param  None
 * @retval None
 */
void Time_Init(void);

/**
 * @brief  Calculates and returns the elapsed time (dt) in seconds since the last call.
 * Handles timer overflow automatically.
 * @param  *pLastMicros
 * @retval float: Elapsed time in seconds (e.g., 0.010523f)
 */
float Time_GetDeltaTime(uint32_t *pLastMicros);
/**
 * @brief  Returns the current system timestamp in microseconds (us).
 * @return uint32_t: Microseconds elapsed since timer start.
 */
uint32_t Time_GetStatusMicros(void);

/**
 * @brief  Returns the current system timestamp in milliseconds (ms).
 * @return uint32_t: Milliseconds elapsed since timer start.
 */
uint32_t Time_GetStatusMillis(void);
#endif /* INC_TIMER_TIME_H_ */
