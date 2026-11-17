/*
 * ENC_DRIVE.h
 *
 *  Created on: Feb 17, 2026
 *      Author: Mohammed Hossam
 */

#ifndef STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ROBOT_DRIVER_H_
#define STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ROBOT_DRIVER_H_

//#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal.h"
#include <math.h>


#define PI_f	3.14159265358979323846f
// --- Configuration ---
#define WHEEL_radius_M   0.0325f
#define ENCODER_PPR       11
#define GEAR_RATIO        90
#define WHEELBASE 	    0.15f
#define TICKS_PER_REV     (ENCODER_PPR * 4 * GEAR_RATIO)
#define RPM_EMA_ALPHA 0.2f
// --- Structs ---

// Encoder struct: Only handles ticks and speed
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t current_tick_value;
    uint32_t last_tick_value;
    float WheelAngVel;
    int32_t last_delta_ticks;
    float wheelDis;
	float  measured_RPM ;
} Encoder_t;

// Robot struct: Handles global position (Odometry)
typedef struct {
    Encoder_t left_enc;
    Encoder_t right_enc;

    // Global Pose (Belongs to the Robot, not the wheels)
    float x;
    float y;
    float delta_theta;
    float theta_rad;
    float linear_x;
    float angular_z;
    uint32_t last_time_ms;
	float measured_RPM_left ;
	float measured_RPM_right ;
}Odometry_t;

void Odom_Init(Odometry_t *odom, TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right);
//static void Update_Single_Encoder(Encoder_t *enc, float dt);
void Odom_Update(Odometry_t *odom , float dt);
#endif /* STM32F4XX_HAL_DRIVER_INC_SENSORS_HEADER_ROBOT_DRIVER_H_ */
