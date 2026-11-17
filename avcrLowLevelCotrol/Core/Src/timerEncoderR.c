/*
 * timerEncoderR.c
 *
 *  Created on: May 21, 2026
 *      Author: Mohammed Hossam
 */


#include <timerEncoder.h>

/*
 * VB = VR+VL/2
 * w = VR-VL/s
 * x_dot= VB * cos(theta) = VL/2*cos(theta) +VR/2*cos(theta)
 * y_dot= VB * sin(theta) = VL/2*sin(theta) +VR/2*sin(theta)
 * theta_dot = -1/s*VL + 1/s * VR
 *
 * actually VL= r*phi_dot_L & VR= r*phi_dot_R
 * while phi_dot is angular velocity of the wheel
 * so ...
 *
 * x_dot= VB * cos(theta) = r*phi_dot_L/2*cos(theta) +r*phi_dot_R/2*cos(theta)
 * y_dot= VB * sin(theta) = r*phi_dot_L/2*sin(theta) +r*phi_dot_R/2*sin(theta)
 * theta_dot = -1/s*r*phi_dot_L + 1/s * r*phi_dot_R
 *
 * w = r*phi_dot_R-r*phi_dot_L/s
 * VB = r*phi_dot_R+r*phi_dot_L/2
 *
 *
 * */

void Odom_Init(Odometry_t *odom, TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right)
{
	//left enc
	odom->left_enc.htim= htim_left;
	odom->left_enc.last_tick_value= __HAL_TIM_GET_COUNTER(htim_left);
	odom->left_enc.WheelAngVel= 0.0f;
	odom->left_enc.wheelDis= 0.0f;
	odom->left_enc.last_delta_ticks = 0;
	odom->left_enc.measured_RPM = 0.0f;
	//right enc
	odom->right_enc.htim = htim_right;
	odom->right_enc.last_tick_value= __HAL_TIM_GET_COUNTER(htim_right);
	odom->right_enc.WheelAngVel= 0.0f;
	odom->right_enc.wheelDis= 0.0f;
	odom->right_enc.last_delta_ticks = 0;
	odom->right_enc.measured_RPM = 0.0f;

	//odom data
	odom->linear_x = 0.0f;
	odom->angular_z = 0.0f;
	odom->x = 0.0f;
	odom->y = 0.0f;
	odom->theta_rad= 0.0f;
	odom->delta_theta = 0.0f;
	odom->measured_RPM_left = 0;
	odom->measured_RPM_right = 0;
	HAL_TIM_Encoder_Start(htim_left, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(htim_right, TIM_CHANNEL_ALL);
}

static void Update_Single_Encoder(Encoder_t *enc, float dt)
{
	int16_t tempDeltaTicks = 0 ;
	float circumference =2.0f *PI_f * WHEEL_radius_M;
	enc->current_tick_value = __HAL_TIM_GET_COUNTER(enc->htim);
	tempDeltaTicks = (int16_t)(enc->current_tick_value - enc->last_tick_value);
	enc->last_delta_ticks = tempDeltaTicks;
	enc->WheelAngVel = (((float)enc->last_delta_ticks/(float)TICKS_PER_REV) * (2.0f *PI_f/dt));
	enc->last_tick_value = enc->current_tick_value;
	enc->wheelDis = ((float)enc->last_delta_ticks/(float)TICKS_PER_REV)* circumference;
	float raw_RPM = ((float)enc->last_delta_ticks / (float)(TICKS_PER_REV)) * (60.0f / dt);
	enc->measured_RPM = (RPM_EMA_ALPHA * raw_RPM) + ((1.0f - RPM_EMA_ALPHA) * enc->measured_RPM);
}
void Odom_Update(Odometry_t *odom,float dt)
{

	uint32_t current_time_ms = HAL_GetTick();

		odom->last_time_ms = current_time_ms;
		Update_Single_Encoder(&odom->left_enc, dt);
		Update_Single_Encoder(&odom->right_enc, dt);

		float dl,dr,dc ,theta_mid, delta_theta = 0.0f;
		//	odom->linear_x = (((WHEEL_radius_M * odom->right_enc.WheelAngVel) + (WHEEL_radius_M * odom->left_enc.WheelAngVel))/2.0f);
		//	odom->angular_z = (((WHEEL_radius_M * odom->right_enc.WheelAngVel) - (WHEEL_radius_M * odom->left_enc.WheelAngVel))/WHEELBASE);
		odom->measured_RPM_left = odom->left_enc.measured_RPM;
		odom->measured_RPM_right = odom->right_enc.measured_RPM;
		dl = odom->left_enc.wheelDis;
		dr = odom->right_enc.wheelDis;
		dc = (dl+dr)/2.0f;
		odom->delta_theta = (dr-dl)/WHEELBASE;
		theta_mid = odom->theta_rad + (delta_theta/2.0f);
		odom->linear_x= (dc/ dt);
		odom->angular_z= (odom->delta_theta/ dt);

		odom->x += dc * cosf(theta_mid);

		odom->y += dc * sinf(theta_mid);

		odom->theta_rad+= odom->delta_theta;
		if(odom->theta_rad > PI_f) {
			odom->theta_rad  -= 2.0f *PI_f;
		}
		else if(odom->theta_rad < -PI_f) {
			odom->theta_rad  += 2.0f *PI_f;
		}
}
