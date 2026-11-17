/*
 * MPU6500.h
 *
 * Created on: Feb 15, 2026
 * Author: Mohammed Hossam
 * Description: Advanced High-Performance Driver for MPU6500 IMU
 * with Static/Dynamic Calibration, Rest Detection, and Dead-Zones.
 */

#ifndef INC_MPU6500_H_
#define INC_MPU6500_H_

#include "main.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

// ============================================================================
// 1. I2C Address & MPU6500 Specific Registers
// ============================================================================
#define MPU6500_ADDR          0xD0

#define SMPLRT_DIV_REG        0x19
#define CONFIG_REG            0x1A
#define GYRO_CONFIG_REG       0x1B
#define ACCEL_CONFIG_REG      0x1C
#define ACCEL_CONFIG_2_REG    0x1D  // Specific to MPU6500 for Accel DLPF
#define ACCEL_XOUT_H_REG      0x3B
#define PWR_MGMT_1_REG        0x6B
#define WHO_AM_I_REG          0x75

// MPU6500 WHO_AM_I Default Value
#define MPU6500_WHO_AM_I_ID   0x70

// ============================================================================
// 2. Constants & Algorithmic Thresholds
// ============================================================================
#define RAD_TO_DEG            57.2957795131f
#define ALPHA                 0.96f        // Complementary filter split

// Rest Detection & Calibration Parameters
#define CALIBRATION_SAMPLES   500
#define VARIANCE_WINDOW       20           // Sliding tracking depth
#define REST_VAR_THRESH_G     0.05f        // Variance threshold for gyro (deg/s)^2
#define REST_VAR_THRESH_A     0.0005f      // Variance threshold for accel (g)^2

// Dead-Zone Thresholds
#define GYRO_DEADZONE_X       0.15f        // deg/s
#define GYRO_DEADZONE_Y       0.15f        // deg/s
#define GYRO_DEADZONE_Z       0.25f        // deg/s (Crucial for Yaw integration drift)

// ============================================================================
// 3. Data Structures
// ============================================================================
typedef struct {
    float x[VARIANCE_WINDOW];
    float y[VARIANCE_WINDOW];
    float z[VARIANCE_WINDOW];
    uint8_t index;
    bool filled;
} WindowFilter_t;

typedef struct {
    // Raw Data Registers
    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;

    // Converted Unbiased Conversions
    float Ax;  // g
    float Ay;
    float Az;
    float Gx;  // deg/s
    float Gy;
    float Gz;

    // Gyro Calibration Offsets (Bg)
    float Gyro_Bias_X;
    float Gyro_Bias_Y;
    float Gyro_Bias_Z;

    // Rest Detection & Dynamic Trackers
    WindowFilter_t AccelWindow ;
    WindowFilter_t GyroWindow;
    bool IsStationary;

    // Geometry Intermediate Orientations
    float AccAngleRoll;
    float AccAnglePitch;

    // Final Processed States
    float FinalAngleRoll;
    float FinalAnglePitch;
    float MPU_Yaw;
} MPU6500_t;

// ============================================================================
// 4. Function Prototypes
// ============================================================================
uint8_t MPU6500_Init(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct);
void MPU6500_Calibrate_Gyro(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct);
void MPU6500_Read_All(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct, float dt);

#endif /* INC_MPU6500_H_ */
