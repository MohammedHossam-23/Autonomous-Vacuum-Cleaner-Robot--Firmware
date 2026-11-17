/*
 * MPU6500.c
 *
 * Created on: Feb 15, 2026
 * Author: Mohammed Hossam
 */

#include "MPU6500.h"

#define HAL_TIMEOUT          10  // 2ms timeout for I2C operations

// Private Helper Functions for Variance Analytics
static void Push_Window(WindowFilter_t *w, float x, float y, float z);
static void Calc_Variance(WindowFilter_t *w, float *varX, float *varY, float *varZ);

// ============================================================================
// 1. Initialization Function
// ============================================================================
uint8_t MPU6500_Init(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct) {
    uint8_t check = 0;
    uint8_t regData = 0;

    // Zero out memory architecture structures
    memset(DataStruct, 0, sizeof(MPU6500_t));

    // 1. Verify Device signature (WHO_AM_I)
    HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, WHO_AM_I_REG, 1, &check, 1, HAL_TIMEOUT);

    if (check == MPU6500_WHO_AM_I_ID) {
        // 2. Reset device and wake up (Clears sleep mode)
        regData = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, PWR_MGMT_1_REG, 1, &regData, 1, HAL_TIMEOUT);
        HAL_Delay(10); // Allow clock setups to settle

        // 3. Configure Hardware DLPF (Targeting ~20Hz Cutoff to eliminate chassis resonance)
        // Gyroscope Cutoff: 20Hz, Delay: 9.9ms, Fs: 1kHz
        regData = 0x04;
        HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, CONFIG_REG, 1, &regData, 1, HAL_TIMEOUT);

        // Accelerometer Cutoff: 21.2Hz, Delay: 8.87ms, Fs: 1kHz
        regData = 0x04;
        HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, ACCEL_CONFIG_2_REG, 1, &regData, 1, HAL_TIMEOUT);

        // 4. Set Sample Rate Divider to achieve 100Hz output data rate (ODR = 1kHz / (1 + 9))
        regData = 0x09;
        HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, SMPLRT_DIV_REG, 1, &regData, 1, HAL_TIMEOUT);

        // 5. Accelerometer Configuration -> Range Select: ±8g
        regData = 0x10; // FS_SEL bits [4:3] = 10b
        HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, ACCEL_CONFIG_REG, 1, &regData, 1, HAL_TIMEOUT);

        // 6. Gyroscope Configuration -> Range Select: ±500 deg/s
        regData = 0x08; // FS_SEL bits [4:3] = 01b
        HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, GYRO_CONFIG_REG, 1, &regData, 1, HAL_TIMEOUT);

        // 7. Perform initial stationary base alignment calibration
        MPU6500_Calibrate_Gyro(I2Cx, DataStruct);

        return 0; // Success
    }
    return 1; // Device failure or wiring issue
}

// ============================================================================
// 2. Gyroscope Static Calibration Routine
// ============================================================================
void MPU6500_Calibrate_Gyro(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct) {
    uint8_t rawBuffer[6];
    int32_t sumX = 0, sumY = 0, sumZ = 0;

    // Collect N stable stationary baseline readings
    for (uint16_t i = 0; i < CALIBRATION_SAMPLES; i++) {
        HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, (ACCEL_XOUT_H_REG + 8), 1, rawBuffer, 6, HAL_TIMEOUT);

        sumX += (int16_t)((rawBuffer[0] << 8) | rawBuffer[1]);
        sumY += (int16_t)((rawBuffer[2] << 8) | rawBuffer[3]);
        sumZ += (int16_t)((rawBuffer[4] << 8) | rawBuffer[5]);

        HAL_Delay(1); // Space samples across time domain
    }

    // Compute average zero-rate bias offsets (Bg)
    DataStruct->Gyro_Bias_X = (float)sumX / CALIBRATION_SAMPLES;
    DataStruct->Gyro_Bias_Y = (float)sumY / CALIBRATION_SAMPLES;
    DataStruct->Gyro_Bias_Z = (float)sumZ / CALIBRATION_SAMPLES;
}

// ============================================================================
// 3. Core Runtime Sample & Update Process
// ============================================================================
void MPU6500_Read_All(I2C_HandleTypeDef *I2Cx, MPU6500_t *DataStruct, float dt) {
    uint8_t rawData[14];
    uint8_t powerStatus;

    // Heartbeat check: safeguard against structural drops resetting power states
    HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, PWR_MGMT_1_REG, 1, &powerStatus, 1, HAL_TIMEOUT);
    if ((powerStatus & 0x40) != 0x00) {
        uint8_t wake = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6500_ADDR, PWR_MGMT_1_REG, 1, &wake, 1, HAL_TIMEOUT);
        return;
    }

    // Read full Burst Map Block (14 Bytes: Accel_X to Gyro_Z) in one continuous bus transition
    if (HAL_I2C_Mem_Read(I2Cx, MPU6500_ADDR, ACCEL_XOUT_H_REG, 1, rawData, 14, HAL_TIMEOUT) != HAL_OK) {
        return;
    }

    // Reconstruct Raw Arrays
    DataStruct->Accel_X_RAW = (int16_t)((rawData[0] << 8)  | rawData[1]);
    DataStruct->Accel_Y_RAW = (int16_t)((rawData[2] << 8)  | rawData[3]);
    DataStruct->Accel_Z_RAW = (int16_t)((rawData[4] << 8)  | rawData[5]);
    DataStruct->Gyro_X_RAW  = (int16_t)((rawData[8] << 8)  | rawData[9]);
    DataStruct->Gyro_Y_RAW  = (int16_t)((rawData[10] << 8) | rawData[11]);
    DataStruct->Gyro_Z_RAW  = (int16_t)((rawData[12] << 8) | rawData[13]);

    // Convert Raw Accelerometer values to engineering unit (g) [Sensitivity Matrix: 4096 LSB/g at ±8g]
    DataStruct->Ax = DataStruct->Accel_X_RAW / 4096.0f;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 4096.0f;
    DataStruct->Az = DataStruct->Accel_Z_RAW / 4096.0f;

    // Apply Gyro Calibration Bias Adjustments: ω_calibrated = ω_raw - Bg [Sensitivity Matrix: 65.5 LSB/(deg/s) at ±500dps]
    DataStruct->Gx = ((float)DataStruct->Gyro_X_RAW - DataStruct->Gyro_Bias_X) / 65.5f;
    DataStruct->Gy = ((float)DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Bias_Y) / 65.5f;
    DataStruct->Gz = ((float)DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Bias_Z) / 65.5f;

    // ==========================================
    // A. REST DETECTION & DYNAMIC RECALIBRATION
    // ==========================================
    Push_Window(&DataStruct->AccelWindow, DataStruct->Ax, DataStruct->Ay, DataStruct->Az);
    Push_Window(&DataStruct->GyroWindow, DataStruct->Gx, DataStruct->Gy, DataStruct->Gz);

    if (DataStruct->AccelWindow.filled && DataStruct->GyroWindow.filled) {
        float varAx, varAy, varAz;
        float varGx, varGy, varGz;

        Calc_Variance(&DataStruct->AccelWindow, &varAx, &varAy, &varAz);
        Calc_Variance(&DataStruct->GyroWindow, &varGx, &varGy, &varGz);

        // Evaluate if variance is under threshold bounds (σ² < ε)
        if (varAx < REST_VAR_THRESH_A && varAy < REST_VAR_THRESH_A && varAz < REST_VAR_THRESH_A &&
            varGx < REST_VAR_THRESH_G && varGy < REST_VAR_THRESH_G && varGz < REST_VAR_THRESH_G) {

            DataStruct->IsStationary = true;

            // Execute silent dynamic recalibration using low-pass tracking on bias arrays
            DataStruct->Gyro_Bias_X = (DataStruct->Gyro_Bias_X * 0.999f) + ((float)DataStruct->Gyro_X_RAW * 0.001f);
            DataStruct->Gyro_Bias_Y = (DataStruct->Gyro_Bias_Y * 0.999f) + ((float)DataStruct->Gyro_Y_RAW * 0.001f);
            DataStruct->Gyro_Bias_Z = (DataStruct->Gyro_Bias_Z * 0.999f) + ((float)DataStruct->Gyro_Z_RAW * 0.001f);
        } else {
            DataStruct->IsStationary = false;
        }
    }

    // ==========================================
    // B. DEAD-ZONE FILTER IMPLEMENTATION
    // ==========================================
    if (fabsf(DataStruct->Gx) < GYRO_DEADZONE_X) DataStruct->Gx = 0.0f;
    if (fabsf(DataStruct->Gy) < GYRO_DEADZONE_Y) DataStruct->Gy = 0.0f;
    if (fabsf(DataStruct->Gz) < GYRO_DEADZONE_Z) DataStruct->Gz = 0.0f;

    // Calculate Mathematical Roll & Pitch reference states using Trigonometric Projection
    DataStruct->AccAngleRoll  = atan2f(DataStruct->Ay, DataStruct->Az) * RAD_TO_DEG;
    DataStruct->AccAnglePitch = atan2f(-DataStruct->Ax, sqrtf(DataStruct->Ay * DataStruct->Ay + DataStruct->Az * DataStruct->Az)) * RAD_TO_DEG;

    // ==========================================
    // C. COMPLEMENTARY SENSOR FUSION & INTEGRATION
    // ==========================================
    DataStruct->FinalAngleRoll  = ALPHA * (DataStruct->FinalAngleRoll + DataStruct->Gx * dt) + (1.0f - ALPHA) * DataStruct->AccAngleRoll;
    DataStruct->FinalAnglePitch = ALPHA * (DataStruct->FinalAnglePitch + DataStruct->Gy * dt) + (1.0f - ALPHA) * DataStruct->AccAnglePitch;

    // Absolute Heading Integration via Dead-Reckoning (Forced 0 base if inside deadzone limits)
    DataStruct->MPU_Yaw += DataStruct->Gz * dt;
}

// ============================================================================
// 4. Private Mathematics Helper Core Units
// ============================================================================
static void Push_Window(WindowFilter_t *w, float x, float y, float z) {
    w->x[w->index] = x;
    w->y[w->index] = y;
    w->z[w->index] = z;
    w->index++;
    if (w->index >= VARIANCE_WINDOW) {
        w->index = 0;
        w->filled = true;
    }
}

static void Calc_Variance(WindowFilter_t *w, float *varX, float *varY, float *varZ) {
    float sumX = 0, sumY = 0, sumZ = 0;
    float meanX, meanY, meanZ;
    float sqSumX = 0, sqSumY = 0, sqSumZ = 0;

    for (uint8_t i = 0; i < VARIANCE_WINDOW; i++) {
        sumX += w->x[i]; sumY += w->y[i]; sumZ += w->z[i];
    }
    meanX = sumX / VARIANCE_WINDOW;
    meanY = sumY / VARIANCE_WINDOW;
    meanZ = sumZ / VARIANCE_WINDOW;

    for (uint8_t i = 0; i < VARIANCE_WINDOW; i++) {
        sqSumX += (w->x[i] - meanX) * (w->x[i] - meanX);
        sqSumY += (w->y[i] - meanY) * (w->y[i] - meanY);
        sqSumZ += (w->z[i] - meanZ) * (w->z[i] - meanZ);
    }
    *varX = sqSumX / VARIANCE_WINDOW;
    *varY = sqSumY / VARIANCE_WINDOW;
    *varZ = sqSumZ / VARIANCE_WINDOW;
}
